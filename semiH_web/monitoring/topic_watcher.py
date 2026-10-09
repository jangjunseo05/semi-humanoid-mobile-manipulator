"""프로세스 상태 조회 전용 레이어 — `pages/main.py`가 한 번에 쓰기 좋은
형태로 `core/launch_runner.py`(bringup/gazebo_instance)와
`core/node_runner.py`(YOLO/target_3d/arm_reach/HDF5 4개 노드)의 상태를
합쳐 보여준다.

**이 모듈은 새 프로세스 관리 로직을 만들지 않는다.** `process_manager`가
이미 각 job의 PID/lock/로그 경로를 디스크(`state/locks/*.json`)에 기록해
추적하고 있고, `launch_runner`/`node_runner`는 그걸 읽는 thin wrapper다 —
`get_system_status()`는 그 wrapper들을 호출한 결과를 한데 모아 재구성만
할 뿐, 자체적으로 lock 파일을 읽거나 subprocess 상태를 새로 조회하지
않는다.

**이번 스코프에서 의도적으로 제외한 것**: 토픽이 실제로 발행되는지
(`ros2 topic hz` 등), `/clock`이 도는지 같은 ROS 그래프 수준의 건강 확인은
포함하지 않는다 — "프로세스가 살아있는가"만 다룬다("프로세스는 떠 있지만
실질적으로 멈춰 있던" semiH_ws/CLAUDE.md 섹션 1 A-5 사례처럼 이 둘이
다르다는 건 알지만, 그 확인은 다음 단계로 미룬다). 아래
`is_topic_publishing`/`get_topic_hz`/`get_active_topics`/
`get_bringup_health`/`get_node_health`는 그 다음 단계를 위해 남겨둔
TODO 스켈레톤이며, `get_system_status()`는 이것들을 전혀 쓰지 않는다.
"""

from __future__ import annotations

from core import launch_runner, node_runner


def _normalize_node_status(node: node_runner.NodeName) -> dict:
    """`node_runner.get_node_status(node)`의 결과(락 노드는 dict, 비-락
    노드는 list)를 `pages/main.py`가 노드 종류를 신경 쓰지 않고 동일하게
    다룰 수 있는 하나의 형태로 통일한다.

    `node_runner.get_node_status()`의 반환 타입 자체가 "이 노드가 하드
    락 노드인지(dict) 아닌지(list)"를 그대로 드러내므로, 그 타입을 보고
    `locked`를 판정한다 — `node_runner`의 내부 구현(`_SINGLE_INSTANCE_NODES`
    등 비공개 이름)을 들여다보지 않고 공개 API의 반환 모양만으로 판단한다
    (semiH_web/CLAUDE.md 3절에서 확정한 정책과 일치: hdf5_collector만 락).
    """
    raw = node_runner.get_node_status(node)

    if isinstance(raw, dict):
        # 락 노드(hdf5_collector): {"running": bool, ...} 하나.
        instances = [raw] if raw.get("running") else []
        return {"locked": True, "instance_count": len(instances), "instances": instances}

    # 비-락 노드(yolo_detection/target_3d/arm_reach): list[dict], 원소마다
    # "stage"(인스턴스 고유 키)가 이미 들어있음(node_runner._running_instances).
    return {"locked": False, "instance_count": len(raw), "instances": raw}


def get_system_status() -> dict:
    """bringup(gazebo_instance 락)과 4개 독립 노드의 현재 상태를 한 번에
    조회해 `pages/main.py`가 바로 렌더링할 수 있는 dict로 반환한다.

    반환 형태:
        {
            "bringup": {
                "running": bool,
                "mode": str | None,       # extra_meta로 저장된 값 (sim/slam/nav/explore)
                "headless": bool | None,
                "pid": int | None,
            },
            "rviz": {"running": bool, "pid": int | None},
            "image_view": {"running": bool, "pid": int | None, "topic": str | None},
            "nodes": {
                "yolo_detection": {"locked": False, "instance_count": int, "instances": [...]},
                "target_3d":      {"locked": False, "instance_count": int, "instances": [...]},
                "arm_reach":      {"locked": False, "instance_count": int, "instances": [...]},
                "hdf5_collector": {"locked": True,  "instance_count": 0|1, "instances": [...]},
            },
        }

    `instances`의 각 원소는 `process_manager`가 lock 파일에 적어둔 그대로
    (`pid`, `started_at`, `cmd`, `log_path`, 그리고 비-락 노드는 `stage`)다
    — 이 함수는 그 값을 다시 가공하지 않고 그대로 통과시킨다(조회 전용
    원칙).

    `rviz`는 2026-08-28에 추가된 `core/launch_runner.get_rviz_status()`의
    thin 재구성 -- bringup/노드와 마찬가지로 이 함수가 직접 lock을 읽지
    않는다는 원칙은 그대로 유지.
    """
    bringup_raw = launch_runner.get_bringup_status()
    bringup = {
        "running": bringup_raw.get("running", False),
        "mode": bringup_raw.get("mode"),
        "headless": bringup_raw.get("headless"),
        "pid": bringup_raw.get("pid"),
    }

    rviz_raw = launch_runner.get_rviz_status()
    rviz = {"running": rviz_raw.get("running", False), "pid": rviz_raw.get("pid")}

    # image_view(rqt_image_view, 2026-08-31 추가): YOLO 인식 결과 창. rviz와
    # 똑같이 launch_runner의 thin wrapper를 통과시킬 뿐 직접 lock을 읽지 않는다.
    image_view_raw = launch_runner.get_image_view_status()
    image_view = {
        "running": image_view_raw.get("running", False),
        "pid": image_view_raw.get("pid"),
        "topic": image_view_raw.get("topic"),
    }

    nodes = {node.value: _normalize_node_status(node) for node in node_runner.NodeName}

    return {
        "bringup": bringup,
        "rviz": rviz,
        "image_view": image_view,
        "nodes": nodes,
    }


# --- 아래는 다음 단계(ROS 그래프 수준 건강 확인)를 위해 남겨둔 TODO
# 스켈레톤이다. get_system_status()는 이것들을 쓰지 않는다 (위 모듈
# docstring 참고 — 이번 스코프에서 의도적으로 제외).


def is_topic_publishing(topic: str, timeout_s: float = 3.0) -> bool:
    """`topic`이 `timeout_s`초 안에 최소 1개 이상의 메시지를 발행하는지 확인한다.

    Args:
        topic: 확인할 토픽 이름 (예: "/clock", "/scan", "/camera/image_raw").
        timeout_s: 대기 시간(초).

    Returns:
        메시지가 하나라도 관측되면 True, 타임아웃까지 없으면 False.
    """
    raise NotImplementedError("TODO: 다음 단계 (이번 스코프 제외)")


def get_topic_hz(topic: str, timeout_s: float = 5.0) -> float | None:
    """`topic`의 평균 발행 주파수(Hz)를 측정해 반환한다. 측정 실패/토픽 없음이면 None.

    `ros2 topic hz`가 내부적으로 첫 메시지가 올 때까지 아무 출력도 안 하는
    특성이 있으므로(semiH_ws 진단 세션에서 실측 확인된 동작), timeout_s 안에
    메시지가 하나도 안 오면 None을 반환하도록 구현할 것(TODO).
    """
    raise NotImplementedError("TODO: 다음 단계 (이번 스코프 제외)")


def get_active_topics() -> list[str]:
    """현재 ROS 2 그래프에 존재하는 전체 토픽 이름 목록을 반환한다
    (`ros2 topic list` 또는 rclpy `get_topic_names_and_types()` 기반, TODO).
    """
    raise NotImplementedError("TODO: 다음 단계 (이번 스코프 제외)")


def get_bringup_health(mode: str) -> dict:
    """현재 실행 중인 bringup(mode: sim/slam/nav/explore)의 핵심 토픽들이
    실제로 살아있는지 종합해 반환한다 — "프로세스 생존"(get_system_status)
    과 "실제로 동작 중"(이 함수)을 구분해서 보여주는 것이 목적
    (semiH_ws/CLAUDE.md 섹션 1 A-5: 프로세스는 떠 있지만 costmap이 멈춰
    있던 사례).

    확인 대상은 mode에 따라 달라야 한다(TODO, 예시 — 확정 아님):
    - 모든 mode 공통: `/clock`(spawn_gazebo.launch.py가 항상 브릿지 —
      semiH_ws/CLAUDE.md 섹션 2-5), `/scan`, `/camera/image_raw`, `/odom`.
    - slam 이상: `/map` 발행 여부.
    - nav 이상: `/local_costmap/costmap`, `/global_costmap/costmap` 갱신 여부.
    - explore: 추가로 `/cmd_vel`이 0이 아닌 값을 내는지, `/explore/frontiers`
      가 갱신되는지.
    """
    raise NotImplementedError("TODO: 다음 단계 (이번 스코프 제외)")


def get_node_health(node: str) -> dict:
    """`core/node_runner.NodeName` 각각에 대응하는 출력 토픽이 실제로
    발행되는지 확인한다.

    예시(TODO, 확정 아님): yolo_detection -> `/yolo/image_annotated`,
    target_3d -> `/target_point`, hdf5_collector -> 자체 로그 기반 프레임
    수 카운트(토픽이 아니라 로그 파싱이 필요할 수 있음 — 이 경우
    `core/process_manager.tail_log()`와 조합해서 구현할 것). arm_reach는
    `/target_point`를 못 받으면 아무 출력도 없는 게 정상 상태라 "발행
    안 함 = 비정상"으로 못 씀 — 확인 방식 자체를 다시 설계해야 함(TODO).
    """
    raise NotImplementedError("TODO: 다음 단계 (이번 스코프 제외)")
