"""`ros2 run <package> <executable>`로 뜨는 독립 노드 4개를 감싸는 실행 레이어.

semiH_ws/CLAUDE.md 섹션 2-1, 2-4에서 확인된 사실을 그대로 반영한다:
- `semih_perception`(yolo_detection_node, target_3d_node, arm_reach_node)과
  `semih_data_collection`(hdf5_collector_node) 두 패키지에는 launch/
  디렉터리 자체가 없다 — 전부 `ros2 run`으로만 기동되는 순수 노드이며, 이
  워크스페이스 어디에도 이들을 감싸는 launch 파일이 없다.
- 이 4개 노드는 `core/launch_runner.py`가 다루는 `bringup`(gazebo_instance
  자원)과 자원이 겹치지 않는다.

**락 필요 여부는 추측하지 않고 라이브로 확인했다** (semiH_web/CLAUDE.md
3절에 방법·로그 전문 기록). 결론:

- **`hdf5_collector`만 하드 락(동시 1개)이 필요하다.** 저장 파일 경로가
  `output_dir/semih_session_<YYYYMMDD_HHMMSS>.h5` (초 단위 타임스탬프,
  세션ID/PID 없음)로 duration_sec 경과 시점(저장 시점)에만 계산된다.
  같은 output_dir로 거의 동시에 띄운 두 인스턴스가 저장 시점까지 같은 초
  안에 들어오면 파일명이 충돌한다 — 라이브 재현 결과, "안전하게 하나가
  덮어씀"이 아니라 **h5py/HDF5 자체의 파일 락 때문에 나중에 `h5py.File(...,
  'w')`를 연 프로세스가 `BlockingIOError: Unable to synchronously create
  file (unable to lock file, errno=11)`로 크래시**했다(그 프로세스가
  수집하던 세션 데이터는 통째로 유실, 먼저 연 쪽은 정상 저장됨). 즉
  카테고리 (c): 두 번째 실행이 자체적으로 에러내고 죽는다 — 그렇다고 안전은
  아니고(데이터 유실 발생), 웹 레이어에서 애초에 막는 게 맞다고 판단했다.
- **`yolo_detection`/`target_3d`/`arm_reach`는 중복 실행이 안전하다.** 셋
  다 라이브로 두 인스턴스씩 동시에 띄워 확인: 크래시/에러 없음, 대상
  토픽(`/yolo/image_annotated`, `/target_point`)에 발행자 2개가 잡히는 것도
  `ros2 topic info -v`로 확인했지만 각자 독립적으로 정상 동작(다만 완전히
  같은 입력을 보므로 결과가 사실상 중복 — 낭비일 뿐 위험은 아님).
  `arm_reach_node` 두 인스턴스에 같은 `/target_point`를 반복 publish해도
  `arm_controller` 액션 서버가 두 클라이언트의 목표를 에러 없이 받아
  처리했고 최종 관절 상태도 의도한 값으로 정상 수렴했다(다만 이 테스트는
  두 인스턴스가 "같은" 목표를 보낸 경우만 재현한 것 — 서로 다른 목표를
  경쟁적으로 보낼 때의 궤적 흔들림 자체는 재현하지 않았음, 액션 서버가
  새 목표로 이전 목표를 프리엠프트하는 것은 ros2_control의 표준 동작이라
  크래시로 이어지지는 않을 것으로 판단하되 이 부분은 "실측"이 아니라
  "표준 동작 근거"임을 구분해서 기록).

그래서 이 파일은 두 가지 락 정책을 구분해서 쓴다:
- `hdf5_collector`: **고정 stage 이름**(`"hdf5_collector"`)으로
  `process_manager`를 호출 — 그 자체가 O_CREAT|O_EXCL 원자적 락이라
  두 번째 시작 시도는 항상 `JobConflictError`.
- 나머지 3개: **인스턴스마다 고유한 stage 키**(`f"{node.value}__{uuid}"`)
  로 `process_manager`를 호출 — 여러 개가 동시에 존재할 수 있고, 서로
  다른 인스턴스라 락이 부딪히지 않는다. `get_node_status()`는 이 경우
  "이미 몇 개 떠 있는지"를 보여주는 목록을 반환한다(1절 "안전한 노드는
  락 없이 상태 추적만" 요구사항).

**셸 래퍼 여부(라이브 확인)**: `ros2 run <pkg> <exe>`는 `ros2run/api.py`
소스 확인 결과 `subprocess.Popen([path] + argv)`로 대상 실행 파일을 직접
띄운다 — `/bin/sh -c`류 셸 래퍼를 전혀 거치지 않는다(gazebo 사례와 다름).
다만 `ros2 run` 프로세스 자신과 실제 노드 프로세스는 여전히 별개의 OS
프로세스 2개(부모/자식)이고, `ros2run/api.py`의 `run_executable()`은 자식이
"신호를 알아서 받아 죽을 것"이라고 가정만 할 뿐 신호를 명시적으로
릴레이하지 않는다 — 즉 부모(`ros2 run`) 프로세스 하나에만 신호를 보내면
자식(진짜 노드)은 프로세스 그룹에 속해 있어야만 같이 신호를 받는다. 라이브
확인(`ps -eo pid,ppid,pgid`)으로 `ros2 run`과 그 자식이 항상 같은 pgid를
공유함을 확인했다 — `core/process_manager.start_job()`이 이미
`start_new_session=True`로 모든 job을 그 job 자신이 pgid인 새 프로세스
그룹으로 띄우고, `stop_job()`이 항상 `os.killpg()`를 쓰므로(2절 근거),
이 4개 노드도 별도 처리 없이 동일한 메커니즘으로 부모+자식이 함께
종료된다 — bringup과 마찬가지로 셸 래퍼가 없어도 프로세스 그룹 단위
종료가 필요하다는 결론은 같다.

**semiH_ws 내부 파일은 이 모듈에서도 절대 읽기 외의 방식으로 수정/이동/
복사하지 않는다.**
"""

from __future__ import annotations

import enum
import uuid
from pathlib import Path

import yaml

from core import process_manager

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SETTINGS_PATH = PROJECT_ROOT / "config" / "settings.yaml"

# 이 노드들은 락 없이 여러 인스턴스가 동시에 떠 있을 수 있다(위 docstring
# 라이브 검증 근거) -- stage 키를 인스턴스마다 다르게 만들어 서로 막지 않는다.
_MULTI_INSTANCE_NODES: set[str] = {"yolo_detection", "target_3d", "arm_reach"}
# hdf5_collector만 고정 stage 이름으로 단일 인스턴스 하드 락을 건다.
_SINGLE_INSTANCE_NODES: set[str] = {"hdf5_collector"}


class NodeName(str, enum.Enum):
    """`ros2 run`으로 개별 기동 가능한 4개 독립 노드 (semiH_ws/CLAUDE.md
    섹션 2-1 확인 기준 — 이 4개 외의 launch-less 노드가 이 워크스페이스에
    더 있는지는 이번 조사 범위에서 전수 확인되지 않았으므로, 새 노드를
    추가할 때는 semiH_ws 소스를 먼저 재확인할 것).
    """

    YOLO_DETECTION = "yolo_detection"
    TARGET_3D = "target_3d"
    ARM_REACH = "arm_reach"
    HDF5_COLLECTOR = "hdf5_collector"


# NodeName -> (ros2 package, executable). semiH_ws/CLAUDE.md 섹션 2-1,
# docs/usage_guide.md 111~154행에 문서화된 실제 `ros2 run` 커맨드 기준.
NODE_REGISTRY: dict[NodeName, tuple[str, str]] = {
    NodeName.YOLO_DETECTION: ("semih_perception", "yolo_detection_node"),
    NodeName.TARGET_3D: ("semih_perception", "target_3d_node"),
    NodeName.ARM_REACH: ("semih_perception", "arm_reach_node"),
    NodeName.HDF5_COLLECTOR: ("semih_data_collection", "hdf5_collector_node"),
}


def _load_settings() -> dict:
    """`config/settings.yaml`을 읽어 dict로 반환한다 (core/launch_runner.py
    의 동명 함수와 동일한 로직 — 공유 설정 모듈이 아직 없어 중복돼 있음,
    TODO: 3개 이상으로 늘어나면 별도 config 모듈로 뽑을 것).
    """
    if not SETTINGS_PATH.exists():
        return {}
    with open(SETTINGS_PATH, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _default_ros_args(node: NodeName) -> dict[str, str]:
    """`config/settings.yaml`에서 이 노드에 쓸 만한 기본 `--ros-args -p` 값을
    끌어온다. 사용자가 `start_node()`에 넘긴 `ros_args`가 있으면 그게
    우선한다(덮어씀).

    - yolo_detection / target_3d: `model_name`을 `perception.yolo_model_path`
      절대경로로 넘긴다 — semiH_ws/CLAUDE.md 섹션 2-6에서 확인된, 상대경로
      `'yolov8n.pt'`가 CWD에 의존하는 문제를 웹 레이어에서 없애기 위함.
    - hdf5_collector: `output_dir`을 `dataset.output_path`로 넘긴다.
    - arm_reach: 넘길 기본값 없음(전부 노드 자체 기본값 사용).
    """
    settings = _load_settings()
    if node in (NodeName.YOLO_DETECTION, NodeName.TARGET_3D):
        model_path = settings.get("perception", {}).get("yolo_model_path")
        return {"model_name": model_path} if model_path else {}
    if node is NodeName.HDF5_COLLECTOR:
        output_path = settings.get("dataset", {}).get("output_path")
        return {"output_dir": output_path} if output_path else {}
    return {}


def build_node_command(node: NodeName, ros_args: dict[str, str] | None = None) -> list[str]:
    """`ros2 run <package> <executable> --ros-args -p key:=value ...`를
    실행하는 argv 리스트를 조립해 반환한다.

    ROS setup.bash + semiH_ws install/setup.bash를 source한 뒤 `exec`로
    `ros2 run`을 실행하는 셸 명령으로 감싼다 — `core/launch_runner.py`의
    `build_bringup_command()`와 동일한 이유(중간에 안 죽는 bash 껍데기가
    안 남게, pid가 진짜 `ros2 run` 프로세스가 되게).

    `ros_args`는 `_default_ros_args(node)`가 주는 기본값 위에 사용자가
    넘긴 값을 덮어써서 합친다.

    Args:
        node: NODE_REGISTRY에 등록된 노드 중 하나.
        ros_args: `--ros-args -p`로 전달할 파라미터 딕셔너리(선택).

    Returns:
        subprocess에 넘길 argv 리스트.
    """
    package, executable = NODE_REGISTRY[node]
    settings = _load_settings()
    workspace = settings.get("workspace", {})
    ros_setup_bash = workspace["ros_setup_bash"]
    install_setup_bash = workspace["install_setup_bash"]

    merged_args = {**_default_ros_args(node), **(ros_args or {})}

    ros2_run_cmd = f"ros2 run {package} {executable}"
    if merged_args:
        param_flags = " ".join(f"-p {k}:={v}" for k, v in merged_args.items())
        ros2_run_cmd += f" --ros-args {param_flags}"

    shell_cmd = (
        f"source {ros_setup_bash} && "
        f"source {install_setup_bash} && "
        f"exec {ros2_run_cmd}"
    )
    return ["bash", "-c", shell_cmd]


def _stage_for_new_instance(node: NodeName) -> str:
    """이 node의 새 인스턴스를 시작할 때 쓸 process_manager stage 키를
    정한다. 단일 인스턴스 노드는 고정 이름(하드 락), 다중 인스턴스 노드는
    매번 새로운 고유 키(락 없음, 상태 추적만) — 모듈 docstring 근거.
    """
    if node.value in _SINGLE_INSTANCE_NODES:
        return node.value
    return f"{node.value}__{uuid.uuid4().hex[:8]}"


def _running_instances(node: NodeName) -> list[dict]:
    """`node`에 대응하는 현재 실행 중인 job 상태를 전부 모아 반환한다.

    단일 인스턴스 노드는 고정 stage 하나만 확인하면 되지만, 다중 인스턴스
    노드는 `state/locks/`에서 `f"{node.value}__"`로 시작하는 stage를 전부
    찾아야 한다 — process_manager는 이런 접두어 스캔 API를 따로 제공하지
    않으므로(원래 so101_web 패턴은 stage 하나당 lock 하나였음), 여기서
    `process_manager.LOCKS_DIR`을 직접 glob한다.
    """
    if node.value in _SINGLE_INSTANCE_NODES:
        status = process_manager.get_job_status(node.value)
        return [status] if status["running"] else []

    results = []
    prefix = f"{node.value}__"
    for path in process_manager.LOCKS_DIR.glob(f"{prefix}*.json"):
        stage = path.stem
        status = process_manager.get_job_status(stage)
        if status["running"]:
            results.append({"stage": stage, **status})
    return results


def start_node(node: NodeName, ros_args: dict[str, str] | None = None) -> dict:
    """새 노드 인스턴스를 시작한다.

    `hdf5_collector`는 이미 하나가 실행 중이면 `process_manager.
    JobConflictError`가 전파된다(모듈 docstring 근거 — 파일명 충돌로 인한
    크래시/데이터 유실 실측 확인). 나머지 3개 노드는 몇 개가 이미 떠
    있든 항상 새 인스턴스로 시작된다(고유 stage 키를 매번 새로 발급하므로
    `process_manager` 레벨의 충돌이 원천적으로 없음).

    Returns:
        시작된 job의 상태 dict. 다중 인스턴스 노드의 경우 `"stage"` 키에
        이번에 발급된 고유 stage 이름이 들어있다(나중에 `stop_node()`로
        이 인스턴스만 콕 집어 종료하려면 필요).
    """
    stage = _stage_for_new_instance(node)
    cmd = build_node_command(node, ros_args=ros_args)
    result = process_manager.start_job(stage, cmd, extra_meta={"node": node.value})
    return {"stage": stage, **result}


def stop_node(node: NodeName, stage: str | None = None, graceful: bool = True) -> bool:
    """실행 중인 노드 인스턴스를 종료한다.

    - `hdf5_collector`(단일 인스턴스 노드): `stage` 인자는 무시하고 고정
      stage 이름으로 종료한다.
    - 다중 인스턴스 노드: `stage`를 명시하면 그 인스턴스만 종료. 생략하면
      `_running_instances(node)`로 찾은 **모든** 실행 중인 인스턴스를
      전부 종료한다(편의 기능 — "이 노드 종류 전부 끄기").

    항상 `process_manager.stop_job(..., graceful=graceful)`을 거치므로
    프로세스 그룹 전체(`os.killpg`)가 대상이 된다 — `ros2 run`이 셸
    래퍼는 안 쓰지만 부모/자식 2개 프로세스 구조라는 점은 bringup과
    같아서 동일한 방식이 필요함(모듈 docstring 근거).

    Returns:
        하나 이상 종료를 시도했으면 True, 애초에 실행 중인 게 없었으면 False.
    """
    if node.value in _SINGLE_INSTANCE_NODES:
        return process_manager.stop_job(node.value, graceful=graceful)

    if stage is not None:
        return process_manager.stop_job(stage, graceful=graceful)

    instances = _running_instances(node)
    if not instances:
        return False
    stopped_any = False
    for info in instances:
        if process_manager.stop_job(info["stage"], graceful=graceful):
            stopped_any = True
    return stopped_any


def get_node_status(node: NodeName) -> dict | list[dict]:
    """`node`의 현재 실행 상태를 반환한다.

    - 단일 인스턴스 노드(`hdf5_collector`): `process_manager.get_job_status()`
      와 동일한 형태의 dict 하나 (`{"running": bool, ...}`).
    - 다중 인스턴스 노드: `_running_instances(node)`가 주는 리스트 그대로
      (0개, 1개, 또는 여러 개일 수 있음 — "이미 떠 있음"을 개수로 보여주는
      용도, 1절 "락 없이 상태 추적만" 요구사항).
    """
    if node.value in _SINGLE_INSTANCE_NODES:
        return process_manager.get_job_status(node.value)
    return _running_instances(node)


def list_available_nodes() -> list[NodeName]:
    """NODE_REGISTRY에 등록된 모든 노드 이름을 반환한다 (UI에서 목록을 그릴 때 사용)."""
    return list(NODE_REGISTRY.keys())
