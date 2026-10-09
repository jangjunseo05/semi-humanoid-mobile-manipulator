"""semiH_web 단일 진입점 (Streamlit 앱).

so101_web(`dashboard/app.py` + `dashboard/lib/ui_components.py`,
`dashboard/lib/next_action.py`, `dashboard/pages/1_데이터_수집.py` 등을
실제로 읽고 확인함 — C:\\Users\\USER\\Desktop\\so101_web)의 검증된 패턴을
톤/컴포넌트 스타일 기준으로 그대로 따른다:
- 카드 헤더 색은 장식이 아니라 역할(action/settings/success/warning)에
  고정 — `_render_card_header()`가 so101_web의 `render_card_header()`와
  동일한 색상표를 쓴다.
- 차단 상태일 땐 카드 헤더 자체를 warning 색으로 바꾸고, 차단 사유를
  버튼보다 위에 캡션으로 보여준다(so101_web 1_데이터_수집.py 66~100행
  패턴 그대로).
- 홈(이 페이지 최상단)은 정적 설명이 아니라 `get_system_status()` 결과를
  해석한 동적 CTA — so101_web의 `next_action.get_next_action()`과 동일한
  "첫 매치 채택" 우선순위 방식.

semiH_web은 so101_web과 달리 파이프라인 단계가 적어(환경 1개 + 독립 노드
4개) 다중 페이지가 아니라 **단일 페이지**로 통합했다(semiH_web/CLAUDE.md
1-4절 결정) — 그래서 `next_action`처럼 다른 페이지로 이동시키는 링크는
없고, 같은 페이지 안 어느 카드를 봐야 하는지 안내만 한다.

**이 파일은 새 프로세스 관리/상태 판단 로직을 만들지 않는다.** 모든 상태는
`monitoring.topic_watcher.get_system_status()` 하나로만 조회하고,
모든 조작은 `core.launch_runner`/`core.node_runner`의 이미 구현된 함수만
호출한다. `_hdf5_save_detected()`만 예외로, `process_manager.tail_log()`가
이미 반환하는 텍스트를 읽어 "저장 완료" 여부를 페이지 코드 안에서
해석하는데, 이는 so101_web의 `pm.looks_like_crash(tail)`을 페이지
코드에서 직접 호출하는 것과 동일한 선례를 따른 것이지 새 핵심 로직이
아니다(semiH_web/CLAUDE.md 5절 "알려진 제약 1번"의 UI 반영).

실행: `streamlit run pages/main.py` (semiH_web 루트에서, 가상환경에
requirements.txt 설치 후)
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import streamlit as st
import yaml
from streamlit_autorefresh import st_autorefresh

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from core import launch_runner, node_runner, process_manager  # noqa: E402
from lib import ui_components  # noqa: E402
from monitoring import topic_watcher  # noqa: E402

BringupMode = launch_runner.BringupMode
NodeName = node_runner.NodeName

SETTINGS_PATH = PROJECT_ROOT / "config" / "settings.yaml"

# role -> (배경색, 글자색). so101_web dashboard/lib/ui_components.py의
# _ROLE_STYLES와 동일한 색상표(BLUE/TEXT_SECONDARY/AQUA/YELLOW) -- "카드
# 색상은 의미 고정" 원칙을 그대로 재사용.
_ROLE_STYLES = {
    "action": ("#2a78d6", "#ffffff"),
    "settings": ("#52514e", "#ffffff"),
    "success": ("#1baf7a", "#ffffff"),
    "warning": ("#eda100", "#2b2200"),
}

_MODE_RANK = {None: 0, "sim": 1, "slam": 2, "nav": 3, "explore": 4}

_MODE_BUTTONS = [
    (BringupMode.SIM, "🟢 sim"),
    (BringupMode.SLAM, "🗺️ slam"),
    (BringupMode.NAV, "🧭 nav"),
    (BringupMode.EXPLORE, "🚀 explore"),
]

# 2026-08-28 추가: "explore라는 이름만으로는 뭘 하는지 추측하기 어렵다"는
# 피드백 반영 -- 각 mode 버튼 옆 popover(❓ 토글)에 이 설명을 그대로 띄운다
# (render_environment_card). mode는 누적이라 뒤로 갈수록 앞 문구를
# 반복하지 않고 "+ 추가되는 것"만 적었다.
_MODE_DESCRIPTIONS = {
    BringupMode.SIM: (
        "Gazebo에 로봇만 스폰합니다. 지도 작성(SLAM)도, 자율주행(Nav2)도 "
        "켜지지 않습니다 — 로봇 모델/센서가 정상인지 확인하거나 팔 조작만 "
        "테스트하고 싶을 때 적합합니다."
    ),
    BringupMode.SLAM: (
        "sim + **SLAM**(slam_toolbox 지도 작성). 라이다로 주변을 스캔해 "
        "실시간으로 지도를 만듭니다. 아직 자율주행 기능은 없어서, 지도를 "
        "채우려면 로봇을 직접 움직여야 합니다(예: teleop, `/cmd_vel` 발행)."
    ),
    BringupMode.NAV: (
        "sim + SLAM + **Nav2**(자율주행). 목표 지점을 주면 장애물을 피해 "
        "경로를 계산해 이동합니다 — **다만 목표는 로봇이 스스로 정하지 "
        "않습니다.** 사람이 RViz에서 지도를 클릭해 목표를 줘야 움직입니다 "
        "(아래 'RViz' 카드로 열 수 있음). 인식·수집 카드(YOLO 등)도 이 "
        "mode부터 활성화됩니다."
    ),
    BringupMode.EXPLORE: (
        "nav가 하는 것 전부 + **목표를 사람이 안 줘도 로봇이 스스로** "
        "미탐색 영역을 찾아다니며 지도를 완성합니다(자율 탐색). 더 이상 "
        "갈 곳이 없으면 스스로 멈춥니다. 시작 20초 후쯤 로봇이 잠깐 앞으로 "
        "움직이는 게 보이는데, 이는 탐색을 시작하기 전 스폰 위치를 코스트맵 "
        "경계에서 벗어나기 위한 의도된 동작입니다(버그 아님)."
    ),
}

_PERCEPTION_NODES = [NodeName.YOLO_DETECTION, NodeName.TARGET_3D, NodeName.ARM_REACH]

_NODE_DISPLAY = {
    NodeName.YOLO_DETECTION: ("🔍", "YOLO 인식", "yolo_detection_node"),
    NodeName.TARGET_3D: ("📐", "3D 좌표 추출", "target_3d_node"),
    NodeName.ARM_REACH: ("🦾", "팔 뻗기 (인식한 물체로 이동)", "arm_reach_node"),
    NodeName.HDF5_COLLECTOR: ("💾", "HDF5 데이터 수집", "hdf5_collector_node"),
}

# 2026-08-28 추가: "'팔 리치' 이름만으로는 뭘 하는지 모호하다"는 피드백
# 반영 -- 카드 헤더 옆 popover에 이 설명을 띄운다. 4개 노드가 이루는
# 파이프라인 순서(YOLO -> 3D 좌표 -> 팔 뻗기)를 각 설명 안에서 서로
# 언급해 "왜 이 순서로 켜야 하는지"가 개별 카드만 봐도 드러나게 했다.
_NODE_DESCRIPTIONS = {
    NodeName.YOLO_DETECTION: (
        "카메라 영상에서 물체를 실시간으로 탐지합니다(YOLOv8, COCO 80종). "
        "탐지 결과를 그려 넣은 영상을 `/yolo/image_annotated` 토픽으로 "
        "발행합니다 — 시작 버튼 위 '인식 결과 창도 함께 열기'가 켜져 있으면 "
        "그 토픽을 보여주는 rqt_image_view 창이 같이 뜹니다. 아래 '3D 좌표 "
        "추출'과 '팔 뻗기'가 동작하려면 **이게 먼저 켜져 있어야** 합니다."
    ),
    NodeName.TARGET_3D: (
        "YOLO가 찾은 물체의 화면상 위치를 뎁스(깊이) 카메라 정보와 결합해 "
        "실제 3D 공간 좌표(x, y, z)로 변환합니다. `/target_point` 토픽으로 "
        "발행하며, '팔 뻗기'가 이 좌표를 받아서 씁니다. YOLO 인식이 먼저 "
        "켜져 있어야 의미 있는 값이 나옵니다."
    ),
    NodeName.ARM_REACH: (
        "'3D 좌표 추출'이 계산한 목표 좌표(`/target_point`)를 받아 로봇 "
        "팔의 역기구학(IK)을 계산하고, 팔을 그 위치까지 움직입니다 — 즉 "
        "**카메라로 인식한 물체 쪽으로 팔을 뻗는** 기능입니다. 앞의 두 "
        "노드가 먼저 켜져 좌표를 발행하고 있어야 실제로 반응합니다."
    ),
    NodeName.HDF5_COLLECTOR: (
        "카메라 · 깊이 · 관절 상태 등을 동기화해서 `duration_sec` 동안 "
        "`.h5` 파일로 저장합니다(학습용 데이터 수집). ⚠️ 저장이 끝나도 "
        "프로세스가 자동으로 꺼지지 않으니, 완료 후에는 직접 정지 버튼을 "
        "눌러야 합니다."
    ),
}

_RVIZ_DESCRIPTION = (
    "지도, 로봇 위치, 라이다 스캔을 3D로 보여주는 뷰어이자 — Nav2에게 "
    "**목표 지점을 클릭으로 줄 수 있는 유일한 방법**입니다. `nav` mode는 "
    "목표가 있어야 움직이는데 `bringup.launch.py`가 RViz를 자동으로 켜주지 "
    "않아서 이 버튼을 따로 뒀습니다. 열리면 툴바의 'Set Goal' 아이콘을 "
    "누르고 지도 위에서 원하는 위치·방향으로 드래그하세요. `explore` "
    "mode처럼 로봇이 스스로 움직이는 경우엔 목표를 줄 필요 없이 지켜보는 "
    "용도로만 써도 됩니다."
)


def _render_card_header(icon: str, title: str, role: str) -> None:
    """`st.container(border=True)` 블록의 첫 줄에서 호출한다 (so101_web
    `render_card_header()`와 동일한 패턴/색상표)."""
    if role not in _ROLE_STYLES:
        raise ValueError(f"unknown card role: {role!r}")
    bg, fg = _ROLE_STYLES[role]
    st.markdown(
        f'<div style="background-color:{bg};color:{fg};padding:0.4em 0.9em;'
        f'border-radius:0.4em;margin-bottom:0.7em;font-weight:600;font-size:1.05em;">'
        f"{icon} {title}</div>",
        unsafe_allow_html=True,
    )


def _load_settings() -> dict:
    """`config/settings.yaml`을 읽어 dict로 반환한다 (표시 전용 —
    `core/launch_runner.py`/`core/node_runner.py`도 각자 이 파일을 읽지만
    그건 이 함수와 별개다, 아직 공유 config 모듈이 없다는 기존 TODO 그대로).
    """
    if not SETTINGS_PATH.exists():
        return {}
    with open(SETTINGS_PATH, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _mode_rank(mode: str | None) -> int:
    return _MODE_RANK.get(mode, 0)


def _hdf5_save_detected(status: dict) -> bool:
    """HDF5 인스턴스가 실행 중이면서 이미 "Saved N synchronized frames
    to: ..." 로그를 남겼는지 확인한다 — `process_manager.tail_log()`가
    반환하는 텍스트를 읽기만 할 뿐, 새 상태 저장/판정 로직을 만들지
    않는다(모듈 docstring 참고). semiH_web/CLAUDE.md 5절 "HDF5는 저장
    후에도 프로세스가 안 죽는다"는 알려진 제약의 UI 반영.
    """
    hdf5 = status["nodes"]["hdf5_collector"]
    if not hdf5["instances"]:
        return False
    log_path = hdf5["instances"][0].get("log_path")
    if not log_path:
        return False
    tail = process_manager.tail_log(log_path, n_lines=30)
    return "synchronized frames to" in tail


def get_next_action(status: dict) -> dict:
    """`{"role": str, "icon": str, "message": str}` -- 현재 상태를 해석해
    "다음 할 일"을 우선순위 순으로 판단한다(so101_web
    `next_action.get_next_action()`과 동일한 "첫 매치 채택" 방식).

    우선순위(위에서부터):
      1. HDF5가 저장을 마쳤는데 아직 떠 있음 -- 지금 당장 조치가 필요한
         경고이므로 다른 무엇보다 위(버튼보다도 위).
      2. 환경(bringup) 자체가 꺼져 있음 -- 아무것도 못 하는 상태.
      3. 환경은 떠 있지만 mode가 nav 미만(sim/slam) -- 인식/수집 카드가
         전부 비활성화된 상태이므로 그 이유를 안내.
      4. 환경이 nav 이상인데 인식/수집 노드가 하나도 안 떠 있음 -- 다음
         행동을 제안.
      5. 그 외(뭔가 정상적으로 돌고 있음) -- 성공/중립 상태 표시.
    """
    if status["nodes"]["hdf5_collector"]["instance_count"] > 0 and _hdf5_save_detected(status):
        return {
            "role": "warning",
            "icon": "⚠️",
            "message": "HDF5 저장이 완료됐지만 프로세스가 아직 실행 중입니다 — "
            "아래 '인식 · 수집' 카드에서 정지 버튼을 눌러 종료하세요.",
        }

    if not status["bringup"]["running"]:
        return {
            "role": "action",
            "icon": "🚀",
            "message": "환경이 꺼져 있습니다 — 아래 '환경' 카드에서 mode를 먼저 선택해 시작하세요.",
        }

    mode = status["bringup"]["mode"]
    if _mode_rank(mode) < _mode_rank("nav"):
        return {
            "role": "action",
            "icon": "➡️",
            "message": f"현재 mode={mode}로 실행 중입니다 — 인식/수집 노드를 쓰려면 "
            "'환경' 카드에서 nav 이상으로 전환하세요.",
        }

    any_running = any(
        status["nodes"][n.value]["instance_count"] > 0 for n in _NODE_DISPLAY
    )
    if not any_running:
        return {
            "role": "action",
            "icon": "➡️",
            "message": "환경이 준비됐습니다 — 아래 '인식 · 수집' 카드에서 노드를 시작할 수 있습니다.",
        }

    return {"role": "success", "icon": "✅", "message": "정상 동작 중입니다."}


def render_top_cta(status: dict) -> None:
    action = get_next_action(status)
    with st.container(border=True):
        _render_card_header(action["icon"], action["message"], action["role"])


def render_environment_card(status: dict) -> None:
    bringup = status["bringup"]
    with st.container(border=True):
        _render_card_header("🖥️", "환경 (Gazebo / SLAM / Nav2 / explore)", "warning" if bringup["running"] else "action")

        if bringup["running"]:
            st.caption(
                f"⚠️ 현재 mode='{bringup['mode']}'로 이미 실행 중입니다 — "
                "다른 mode로 바꾸려면 먼저 종료하세요."
            )
            c1, c2, c3 = st.columns(3)
            c1.metric("mode", bringup["mode"])
            c2.metric("PID", bringup["pid"])
            c3.metric("headless", str(bringup["headless"]))

            if st.button("⏹ 환경 종료", type="primary", key="stop_bringup"):
                launch_runner.stop_bringup(graceful=True)
                st.success("환경을 종료했습니다.")
                st.rerun()
        else:
            st.caption(
                "각 mode는 누적입니다: sim ⊂ slam ⊂ nav ⊂ explore "
                "(예: nav = sim + SLAM + Nav2)."
            )

            # Gazebo GUI(Ogre2) 창은 이 머신처럼 GPU 가속 GL이 없는 WSLg 환경에서는
            # 화면에 안 뜨고 작업표시줄 아이콘만 남는다(커서에 "copy mode" —
            # semiH_web/CLAUDE.md 9절, weston-rdprail cursor-role 충돌). gzserver /
            # SLAM / Nav2 / explore 는 창 없이도 완전히 동작하고 시각화는 아래
            # 'RViz' 카드로 하므로, 기본값을 headless로 둔다. Mesa를 업그레이드해
            # (scripts/upgrade_mesa_wslg.sh) GPU 경로가 살아난 뒤에만 이 체크박스를
            # 켜서 Gazebo 창을 띄우는 것이 의미 있다.
            default_headless = bool(
                _load_settings().get("bringup", {}).get("default_headless", True)
            )
            show_gui = st.checkbox(
                "Gazebo GUI 창도 띄우기 (headless 끄기)",
                value=not default_headless,
                key="bringup_show_gui",
                help=(
                    "이 머신의 WSLg는 GPU 가속 GL이 없어 Gazebo 창이 화면에 안 뜰 수 "
                    "있습니다(작업표시줄 아이콘만 남고 커서가 'copy mode'). Mesa "
                    "업그레이드 전에는 꺼두고 RViz로 보세요."
                ),
            )
            if show_gui:
                st.caption(
                    "⚠️ Gazebo 창이 화면에 안 뜨고 아이콘만 남으면 이 체크를 끄고 "
                    "RViz로 보세요"
                )

            cols = st.columns(4)
            for col, (mode, label) in zip(cols, _MODE_BUTTONS):
                if col.button(label, key=f"start_{mode.value}"):
                    try:
                        launch_runner.start_bringup(mode, headless=not show_gui)
                        st.success(
                            f"mode={mode.value}로 시작했습니다"
                            + (" (Gazebo 창 포함)." if show_gui else " (headless — RViz로 시각화).")
                        )
                        st.rerun()
                    except process_manager.JobConflictError as e:
                        st.error(str(e))
                with col.popover("설명"):
                    st.write(_MODE_DESCRIPTIONS[mode])


def render_rviz_card(status: dict) -> None:
    """2026-08-28 추가: RViz를 웹에서 켜고 끄는 카드.

    `bringup`과 자원이 겹치지 않으므로(launch_runner.py RVIZ_STAGE 근거)
    환경이 꺼져 있어도 버튼 자체는 막지 않는다 -- 다만 그 상태에서 열면
    지도/스캔이 안 보일 뿐이므로 캡션으로 안내한다.

    `status.get("rviz", ...)`로 방어적으로 읽는다 -- Streamlit이 메인
    스크립트(`pages/main.py`)는 파일이 바뀔 때마다 항상 다시 읽어 실행하지만,
    `import`된 하위 모듈(`monitoring.topic_watcher` 등)은 프로세스를 완전히
    재시작하지 않으면 이전에 캐시된 버전이 그대로 남을 수 있다(2026-08-28,
    사용자가 실제로 `KeyError: 'rviz'`로 재현 -- `monitoring/topic_watcher.py`
    에 `"rviz"` 키를 새로 추가했는데, 오래전에 띄워둔 streamlit 프로세스가
    그 변경 전 버전의 `topic_watcher`를 여전히 물고 있어서 발생). 근본
    해결책은 `streamlit run`을 완전히 재시작하는 것이지만(코드 수정 후
    권장되는 절차), 이 화면 자체가 그런 일시적 불일치로 통째로 죽지는
    않도록 기본값 fallback을 둔다.
    """
    rviz = status.get("rviz") or {"running": False, "pid": None}
    with st.container(border=True):
        header_role = "success" if rviz["running"] else "action"
        _render_card_header("🖼️", "RViz — 지도 보기 / 목표 주기", header_role)
        with st.popover("설명"):
            st.write(_RVIZ_DESCRIPTION)

        if not status["bringup"]["running"]:
            st.caption("환경이 꺼져 있습니다 — 지금 열어도 빈 화면입니다. 먼저 '환경' 카드에서 mode를 시작하세요.")
        elif _mode_rank(status["bringup"]["mode"]) < _mode_rank("nav"):
            st.caption(f"현재 mode='{status['bringup']['mode']}' — 목표를 주는 기능은 nav 이상에서만 의미가 있습니다.")

        if rviz["running"]:
            st.metric("PID", rviz["pid"])
            if st.button("⏹ RViz 닫기", key="stop_rviz"):
                launch_runner.stop_rviz()
                st.rerun()
        else:
            if st.button("🖼️ RViz 열기", key="start_rviz"):
                try:
                    launch_runner.start_rviz()
                    st.success("RViz를 시작했습니다 (별도 창으로 뜹니다).")
                    st.rerun()
                except process_manager.JobConflictError as e:
                    st.error(str(e))


def _render_yolo_viewer_row(status: dict) -> None:
    """YOLO 카드 안에 인식 결과 창(rqt_image_view, `/yolo/image_annotated`)의
    상태와 열기/닫기 버튼을 그린다.

    2026-08-31 추가: "YOLO 버튼을 누르면 결과 영상 창도 같이 떠야 한다"는
    요청 반영. 시작 버튼 쪽(`_render_multi_instance_node`)에서 체크박스가
    켜져 있으면 `launch_runner.start_image_view()`를 자동 호출하고, 여기서는
    그 창을 수동으로 다시 열거나 닫을 수 있게 한다. 창은 하나만 뜨는
    고정 stage라, YOLO 인스턴스가 여러 개여도 뷰어는 1개다.
    """
    iv = status.get("image_view") or {"running": False, "pid": None}
    if iv["running"]:
        c1, c2 = st.columns([3, 1])
        c1.caption(f"🖼 인식 결과 창 실행 중 · PID {iv['pid']} · `{iv.get('topic') or launch_runner.IMAGE_VIEW_DEFAULT_TOPIC}`")
        if c2.button("⏹ 창 닫기", key="stop_image_view"):
            launch_runner.stop_image_view()
            st.rerun()
    else:
        if st.button("🖼 인식 결과 창 열기", key="start_image_view_manual"):
            try:
                launch_runner.start_image_view()
                st.success("인식 결과 창을 열었습니다 (별도 rqt_image_view 창).")
                st.rerun()
            except process_manager.JobConflictError as e:
                st.error(str(e))


def _render_multi_instance_node(node: NodeName, status: dict, blocked: bool, block_reason: str) -> None:
    icon, title, exe = _NODE_DISPLAY[node]
    info = status["nodes"][node.value]
    count = info["instance_count"]
    is_yolo = node is NodeName.YOLO_DETECTION

    with st.container(border=True):
        _render_card_header(icon, title, "warning" if blocked else "action")
        with st.popover("설명"):
            st.write(_NODE_DESCRIPTIONS[node])
        if blocked:
            st.caption(f"⚠️ {block_reason}")

        st.caption(f"`ros2 run semih_perception {exe}` — 여러 개 동시 실행 가능, 인스턴스별로 개별 종료합니다.")

        if node is NodeName.ARM_REACH and count > 0 and not blocked:
            st.caption(
                f"⚠️ 이미 {count}개가 실행 중입니다 — arm_reach를 여러 개 동시에 켜면 "
                "팔이 서로 다른 목표를 두고 경합할 수 있습니다 "
                "(semiH_web/CLAUDE.md 5절: 크래시는 안 나는 것까지만 확인, 궤적 흔들림 정도는 미검증)."
            )

        st.caption(f"현재 {count}개 실행 중")

        auto_open_viewer = False
        if is_yolo:
            auto_open_viewer = st.checkbox(
                "인식 결과 창(rqt_image_view)도 함께 열기",
                value=True,
                disabled=blocked,
                key="yolo_auto_open_viewer",
                help=f"YOLO를 시작할 때 {launch_runner.IMAGE_VIEW_DEFAULT_TOPIC} 토픽을 "
                "미리 물린 rqt_image_view 창을 같이 엽니다. 이미 열려 있으면 그대로 둡니다.",
            )

        if st.button(f"▶ {title} 새로 시작", disabled=blocked, key=f"start_{node.value}"):
            try:
                node_runner.start_node(node)
                if is_yolo and auto_open_viewer:
                    try:
                        launch_runner.start_image_view()
                    except process_manager.JobConflictError:
                        pass  # 이미 창이 떠 있으면 그대로 둔다
                st.success(f"{title} 인스턴스를 시작했습니다.")
                st.rerun()
            except process_manager.JobConflictError as e:
                st.error(str(e))

        for inst in info["instances"]:
            elapsed = time.time() - inst["started_at"]
            c1, c2, c3 = st.columns([2, 2, 1])
            c1.caption(f"PID {inst['pid']}")
            c2.caption(f"{int(elapsed // 60)}분 {int(elapsed % 60)}초 경과")
            if c3.button("⏹ 종료", key=f"stop_{inst['stage']}"):
                node_runner.stop_node(node, stage=inst["stage"])
                st.rerun()

        if is_yolo:
            _render_yolo_viewer_row(status)


def _render_hdf5_node(status: dict, blocked: bool, block_reason: str) -> None:
    icon, title, exe = _NODE_DISPLAY[NodeName.HDF5_COLLECTOR]
    info = status["nodes"]["hdf5_collector"]
    locked = info["instance_count"] > 0

    with st.container(border=True):
        _render_card_header(icon, title, "warning" if blocked else "action")
        with st.popover("설명"):
            st.write(_NODE_DESCRIPTIONS[NodeName.HDF5_COLLECTOR])
        if blocked:
            st.caption(f"⚠️ {block_reason}")
        elif locked:
            st.caption("⚠️ 이미 실행 중입니다 — 중복 실행은 저장 파일 충돌로 크래시하므로 새로 시작하는 버튼은 막혀 있습니다.")

        st.caption(f"`ros2 run semih_data_collection {exe}` — 동시에 1개만 실행 가능(하드 락).")

        duration_sec = st.number_input(
            "duration_sec", min_value=1.0, value=30.0, step=5.0,
            disabled=blocked or locked, key="hdf5_duration_sec",
        )

        if st.button("▶ HDF5 수집 시작", disabled=blocked or locked, key="start_hdf5_collector"):
            try:
                node_runner.start_node(
                    NodeName.HDF5_COLLECTOR, ros_args={"duration_sec": str(duration_sec)}
                )
                st.success("HDF5 수집을 시작했습니다.")
                st.rerun()
            except process_manager.JobConflictError as e:
                st.error(str(e))

        if locked:
            inst = info["instances"][0]
            elapsed = time.time() - inst["started_at"]
            c1, c2 = st.columns(2)
            c1.metric("PID", inst["pid"])
            c2.metric("경과 시간", f"{int(elapsed // 60)}분 {int(elapsed % 60)}초")
            if _hdf5_save_detected(status):
                st.warning("저장 완료 로그가 확인됐습니다 — 정지 버튼을 눌러 종료하세요.")
            if st.button("⏹ HDF5 수집 정지", type="primary", key="stop_hdf5_collector"):
                node_runner.stop_node(NodeName.HDF5_COLLECTOR)
                st.success("정지했습니다.")
                st.rerun()


def render_perception_card(status: dict) -> None:
    mode = status["bringup"]["mode"]
    blocked = not status["bringup"]["running"] or _mode_rank(mode) < _mode_rank("nav")
    if not status["bringup"]["running"]:
        block_reason = "환경이 꺼져 있습니다 — 먼저 '환경' 카드에서 nav 이상으로 시작하세요."
    elif blocked:
        block_reason = f"현재 mode='{mode}' — nav 이상이어야 인식/수집 노드를 쓸 수 있습니다."
    else:
        block_reason = ""

    st.subheader("인식 · 수집")
    for node in _PERCEPTION_NODES:
        _render_multi_instance_node(node, status, blocked, block_reason)
    _render_hdf5_node(status, blocked, block_reason)


def render_meta_footer(status: dict) -> None:
    with st.expander("메타정보 (로그 경로 / 설정값 요약)", expanded=False):
        st.markdown("**실행 중인 job의 로그 경로**")
        any_log = False
        if status["bringup"]["running"]:
            any_log = True
            st.code(process_manager.get_job_status("bringup").get("log_path", ""), language=None)
        if status.get("rviz", {}).get("running"):
            any_log = True
            st.code(process_manager.get_job_status("rviz").get("log_path", ""), language=None)
        if status.get("image_view", {}).get("running"):
            any_log = True
            st.code(process_manager.get_job_status("image_view").get("log_path", ""), language=None)
        for node in _NODE_DISPLAY:
            for inst in status["nodes"][node.value]["instances"]:
                any_log = True
                st.code(inst.get("log_path", ""), language=None)
        if not any_log:
            st.caption("실행 중인 job이 없습니다.")

        st.markdown("**config/settings.yaml 값 요약**")
        settings = _load_settings()
        st.json(settings)


def render_guide_tab() -> None:
    """2026-08-28 추가: "처음 쓰는 사용자가 최종적으로 자율주행+객체인식을
    경험하기까지 뭘 순서대로 눌러야 하는지 모르겠다"는 피드백에 대한 응답.

    새 프로세스 조작은 전혀 하지 않는 정적 안내문이다 -- 버튼은 전부
    "제어판" 탭에 그대로 있고, 이 탭은 그 버튼들을 어떤 순서로 눌러야
    하는지만 설명한다. semiH_web/CLAUDE.md 1-4절의 "단일 페이지" 결정을
    깨지 않기 위해 별도 페이지 파일(`pages/2_...py`, `st.navigation()`)이
    아니라 같은 `pages/main.py` 안 `st.tabs()`로 구현했다 -- URL/엔트리
    포인트는 여전히 하나뿐이다.
    """
    st.markdown("### 처음 사용 가이드 — 자율주행 + 객체 인식까지")
    st.caption("아래 단계를 위에서부터 순서대로 진행하면 '제어판' 탭의 버튼만으로 전체 파이프라인을 경험할 수 있습니다.")

    st.markdown("#### 1️⃣ 환경 켜기 — `nav` 또는 `explore`")
    st.markdown(
        "- **`nav`**: Gazebo + 지도 작성(SLAM) + 자율주행(Nav2)까지 켜지지만, "
        "**목표는 직접 줘야** 움직입니다(2단계 참고).\n"
        "- **`explore`**: `nav`가 하는 것 전부 + **목표 없이 로봇이 스스로** "
        "미탐색 영역을 찾아 돌아다니며 지도를 완성합니다. 클릭 한 번으로 "
        "'자율주행이 도는 모습'을 바로 보고 싶다면 이쪽이 더 빠릅니다.\n"
        "- `sim`/`slam`은 그 이전 준비 단계라 이 목적(자율주행 경험)에는 "
        "부족합니다 — 인식·수집 카드도 `nav` 이상이어야 열립니다.\n\n"
        "각 버튼의 정확한 차이는 '제어판' 탭 환경 카드의 ❓ 설명을 참고하세요."
    )

    st.markdown("#### 2️⃣ (`nav`를 골랐다면) RViz로 목표 주기")
    st.markdown(
        "환경 카드 아래 **'RViz — 지도 보기 / 목표 주기'** 카드에서 "
        "'🖼️ RViz 열기'를 누르면 별도 창이 뜹니다. 툴바의 **Set Goal** "
        "아이콘을 누르고 지도 위에서 원하는 위치와 방향으로 드래그하면 "
        "로봇이 그쪽으로 경로를 계산해 이동합니다.\n\n"
        "`explore`를 골랐다면 이 단계는 건너뛰어도 됩니다 — 로봇이 알아서 "
        "움직입니다. RViz는 그 모습을 지켜보는 용도로만 열면 됩니다."
    )

    st.markdown("#### 3️⃣ 인식 파이프라인 켜기 — 반드시 이 순서로")
    st.markdown(
        "'인식 · 수집' 카드에서 아래 순서대로 시작 버튼을 누르세요 (역순으로 "
        "켜면 앞 단계가 없어 뒤 노드가 의미 있는 값을 못 냅니다):\n\n"
        "1. **🔍 YOLO 인식** — 카메라 영상에서 물체를 탐지합니다. 시작할 때 "
        "'인식 결과 창도 함께 열기'가 켜져 있으면 `/yolo/image_annotated`를 "
        "보여주는 rqt_image_view 창이 같이 뜹니다(박스·라벨이 그려진 영상). "
        "체크를 꺼두었거나 창을 닫았다면 카드 안 '🖼 인식 결과 창 열기'로 "
        "다시 열 수 있습니다.\n"
        "2. **📐 3D 좌표 추출** — YOLO가 찾은 물체의 실제 3D 위치를 계산합니다.\n"
        "3. **🦾 팔 뻗기 (인식한 물체로 이동)** — 그 위치로 로봇 팔을 움직입니다."
    )

    st.markdown("#### 4️⃣ (선택) 데이터로 남기기 — HDF5 수집")
    st.markdown(
        "지금까지의 카메라/깊이/관절 상태를 파일로 저장하고 싶다면 "
        "'💾 HDF5 데이터 수집'을 시작하세요. **저장이 끝나도 프로세스가 "
        "자동으로 꺼지지 않으니**, 화면에 저장 완료 경고가 뜨면 직접 정지 "
        "버튼을 눌러 종료하세요."
    )

    st.markdown("#### ✅ 다 됐다면")
    st.markdown(
        "RViz 창에는 로봇이 지도 위를 이동하는 모습이, YOLO 창(또는 "
        "`/yolo/image_annotated` 토픽)에는 인식된 물체가, 그리고 물체가 "
        "로봇 팔 반경 안에 있다면 실제로 팔이 그쪽으로 움직이는 모습이 "
        "동시에 보이면 자율주행 + 객체 인식 파이프라인이 전부 정상 동작 "
        "중인 것입니다."
    )

    st.info("막히는 부분이 있으면 '제어판' 탭에서 해당 카드의 ❓ 버튼을 눌러 그 기능만의 상세 설명을 볼 수 있습니다.")


def render_control_tab(status: dict) -> None:
    render_top_cta(status)
    st.divider()
    render_environment_card(status)
    st.divider()
    render_rviz_card(status)
    st.divider()
    render_perception_card(status)
    st.divider()
    render_meta_footer(status)


def _check_stale_module_cache() -> None:
    """2026-08-28 추가: 사용자가 실제로 `AttributeError: module 'core.launch_runner'
    has no attribute 'start_rviz'`를 겪은 것에 대한 응답 -- `KeyError: 'rviz'`와
    같은 근본 원인(8-7절)이다. Streamlit은 엔트리 스크립트(`pages/main.py`)는
    파일이 바뀔 때마다 항상 다시 읽어 실행하지만, 그 스크립트가 `import`하는
    하위 모듈(`core.launch_runner` 등)은 `sys.modules`에 캐시돼 있어서 **그
    streamlit 프로세스를 완전히 재시작하지 않는 한** 코드를 수정해도 반영되지
    않는다.

    이걸 막을 방법은 없다(우리 코드가 아니라 Python의 import 캐싱 자체가
    원인) -- 대신 최소한 사용자가 원인을 못 알아채고 새로고침만 반복하는
    상황을 막기 위해, `main()` 맨 앞에서 "이번 세션에서 추가된 게 확실한
    속성"이 실제로 있는지 확인해 없으면 원인과 조치를 명확히 알려주고
    `st.stop()`으로 이후 렌더링을 막는다(어차피 이후 코드가 그 속성을 호출
    하면 똑같이 죽으므로, 알아보기 쉬운 에러로 먼저 잡는 것).
    """
    missing = [
        attr
        for attr in ("start_rviz", "start_image_view")
        if not hasattr(launch_runner, attr)
    ]
    if missing:
        st.error(
            "⚠️ 이 streamlit 프로세스가 이전 버전의 코드를 아직 메모리에 캐시하고 "
            f"있습니다(`core.launch_runner`에 `{', '.join(missing)}`가 없음) — 파일은 "
            "이미 최신인데, 파이썬이 하위 모듈을 다시 불러오지 않아서 생기는 문제입니다.\n\n"
            "**해결: 터미널에서 이 streamlit 프로세스를 완전히 종료(Ctrl+C)한 뒤 "
            "`streamlit run pages/main.py ...`로 다시 실행해 주세요.** 브라우저 "
            "새로고침이나 'Always rerun'만으로는 해결되지 않습니다."
        )
        st.stop()


def main() -> None:
    st.set_page_config(page_title="semiH_web", page_icon=ui_components.get_logo_icon(), layout="wide")
    _check_stale_module_cache()

    # 새로고침/재방문마다(같은 브라우저 탭에서의 버튼 클릭 rerun 제외) 스플래시를
    # 다시 보여준다 -- semiH_web/CLAUDE.md 7절 "매번 vs 세션당 1회" 논의 참고.
    # st.session_state는 세션(=브라우저 탭 접속)마다 새로 초기화되므로, 이
    # 블록이 "아직 이번 세션에서 스플래시를 안 보여줬을 때만" 실행된다 -- 같은
    # 세션 안에서 버튼을 눌러 st.rerun()이 여러 번 일어나도 그때마다 다시
    # 뜨지는 않는다(그러면 매 클릭마다 1초씩 막혀서 UX가 나빠짐).
    if "splash_shown" not in st.session_state:
        ui_components.render_splash_screen()
        time.sleep(1.5)  # 2026-08-18: 1.0 -> 1.5 (semiH_web/CLAUDE.md 7절 재측정 참고)
        st.session_state["splash_shown"] = True
        st.rerun()

    ui_components.render_brand_header()
    st.title("세미 휴머노이드 자율주행")
    st.caption("semiH_ws(ROS 2 워크스페이스)를 감싸는 제어 대시보드 — semiH_ws 코드는 이 앱에서 수정되지 않습니다.")

    status = topic_watcher.get_system_status()

    tab_control, tab_guide = st.tabs(["🎛 제어판", "📖 처음 사용 가이드"])
    with tab_control:
        render_control_tab(status)
    with tab_guide:
        render_guide_tab()

    if (
        status["bringup"]["running"]
        or status.get("rviz", {}).get("running")
        or status.get("image_view", {}).get("running")
        or any(status["nodes"][n.value]["instance_count"] > 0 for n in _NODE_DISPLAY)
    ):
        st_autorefresh(interval=3000, key="main_autorefresh")


if __name__ == "__main__":
    main()
