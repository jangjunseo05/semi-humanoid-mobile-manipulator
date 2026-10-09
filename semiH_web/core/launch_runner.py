"""`ros2 launch semih_description bringup.launch.py`를 감싸는 실행 레이어.

semiH_ws/CLAUDE.md 섹션 3-1에서 확인된 사실을 그대로 반영한다:
- `bringup.launch.py`의 `mode` 인자는 코드상 `sim`/`slam`/`nav`/`explore`
  4개 값만 의미 있게 처리하며(각각 누적: slam은 sim+slam, nav는 sim+slam+nav2,
  explore는 그 전부 + 20초/33초 지연 시퀀싱), 그 외의 값(오타 포함)을 줘도
  아무 `IfCondition`도 참이 안 돼 조용히 `mode=sim`과 동일하게 동작한다 —
  즉 원본 코드에는 값 검증이 없다.
- 이 버그를 웹 레이어에서 반복하지 않기 위해, `BringupMode`를 자유 텍스트가
  아니라 4개 값으로 제한된 Enum으로 강제한다.
- `spawn_gazebo.launch.py`는 mode 값과 무관하게 bringup.launch.py 안에서
  항상 include되므로(섹션 2-2), 4개 mode 전부가 `gazebo_instance` 자원을
  요구한다 — `core/process_manager.STAGE_RESOURCES`에서 `bringup` 하나로
  묶여 있는 것과 일치한다.

**semiH_ws 내부 파일은 이 모듈에서도 절대 읽기 외의 방식으로 수정/이동/
복사하지 않는다** — 오직 `ros2 launch` subprocess를 바깥에서 감싸 실행할
뿐이다.
"""

from __future__ import annotations

import enum
from pathlib import Path

import yaml

from core import process_manager

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SETTINGS_PATH = PROJECT_ROOT / "config" / "settings.yaml"

STAGE = "bringup"


class BringupMode(str, enum.Enum):
    """`bringup.launch.py`의 `mode` 인자로 허용되는 값 전체 (semiH_ws/CLAUDE.md
    섹션 3-1 기준, 4개 누적 값 — 이 Enum 밖의 값은 애초에 만들어질 수 없다).
    """

    SIM = "sim"
    SLAM = "slam"
    NAV = "nav"
    EXPLORE = "explore"


def _load_settings() -> dict:
    """`config/settings.yaml`을 읽어 dict로 반환한다.

    파일이 없거나 파싱에 실패하면 빈 dict를 반환한다 — 이 경우
    `build_bringup_command()`가 `workspace.*` 키 부재로 `KeyError`를 내며
    명확히 실패하게 두는 편이, 잘못된 기본값으로 조용히 넘어가는 것보다
    안전하다고 판단했다(경로가 틀리면 `ros2 launch` 자체가 실패하므로
    어차피 사용자가 알아채야 함).
    """
    if not SETTINGS_PATH.exists():
        return {}
    with open(SETTINGS_PATH, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _gui_env() -> dict:
    """창을 띄우는 job(rviz2 / headless=false인 gazebo GUI)에만 주입할 환경변수를
    `config/settings.yaml`의 `gui_env` 블록에서 읽어 반환한다(없으면 빈 dict).

    WSLg에서 Qt 앱이 xcb(X11/GLX)로 뜨면 GPU 가속 GL 경로를 못 타고 소프트웨어
    렌더(swrast)로 폴백돼, Ogre2(gazebo GUI)/rviz 창이 첫 프레임을 제때 못 그리고
    작업표시줄 아이콘만 남은 채 화면에 안 뜬다. `gui_env`의 `QT_QPA_PLATFORM=
    "wayland;xcb"`로 Qt를 네이티브 Wayland 클라이언트로 띄우면 컴포지터가 즉시
    합성하고 EGL 경로라 d3d12(GPU) 드라이버가 선택된다 — semiH_web/CLAUDE.md 9절.

    headless bringup·`ros2 run` 노드에는 이 값을 넘기지 않는다(창이 없으므로 무의미).
    """
    return _load_settings().get("gui_env", {}) or {}


def build_bringup_command(mode: BringupMode, headless: bool = True) -> list[str]:
    """`ros2 launch semih_description bringup.launch.py mode:=<mode> headless:=<headless>`
    를 실행하는 argv 리스트를 조립해 반환한다.

    ROS 배포판 setup.bash와 semiH_ws의 install/setup.bash를 순서대로
    source한 뒤 `exec`로 `ros2 launch`를 실행하는 셸 명령으로 감싼다.
    `exec`를 쓰는 이유: 감싸는 bash 프로세스를 `ros2 launch` 프로세스 자체로
    치환해, `process_manager.start_job()`이 기록하는 pid가 진짜 `ros2
    launch` 프로세스의 pid가 되게 하기 위함(중간에 안 죽는 bash 껍데기가
    남지 않음) — semiH_web/CLAUDE.md 2절 라이브 검증에 쓴 것과 동일한 형태.

    Args:
        mode: BringupMode 중 하나 (자유 텍스트 금지).
        headless: bringup.launch.py의 headless 인자에 그대로 대응.

    Returns:
        subprocess에 넘길 argv 리스트.
    """
    settings = _load_settings()
    workspace = settings.get("workspace", {})
    ros_setup_bash = workspace["ros_setup_bash"]
    install_setup_bash = workspace["install_setup_bash"]

    headless_str = "true" if headless else "false"
    shell_cmd = (
        f"source {ros_setup_bash} && "
        f"source {install_setup_bash} && "
        f"exec ros2 launch semih_description bringup.launch.py "
        f"mode:={mode.value} headless:={headless_str}"
    )
    return ["bash", "-c", shell_cmd]


def start_bringup(mode: BringupMode, headless: bool = True) -> dict:
    """`build_bringup_command()`로 만든 명령을 `process_manager.start_job()`
    으로 넘겨 실행한다.

    이미 어떤 mode로든 bringup이 실행 중이면(자원: gazebo_instance)
    `process_manager.JobConflictError`가 그대로 전파된다 — 여기서 삼키거나
    강제로 이전 걸 죽이고 새로 띄우지 않는다(사용자가 명시적으로
    stop_bringup() 후 재시도).

    Returns:
        시작된 job의 상태 dict.
    """
    cmd = build_bringup_command(mode, headless=headless)
    # headless=false일 때만 gazebo GUI 창이 뜨므로 그 경우에만 gui_env를 주입한다
    # (headless bringup은 창이 없어 무의미) — semiH_web/CLAUDE.md 9절.
    env_overlay = None if headless else _gui_env()
    return process_manager.start_job(
        STAGE,
        cmd,
        extra_meta={"mode": mode.value, "headless": headless},
        env_overlay=env_overlay,
    )


#  process_manager.stop_job()의 기본 graceful_timeout_s(5.0s)는 가벼운 단일
# ros2 run 노드(core/node_runner.py 쪽)에는 맞지만, bringup은 nav2 lifecycle
# manager가 여러 서버를 순서대로 deactivate/cleanup하고 gzserver까지 내려가는
# 훨씬 무거운 종료 시퀀스라 그걸로는 부족하다 -- 라이브 실측(semiH_web/CLAUDE.md
# 2절): 그룹 전체 SIGINT 이후 launch 부모 프로세스가 스스로 죽기까지 8.43초
# 걸렸다(mode=explore, 노드 33개 기준). 5.0s로 뒀을 때는 실제로 그 안에 못
# 끝나 매번 SIGKILL로 escalate되는 것까지 실측 확인됨 -- 즉 "graceful"이라는
# 이름과 달리 사실상 매번 강제종료였다는 뜻이라 잘못된 기본값이었다. 8.43초에
# 여유를 더해 15초로 둔다.
BRINGUP_GRACEFUL_TIMEOUT_S = 15.0


def stop_bringup(graceful: bool = True) -> bool:
    """실행 중인 bringup(어떤 mode든)을 종료한다.

    `process_manager.stop_job("bringup", graceful=graceful,
    graceful_timeout_s=BRINGUP_GRACEFUL_TIMEOUT_S)`의 thin wrapper.
    내부적으로 프로세스 그룹 전체(`os.killpg`)에 신호를 보내 `ign gazebo`
    등 셸 래퍼를 통해 기동된 손자 프로세스까지 정리한다(semiH_web/CLAUDE.md
    2절 라이브 검증 근거).
    """
    return process_manager.stop_job(
        STAGE, graceful=graceful, graceful_timeout_s=BRINGUP_GRACEFUL_TIMEOUT_S
    )


def get_bringup_status() -> dict:
    """현재 bringup의 실행 상태를 반환한다
    (`process_manager.get_job_status("bringup")`의 thin wrapper).
    `start_bringup()`이 `extra_meta`로 저장해둔 `mode`/`headless` 값도
    함께 노출된다.
    """
    return process_manager.get_job_status(STAGE)


# --- RViz ---------------------------------------------------------------
#
# 2026-08-28 추가: mode=nav/explore로 nav2를 띄워도 목표(goal)를 줄 방법이
# 웹 UI에 없어서 "로봇이 안 움직인다"는 혼란이 생겼다(사용자 대화 중 실제로
# 재현됨) -- nav2는 목표를 받아야 움직이는 스택이고, `bringup.launch.py`는
# RViz를 자동으로 띄우지 않는다(semih_description/launch/nav2.launch.py에
# rviz 노드 없음, 확인함). RViz 자체를 semiH_ws 안에 새로 만들거나 그
# launch 파일을 고치는 대신(1절 "semiH_ws 비수정 원칙"), 웹 레이어에서
# 독립적으로 `rviz2`를 띄우는 버튼을 추가하는 쪽을 택했다.
#
# `bringup`(gazebo_instance)과는 별개 자원이라 STAGE_RESOURCES["rviz"] = []
# (동시에 떠 있는 게 정상 사용 형태 -- RViz로 bringup이 만드는 지도를 보는
# 것 자체가 목적). 여러 개 띄울 이유가 없으므로 고정 stage 이름 하나만
# 쓴다(hdf5_collector와 동일한 "고정 stage = 하드 락" 패턴, 3절 근거).

RVIZ_STAGE = "rviz"
RVIZ_CONFIG_PATH = PROJECT_ROOT / "config" / "nav_view.rviz"


def build_rviz_command() -> list[str]:
    """`rviz2 -d config/nav_view.rviz`를 실행하는 argv 리스트를 조립한다.

    ROS setup.bash + semiH_ws install/setup.bash를 순서대로 source한 뒤
    `exec`로 치환한다 -- `build_bringup_command()`/`build_node_command()`와
    동일한 이유(bash 껍데기가 안 남고 pid가 진짜 rviz2 프로세스가 되게).
    `-d`로 넘기는 `config/nav_view.rviz`는 Map/LaserScan/RobotModel/TF
    디스플레이와 Fixed Frame=map, "2D Nav Goal" 툴이 미리 채워진 시작
    설정이다(라이브 로드 테스트 완료 -- 파일 자체의 주석 참고). 설정 파일이
    없으면(예: 실수로 지워진 경우) `-d` 없이 빈 화면으로라도 뜨게 조용히
    폴백한다 -- 설정 파일 누락이 RViz 자체를 못 열게 막으면 안 되므로.
    """
    settings = _load_settings()
    workspace = settings.get("workspace", {})
    ros_setup_bash = workspace["ros_setup_bash"]
    install_setup_bash = workspace["install_setup_bash"]

    rviz_cmd = "rviz2"
    if RVIZ_CONFIG_PATH.is_file():
        rviz_cmd += f" -d {RVIZ_CONFIG_PATH}"

    shell_cmd = (
        f"source {ros_setup_bash} && "
        f"source {install_setup_bash} && "
        f"exec {rviz_cmd}"
    )
    return ["bash", "-c", shell_cmd]


def start_rviz() -> dict:
    """RViz를 시작한다. 이미 떠 있으면 `process_manager.JobConflictError`가
    그대로 전파된다(고정 stage 하나뿐이므로 두 번째 시작 시도는 항상 충돌).
    """
    cmd = build_rviz_command()
    # rviz2는 항상 창을 띄우므로 gui_env를 주입한다 — semiH_web/CLAUDE.md 9절.
    return process_manager.start_job(RVIZ_STAGE, cmd, env_overlay=_gui_env())


def stop_rviz(graceful: bool = True) -> bool:
    """실행 중인 RViz를 종료한다 (`process_manager.stop_job()`의 thin
    wrapper -- rviz2도 셸 래퍼 없이 `ros2 run`류와 동일하게 부모/자식이
    없는 단일 GUI 프로세스이므로 기본 graceful_timeout_s(5.0s)로 충분할
    것으로 판단했다, bringup처럼 별도로 늘리지 않음 -- 이 판단은 node_runner
    쪽과 마찬가지로 아직 실측 검증되지 않았음, "확인 안 됨"으로 남김).
    """
    return process_manager.stop_job(RVIZ_STAGE, graceful=graceful)


def get_rviz_status() -> dict:
    """현재 RViz의 실행 상태를 반환한다
    (`process_manager.get_job_status("rviz")`의 thin wrapper)."""
    return process_manager.get_job_status(RVIZ_STAGE)


# --- rqt_image_view (YOLO 인식 결과 창) ---------------------------------
#
# 2026-08-31 추가: YOLO 노드는 `/yolo/image_annotated` 토픽으로만 결과를
# 내보내고 창을 스스로 띄우지 않는다. 사용자가 웹에서 "YOLO 인식 새로
# 시작"을 누르면 그 결과 영상 창도 같이 뜨길 원해서, `rqt_image_view`를
# 그 토픽에 미리 물려 여는 래퍼를 추가했다(RViz 래퍼와 완전히 동일한
# 패턴 -- setup.bash 2개 source 후 exec, 고정 stage 하나, gui_env 주입,
# 그룹 SIGINT 종료).
#
# `bringup`/`rviz`와 자원이 겹치지 않아 STAGE_RESOURCES["image_view"] = []
# (셋 다 동시에 떠 있는 게 정상 사용 형태). 여러 개 띄울 이유가 없으므로
# 고정 stage 이름 하나만 쓴다 -- 두 번째 시작 시도는 JobConflictError.

IMAGE_VIEW_STAGE = "image_view"
IMAGE_VIEW_DEFAULT_TOPIC = "/yolo/image_annotated"


def build_image_view_command(topic: str = IMAGE_VIEW_DEFAULT_TOPIC) -> list[str]:
    """`ros2 run rqt_image_view rqt_image_view <topic>`를 실행하는 argv 리스트를
    조립한다.

    `rqt_image_view`는 positional 인자로 토픽 이름을 받아 그 토픽을 미리
    선택한 채 창을 연다(`--help`로 확인함: `topic  The topic name to
    subscribe to`). setup.bash 2개를 source한 뒤 `exec`로 치환하는 것은
    `build_rviz_command()`와 동일한 이유(bash 껍데기가 안 남고 pid가 진짜
    rqt_image_view 프로세스가 되게).
    """
    settings = _load_settings()
    workspace = settings.get("workspace", {})
    ros_setup_bash = workspace["ros_setup_bash"]
    install_setup_bash = workspace["install_setup_bash"]

    shell_cmd = (
        f"source {ros_setup_bash} && "
        f"source {install_setup_bash} && "
        f"exec ros2 run rqt_image_view rqt_image_view {topic}"
    )
    return ["bash", "-c", shell_cmd]


def start_image_view(topic: str = IMAGE_VIEW_DEFAULT_TOPIC) -> dict:
    """인식 결과 뷰어(rqt_image_view)를 시작한다. 이미 떠 있으면
    `process_manager.JobConflictError`가 그대로 전파된다(고정 stage 하나뿐).
    """
    cmd = build_image_view_command(topic)
    # rqt_image_view도 항상 창을 띄우므로 gui_env를 주입한다 — semiH_web/CLAUDE.md 9절.
    return process_manager.start_job(
        IMAGE_VIEW_STAGE, cmd, extra_meta={"topic": topic}, env_overlay=_gui_env()
    )


def stop_image_view(graceful: bool = True) -> bool:
    """실행 중인 인식 결과 뷰어를 종료한다 (`process_manager.stop_job()`의 thin
    wrapper -- rviz2와 마찬가지로 셸 래퍼 없는 단일 GUI 프로세스라 기본
    graceful_timeout_s(5.0s)로 충분할 것으로 판단, 별도로 늘리지 않음)."""
    return process_manager.stop_job(IMAGE_VIEW_STAGE, graceful=graceful)


def get_image_view_status() -> dict:
    """현재 인식 결과 뷰어의 실행 상태를 반환한다
    (`process_manager.get_job_status("image_view")`의 thin wrapper).
    `start_image_view()`가 `extra_meta`로 저장한 `topic`도 함께 노출된다."""
    return process_manager.get_job_status(IMAGE_VIEW_STAGE)
