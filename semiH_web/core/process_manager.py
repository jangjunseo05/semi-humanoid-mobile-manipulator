"""로컬 subprocess job/lock 관리.

패턴 출처: `so101_web/dashboard/lib/process_manager.py` (실제 파일을 찾아서
확인 후 참고함 — C:\\Users\\USER\\Desktop\\so101_web\\dashboard\\lib\\process_manager.py).
그 프로젝트와 동일하게, Streamlit의 `st.session_state`가 아니라 디스크 lock
파일(`state/locks/*.json`)에 상태를 둬서 페이지 새로고침/웹 서버 재시작에도
"지금 뭐가 돌고 있는지"를 잃지 않는 것을 목표로 한다.

semiH_web과 so101_web의 핵심 차이 (semiH_web/CLAUDE.md 1절 참고):
- so101_web은 stage마다(data_collection/training/inference) 서로 다른 자원
  조합(mujoco_viewer/serial_port/gpu)이 겹치는지를 확인해야 했다.
- semiH_web은 자원 락이 `gazebo_instance` 단 하나뿐이다. `bringup` stage(=
  ros2 launch semih_description bringup.launch.py, mode 값이 무엇이든)만
  이 자원을 점유하며, 동시에 하나의 bringup 프로세스 트리만 존재할 수 있다.
  YOLO/3D좌표+IK/HDF5 노드(core/node_runner.py)는 독립 ros2 run 노드라
  gazebo_instance와 충돌하지 않고, 서로 간에도 이번 스코프에서는 자원 경합이
  없다고 간주한다(각 노드 자체의 중복 실행 방지만 이 모듈이 담당).
- GPU 락은 이번 스코프에서 제외한다(다음 단계에서 필요해지면 STAGE_RESOURCES에
  추가하는 식으로 확장 가능하도록 구조만 so101_web과 동일하게 유지함).

이 파일의 두 가지 핵심 동작은 so101_web과 다르게, semiH_web/CLAUDE.md 2절에
기록된 실측 검증을 거쳐 결정했다:

1. **stale lock 판단은 timeout이 아니라 PID 생존 여부**. so101_web은 lock
   생성 후 pid가 채워지지 않은 "starting" placeholder 상태에만 timeout을
   쓰고, pid가 있는 lock은 `psutil.pid_exists(pid)`로 생존을 확인했다 —
   semiH_web도 동일하게, **pid가 기록된 lock에는 timeout을 전혀 적용하지
   않는다**. `mode=explore`처럼 사용자가 수 시간 띄워두는 게 정상 상태인
   워크로드에서 "오래 떠 있으니 stale"로 잘못 정리하면 안 되기 때문
   (`STARTING_TIMEOUT_S`는 pid가 아직 없는 아주 짧은 부트스트랩 구간에만
   적용됨 — 아래 참고).

2. **graceful 종료는 프로세스 그룹 전체(`os.killpg`)를 대상으로 한다**,
   launch 부모 프로세스 하나만이 아니라. semiH_web/CLAUDE.md 2절에 기록된
   라이브 검증에서, `ros2 launch ... mode:=explore`의 부모 PID에만
   SIGINT를 보냈을 때 launch 자신과 그 직계 ROS 노드들은 정상 종료됐지만,
   `ign gazebo` 실제 프로세스(`/bin/sh -c 'ruby ign gazebo ...'` 셸
   래퍼를 통해 기동됨)는 고아 프로세스로 남았다 — 셸 래퍼가 죽으면서
   자신의 자식에게 신호를 전달하지 않았기 때문. 반대로 같은 프로세스
   트리를 process group 전체(`os.killpg`)로 SIGINT했을 때는 `ign gazebo`
   를 포함한 전체(그 세션의 21개 프로세스)가 잔여물 없이 종료됐다(라이브
   재현·재확인 완료). 그래서 `start_job()`은 `subprocess.Popen(...,
   start_new_session=True)`로 각 job을 새 세션/프로세스 그룹의 leader로
   띄우고(그 job의 pid == 그 그룹의 pgid), `stop_job()`은 항상
   `os.killpg(pid, signal)`을 쓴다.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
LOCKS_DIR = PROJECT_ROOT / "state" / "locks"
JOB_LOGS_DIR = PROJECT_ROOT / "logs" / "jobs"

LOCKS_DIR.mkdir(parents=True, exist_ok=True)
JOB_LOGS_DIR.mkdir(parents=True, exist_ok=True)

# stage별로 독점 점유하는 자원 태그. semiH_web은 gazebo_instance 하나뿐이며,
# bringup 외 나머지 stage(개별 ros2 run 노드)는 빈 리스트 — 즉 서로 충돌하지
# 않는다. GPU 락은 이번 스코프 제외(1절 참고) — 필요해지면 이 표에 항목만
# 추가하면 되도록 so101_web과 동일한 확장 구조를 유지한다.
STAGE_RESOURCES: dict[str, list[str]] = {
    "bringup": ["gazebo_instance"],
    "yolo_detection": [],
    "target_3d": [],
    "arm_reach": [],
    "hdf5_collector": [],
    # rviz2는 GPU를 쓰지만 gazebo_instance와는 별개 자원(둘 다 동시에 떠 있는
    # 게 정상적인 사용 형태 -- RViz로 bringup이 만드는 지도/costmap을 보는
    # 것이 목적)이라 빈 리스트. 여러 개 띄울 이유가 없어 core/launch_runner.py
    # 의 start_rviz()가 고정 stage 이름 하나만 쓰지만, 그건 여기 자원 충돌이
    # 아니라 process_manager의 "이미 실행 중" 락(같은 stage 재사용) 때문이다.
    "rviz": [],
    # rqt_image_view (2026-08-31 추가): YOLO 인식 결과(/yolo/image_annotated)를
    # 보는 뷰어 창. rviz와 같은 이유로 빈 리스트 -- gazebo/노드와 동시에 떠
    # 있는 게 정상이고, 하나만 있으면 충분하므로 고정 stage 이름 하나만 쓴다.
    "image_view": [],
}

# start_job()이 lock 파일을 원자적으로 생성한 직후, 아직 실제 subprocess pid로
# 덮어쓰기 전인 "starting" placeholder 상태(pid가 아직 None)가 이보다 오래
# 지속되면(정상이라면 수 초 내로 끝남) 시작 도중 죽은 것으로 본다. pid가 이미
# 채워진 lock에는 이 타임아웃을 적용하지 않는다(위 docstring 1번 참고).
STARTING_TIMEOUT_S = 60


class JobConflictError(Exception):
    """이미 실행 중이거나, 겹치는 자원을 다른 stage가 점유 중일 때."""


def _lock_path(stage: str) -> Path:
    """`stage`에 대응하는 lock 파일의 절대경로를 반환한다."""
    return LOCKS_DIR / f"{stage}.json"


def _read_lock(stage: str) -> dict | None:
    """`stage`의 lock 파일을 읽어 dict로 반환한다. 파일이 없거나 손상됐으면 None."""
    path = _lock_path(stage)
    if not path.exists():
        return None
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (json.JSONDecodeError, OSError):
        return None


def _pid_alive(pid: int) -> bool:
    """`pid`가 살아있는 프로세스인지 확인한다.

    **좀비 프로세스 처리(2026-08-17, pages/main.py 라이브 검증 중 발견)**:
    `os.kill(pid, 0)` 하나만으로는 좀비(이미 종료됐지만 부모가 아직
    `wait()`로 거두지 않은 프로세스)를 "살아있음"으로 잘못 판단한다 —
    실측 확인(`os.kill(zombie_pid, 0)`이 예외 없이 성공함). 우리
    자식이면(`start_job()`이 이 프로세스 자신을 부모로 `subprocess.Popen`
    했으므로 그 job은 항상 우리 프로세스의 직계 자식) `os.waitpid(pid,
    os.WNOHANG)`로 먼저 거두기를 시도한다 — 좀비였다면 이 호출이 그
    자리에서 pid를 반환하며 정리되고(진짜로 죽음 확정), 아직 실행
    중이면 `(0, 0)`을 즉시 반환(비차단), 애초에 우리 자식이 아니면(예:
    이 프로세스가 재시작돼 fork 계보가 끊긴 경우) `ChildProcessError`를
    던진다 — 이 경우에만 기존의 `os.kill(pid, 0)` 방식으로 폴백한다.

    권한 문제로 죽었는지 살았는지 못 물어보는 경우(PermissionError)는
    "존재한다"로 간주한다 — 이 프로젝트는 단일 사용자(WSL 내 jangjunseo)
    환경이라 실제로는 거의 발생하지 않는 경로지만, 방어적으로 stale이
    아닌 쪽으로 판단한다(잘못 지워서 실제 실행 중인 프로세스의 lock을
    날리는 것보다 안전한 방향).
    """
    try:
        reaped_pid, _ = os.waitpid(pid, os.WNOHANG)
        if reaped_pid == pid:
            return False  # 좀비였고, 지금 거뒀다 -- 확정적으로 죽음
    except ChildProcessError:
        pass  # 우리 자식이 아님(이미 거둬졌거나 fork 계보가 끊김) -- 아래로 폴백

    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    else:
        return True


def _all_locks() -> dict[str, dict]:
    """현재 살아있는(또는 막 시작 중인) 모든 stage의 lock 정보를 모아 반환한다.

    pid가 기록돼 있는데 그 pid가 이미 죽은 stale lock은 이 호출 중에
    정리(삭제)한다 — `_pid_alive()` 기준, timeout 아님(위 docstring 1번).
    pid가 아직 None인 "starting" placeholder는 `STARTING_TIMEOUT_S`를 넘겼을
    때만 stale로 정리한다.
    """
    result: dict[str, dict] = {}
    for path in LOCKS_DIR.glob("*.json"):
        stage = path.stem
        info = _read_lock(stage)
        if info is None:
            continue
        pid = info.get("pid")
        if pid is None:
            if time.time() - info.get("started_at", 0) > STARTING_TIMEOUT_S:
                path.unlink(missing_ok=True)
            else:
                result[stage] = info
            continue
        if _pid_alive(pid):
            result[stage] = info
        else:
            path.unlink(missing_ok=True)
    return result


def get_job_status(stage: str) -> dict:
    """`{"running": bool, ...lock 정보}` 형태로 stage의 현재 상태를 반환한다.

    stage가 STAGE_RESOURCES에 없는 이름이어도 에러 없이 `{"running": False}`
    를 반환한다(방어적 조회).
    """
    info = _all_locks().get(stage)
    if info is None:
        return {"running": False}
    return {"running": True, **info}


def check_conflict(stage: str) -> str | None:
    """`stage`를 시작하려 할 때 자원이 겹치는, 이미 실행 중인 다른 stage 이름을
    반환한다(없으면 None).

    semiH_web에서는 사실상 `stage == "bringup"`일 때만 다른 실행 중인
    "bringup" lock과 부딪힐 수 있다 — mode가 sim/slam/nav/explore 무엇이든
    같은 gazebo_instance 자원을 요구하기 때문에, 이미 어떤 mode로든
    bringup이 떠 있으면 다른 mode의 bringup을 또 띄울 수 없다.
    """
    needed = set(STAGE_RESOURCES.get(stage, []))
    if not needed:
        return None
    for other_stage, info in _all_locks().items():
        if other_stage == stage:
            continue
        if needed & set(info.get("resources", [])):
            return other_stage
    return None


def _merged_env(env_overlay: dict | None) -> dict | None:
    """`env_overlay`가 있으면 현재 프로세스 환경(`os.environ`) 위에 덮어쓴 새 dict를
    반환한다. 없거나 비어 있으면 `None`을 반환해 `Popen`이 평소처럼 부모 환경을
    그대로 상속하게 둔다(기존 동작과 100% 동일).

    GUI job(rviz2 / gazebo GUI)에 `QT_QPA_PLATFORM` 등을 주입하기 위한 것이며,
    값은 `core/launch_runner.py`의 `_gui_env()`가 `config/settings.yaml`의
    `gui_env` 블록에서 읽어 넘긴다(semiH_web/CLAUDE.md 9절).
    """
    if not env_overlay:
        return None
    merged = dict(os.environ)
    merged.update({str(k): str(v) for k, v in env_overlay.items()})
    return merged


def start_job(
    stage: str,
    cmd: list[str],
    extra_meta: dict | None = None,
    env_overlay: dict | None = None,
) -> dict:
    """subprocess를 새로 띄우고 lock 파일 생성 + stdout/stderr 로그 캡처를 시작한다.

    - lock 파일 생성 자체를 `os.open(..., O_CREAT | O_EXCL)`로 원자적 연산으로
      만들어 동시 요청(예: 웹 버튼 더블클릭) 중 단 하나만 성공하게 한다.
    - 먼저 `check_conflict(stage)`로 충돌을 확인하고, 충돌 시
      `JobConflictError`를 던진다.
    - 이미 있는 lock 파일과 부딪히면(O_EXCL 실패) 역시 `JobConflictError`.
    - `subprocess.Popen(..., start_new_session=True)`로 띄운다 — 이 job의
      pid가 곧 새 프로세스 그룹의 pgid가 되어, `stop_job()`이 그룹 전체를
      한 번에 겨냥할 수 있게 한다(docstring 2번 근거).
    - `cmd`는 `core/launch_runner.py`/`core/node_runner.py`가 만들어 넘기며,
      ROS setup.bash 소싱 등 셸이 필요한 부분은 이미 `cmd` 안에
      (`["bash", "-c", "source ... && exec ros2 ..."]` 형태로) 포함돼 있다고
      가정한다 — 여기서는 그걸 그대로 Popen에 넘길 뿐, 내용을 해석하지 않는다.
    - `env_overlay`가 주어지면 `os.environ` 위에 그 키/값을 덮어쓴 환경으로
      subprocess를 띄운다(없으면 부모 환경을 그대로 상속 — 기존 동작). GUI
      job(rviz2 / gazebo GUI)에 `QT_QPA_PLATFORM` 등을 주입하는 용도이며,
      `core/launch_runner.py`의 `_gui_env()`가 `config/settings.yaml`의
      `gui_env` 블록에서 읽어 넘긴다(semiH_web/CLAUDE.md 9절). headless
      bringup·`ros2 run` 노드는 이 인자를 넘기지 않으므로 영향이 없다.

    Returns:
        시작된 job의 상태 dict (get_job_status(stage)와 동일한 형태).

    Raises:
        JobConflictError: 자원 충돌 또는 이미 실행 중인 경우.
    """
    conflict = check_conflict(stage)  # _all_locks() 호출로 stale lock 정리도 겸함
    if conflict is not None:
        raise JobConflictError(
            f"'{conflict}'가 겹치는 자원({', '.join(STAGE_RESOURCES.get(stage, []))})을 "
            f"사용 중이라 '{stage}'를 시작할 수 없습니다."
        )

    lock_path = _lock_path(stage)
    placeholder = {
        "pid": None,
        "starting": True,
        "started_at": time.time(),
        "cmd": cmd,
        "resources": STAGE_RESOURCES.get(stage, []),
        "log_path": None,
        **(extra_meta or {}),
    }
    try:
        fd = os.open(lock_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    except FileExistsError as exc:
        raise JobConflictError(f"'{stage}'가 이미 실행 중이거나 지금 막 시작되고 있습니다.") from exc
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(placeholder, f, ensure_ascii=False, indent=2)
    except Exception:
        lock_path.unlink(missing_ok=True)
        raise

    # 여기부터는 이 stage의 lock을 우리만 쥐고 있음이 원자적으로 보장된다 --
    # 이제 실제 subprocess를 띄우고, 완성된 정보(진짜 pid)로 lock을 덮어쓴다.
    try:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        log_path = JOB_LOGS_DIR / f"{stage}_{timestamp}.log"
        log_file = open(log_path, "w", encoding="utf-8")

        proc = subprocess.Popen(
            cmd,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            cwd=str(PROJECT_ROOT),
            start_new_session=True,  # 새 세션/프로세스 그룹 leader (docstring 2번)
            env=_merged_env(env_overlay),  # None이면 부모 환경 그대로 상속(기존 동작)
        )
        # 자식 프로세스가 OS 레벨에서 파일 핸들을 복제해 갖고 있으므로, 부모(이
        # 웹 프로세스)가 파일 객체를 닫아도 자식의 로그 기록은 계속된다.
        log_file.close()
    except Exception:
        lock_path.unlink(missing_ok=True)
        raise

    lock_info = {
        "pid": proc.pid,
        "started_at": placeholder["started_at"],
        "cmd": cmd,
        "resources": STAGE_RESOURCES.get(stage, []),
        "log_path": str(log_path),
        **(extra_meta or {}),
    }
    with open(lock_path, "w", encoding="utf-8") as f:
        json.dump(lock_info, f, ensure_ascii=False, indent=2)

    return {"running": True, **lock_info}


def _signal_group(pid: int, sig: signal.Signals) -> bool:
    """`pid`가 leader인 프로세스 그룹 전체에 `sig`를 보낸다 (`os.killpg`).

    `start_job()`이 `start_new_session=True`로 띄웠으므로 이 job의 pid는
    항상 자기 프로세스 그룹의 pgid와 같다 — 이 전제가 깨지면(예: 다른
    경로로 lock 파일에 pid가 잘못 채워진 경우) `os.killpg`가
    `ProcessLookupError`/`PermissionError`를 던질 수 있으므로 잡아서
    False를 반환한다.
    """
    try:
        os.killpg(pid, sig)
        return True
    except (ProcessLookupError, PermissionError):
        return False


def stop_job(stage: str, graceful: bool = True, graceful_timeout_s: float = 5.0) -> bool:
    """실행 중인 job을 종료한다.

    `graceful=True`면 먼저 프로세스 그룹 전체에 SIGINT를 보내고
    (`_signal_group`), `graceful_timeout_s`초 동안 leader pid가 스스로
    사라지길 기다린 뒤, 그래도 살아있으면 프로세스 그룹 전체에 SIGKILL로
    escalate한다. `graceful=False`면 SIGINT 단계를 건너뛰고 곧바로
    SIGKILL한다.

    항상 프로세스 그룹 전체(`os.killpg`)를 겨냥한다 — leader pid 하나만
    시그널했을 때 `ign gazebo`(셸 래퍼를 통해 기동되는 손자 프로세스)가
    고아로 남는 것을 라이브로 확인했기 때문(모듈 docstring 2번,
    semiH_web/CLAUDE.md 2절 실측 근거).

    Returns:
        실제로 종료 시도를 했으면 True, 애초에 실행 중이 아니었으면 False.
    """
    info = get_job_status(stage)
    if not info["running"]:
        return False
    pid = info["pid"]
    if pid is None:
        # 원자적 lock 생성 직후 ~ 실제 subprocess pid 확정 전의 극히 짧은
        # "starting" 구간 -- 죽일 대상 프로세스가 아직 없으므로 lock만 정리.
        _lock_path(stage).unlink(missing_ok=True)
        return True

    if graceful and _signal_group(pid, signal.SIGINT):
        deadline = time.time() + graceful_timeout_s
        while time.time() < deadline:
            if not _pid_alive(pid):
                _lock_path(stage).unlink(missing_ok=True)
                return True
            time.sleep(0.2)

    _signal_group(pid, signal.SIGKILL)
    _lock_path(stage).unlink(missing_ok=True)
    return True


def tail_log(log_path: str, n_lines: int = 200) -> str:
    """`log_path`의 마지막 `n_lines`줄을 문자열로 반환한다. 파일이 없으면 빈 문자열."""
    path = Path(log_path)
    if not path.exists():
        return ""
    with open(path, encoding="utf-8", errors="replace") as f:
        lines = f.readlines()
    return "".join(lines[-n_lines:])


# ROS 2 launch/노드 로그에서 실제로 관찰된 에러 계열 문자열 기준(이번 세션 중
# semiH_ws 라이브 실행 로그에서 실측: "Timed out waiting for transform",
# "died before starting" 유의 launch 자체 에러 메시지, Python 노드의
# 트레이스백 등). so101_web처럼 검증된 대규모 실사용 이력이 있는 목록은
# 아니므로, 새 크래시 패턴이 관찰되면 이 목록을 갱신할 것(추측으로 미리
# 채워 넣지 않음).
_CRASH_PATTERNS = (
    "traceback (most recent call last)",
    "died before starting",
    "rlexception",
    "modulenotfounderror",
    "filenotfounderror",
    "permissionerror",
    "connectionerror",
)


def looks_like_crash(log_text: str) -> bool:
    """로그 텍스트에 흔한 크래시 패턴이 있는지 최소한의 문자열 휴리스틱으로 판단한다."""
    lowered = log_text.lower()
    return any(pattern in lowered for pattern in _CRASH_PATTERNS)
