# semiH_web 실행/확인 가이드

목적별로 정리한 명령어 모음. 전부 **WSL2(Ubuntu-22.04) 안에서** 실행합니다
(Windows PowerShell이 아님 — ROS 2/Gazebo가 WSL2 안에만 설치돼 있음). 여기
적힌 명령은 전부 이 세션에서 실제로 실행해 확인한 것이며, 경로는 이
개발 머신(`jangjunseo` 사용자) 기준입니다 — 다른 머신이라면 1절부터
확인하세요.

---

## 0. 사전 확인 (다른 머신에서 처음 쓸 때만)

`config/settings.yaml`의 `workspace.*` 값이 실제 환경과 맞는지 확인:

```bash
cat ~/semiH_web/config/settings.yaml
```

- `workspace.path`: `semiH_ws` 워크스페이스 절대경로
- `workspace.ros_setup_bash`: ROS 배포판 `setup.bash` 절대경로
- `workspace.install_setup_bash`: `semiH_ws/install/setup.bash` 절대경로
- `dataset.output_path`, `perception.yolo_model_path`도 함께 확인

이 값이 틀리면 `launch_runner`/`node_runner`가 만드는 `bash -c "source
... && ..."` 명령 자체가 실패합니다(파일 없음 에러가 로그에 남음 —
아래 4절 로그 확인 명령으로 진단).

`semiH_ws`가 이미 `colcon build`까지 끝나 있어야 합니다(`install/`
디렉터리 존재 여부로 확인):

```bash
ls ~/semiH_ws/install/setup.bash
```

---

## 1. 최초 1회: 의존성 설치

```bash
cd ~/semiH_web
pip install --user -r requirements.txt
```

`streamlit`, `streamlit-autorefresh`, `psutil`, `PyYAML`이 설치됩니다.
`~/.local/bin`이 PATH에 있는지 확인(로그인 셸이면 보통 자동):

```bash
which streamlit
```

경로가 안 뜨면(`command not found`) 아래 2절에서 `streamlit` 대신
`python3 -m streamlit`을 쓰거나 `~/.local/bin/streamlit`처럼 전체 경로로
실행하세요.

---

## 2. 웹 서버 실행

```bash
cd ~/semiH_web
streamlit run pages/main.py
```

기본 포트는 8501입니다. 포트를 지정하거나 백그라운드(헤드리스)로 띄우려면:

```bash
cd ~/semiH_web
streamlit run pages/main.py --server.headless true --server.port 8501
```

터미널을 계속 띄워두기 싫으면 백그라운드 + 로그 파일로:

```bash
cd ~/semiH_web
nohup streamlit run pages/main.py --server.headless true --server.port 8501 \
  > /tmp/semih_web_streamlit.log 2>&1 &
disown
```

> **주의**: `streamlit run pages/main.py`를 실행하는 이 셸 자체는 ROS 2를
> source할 필요가 없습니다 — `core/launch_runner.py`/`core/node_runner.py`가
> `ros2 launch`/`ros2 run`을 실행할 때마다 그때그때 자체적으로 `source
> {ros_setup_bash} && source {install_setup_bash}`를 하기 때문입니다
> (0절의 `config/settings.yaml` 값을 읽어서 씀).

---

## 3. 웹 브라우저로 접속

서버가 뜬 뒤(`You can now view your Streamlit app...` 로그 확인):

- **WSL2 안에서 확인**: `curl -sS -o /dev/null -w '%{http_code}\n' http://localhost:8501`
  → `200`이면 정상.
- **Windows 브라우저에서 접속**: `http://localhost:8501`을 그대로 열면
  됩니다 — WSL2의 localhost 자동 포워딩 덕분에 별도 설정 없이 Windows
  쪽에서 바로 접속 가능함을 확인했습니다(별도 IP/포트포워딩 설정 불필요).

---

## 4. 실행 상태 확인 (터미널에서, UI 없이도 가능)

**현재 뭐가 떠 있는지(락 파일 기준, UI가 보여주는 것과 동일한 소스)**:

```bash
cat ~/semiH_web/state/locks/*.json 2>/dev/null
# 아무 출력도 없으면 아무것도 안 떠 있는 상태
```

**실제 OS 프로세스로 교차 확인**:

```bash
ps aux | grep -E 'ros2|gazebo|gz|parameter_bridge|nav2|slam|explore_lite|robot_state|controller|static_transform|yolo_detection_node|target_3d_node|arm_reach_node|hdf5_collector_node' \
  | grep -v grep | grep -v ros2-daemon
```

**특정 job의 실시간 로그 보기**(경로는 위 lock 파일의 `log_path` 필드에서 확인):

```bash
tail -f ~/semiH_web/logs/jobs/<파일명>.log
```

**ROS 2 그래프 자체를 직접 보고 싶을 때** (환경이 떠 있을 때만 의미 있음):

```bash
source /opt/ros/humble/setup.bash
ros2 node list
ros2 topic list
```

---

## 5. UI를 거치지 않고 명령어로 직접 조작 (디버깅용)

웹 UI 없이 파이썬으로 직접 시작/종료하고 싶을 때 (예: 스크립트 자동화,
UI 버그 의심 시 백엔드만 따로 확인):

```bash
cd ~/semiH_web
python3 -c "
import sys; sys.path.insert(0, '.')
from core import launch_runner
print(launch_runner.start_bringup(launch_runner.BringupMode.NAV, headless=True))
"
```

```bash
python3 -c "
import sys; sys.path.insert(0, '.')
from core import node_runner
print(node_runner.start_node(node_runner.NodeName.YOLO_DETECTION))
"
```

상태 조회(웹 UI 상단 카드가 쓰는 것과 동일한 함수):

```bash
python3 -c "
import sys; sys.path.insert(0, '.')
from monitoring import topic_watcher
import json
print(json.dumps(topic_watcher.get_system_status(), indent=2, default=str))
"
```

종료:

```bash
python3 -c "
import sys; sys.path.insert(0, '.')
from core import launch_runner, node_runner
launch_runner.stop_bringup(graceful=True)
for n in node_runner.NodeName:
    node_runner.stop_node(n)
"
```

---

## 6. 종료 / 정리

**권장 순서**: 웹 UI에서 먼저 각 카드의 종료 버튼(YOLO/target_3d/arm_reach/
HDF5 → 환경 순서)을 눌러 정리한 뒤 Streamlit 서버를 끄세요 — UI로 정리하면
`stop_job()`의 graceful(정상 종료 우선) 경로를 타지만, Streamlit 서버부터
먼저 죽이면 떠 있던 job들이 아무도 정리 안 된 채 남습니다.

**Streamlit 서버 자체 종료**:
- 포그라운드로 띄웠다면: 그 터미널에서 `Ctrl+C`
- 백그라운드(nohup)로 띄웠다면:
  ```bash
  pkill -f "streamlit run pages/main.py"
  ```

**UI로 못 끄고 서버까지 죽여버린 경우(잔여 프로세스 수동 정리)**:

```bash
ps aux | grep -E 'ros2|gazebo|gz|parameter_bridge|nav2|slam|explore_lite|robot_state|controller|static_transform|yolo_detection_node|target_3d_node|arm_reach_node|hdf5_collector_node' \
  | grep -v grep | grep -v ros2-daemon
# 나온 PID들을 확인하고
kill -9 <PID> <PID> ...
```

정리 후 락 파일도 비어있는지 확인:

```bash
ls ~/semiH_web/state/locks/
# 비어있어야 정상
```

---

## 7. 문제 해결

| 증상 | 확인/조치 |
|---|---|
| `streamlit: command not found` | `python3 -m streamlit run pages/main.py`로 대체, 또는 `~/.local/bin`을 PATH에 추가 |
| 포트 이미 사용 중 | `--server.port 8502`처럼 다른 포트로 |
| 버튼을 눌러도 "JobConflictError" 계속 뜸 | `cat ~/semiH_web/state/locks/*.json`으로 실제로 떠 있는지 확인 → 4절 `ps aux`로 실제 프로세스 존재 여부 교차 확인 → 프로세스는 없는데 락만 남아있으면(비정상) 해당 `.json` 파일을 수동 삭제 |
| `ros2 launch`/`ros2 run` 자체가 실패(로그에 `No such file or directory`) | 0절의 `config/settings.yaml` 경로들이 이 머신과 맞는지 재확인 |
| HDF5 수집이 "Saved N frames" 로그 이후에도 안 꺼짐 | 정상입니다(알려진 제약, `semiH_web/CLAUDE.md` 5절) — UI에서 정지 버튼을 눌러야 실제로 종료됩니다 |
| 종료 후에도 `ps aux`에 `<defunct>`(좀비)로 남음 | `semiH_web/CLAUDE.md` 6-3절 참고 — 같은 Streamlit 프로세스 안에서 새 job을 하나 더 시작하면 그 시점에 자동으로 정리됩니다(CPython의 자체 정리 동작) |
