# semiH 프로젝트 — P1~P4 개선 세션 트러블슈팅 로그

**대상 범위**: 초기 구축 완료 후 진행한 실사용성/코드 품질 개선 작업 (P1: CPU 병목, P2: 미해결 4가지 문제 실측 검증, P3: 코드 품질, P4: LeRobotDataset v3.0 연동)
**선행 문서**: `semiH_troubleshooting_log.md` (환경 구축 단계, 16개 항목) — 이 문서는 그 이후 세션을 다룸

이 문서의 항목들은 전부 **실제로 재현하고 고쳐서 확인한 것**입니다 (로그 읽고 추정만 한 게 아니라, 매번 재빌드 → 실행 → 결과 확인까지 거쳤습니다).

---

## 1. 3시간 넘게 방치된 Gazebo 세션 — CPU 800%+ 재현

**증상**: P1 진단을 시작하자마자 `ps aux`에서 `ign gazebo server`/`gui`가 각각 412%/439% CPU를 3시간 21분째 먹고 있는 걸 발견. 이번 세션과 무관하게 이전 작업이 끝난 뒤 정리가 안 된 채 계속 돌고 있었음.

**원인**: `ros2 launch`를 Ctrl-C 없이(또는 터미널을 그냥 닫아서) 종료했을 때, Gazebo 서버/GUI + 브릿지 6개 + `robot_state_publisher`가 부모 프로세스만 죽고 자식들은 살아남음.

**해결**: `cleanup.sh` 작성 (아래 4번 참고). SIGTERM 후 3초 대기, 안 죽으면 SIGKILL.

**교훈**: "잔여 프로세스"는 추정이 아니라 세션 시작 전 항상 `ps aux`로 먼저 확인해야 하는 대상. 오늘 이후 모든 테스트 스크립트는 시작/종료에 `cleanup.sh`를 넣는 걸 기본값으로 함.

---

## 2. GPU 패스스루는 있는데 소프트웨어 렌더링 중

**증상**: `~/.gz/rendering/ogre2.log`에 `GL_RENDERER = llvmpipe (LLVM 15.0.7, 256 bits)` — CPU 렌더링.

**진단**:
- `nvidia-smi`: RTX 5050 GPU가 WSL2에 정상 전달됨 (드라이버 레벨은 문제없음)
- `/dev/dxg`는 존재, `/dev/dri`는 **존재하지 않음** — Mesa의 DRM 기반 GPU 열거 경로가 GPU를 못 찾음
- `d3d12_dri.so` 드라이버 파일 자체는 시스템에 있음 (`/usr/lib/x86_64-linux-gnu/dri/d3d12_dri.so`)
- `MESA_D3D12_DEFAULT_ADAPTER_NAME`/`GALLIUM_DRIVER=d3d12` 환경변수로 강제 시도했으나 Fuel 모델 다운로드 대기 때문에 짧은 시간 안에 결론 못 냄

**결론**: `/dev/dri` 부재는 WSL 커널/드라이버 레벨 이슈로, 이번 세션에서 근본 해결은 하지 않음 (별도 조사 항목으로 보류). 대신 **headless 모드**로 실질적 완화.

**교훈**: WSL2 + Gazebo 조합에서 `nvidia-smi`가 GPU를 보여준다고 해서 실제 렌더링에 GPU가 쓰이는 게 보장되지 않음. 반드시 `ogre2.log`의 `GL_RENDERER` 라인으로 실측 확인할 것.

---

## 3. Gazebo headless 모드 적용

**변경**: `spawn_gazebo.launch.py`에 `headless` launch 인자 추가 (기본값 `true`, `-s` 플래그).

**효과 (실측)**: GUI 프로세스 자체가 안 뜸. 서버 단독 steady-state CPU가 기존 GUI 포함 800%대에서 **~70~100%대**로 감소.

**주의**: 헤드리스여도 카메라/뎁스카메라/라이다 센서 렌더링(ogre2)은 여전히 서버 프로세스 안에서 CPU로 돌아감 — 완전한 해결이 아니라 완화. GUI 디버깅이 필요하면 `headless:=false`.

---

## 4. `cleanup.sh` — 자기 자신을 죽이는 버그

**증상**: `cleanup.sh`를 `bash -lc "... ros2 launch semih_description ... ; ~/semiH_ws/cleanup.sh"` 형태로 **인라인 커맨드 문자열 안에서** 호출했더니, `pgrep -f`가 그 커맨드 문자열 자체(자기 자신의 argv)에서 `"ros2 launch semih_description"` 패턴을 찾아내 **현재 실행 중인 쉘 프로세스를 자기 자신에게 SIGTERM**을 보내버림.

**원인**: `pgrep -f`는 프로세스의 전체 커맨드라인을 대상으로 매칭한다. 스크립트를 파일로 안 만들고 긴 커맨드를 그대로 `bash -lc "..."`에 넣으면, 그 문자열 자체가 검색 대상 프로세스 목록에 포함됨.

**해결**: 이후로는 `cleanup.sh`를 포함하는 모든 테스트는 반드시 **별도 `.sh` 파일로 저장 후 `bash /path/to/script.sh`로 실행**. 파일 경로(`bash /tmp/xxx.sh`)는 패턴에 안 걸림.

**교훈**: `pgrep -f` 기반 정리 스크립트를 쓸 때는 그 스크립트를 호출하는 커맨드라인 자체가 검색 패턴에 걸리지 않는지 항상 의심할 것.

---

## 5. Windows Git Bash의 자동 경로 변환 (MSYS pathconv)

**증상**: `wsl.exe -d Ubuntu-22.04 -e bash -lc '/tmp/test_nav2_p2.sh > /tmp/... 2>&1'`가 `bash: line 1: C:/Users/USER/AppData/Local/Temp/test_nav2_p2.sh: No such file or directory`로 실패.

**원인**: 이 세션은 Windows의 Git Bash에서 `wsl.exe`를 호출하는 구조인데, Git Bash(MSYS)는 인자가 `/`로 시작하는 절대경로처럼 보이면 **Windows 경로로 자동 변환**한다. `/tmp/...`가 인자의 맨 앞에 오면 변환 대상이 되어 `wsl.exe`에 엉뚱한 Windows 경로가 전달됨.

**해결**: 경로를 인자의 맨 앞에 두지 않기 — `bash /tmp/script.sh`처럼 `bash `를 앞에 붙이면 변환을 피함.

**교훈**: Windows(Git Bash) → WSL 브리지 환경에서 `/`로 시작하는 문자열을 통째로 커맨드 인자로 넘길 때는 항상 이 변환을 의심할 것.

---

## 6. HDF5 수집 0프레임 — 근본 원인 3단 체인 (가장 중요한 발견)

트러블슈팅 로그 #16에서 "CPU 과부하"로 추정했던 문제. 실제로는 CPU와 거의 무관한, 세 가지 독립된 버그가 겹쳐 있었다.

### 6-1. `/odom` gz 브릿지 토픽명 불일치 (진짜 근본 원인)

`joint_state_bridge.yaml`이 gz 토픽 `/model/semih/odometry`를 구독하도록 설정돼 있었는데, 실제 `DiffDrive` 플러그인은 URDF의 `<odom_topic>odom</odom_topic>` 설정에 따라 그냥 `/odom`이라는 gz 토픽에 발행하고 있었다. **이름이 아예 안 맞아서 브릿지가 존재하지도 않는 gz 토픽을 구독** — ROS 쪽 `/odom` 토픽은 `ros2 topic list`엔 항상 나왔지만 (브릿지 프로세스 자체는 살아있으니) **메시지가 단 한 번도 흐른 적이 없었다.**

- 확인 방법: `ign topic -l | grep odom` → gz 쪽 실제 토픽명은 `/odom` 하나뿐, `/model/semih/odometry`는 존재하지 않음.
- TF는 별도의 (이름이 정확히 맞는) `tf_topic` 브릿지를 통해서 흘렀기 때문에, TF 기반인 Nav2는 이 버그와 무관하게 정상 동작했다 — "Nav2는 되는데 왜 /odom만 안 되지"라는 혼란의 원인.
- **수정**: `joint_state_bridge.yaml`의 `gz_topic_name`을 `/model/semih/odometry` → `/odom`으로 변경.

### 6-2. `/joint_states` 878Hz 무제한 발행

`ignition::gazebo::systems::JointStatePublisher` 플러그인에 `<update_rate>`가 없어서 물리 스텝에 가까운 속도(**실측 848~878Hz**)로 무제한 발행 중이었음. `ApproximateTimeSynchronizer`의 `queue_size=20`짜리 큐가 이 토픽 하나로 20/878초(≈23ms) 만에 꽉 차서, 카메라(22Hz)/라이다(8Hz) 프레임이 도착했을 때는 이미 매칭 대상이 밀려나 있었다 — 이게 "0프레임 수집"의 직접적 트리거.

- `<update_rate>` 태그를 추가해봤지만 **Fortress(ign-gazebo6)에서는 무시됨** (실측 확인, 878Hz 그대로).
- **수정**: 근본적으로 발행 속도를 못 줄이므로, `/joint_states`를 동기화 대상에서 아예 빼고 "최신값 캐싱" 방식(콜백에서 값만 저장, 동기화된 프레임 저장 시점에 최신 캐시값을 붙임)으로 변경. 관절각은 완만하게 변하는 값이라 이 방식이 오히려 더 적합.

### 6-3. `/joint_states`에 발행자가 2개 — 배열 길이 불일치로 크래시

6-2를 고친 직후 재테스트에서 `np.stack()`이 `all input arrays must have the same shape`로 크래시. 원인은 `/joint_states`에 **서로 다른 두 발행자**가 있었던 것:
- `joint_state_broadcaster` (ros2_control, arm_joint1~3만, 3개)
- Gazebo의 raw `JointStatePublisher` 브릿지 (arm+wheel 전부, 5개)

최신값 캐싱 콜백이 둘 중 아무거나 마지막에 온 걸 저장하다 보니 프레임마다 배열 길이가 3 또는 5로 들쭉날쭉.

- **수정**: 콜백에서 `'left_wheel_joint' in msg.name`이면 무시하고 arm 전용(3개) 메시지만 캐싱.

### 최종 결과 (실측)
15~20초 수집에 **60~109프레임** 정상 수집, `verify_dataset.py` 전항목 통과 (drift 전부 100ms 이내).

**교훈**: "CPU 과부하일 가능성이 높다"는 진단은 진짜 원인을 못 찾았을 때의 그럴듯한 자기위안일 수 있다. 토픽 이름이 실제로 매칭되는지(`ign topic -l` vs 브릿지 yaml), 발행 속도가 합리적인지(`ros2 topic hz`)는 항상 직접 찍어봐야 한다.

---

## 7. Nav2 `bt_loop_duration` 10ms는 이 환경엔 비현실적

**증상**: "Behavior Tree tick rate exceeded" 경고.

**원인**: `nav2_bringup` 기본값 `bt_loop_duration: 10`(ms)는 100Hz BT 틱을 요구하는데, WSL2에서 Gazebo+SLAM+Nav2를 동시에 돌리는 상황에선 그 예산을 못 맞춤.

**수정**: `nav2_params.yaml`의 `bt_loop_duration`을 `100`으로 완화.

---

## 8. `ros_gz_bridge`는 센서 토픽을 best-effort로 발행 — reliable 구독자는 무응답

**증상**: `hdf5_collector_node`가 `/scan`, `/odom`을 구독해도 반응 없음 (6번 문제와 겹쳐서 나타났지만 별개 이슈).

**원인**: `ros_gz_bridge`가 센서성 GZ→ROS 브릿지는 기본적으로 best-effort QoS로 발행하는데, `rclpy`의 `create_subscription` 기본 QoS는 reliable. DDS 규칙상 reliable 구독자는 best-effort 발행자를 만나면 아예 매칭이 안 돼서 메시지를 못 받음.

**수정**: `yolo_detection_node.py`, `target_3d_node.py`, `hdf5_collector_node.py`의 센서 구독을 전부 `qos_profile_sensor_data`(best-effort)로 변경.

**참고**: 6번 문제(odom 토픽명 불일치)를 고치기 전까지는 QoS를 고쳐도 `/odom`은 여전히 0프레임이었음 — 즉 이 QoS 이슈와 6번은 **서로 다른 버그가 우연히 같은 증상(무응답)으로 겹쳐** 있었던 것. 하나 고쳤다고 다른 하나가 저절로 고쳐지지 않는다는 걸 실측으로 확인.

---

## 9. XML 주석 안에 `--`를 쓰면 xacro 파싱이 깨짐

**증상**: `ros2 launch`가 `XML parsing error: not well-formed (invalid token)`로 즉시 실패.

**원인**: `semih.urdf.xacro`에 추가한 주석 안에 "... frames -- not CPU load ..."처럼 더블하이픈(`--`)을 썼음. XML 주석(`<!-- -->`) 내부에는 `--`가 (마지막을 제외하고) 올 수 없다는 게 XML 스펙.

**수정**: 주석 문구에서 `--`를 제거.

**교훈**: xacro/URDF 주석을 쓸 때 자연스럽게 "-- 이런 식으로" 강조하는 습관이 있으면 특히 주의. 변경 후 `xacro <file> > /dev/null`로 파싱만이라도 확인하는 습관이 저렴하고 확실함.

---

## 10. `set -u`와 ROS2 `setup.bash`는 상극

**증상**: 자동화 테스트 스크립트 맨 위에 `set -u`를 넣고 `source /opt/ros/humble/setup.bash`를 실행하면 `AMENT_TRACE_SETUP_FILES: unbound variable`로 즉시 죽음.

**원인**: ROS2의 `setup.bash` 체인이 내부적으로 아직 정의되지 않았을 수 있는 환경변수를 참조하는 부분이 있음. `set -u`(미정의 변수 참조 시 에러)와 충돌.

**해결**: ROS 환경을 소싱하는 스크립트에서는 `set -u`를 쓰지 않음.

---

## 11. `lerobot` pip 의존성의 opencv 버전 충돌 위험

**증상 (사전 확인, 실제로 겪지는 않음)**: `lerobot==0.4.4`는 `opencv-python-headless<4.13,>=4.9`를 요구하는데, 현재 ROS 환경은 apt로 설치된 `opencv 5.0.0`을 `cv_bridge`/YOLO가 쓰고 있음.

**대응**: 시스템/ROS 파이썬 환경에 직접 설치하지 않고 `python3 -m venv ~/lerobot_venv`로 완전히 격리. ROS 쪽 recording 노드는 `lerobot`을 전혀 import하지 않고, 세션 종료 후 별도 venv에서 오프라인 변환 스크립트(`hdf5_to_lerobot.py`)만 실행하는 구조로 설계.

**부수적으로 겪은 것**: `python3 -m venv`가 `ensurepip is not available` 에러로 실패 — `python3.10-venv` apt 패키지가 안 깔려 있었음 (`sudo apt install -y python3.10-venv` 필요, sudo 비번 필요해서 사용자가 직접 실행).

**교훈**: 무거운 ML 스택(torch 등)을 끌고 오는 패키지는 설치 전에 반드시 `pip index versions`/PyPI JSON API로 의존성 목록부터 확인하고, 기존 환경과 충돌 여지가 있으면 venv 격리를 기본값으로 삼을 것.

---

## 12. `explore_lite` 완전자율 탐색 — 로봇 스폰 위치가 코스트맵 경계에 걸리는 문제

P4 이후 추가 작업: `explore_lite`(frontier 기반 자율 탐색, `src/m-explore-ros2`에서 소스 빌드 — Humble용 apt 바이너리가 없음)를 붙여서 사람이 목표를 안 줘도 로봇이 스스로 탐색하도록 만드는 과정에서 겪은 문제.

**증상**: `explore_lite`를 켜면 시작하자마자 다음 로그를 남기고 영구 정지:
```
[FrontierSearch] Robot out of costmap bounds, cannot search for frontiers
No frontiers found, stopping.
Exploration stopped.
```

**원인 규명 과정**:
1. 처음엔 `costmap_topic: map`(slam_toolbox 원본 맵)을 `costmap_topic: /global_costmap/costmap`(Nav2 글로벌 코스트맵, 로봇 중심으로 항상 안정적일 거라 예상)으로 바꿔봤으나 재현됨.
2. `/global_costmap/costmap`을 직접 `rclpy`로 구독 테스트(volatile/transient_local 둘 다) 했으나 **메시지가 한 번도 안 옴** — 원인 불명인 채로 이 방향은 포기하고 `map`으로 원복.
3. `planner_server`(글로벌 코스트맵을 담고 있는 프로세스)의 실제 로그를 직접 확인해서 근본 원인 확정:
   ```
   StaticLayer: Resizing costmap to 159 X 180 at 0.050000 m/pix
   Robot is out of bounds of the costmap!
   Sensor origin at (-0.00, -0.00) is out of map bounds (0.00, -5.97) to (7.93, 3.01)
   ```
   로봇 스폰 좌표 (0,0)이 코스트맵 경계의 **정확히 X=0.00 모서리**에 걸쳐 있어서 `worldToMap()` 경계 체크가 실패. `explore_lite`는 이 체크를 시작 시 **딱 한 번만** 하고, 실패하면 `stop(true)`를 호출해 타이머를 취소 — 이후 **재시도를 절대 안 함**.

**해결**: `bringup.launch.py`의 `mode:=explore`에 자동 "프라이밍" 동작 추가 — 시작 20초 후 짧게 전진(`linear.x`)해서 로봇의 (x,y)를 경계 밖으로 옮긴 뒤, `explore_lite`를 시작. 시행착오:
- 처음엔 회전(`angular.z`)만 시켰더니 **전혀 효과 없음** — 회전은 방향만 바꾸지 위치(x,y)는 안 바뀜. 원점 문제는 위치 문제라 직진이 필요.
- `timeout 1.5`로 짧게 줬더니 `ros2 topic pub`의 디스커버리/핸드셰이크 오버헤드가 예산 대부분을 먹어서 메시지가 3개(≈0.03m 이동)밖에 안 나감 — 너무 짧아 경계를 확실히 못 벗어남. `timeout 5`로 늘려서 해결.
- 정지 명령(`"{}"` 빈 Twist 1회)도 한 번으론 확실히 안 멈춰서, `-r 10`으로 3초간 반복 발행하도록 강화.

**최종 확인 (실측)**: 사람 개입 없이 로봇이 t=40s에 0.68m, t=70s에 1.34m까지 자율 이동, 이후 `"All frontiers traversed/tried out, stopping."` → `"Exploration stopped."` → `"Returning to initial pose."`로 정상 종료(에러 아님, 탐색 가능 영역을 다 돈 것).

**교훈**:
- 로봇 스폰 좌표를 원점(0,0)처럼 "딱 떨어지는" 값으로 두면, 그리드 기반 경계 계산에서 부동소수점 경계 케이스에 걸릴 수 있다. 완전히 못 피하는 문제는 아니지만, 이런 종류의 버그가 나올 수 있다는 걸 염두에 둘 것.
- 서드파티 패키지가 "실패 시 조용히 영구 정지하고 재시도 안 함" 같은 동작을 하는지는 소스를 직접 봐야 안다 — 로그 메시지만 보고 "언젠가 재시도하겠지"라고 가정하지 말 것.
- 회전과 직진은 전혀 다른 효과다 — "로봇을 움직여서 뭔가를 우회한다"는 계획을 세울 때 정확히 어떤 상태(위치 vs 방향)를 바꿔야 하는지 먼저 명확히 할 것.
- `ros2 topic pub`을 스크립트/launch에서 타임박스로 쓸 때는 디스커버리 오버헤드를 감안해 넉넉하게 잡을 것 — 실제 발행 시간이 아니라 벽시계 시간 기준으로 timeout이 걸린다.

---

## 13. `cleanup.sh`의 `robot_state_publisher` 패턴이 하루 종일 전혀 매칭되지 않았음

**증상**: `explore_lite` 라이브 세션 CPU가 다시 튀어서(`load average` 70대) 확인해보니, **4.5시간 전부터 쌓인 `robot_state_publisher` 프로세스가 27개** 동시에 떠있었음. 다른 프로세스(Gazebo, Nav2, SLAM 등)는 `cleanup.sh`가 정상적으로 정리하고 있었는데 이것만 계속 새어나갔음.

**원인**: `cleanup.sh`(P1에서 작성)의 패턴이 `'robot_state_publisher --ros-args.*robot_description'`였는데, 실제 프로세스 명령줄에는 `robot_description`이라는 문자열이 **전혀 등장하지 않음** — 그 값은 `--params-file`로 넘기는 임시 YAML 파일 *내용* 안에 들어있고, `pgrep -f`는 프로세스의 실제 argv만 보지 파일 내용은 못 봄. 그래서 이 패턴은 처음 작성된 순간부터 단 한 번도 매칭된 적이 없었고, `cleanup.sh`를 몇 번을 돌려도 이 프로세스만 계속 살아남아 쌓였음.

**해결**: 패턴을 `'robot_state_publisher/robot_state_publisher --ros-args'`로 변경 (실행 파일 경로 자체가 이미 충분히 특정적이라 추가 조건 불필요).

**교훈**: `pgrep -f` 패턴을 작성한 뒤에는 반드시 `pgrep -af "패턴"`으로 **실제 매칭되는지 직접 확인**할 것. "이 정도면 맞겠지"로 작성하고 안 돌아온 적이 있는지(=아예 매칭이 안 되는 상태로 몇 시간 동안 조용히 새고 있었는지) 자체를 감지하기 어렵다는 게 이런 버그의 무서운 점 — 에러가 안 나고 그냥 "정리 대상 없음"처럼 보이기 때문. 정리 스크립트를 새로 만들 때마다 일부러 정리 대상이 있는 상태에서 한 번 실행해보고 개수가 기대한 만큼 줄어드는지 확인하는 습관이 필요함.

---

## 14. teleop 수동 조작 중 회전만 안 먹힘 — `velocity_smoother`와 `/cmd_vel` 충돌

**증상**: `mode:=nav`(또는 `explore`)가 떠있는 상태에서 `teleop_twist_keyboard`로 조작하면 전진/후진/정지는 되는데 좌/우 회전만 안 먹힘.

**원인**: `teleop_twist_keyboard`가 기본적으로 `/cmd_vel`에 직접 발행하는데, Nav2의 `velocity_smoother`도 **같은 `/cmd_vel`에 동시에 발행 중**(`nav2_params.yaml`에서 `cmd_vel_nav`를 입력받아 smoothing 후 `/cmd_vel`로 내보내는 리매핑 구조). 두 발행자가 같은 토픽을 놓고 경합하는데, `velocity_smoother`의 `max_decel` 설정이 `[-1.0, 0.0, -2.0]`(x, y, theta 순)이라 **회전 감속이 직진 감속보다 2배 빠름** — 그래서 회전 명령이 직진보다 훨씬 빨리 지워져 "회전만 안 되는" 것처럼 보임.

**해결**: teleop을 `velocity_smoother`의 입력 쪽으로 보내면 경합이 사라짐:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_nav
```

**교훈**: Nav2 스택이 떠있는 상태에서 `/cmd_vel`에 뭔가를 직접 발행하려는 다른 도구(teleop, 커스텀 스크립트 등)를 쓸 때는, Nav2 내부적으로 이미 그 토픽을 리매핑해서 쓰고 있는 노드가 없는지 먼저 확인할 것. 증상이 "일부 축만 이상하게 동작"으로 나타나면 완전 실패보다 오히려 원인 파악이 더 어려움 (설정값 하나하나를 비교해봐야 비대칭성이 드러남).

---

## 15. Nav2 config 변경은 관련 서버를 껐다 켜야 반영됨 — 부분 재시작으로는 안 됨

**증상**: `inflation_radius`를 0.4→0.25로 고치고 재빌드했는데, `explore_lite`만 재시작(`Ctrl+C` 후 `ros2 launch semih_description explore.launch.py`만 재실행)하니 동일한 지점에서 계속 막힘.

**원인**: `inflation_radius`는 `nav2_params.yaml`에 있고, 이 값은 `planner_server`/`controller_server`(코스트맵을 들고 있는 프로세스)가 **시작할 때 한 번만** 읽어들임. `explore_lite`는 Nav2 코스트맵을 구독만 하는 별개 프로세스라, 이걸 재시작해도 이미 떠있는 Nav2 프로세스들이 들고 있는 설정은 그대로임. `ps -o etime`으로 확인해보니 Nav2 프로세스들이 설정 변경 훨씬 전부터(23분째) 그대로 떠있었음.

**해결**: `nav2_params.yaml`이나 `slam_params.yaml`처럼 서버 프로세스 자체가 기동 시 읽는 설정을 바꿨다면, **`mode:=nav`/`mode:=explore` 전체를 처음부터 다시 launch**해야 함. 부분 재시작(개별 노드만 `ros2 run`으로 다시 켜기)은 그 노드 자신의 파라미터만 갱신됨.

**교훈**: 설정 파일을 고친 뒤 "반영이 안 된 것 같다"는 증상이 나오면, 제일 먼저 `ps -o pid,etime,cmd -p <PID>`로 그 설정을 실제로 들고 있는 프로세스가 **내가 고친 시점 이후에 새로 뜬 게 맞는지**부터 확인할 것. 재시작 범위를 정확히 아는 것 자체가 디버깅 시간을 크게 좌우함.

---

## 16. (미해결) 자율 탐색이 미로 완주 전에 간헐적으로 멈춤

`inflation_radius` 완화 후에도 로봇이 조금 움직이다가 다시 멈추는 현상이 재현됨. 오늘 세션에서 근본 원인까지는 못 잡고 다음 세션으로 넘김.

**지금까지 확인된 것**:
- CPU 문제 아님 (막혔을 때 idle 45%로 여유 있었음)
- 최소 한 가지 사례는 특정 좌표(벽 모서리 근처 미도달 지점)를 `explore_lite`가 계속 재시도하면서 멈춘 것으로 확인됨(`planner_server` 로그의 반복되는 `failed to create plan` / `Aborting handle`) — `inflation_radius` 완화로 이 특정 케이스는 해결됨
- 완화 후에도 "조금 움직이다 멈춤"이 재현 — 아직 라이브로 새 정지 지점을 못 잡음(사용자가 세션을 반복 재시작하는 동안 Nav2 설정 미반영 등 다른 변수가 섞여있어 깨끗한 재현 케이스를 못 얻음)

**다음 세션 시작점**:
1. 완전히 새로 `mode:=explore` 기동 후, `planner_server`/`controller_server`/`explore_node` 로그를 실시간으로 tail하며 정확히 어느 시점에 멈추는지 잡을 것 (오늘처럼 로그 파일을 사후에 찾아보는 것보다, `tail -f`로 붙여서 실시간 관찰하는 게 더 빠를 것)
2. `explore_lite`의 `progress_timeout`(현재 30초) 기반 블랙리스트 로직이 실제로 발동하는지 확인 — `explore.cpp`의 `last_progress_` 갱신 조건(`prev_distance_ > frontier->min_distance`)이 미세한 진동성 움직임에도 계속 갱신되어 타임아웃이 영영 안 걸릴 가능성을 코드로 확인했으나 실측 검증은 아직 안 함
3. 미로 벽 좌표를 기준으로 통로 폭을 직접 계산해서, `inflation_radius`(현재 0.25) + 로봇 반폭(0.15~0.2m)을 감안했을 때 통과 불가능한 폭 좁은 구간이 있는지 사전 확인 (오늘은 반응형으로 하나씩 잡았는데, 미리 좌표 기반으로 계산해두면 더 빠를 것)
