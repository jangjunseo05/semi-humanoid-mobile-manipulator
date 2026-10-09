# semiH 사용법 튜토리얼

모든 명령은 WSL2 (Ubuntu 22.04) 안에서 실행합니다. 매번 새 터미널을 열 때마다:

```bash
source /opt/ros/humble/setup.bash
source ~/semiH_ws/install/setup.bash
```

## 0. 빌드

```bash
cd ~/semiH_ws
colcon build
source install/setup.bash
```

특정 패키지만 다시 빌드하고 싶으면:
```bash
colcon build --packages-select semih_description semih_perception semih_data_collection
```

## 1. 세션 시작 전 잔여 프로세스 정리 (습관화할 것)

```bash
~/semiH_ws/cleanup.sh          # 실제 종료
~/semiH_ws/cleanup.sh --dry-run # 뭐가 걸리는지만 미리 확인
```

## 2. 시뮬레이션만 띄우기

```bash
ros2 launch semih_description spawn_gazebo.launch.py
```

- 기본값은 **headless**(`-s`, GUI 없음) — WSL2에서 GPU 가속이 안 되는 환경이라 GUI를 띄우면 CPU가 크게 뜁니다 (`docs/troubleshooting_p1-p4.md` #2, #3 참고).
- GUI로 눈으로 보고 싶으면:
  ```bash
  ros2 launch semih_description spawn_gazebo.launch.py headless:=false
  ```
- 카메라 화면 확인: `rqt_image_view` (Gazebo 내장 Image Display는 메뉴에 안 보이는 이슈가 있음, 기능상 문제는 아님)
- 로봇을 손으로 움직이기: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

기동 후 토픽이 전부 올라오기까지 (Fuel 모델 로딩 포함) **약 1분** 정도 걸립니다. 조급해하지 말 것.

## 3. SLAM + Nav2 (통합 launch, 한 줄로 실행)

터미널 4~5개 띄우던 걸 하나로 합친 `bringup.launch.py`를 씁니다.

```bash
# Gazebo만 (spawn_gazebo.launch.py와 동일)
ros2 launch semih_description bringup.launch.py mode:=sim

# Gazebo + slam_toolbox
ros2 launch semih_description bringup.launch.py mode:=slam

# Gazebo + slam_toolbox + Nav2 (제일 자주 쓰게 될 모드)
ros2 launch semih_description bringup.launch.py mode:=nav

# GUI로 보면서 nav 모드
ros2 launch semih_description bringup.launch.py mode:=nav headless:=false

# Gazebo + slam_toolbox + Nav2 + 완전자율 탐색 (사람이 목표를 안 줘도 알아서 돌아다님)
ros2 launch semih_description bringup.launch.py mode:=explore
```

내부적으로 `slam.launch.py` / `nav2.launch.py`를 include하는 구조라, SLAM/Nav2만 따로 개별 실행하고 싶으면 (예: Gazebo를 이미 딴 터미널에 띄워둔 상태에서):
```bash
ros2 launch semih_description slam.launch.py
ros2 launch semih_description nav2.launch.py
```

### mode:=explore (완전자율 탐색)

`src/m-explore-ros2`의 `explore_lite` 패키지를 붙여서, 사람이 목표를 하나도 안 줘도 로봇이 스스로 미탐색 경계(frontier)를 찾아 Nav2에 목표를 계속 보내며 돌아다닙니다. 더 이상 갈 곳이 없으면 알아서 멈추고 시작 위치로 돌아옵니다(`return_to_init: true`).

- 시작 후 20초 뒤 자동으로 짧게 전진하는 "프라이밍" 동작이 있습니다. 로봇이 정확히 (0,0)에 스폰되는데, 이 좌표가 코스트맵 경계에 딱 걸쳐서 그대로 두면 `explore_lite`가 첫 시도에서 "Robot is out of costmap bounds"로 실패하고 재시도 없이 영구 정지해버립니다(`docs/troubleshooting_p1-p4.md` #12 참고). 별도 조치 필요 없이 자동으로 처리됩니다.
- 설정은 `config/explore_params.yaml`에 있습니다. 월드가 좁으면(`min_frontier_size`) 더 작게, 넓으면 더 크게 조정하세요.
- 진행 상황 확인: `ros2 topic echo /explore/frontiers` (RViz에 마커로 표시됨) 또는 그냥 `mode:=explore headless:=false`로 GUI를 보세요.
- **현재 상태(미해결)**: `inflation_radius` 튜닝(0.4→0.25) 후에도 미로 완주 전에 간헐적으로 멈추는 현상이 있습니다. `docs/troubleshooting_p1-p4.md` #16 참고, 다음 세션에서 이어서 진단할 것.

### 수동으로 먼저 지도 넓혀두고 자율주행 넘기기 (더 안정적인 대안)

`mode:=explore`의 자동 프라이밍보다, 사람이 먼저 조금 돌아다니면서 지도를 넓혀준 뒤 자율 탐색으로 넘기는 게 더 안정적입니다 (더 넓은 지도에서 시작하니 프론티어 선택지가 많아짐).

```bash
# 1) Nav2까지만 (auto-explore 없이)
ros2 launch semih_description bringup.launch.py mode:=nav headless:=false
```
새 터미널에서 수동 조작 (**주의**: `mode:=nav`가 떠있으면 `velocity_smoother`가 `/cmd_vel`을 이미 쓰고 있어서, 기본 설정으로 teleop을 켜면 회전이 제대로 안 먹힙니다 — `docs/troubleshooting_p1-p4.md` #14):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_nav
```
지도가 어느 정도 넓어졌으면 teleop 터미널만 `Ctrl+C`, 나머지는 그대로 두고:
```bash
ros2 launch semih_description explore.launch.py
```

> `nav2_params.yaml`/`slam_params.yaml`처럼 서버가 기동 시에만 읽는 설정을 고쳤다면, 이 방식으로도 `explore_lite`만 재시작해서는 반영이 안 됩니다 — `mode:=nav`부터 전체를 다시 켜야 합니다 (`docs/troubleshooting_p1-p4.md` #15).

### 목표 전송

RViz2로 "2D Goal Pose"를 찍거나, CLI로:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

> 로봇이 아직 스캔 안 한 영역(맵 밖)으로 목표를 보내면 `off the global costmap`으로 실패합니다. 처음엔 스폰 지점 근처(1~2m 이내)로 목표를 잡거나, teleop으로 먼저 한 바퀴 돌려서 맵을 넓힌 뒤 시도하세요.

## 4. YOLO 인식 / 3D 좌표 추출 / 팔 리치

시뮬레이션이 이미 떠 있는 상태에서 (별도 터미널마다):

```bash
# 인식 결과를 이미지로 보고 싶을 때
ros2 run semih_perception yolo_detection_node
rqt_image_view   # /yolo/image_annotated 선택

# 3D 좌표 추출 (YOLO + depth camera)
ros2 run semih_perception target_3d_node

# 팔 리치 (target_3d_node가 필요, 또는 아래처럼 직접 좌표를 줘도 됨)
ros2 run semih_perception arm_reach_node
```

### 팔만 따로 테스트하고 싶을 때 (인식 파이프라인 없이)

팔 사거리는 base_link 기준 최대 0.40m입니다. 사거리 안의 좌표를 직접 publish:
```bash
ros2 topic pub --once /target_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'base_link'}, point: {x: 0.3, y: 0.0, z: 0.35}}"
```
잘 되면 `ros2 topic echo /joint_states --once`로 `arm_joint2`/`arm_joint3`가 변하는 걸 확인할 수 있습니다.

> 월드에 배치된 인식 대상(Chair, Backpack, Suitcase)은 로봇에서 1~3m 떨어져 있어 팔 사거리 밖입니다. 팔이 실제로 물체에 뻗는 걸 보려면 먼저 Nav2로 물체 근처(0.4m 이내)까지 이동시켜야 합니다.

## 5. 데이터 수집 (HDF5)

```bash
ros2 run semih_data_collection hdf5_collector_node --ros-args \
  -p duration_sec:=20.0 \
  -p output_dir:=$HOME/semih_datasets
```

주요 파라미터:
| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `duration_sec` | 10.0 | 수집 시간(초) |
| `output_dir` | `~/semih_datasets` | 저장 경로 |
| `camera_topic` / `scan_topic` / `joint_states_topic` / `odom_topic` / `action_topic` | 각 기본 토픽 | 필요시 리매핑 |
| `sync_slop` | 0.1 | 카메라/라이다/odom 동기화 허용 오차(초) |

**팔 관련 데이터를 같이 모으고 싶다면** `arm_reach_node`도 같이 띄워두세요 — 그래야 `action/arm_target_positions`에 실제 명령값이 채워집니다. 안 띄우면 전부 NaN으로 기록됩니다 (수집 자체는 정상 진행).

### 검증

```bash
python3 ~/semiH_ws/verify_dataset.py ~/semih_datasets/semih_session_YYYYMMDD_HHMMSS.h5
```
프레임 수, 동기화 간격 통계, 토픽별 drift, 각 데이터셋 shape을 출력합니다. `WARNING`이 뜨면 프레임 드랍/스톨 의심 구간입니다.

## 6. LeRobotDataset v3.0으로 변환

**최초 1회만**: 전용 venv 준비 (ROS 파이썬 환경과 절대 섞지 말 것 — `docs/troubleshooting_p1-p4.md` #11 참고)
```bash
sudo apt install -y python3.10-venv   # 처음 한 번만
python3 -m venv ~/lerobot_venv
source ~/lerobot_venv/bin/activate
pip install lerobot h5py
deactivate
```

**변환 (매번)**:
```bash
source ~/lerobot_venv/bin/activate
python3 ~/semiH_ws/hdf5_to_lerobot.py \
  ~/semih_datasets/semih_session_YYYYMMDD_HHMMSS.h5 \
  --repo-id local/semih-arm-demo \
  --root ~/lerobot_datasets/semih-arm-demo
deactivate
```

옵션:
- `--task "..."`: 프레임마다 붙는 태스크 설명 (기본값 `"reach toward target"`)
- `--camera-key front`: `observation.images.<key>` 이름
- `--repo-id`만 주고 `--root` 생략 시 `~/.cache/huggingface/lerobot/<repo-id>`에 저장됨

변환이 끝나면 스크립트가 자동으로 재로드해서 프레임 수/피처 목록을 출력합니다. lidar/odom은 LeRobot 표준 모달리티가 아니라서 변환 결과물에는 포함되지 않습니다 (원본 `.h5` 파일에는 그대로 남아있음).

## 7. 유닛 테스트

```bash
cd ~/semiH_ws
colcon build --packages-select semih_perception
source install/setup.bash
python3 -m pytest src/semih_perception/test/test_arm_ik.py -v
```

IK 로직(`solve_ik`)만 따로 검증하는 테스트입니다. 팔 기하값(`arm_link*_len` 등)을 바꾸면 `GEOM` 딕셔너리도 같이 맞춰줘야 합니다.

## 8. 자주 겪는 문제 빠른 체크리스트

- 로봇이 안 움직인다 → `ros2 topic list`에 `/cmd_vel`이 있는지, `ros2 topic hz /odom`으로 실제 데이터가 흐르는지 확인
- Nav2가 "off the global costmap" → 스캔 안 된 영역. 먼저 탐색.
- 수집 프레임이 0개 → `ros2 topic hz /camera/image_raw /scan /odom /joint_states` 각각 실제로 도는지 확인 (특정 토픽만 안 돌면 `docs/troubleshooting_p1-p4.md` #6 패턴 의심)
- 이전 세션이 안 죽고 남아있는 것 같다 → `~/semiH_ws/cleanup.sh --dry-run`으로 먼저 확인
