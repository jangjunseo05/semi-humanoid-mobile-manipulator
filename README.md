# semiH — 세미 휴머노이드 모바일 매니퓰레이터

[English](README.en.md)

세미 휴머노이드 모바일 매니퓰레이터를 위한 ROS 2(Humble) 스택입니다. Gazebo 시뮬레이션,
SLAM/Nav2 자율 주행, YOLO 기반 인식과 해석적 팔 IK, 동기화된 멀티모달 데이터 수집으로 구성되며,
터미널 없이 전체 스택을 실행하고 모니터링하는 Streamlit 웹 대시보드(`semiH_web/`)가 함께 들어 있습니다.

## 저장소 구성

```
src/
├── semih_description/      URDF/xacro 로봇 모델, Gazebo 월드, bringup/SLAM/Nav2/explore launch 파일
├── semih_perception/       YOLO 물체 검출, 목표 3D 위치 추정, 해석적 팔 IK
├── semih_data_collection/  동기화 HDF5 데이터 수집 (카메라, 라이다, 관절 상태, 오도메트리)
└── m-explore-ros2/         외부 의존성 (frontier 탐색) — 저장소에 포함하지 않음, 설치 항목 참고

semiH_web/                  위 ROS 2 스택을 실행/중지/모니터링하는 Streamlit 대시보드
```

## 로봇 스택 (`src/`)

### `semih_description` — 시뮬레이션과 bringup
Gazebo, SLAM, Nav2를 터미널 여러 개에서 따로 실행하는 대신, 누적되는 `mode` 인자 하나로
`bringup.launch.py` 하나에서 실행합니다.

```bash
ros2 launch semih_description bringup.launch.py mode:=sim      # Gazebo만 (기본값)
ros2 launch semih_description bringup.launch.py mode:=slam     # + slam_toolbox
ros2 launch semih_description bringup.launch.py mode:=nav      # + slam_toolbox + Nav2
ros2 launch semih_description bringup.launch.py mode:=explore  # + slam_toolbox + Nav2 + explore_lite (자율 frontier 탐색)
```

`mode:=explore`는 사람이 목표를 주지 않아도 스스로 목표를 골라 주행하고, 도달 가능한 미탐색
frontier가 더 이상 없으면 멈춥니다. 개발 중 발견해 우회한 버그가 하나 있습니다. 로봇이 정확히
`(0,0)`에 스폰되는데, 이 지점이 SLAM으로 처음 만들어진 costmap의 경계에 걸려 `explore_lite`의 첫
frontier 탐색이 바로 실패하고 다시 시도하지 않습니다. 그래서 launch 파일이 시작 약 20초 뒤 로봇을
앞으로 살짝 자동 이동시켜, 탐색이 시작되기 전에 그 경계 좌표에서 벗어나게 합니다(제자리 회전으로는
해결되지 않고, 직선 이동이어야 합니다).

### `semih_perception` — 인식과 팔 제어
- `yolo_detection_node` — YOLOv8(Ultralytics) 물체 검출
- `target_3d_node` — 검출한 목표의 3D 좌표를 뎁스 카메라로 추출
- `arm_reach_node` — 위치를 구한 목표에 팔을 뻗는 해석적 역기구학(IK)
- `target_approach_node` — 멀리서 검출한 목표 좌표를 odom에 저장하고, 팔이 닿는 거리(수평 0.33m)까지
  주행한 뒤 그 좌표를 팔에 넘깁니다. 가까이 가면 검출이 불안정해지는 문제를 피하기 위한 구성입니다.
  Gazebo에서 같은 조건 10회 중 9회 팔 끝이 측정한 가방 표면점 2cm 안까지 갔고(YOLO 신뢰도 임계값 0.2
  조건), 이후 팔 끝 링크에 단 접촉 센서로 같은 조건 3회 모두 가방에 닿는 것을 확인했습니다. 남은 약 2cm는
  링크 두께(반지름 1.6cm) 때문입니다. 다른 시작 위치 8곳에서는 YOLOv8n이 이 가방을 suitcase/umbrella로
  보아 backpack 조건으로는 모두 검출에 실패했고, 인식 클래스를 넓히면 7곳 모두 접촉했습니다(병목은 인식).
  물체를 잡는(grasp) 기능은 없습니다. 사용법은 `docs/usage_guide.md` 4절.

### `semih_data_collection` — 데이터셋 녹화
- `hdf5_collector_node` — 카메라, 라이다, 관절 상태, 오도메트리 토픽을 `message_filters`로
  동기화해, 모방학습용 데이터셋 형태의 HDF5 파일로 저장합니다.
  저장소 루트의 `hdf5_to_lerobot.py`는 녹화한 HDF5를 LeRobot 데이터셋 형식으로 변환합니다.

## 웹 대시보드 (`semiH_web/`)

위의 `ros2 launch`/`ros2 run` 프로세스를 감싸, 터미널 4~5개 대신 브라우저에서 스택을 시작·중지·
모니터링할 수 있게 한 단일 페이지 Streamlit 앱입니다. `semiH_web`은 위 로봇 스택을 읽기만 하며,
`src/` 아래 파일은 절대 수정하지 않습니다.

- **`core/process_manager.py`** — 작업/잠금 엔진. 실행 상태를 디스크의 `state/locks/*.json`에
  저장해(Streamlit 세션 상태만이 아니라) 페이지 새로고침이나 서버 재시작 뒤에도 유지됩니다.
  `O_CREAT | O_EXCL` 원자적 파일 생성으로 중복 시작 요청에 따른 경쟁 상태를 막고, 모든 작업을
  새 프로세스 그룹의 리더로 실행(`start_new_session=True`)한 뒤 `os.killpg`로 종료(SIGINT → 시간
  초과 시 SIGKILL)해, 셸 래퍼를 거쳐 생긴 손자 프로세스(예: `ign gazebo`)가 고아로 남지 않게
  합니다. 잠금이 유효한지는 시간 초과가 아니라 실제 PID 생존 여부(`psutil`/`os.waitpid`)로 판단해
  좀비 프로세스를 잘못 판단하지 않게 했고, 단계별 `STAGE_RESOURCES` 표로 자원 충돌을 표시합니다.
- **`core/launch_runner.py`** — `bringup.launch.py`의 네 가지 모드를 감싸며 웹 계층에서 모드 값을
  검증합니다(원래 launch 파일은 잘못된 모드 값이 오면 조용히 `sim`으로 넘어갑니다). Nav2 목표를
  설정할 UI가 따로 없어 독립 RViz2 실행 버튼을, YOLO 결과 이미지 토픽을 보기 위한 `rqt_image_view`
  버튼을 추가했고, GPU 가속이 없는 WSLg 환경을 위해 소프트웨어 렌더링 환경 변수를 넣어 줍니다.
- **`core/node_runner.py`** — 자체 launch 파일이 없는 `ros2 run` 노드 4개를 시작·중지합니다. 잠금
  정책은 실제로 관찰한 동작에 따라 노드마다 다릅니다. `hdf5_collector`는 동시 실행 시 파일 이름
  충돌로 데이터가 사라지는 버그를 확인한 뒤 단일 인스턴스 잠금을 강하게 걸었고, 나머지 3개는
  동시 실행이 안전한 것을 확인해 서로 다른 키로 여러 인스턴스를 허용합니다.
- **`monitoring/topic_watcher.py`** — `launch_runner`/`node_runner` 상태를 UI용 딕셔너리 하나로
  합치며, 프로세스 생존 여부와 실제 토픽 발행 여부를 구분합니다(토픽 발행 확인은 범위를 정해 둔 TODO).
- **`pages/main.py` / `lib/ui_components.py`** — 대시보드 본체입니다. 환경(bringup) 카드, 인식
  파이프라인 카드(YOLO → 3D 위치 추정 → 팔 뻗기), 데이터 수집 카드로 구성되며, 상태에 따라
  다음 행동을 안내하는 문구와 처음 사용하는 사람을 위한 가이드 탭이 있습니다.

## 설치

1. **ROS 2 워크스페이스** (ROS 2 Humble 필요):
   ```bash
   cd src && git clone https://github.com/robo-friends/m-explore-ros2.git  # frontier 탐색 의존성, 저장소에 포함하지 않음
   cd .. && colcon build
   ```
2. **YOLO 가중치**: `yolov8n.pt`(Ultralytics YOLOv8n)를 워크스페이스 루트에 둡니다 —
   `semiH_web/config/settings.yaml` → `yolo_model_path`에서 참조합니다.
3. **웹 대시보드**:
   ```bash
   cd semiH_web && pip install -r requirements.txt
   streamlit run pages/main.py
   ```
   `semiH_web/config/settings.yaml`에 워크스페이스 경로 등 기기별 설정이 들어 있습니다 —
   워크스페이스 위치가 개발 당시 경로와 다르면 이 파일을 수정하세요.

## 기술 스택

- **로보틱스**: ROS 2 Humble, Gazebo, slam_toolbox, Nav2, explore_lite, RViz2
- **인식**: YOLOv8(Ultralytics), 뎁스 카메라 3D 위치 추정, 해석적 IK
- **데이터**: HDF5(h5py) 동기화 멀티모달 녹화, LeRobot 형식 변환
- **대시보드**: Python, Streamlit, PyYAML, psutil, Pillow
- **실행 환경**: WSL2(Ubuntu 22.04), `subprocess` 기반 프로세스 그룹 관리
