# 포트폴리오 영상 가이드 — 세미휴머노이드 시뮬레이션 (STEP 3)

이 저장소의 시뮬레이션 데모 영상을 포트폴리오에 넣을 때 **어떤 파일을, 어디에, 어떤 문구로** 넣는지 정리한 문서다.
영상은 2026-11-16(장면 ①~⑤)과 2026-11-17(`semih_reach_demo`, `semih_dashboard_demo`)에 Gazebo Fortress + ROS 2 Humble(WSL2, RTX 4070 Laptop)에서 녹화했다.
녹화 과정과 문제 기록은 [semiH_troubleshooting_log.md](semiH_troubleshooting_log.md)와 포트폴리오 폴더의 `STEP3_결과보고.md`에 있다.

## 1. 저장 위치와 파일 이름

영상 파일은 용량 때문에 저장소에 넣지 않는다. 포트폴리오 폴더(`바탕 화면\포트폴리오\`)에 아래 이름 그대로 둔다.
각 mp4에는 첫 프레임 썸네일이 같은 이름의 `.png`로 함께 있다.

| 파일 | 길이 | 배속 | 용도 |
|---|---|---|---|
| `semih_demo_summary.mp4` (+`.png`) | 34.0s | 2× / 2× / 2× / 1× (구간별, 라벨 표기) | **대표 영상** — STEP 3 박스 맨 위 |
| `semih_slam.mp4` (+`.png`) | 18.0s | 2× | 장면 ① SLAM |
| `semih_nav2.mp4` (+`.png`) | 18.0s | 2× | 장면 ② Nav2 |
| `semih_yolo.mp4` (+`.png`) | 18.0s | 1× | 장면 ③ YOLO + 뎁스 3D 좌표 |
| `semih_arm_ik.mp4` (+`.png`) | 13.0s | 2× | 장면 ④ 팔 IK |
| `semih_perception_arm.mp4` (+`.png`) | 14.0s | 2× | (선택) 장면 ⑤ 인식→팔 연결 — 옛 팔(2026-11-16)이라 닿지 않음. 아래 `semih_reach_demo.mp4`로 대체 권장 |
| `semih_reach_demo.mp4` (+`.png`) | 28.7s | 2× (전 구간 동일, 벽시계 기준) | **2026-11-17** 탐지 → 위치 저장 → 접근 → 팔 뻗기 → 정지, 1회분 무편집. 1920×720 |
| `semih_dashboard_demo.mp4` (+`.png`) | 39.1s | 1× (장면 4만 2×) | **2026-11-17** 웹 대시보드(semiH_web) 조작 6장면. 1920×1080 |

- 원본 녹화본은 `포트폴리오\media_raw\`에 있다. 포트폴리오에는 올리지 않는다.
  - `semih_reach_demo_raw.mp4`(데스크톱 캡처 2560×1600, 120s) + `semih_reach_demo_run\`(같은 회차의 카메라 프레임 570장·노드 로그·팔 끝 측정값)
  - `semih_dashboard_demo_raw.mp4`(데스크톱 캡처, 80s, 자르지 않음) + `semih_dashboard_demo_events.json`(동작별 시각, 검증값)
  - `semih_reach_repeat10\`(반복 시험 10회의 로그와 `summary.md`)
- 새 영상 2개는 가로 1920이라 위 웹 규격(1280)보다 크다. 용량은 0.92MB / 1.36MB다.
- 웹 규격: 무음, H.264, 가로 1280, `-crf 24 -preset slow -pix_fmt yuv420p -movflags +faststart`. 모든 파일이 1MB 미만이다.
- 웹 페이지에 넣을 때는 `<video src="semih_demo_summary.mp4" poster="semih_demo_summary.png" muted loop playsinline controls>`처럼 썸네일 png를 poster로 쓴다.

## 2. 캡션 (그대로 써도 되는 문구)

| 파일 | 한국어 | English |
|---|---|---|
| `semih_demo_summary.mp4` | 시뮬레이션에서 SLAM·Nav2·팔 IK·YOLO+뎁스 3D 좌표 파이프라인이 동작하는 장면 | SLAM, Nav2, arm IK and YOLO + depth 3D localization running in a Gazebo simulation |
| `semih_slam.mp4` | 미리 정한 경로로 주행하며 slam_toolbox로 2D 지도를 실시간 작성 (2배속) | Real-time 2D mapping with slam_toolbox along a scripted route (2×) |
| `semih_nav2.mp4` | 지도 위 목표 지점을 Nav2로 자율 주행 (2배속) | Autonomous goal navigation with Nav2 on the SLAM map (2×) |
| `semih_yolo.mp4` | YOLOv8n 검출 결과와 뎁스 기반 3D 좌표를 RViz에 함께 표시 | YOLOv8n detections with depth-based 3D target points shown in RViz |
| `semih_arm_ik.mp4` | 해석적 IK로 목표점 6개에 팔 끝을 위치 (오차 0.0cm, 2배속) | Analytic IK places the arm tip on six target points (0.0 cm error, 2×) |
| `semih_perception_arm.mp4` | 검출된 물체의 3D 좌표로 팔을 뻗는 연결 동작. 목표가 작업영역 밖이라 최대 도달 거리에서 정지 | Detection-to-arm pipeline: the arm extends toward the detected object but stops at max reach (target out of workspace) |
| `semih_reach_demo.mp4` | 멀리서 YOLO로 가방을 찾아 위치를 저장하고, 그 위치로 다가가 팔을 뻗어 가방 표면 앞 2cm에서 정지 (Gazebo 시뮬레이션, 같은 조건 10회 중 9회 성공, YOLO 신뢰도 임계값 0.2, 접촉 센서 미확인, 2배속) | Detects the backpack from afar, stores its position, drives up and reaches to 2 cm short of its surface (Gazebo sim, 9/10 runs under identical conditions, YOLO confidence threshold 0.2, no contact sensor, 2×) |
| `semih_dashboard_demo.mp4` | 웹 대시보드에서 시뮬레이션 환경과 인식 노드를 켜고 끄는 장면 — 새로고침해도 상태 유지, 중복 실행 방지, 종료 시 Gazebo 창까지 정리 | Web dashboard starting/stopping the simulation and perception nodes — state survives reload, duplicate launch is blocked, stopping closes Gazebo |

### `semih_reach_demo.mp4` — 무엇이 찍혀 있나

- 왼쪽: Gazebo GUI 화면(데스크톱 캡처를 잘라 1280×720). 오른쪽: **같은 회차** 로봇 카메라 프레임.
- 단계 라벨(0/4~4/4): 시작 자세 → 탐지·위치 저장(검출 10개 중앙값을 odom에 고정) → 저장 위치로 주행(0.33m에서 정지, 이후 검출 무시) → 팔 뻗기(팔꿈치 접기 → 어깨 내리기 → 펴기) → 정지.
- 이 회차 값: 저장 좌표 odom (2.852, -0.920, 0.092), 도착 시 목표 base_link (0.340, 0.000, 0.092), 팔 끝 (0.325, 0.000, 0.079) → **표면점과 2.0cm**, clamp 없음.
- 오른쪽 YOLO 박스는 **녹화한 카메라 프레임에 같은 모델(YOLOv8n)·같은 임계값(0.2)으로 다시 그린 것**이다. 노드가 실시간으로 낸 박스 영상이 아니다. 이유: 노드의 기본 그림(`yolo_detection_node`)은 같은 박스의 umbrella 0.41 라벨이 backpack 0.24 라벨을 덮어서, 파이프라인이 실제로 쓴 backpack 박스가 안 보였다(1차 녹화에서 확인). JPEG로 저장한 프레임이라 실시간 결과와 2순위 클래스가 조금 다를 수 있다(이 회차 첫 장면: suitcase 0.47 / backpack 0.26).
- 시작 위치(odom (1.87, -0.54), 가방 쪽 -21°)까지는 테스트 스크립트로 옮겼다. 영상은 그 뒤부터다.
- GUI를 켜면 시뮬레이션이 실시간보다 느리게 돈다(화면 우하단 RTF 약 18~90%). 배속 2×는 벽시계 기준이다.
- 녹화는 2번 했다. 1차는 위 YOLO 라벨 문제로 다시 찍었고, 1차도 성공(clamp 없음, 약 2.1cm)이었다. 영상은 2차 1회분을 자르지 않고 썼다.

### `semih_dashboard_demo.mp4` — 무엇이 찍혀 있나

녹화 한 번(80초)에서 장면 6개를 잘라 이었다(장면 사이는 컷). Chrome 창을 DevTools 프로토콜로 자동 조작했고, 누르기 직전에 빨간 테두리로 표시한 뒤 실제 마우스 이벤트로 눌렀다.
1. 첫 화면 — 모든 카드 정지
2. 환경 카드: "Gazebo GUI 창도 띄우기" 체크 → nav 시작 → Gazebo 창이 뜨고 상태가 실행 중(mode nav, headless False)
3. 새로고침 → 실행 상태 유지(상태가 브라우저가 아니라 디스크 lock 파일에 있음)
4. 인식 카드: YOLO → 3D 좌표 추출 → 팔 뻗기 순서로 시작 (2×)
5. **중복 실행 방지:** 시작 전에 열어 둔 두 번째 탭(옛 화면, 시작 버튼이 보이는 상태)에서 nav를 다시 누름 → 두 번째 실행 없이 "이미 실행 중"으로 바뀜. Gazebo 서버 프로세스 수를 누르기 전후로 셌다: 1 → 1.
6. 환경 종료 → Gazebo 창이 닫힘

- **"전체 중지" 버튼은 없다.** "환경 종료"는 Gazebo/SLAM/Nav2만 끈다. 인식 노드 3개는 카드별 "⏹ 종료"로 따로 껐다(원본 65~76초, 편집본에서는 길이 때문에 뺐다). 마지막에 lock 0개, 관련 프로세스 0개를 확인했다.
- 인식 노드는 원래 여러 개 동시 실행을 허용하는 설계라(카드에 "여러 개 동시 실행 가능"이라고 적혀 있음) 노드 쪽 중복 실행은 막지 않는다. 장면 5는 **환경(bringup)** 의 중복 실행 방지다.
- 이 녹화에서 인식 노드는 대시보드 기본값(클래스 필터 없음, 신뢰도 0.4)으로 켜졌다. 이 영상은 대시보드 조작만 보여 주고, 팔이 가방에 닿는 동작은 `semih_reach_demo.mp4`에서 보인다.
- 녹화 전용 처리: Streamlit 자체 버튼(Deploy·메뉴)은 숨겼다. 화면 뒤쪽은 단색 창으로 가렸다(다른 창이 찍히지 않게).
- 대시보드는 이 PC의 별도 가상환경(`~/semih_web_venv`, streamlit 1.65.0)에서 `127.0.0.1:8502`로 띄웠다. 저장소 코드는 바꾸지 않았다.

## 3. 함께 적어야 하는 사실 (빼면 과장이 되는 것)

- **시뮬레이션 영상이다.** 실물 로봇 영상이 아니다. 캡션이나 박스 제목에 "Gazebo 시뮬레이션"을 밝힌다.
- **배속:** 2×로 편집한 클립은 캡션에 2배속을 적는다(영상 라벨에도 표기돼 있음).
- **스크립트로 준 입력:** 사람이 실시간으로 조종한 것이 아니다.
  - SLAM 주행: 미리 정한 웨이포인트를 odom 기반 P제어로 `/cmd_vel`에 보냈다(키보드 조작 대체).
  - Nav2: 목표 4개를 `navigate_to_pose` 액션으로 보냈다(4개 모두 SUCCEEDED, 클립에는 앞의 2개).
  - 팔 IK: `/target_point`에 목표 6개를 4초 간격으로 보냈다.
  - YOLO / 인식→팔: 로봇 이동만 스크립트, 검출·3D 좌표·팔 목표는 노드가 실제로 낸 값이다.
- **오검출이 영상에 남아 있다.** 여행가방을 traffic light로, 가까운 가방을 umbrella / person으로 잘못 분류하는 장면이 있다. 잘라내지 않았다.
- **기존 ⑤ 클립(`semih_perception_arm.mp4`)은 2026-11-16의 옛 팔(끝 링크 0.15m)로 찍은 것이라 물체에 닿지 않는다.** 최대 도달 거리에서 멈춘다. 원인은 처음에 적은 "목표가 멀어서"가 아니라 팔 설계였다 — 원래 팔은 관절 한계 때문에 팔 끝이 z 0.125m 아래로 못 내려가서, z 0.10m인 가방 목표점에는 거리와 상관없이 닿지 않았다(`semiH_troubleshooting_log.md` 14절 정정).
- **2026-11-17 이후 코드(끝 링크 0.21m + `target_approach_node`)의 반복 시험: 10회 중 9회 성공.** 아래 조건을 함께 적는다.
  - 성공 기준(시험 전에 정함): 탐지 고정 → 도착 → 팔 명령이 clamp 없이 끝나고, 팔 끝이 **깊이로 잰 가방 표면점** 3cm 이내.
  - 같은 월드, 같은 시작 위치, 회차마다 Gazebo를 새로 띄움(headless). 성공 9회의 팔 끝–표면점 거리 2.0~2.1cm, 오도메트리 오차(Gazebo 실제 위치 대비) 1.7~1.8cm.
  - 실패 1회(5회차): 측정된 표면점 높이가 z 0.084로 다른 회차(0.092~0.095)보다 낮아 어깨 관절이 한계(90°)에 걸려 IK가 clamp를 냈다. 팔 끝 자체는 1.9cm였지만 기준대로 실패로 셌다. 이 높이에서 작업영역 여유가 얇다는 뜻이다.
  - **접촉 센서로 확인하지 않았다.** "닿았다"가 아니라 "표면 앞 2cm까지 도달"로 쓴다. 측정 기준점도 실제 표면이 아니라 뎁스로 잰 점이다.
  - **YOLO 신뢰도 임계값 0.2 조건**이다(이 가방은 backpack 0.21~0.26으로만 검출됨). 코드 기본값은 0.4 그대로다.
  - 시뮬레이션이 결정적이라 10회 결과가 거의 같다(저장 좌표 차이 2mm 이내). 다양한 위치·물체에 대한 일반화가 아니다.
  - 어느 경우든 "물체를 집는다"고 쓰지 않는다(파지 기능 없음).
- **3D 좌표 정확도:** 뎁스로 얻은 점과 물체 원점 사이 수평 거리는 의자 0.24~0.28m, 가방 0.14~0.19m다(물체 표면 기준이라 생기는 차이). 정밀도를 내세우지 않는다.
- **편집 원칙:** 각 클립은 녹화 한 번에서 연속된 한 구간만 잘랐다. 여러 시도를 이어 붙이지 않았다. 예외: `semih_dashboard_demo.mp4`는 녹화 한 번에서 장면 6개를 잘라 이었다(장면 사이 컷, 라벨에 번호 표기).

## 4. 쓰지 말아야 할 표현

- "실제 로봇에서 동작" / "실환경 검증" — 시뮬레이션만 했다.
- "자율 탐색(explore)" — 탐색은 시도하지 않았다.
- "인식한 물체를 집는다" / "모바일 매니퓰레이션 완성" — 파지(grasp)는 구현하지 않았다. 기존 ⑤ 클립은 도달하지 못했고, 새 코드도 도달(reach)까지만 한다.
- "물체에 닿았다" — 접촉 센서로 확인하지 않았다. "표면 앞 N cm까지 도달"로 쓴다.
- "성공률 90%" 단독 — "시뮬레이션, 같은 조건 10회, 신뢰도 임계값 0.2" 조건 없이 쓰지 않는다.
- "대시보드에서 전체 중지" — 그런 버튼은 없다. 환경 종료와 노드별 종료가 따로다.
- "중복 실행을 모두 막는다" — 막는 것은 환경(bringup)과 HDF5 수집이다. 인식 노드는 여러 개 실행을 허용한다.
- "IK 오차 0" 단독 표기 — 시뮬레이션에서 TF로 잰 값(0.0cm)임을 함께 적는다.
