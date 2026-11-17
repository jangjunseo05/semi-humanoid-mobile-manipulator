# 포트폴리오 영상 가이드 — 세미휴머노이드 시뮬레이션 (STEP 3)

이 저장소의 시뮬레이션 데모 영상을 포트폴리오에 넣을 때 **어떤 파일을, 어디에, 어떤 문구로** 넣는지 정리한 문서다.
영상은 2026-11-16에 Gazebo Fortress + ROS 2 Humble(WSL2, RTX 4070 Laptop)에서 녹화했다.
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
| `semih_perception_arm.mp4` (+`.png`) | 14.0s | 2× | (선택) 장면 ⑤ 인식→팔 연결 — 대표 영상에는 넣지 않음 |

- 원본 녹화본은 `포트폴리오\media_raw\`에 있다. 포트폴리오에는 올리지 않는다.
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

## 3. 함께 적어야 하는 사실 (빼면 과장이 되는 것)

- **시뮬레이션 영상이다.** 실물 로봇 영상이 아니다. 캡션이나 박스 제목에 "Gazebo 시뮬레이션"을 밝힌다.
- **배속:** 2×로 편집한 클립은 캡션에 2배속을 적는다(영상 라벨에도 표기돼 있음).
- **스크립트로 준 입력:** 사람이 실시간으로 조종한 것이 아니다.
  - SLAM 주행: 미리 정한 웨이포인트를 odom 기반 P제어로 `/cmd_vel`에 보냈다(키보드 조작 대체).
  - Nav2: 목표 4개를 `navigate_to_pose` 액션으로 보냈다(4개 모두 SUCCEEDED, 클립에는 앞의 2개).
  - 팔 IK: `/target_point`에 목표 6개를 4초 간격으로 보냈다.
  - YOLO / 인식→팔: 로봇 이동만 스크립트, 검출·3D 좌표·팔 목표는 노드가 실제로 낸 값이다.
- **오검출이 영상에 남아 있다.** 여행가방을 traffic light로, 가까운 가방을 umbrella / person으로 잘못 분류하는 장면이 있다. 잘라내지 않았다.
- **인식→팔(⑤)은 물체에 닿지 않는다.** 목표가 팔 작업영역 밖이라 최대 도달 거리에서 멈춘다. "물체를 집는다"고 쓰지 않는다.
- **3D 좌표 정확도:** 뎁스로 얻은 점과 물체 원점 사이 수평 거리는 의자 0.24~0.28m, 가방 0.14~0.19m다(물체 표면 기준이라 생기는 차이). 정밀도를 내세우지 않는다.
- **편집 원칙:** 각 클립은 녹화 한 번에서 연속된 한 구간만 잘랐다. 여러 시도를 이어 붙이지 않았다.

## 4. 쓰지 말아야 할 표현

- "실제 로봇에서 동작" / "실환경 검증" — 시뮬레이션만 했다.
- "자율 탐색(explore)" — 탐색은 시도하지 않았다.
- "인식한 물체를 집는다" / "모바일 매니퓰레이션 완성" — ⑤는 도달하지 못했다.
- "IK 오차 0" 단독 표기 — 시뮬레이션에서 TF로 잰 값(0.0cm)임을 함께 적는다.
