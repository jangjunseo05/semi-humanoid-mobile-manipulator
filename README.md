<p align="center">
  <img src="logo.png" alt="semiH_web logo" width="120">
</p>

<h1 align="center">semiH_web</h1>

<p align="center">
  ROS 2 기반 세미 휴머노이드 모바일 매니퓰레이터(<code>semiH_ws</code>) 운영을 위한<br>
  Streamlit 단일 페이지 제어/모니터링 대시보드
</p>

---

## Overview

`semiH_web`은 별도의 ROS 2 워크스페이스(`semiH_ws`)로 존재하는 모바일 매니퓰레이터
스택 — Gazebo 시뮬레이션, SLAM/Nav2 자율주행, YOLO 기반 인식, 뎁스 카메라 3D 좌표
추출, IK 기반 팔 제어, HDF5 학습 데이터 수집 — 을 터미널 명령 없이 웹 UI에서
시작/종료/모니터링할 수 있게 만든 운영 레이어입니다. `semiH_ws` 내부 파일은
읽기 전용으로만 참조하며 절대 수정하지 않는다는 원칙 아래, `ros2 launch`/`ros2 run`
subprocess를 바깥에서 감싸는 방식으로 동작합니다.

디스크에 상태를 영속화하는 lock 파일, 프로세스 그룹 단위(`os.killpg`) 종료,
자원 충돌 감지, 좀비 프로세스 처리 등은 실제 라이브 환경(WSL2 + ROS 2 Humble)에서
재현·검증한 결과를 바탕으로 구현했습니다.

## Key Components

### `core/process_manager.py` — Job/Lock 관리 엔진
- `state/locks/*.json` 파일 기반으로 실행 상태를 영속화(Streamlit 세션 상태가
  아닌 디스크 기준이라 페이지 새로고침/서버 재시작에도 상태 유지)
- `O_CREAT | O_EXCL` 원자적 파일 생성으로 동시 시작 요청(버튼 더블클릭 등) 레이스 방지
- 모든 job을 `start_new_session=True`로 새 프로세스 그룹의 leader로 실행하고,
  종료 시 `os.killpg`로 그룹 전체에 SIGINT → (timeout 시) SIGKILL 에스컬레이션
  — 셸 래퍼(`ign gazebo`)를 통해 기동되는 손자 프로세스가 단일 PID 종료로는
  고아로 남는 문제를 라이브로 재현 후 해결
- stale lock 판단을 timeout이 아닌 `psutil`/`os.waitpid` 기반 PID 생존 확인으로 처리
  (좀비 프로세스 오판 방지 포함)
- stage별 점유 자원(`STAGE_RESOURCES`) 테이블로 자원 충돌 감지

### `core/launch_runner.py` — Bringup / RViz / 인식 결과 뷰어
- `ros2 launch semih_description bringup.launch.py`를 `sim` / `slam` / `nav` /
  `explore` 4단계 누적 모드로 감싸며, 원본 launch 파일에 없는 모드 값 검증을
  Enum으로 웹 레이어에서 강제(원본은 잘못된 값이 조용히 `sim`으로 폴백되는 버그 존재)
- Nav2 목표 지점을 지정할 UI가 없는 문제를 해결하기 위해 RViz2를 독립적으로
  띄우는 버튼 추가, YOLO 인식 결과(`/yolo/image_annotated`)를 바로 볼 수 있는
  `rqt_image_view` 뷰어 버튼 추가
- WSLg GPU 가속 미지원 환경을 위한 소프트웨어 렌더링 환경변수 주입(`gui_env`)

### `core/node_runner.py` — 독립 인식/수집 노드 실행
- launch 파일이 없는 4개 `ros2 run` 노드(`yolo_detection_node`, `target_3d_node`,
  `arm_reach_node`, `hdf5_collector_node`)를 개별 시작/종료
- 라이브 테스트로 노드별 락 정책을 다르게 적용: `hdf5_collector`는 파일명 충돌로
  인한 데이터 유실이 실측 확인되어 단일 인스턴스 하드 락, 나머지 3개는 중복
  실행이 안전함을 확인하여 인스턴스별 고유 키로 복수 실행 허용

### `monitoring/topic_watcher.py` — 상태 집계
- `launch_runner`/`node_runner`의 상태를 하나의 dict로 통합해 UI가 쓰기 좋게 제공
- 프로세스 생존 여부와 실제 토픽 발행 여부(ROS 그래프 수준 헬스체크)를 구분하며,
  후자는 범위를 명시한 TODO로 남겨둠

### `pages/main.py` / `lib/ui_components.py` — Streamlit UI
- 환경(bringup) 카드, 인식 파이프라인(YOLO → 3D 좌표 추출 → 팔 뻗기) 카드,
  데이터 수집 카드로 구성된 단일 페이지 대시보드
- 현재 시스템 상태를 해석해 다음에 눌러야 할 동작을 안내하는 동적 CTA
- 역할 기반 카드 색상(action/settings/success/warning), 모드/노드별 설명 popover,
  최초 사용자용 가이드 탭, 브랜드 스플래시 화면

### `scripts/upgrade_mesa_wslg.sh`
- WSL2(WSLg) 환경에서 Gazebo/RViz GUI 창이 뜨지 않는 문제의 원인(GPU-PV 가상화
  채널 불능)을 진단하는 스크립트

## Tech Stack

- **Web/App**: Python, Streamlit, streamlit-autorefresh, PyYAML, psutil, Pillow
- **Robotics**: ROS 2 Humble, Gazebo, slam_toolbox, Nav2, explore_lite, RViz2, rqt_image_view
- **Perception**: YOLOv8(Ultralytics) 기반 객체 탐지, 뎁스 카메라 3D 좌표 변환
- **Data**: HDF5(h5py) 기반 센서/관절 상태 동기 수집
- **Runtime**: WSL2(Ubuntu 22.04) 위에서 `subprocess` 기반 프로세스 그룹 관리

---

> 이 저장소는 `semiH_ws`(로봇 URDF 모델링, Gazebo 월드, 인식/제어 노드 구현,
> launch 파일 등 실제 로봇 스택)를 수정하지 않고 바깥에서 운영/모니터링하는
> 웹 레이어만을 포함합니다.
