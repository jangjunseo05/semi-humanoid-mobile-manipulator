# semiH_ws — 프로젝트 메모

이 파일은 지금까지 없었고, 이번 진단 세션에서 처음 생성했습니다. 환경 구축 단계 히스토리는
`semiH_troubleshooting_log.md`(16개 항목), 이후 개선 세션은 `docs/troubleshooting_p1-p4.md`,
`docs/senior_review_notes.md`에 이미 정리되어 있으니 함께 참고하세요. 아래 섹션은 그 문서들과
번호 체계가 다른 새 문서이므로 "## 1."부터 시작합니다.

---

## 1. 자율주행 정지 및 HDF5 0 frames 진단 리포트 (2026-08-11)

**조사 방법**: 코드/설정 정적 분석 + `mode:=explore headless:=true`로 실제 시뮬레이션을 라이브 기동해
`tf2_echo`, `ros2 topic hz`, `ros2 topic info -v`, `ros2 topic echo /behavior_tree_log`, 실제 launch
로그, `top` 실측, 공식 문서(nav2 docs, ros_gz README, ros2/rclcpp·navigation2 GitHub 이슈) 대조로 검증.
이번 세션에서는 코드/설정을 전혀 수정하지 않았고, 조사 중 뜬 프로세스는 세션 종료 시 정리만 함.

### A. 자율주행 정지 문제

#### A-1. TF(map→base_link) 갱신이 끊기는 시점이 있는가— **[배제됨 / 조건부]**
`ros2 run tf2_ros tf2_echo map base_link`를 10초간 라이브 실행한 결과, sim time 23.5s~27.0s 구간
동안 TF는 끊김 없이 계속 갱신됐다(약 0.5~1.1s 간격, 정상):
```
At time 23.528000000 ... Translation: [0.261, 0.000, 0.000]
At time 24.684000000 ... Translation: [0.261, 0.000, 0.000]
At time 25.262000000 ... Translation: [0.261, 0.000, 0.000]
At time 25.874000000 ... Translation: [0.261, 0.000, 0.000]
At time 26.418000000 ... Translation: [0.261, 0.000, 0.000]
At time 26.996000000 ... Translation: [0.261, 0.000, 0.000]
```
다만 **이 값이 계속 [0.261, 0, 0]으로 고정된 것 자체가 A-5의 결과**다 — TF 파이프라인(전송 계층)은
안 끊기지만, 로봇이 실제로 이동하지 않기 때문에 좌표가 안 바뀐 것뿐이다. "주행 중 TF가 끊긴다"는
가설은 이번 관측 구간에서는 배제되지만, 애초에 A-5의 원인 때문에 "주행"이 성립하지 않아 완전한
검증(수 미터 주행 중 TF 드롭 여부)은 하지 못했다 — 추가조사 필요 항목으로 남김.

#### A-2. nav2_params.yaml costmap 설정이 nav2 공식 문서와 어긋나는가 — **[배제됨]**
`src/semih_description/config/nav2_params.yaml`의 구조:
- `local_costmap.plugins: [voxel_layer, inflation_layer]`
- `global_costmap.plugins: [static_layer, obstacle_layer, inflation_layer]`

nav2 공식 문서([Costmap 2D — Nav2 docs](https://docs.nav2.org/configuration/packages/configuring-costmaps.html))의
표준 패턴(global=static+inflation 또는 +obstacle, local=obstacle/voxel+inflation)과 일치한다.
`inflation_radius: 0.25`, `cost_scaling_factor: 3.0`은 `docs/troubleshooting_p1-p4.md` 12번 항목에서
실측 후 조정된 값 그대로 남아 있음을 확인. `observation_sources`에 `sensor_frame`을 명시하지 않았는데,
이는 nav2 기본 동작(메시지의 `frame_id`를 그대로 사용)과 일치해 문제가 아니다. **설정 자체는 표준
패턴에서 벗어나지 않음** — 문제는 설정값이 아니라 A-5(코스트맵이 아예 갱신되지 않음)에 있다.

#### A-3. lidar/depth camera 실측 publish rate vs 설계 스펙 — **[확인됨: 스펙 대비 30~45% 저하, 그러나 정지의 직접 원인은 아님]**
`semih.urdf.xacro`의 스펙과 라이브 실측(`ros2 topic hz`, headless + nav2 + slam 전체 스택 구동 중):

| 센서 | 스펙(xacro `update_rate`) | 실측 평균 | 스펙 대비 |
|---|---|---|---|
| lidar(`/scan`) | 10 Hz | 6.81~6.93 Hz | 약 -32% |
| depth_camera | 15 Hz | 8.5~9.2 Hz | 약 -40% |
| camera(`/camera/image_raw`) | 30 Hz | 15.8~18.5 Hz | 약 -45% |
| joint_states | (xacro에 `update_rate: 30` 명시) | **734~757 Hz** | 스펙 무관, 태그 자체가 무시됨 |

`joint_states`는 xacro 주석(283~286행)에 "`update_rate:30`으로 바운딩해서 878Hz 플러딩 문제를
고쳤다"고 적혀 있으나, 실측 결과 **이 태그는 여전히 무시되고 있다** (734~757Hz는 `semiH_troubleshooting_log.md`
#16과 `hdf5_collector_node.py` 자체 주석의 "Fortress(ign-gazebo6)는 `<update_rate>`를 무시한다"는
서술과 일치, xacro 주석이 최신 상태를 반영 못 하는 오기록). 실제 완화책은 `hdf5_collector_node.py`가
joint_states를 동기화 대상에서 빼고 최신값 캐싱하는 것이며 이는 태그 동작 여부와 무관하게 유효함.

CPU 근거: 이 실측 시점 `top` 스냅샷에서 `ign gazebo` 서버(표시상 프로세스명 `ruby` — Ignition의 `ign`
CLI 자체가 Ruby로 구현된 디스패처라 top/ps에는 `ruby`로 나타남)가 **426~448% CPU**, load average
9.05였다. headless인데도 카메라+뎁스카메라+라이다(gpu_lidar) 3개 센서의 ogre2 렌더링이 모두
서버 프로세스 내부에서 CPU(llvmpipe 소프트웨어 렌더링)로 처리되기 때문으로, `docs/troubleshooting_p1-p4.md`
2·3번 항목에 이미 기록된 것과 같은 종류의 병목이 오늘도 재현됨. 다만 이 저하율만으로는 "정지"를
설명하지 못한다 — 아래 A-5가 훨씬 결정적이며, HDF5 수집(B번 문제)에도 이 CPU 부하 자체는 실측상
악영향을 주지 않았다(B-5 참고).

#### A-4. 정지 시점 behavior_tree_log / recovery 트리거 여부 — **[확인됨: BT가 무의미하게 초고속 반복]**
`/behavior_tree_log`를 10초간 라이브 캡처한 결과:
```
FollowPath: RUNNING -> SUCCESS -> IDLE
NavigateWithReplanning: RUNNING -> SUCCESS -> IDLE
NavigateRecovery: RUNNING -> SUCCESS -> IDLE
(nanosec 190308641 -> 190317536, 약 9μs 만에 전체 사이클 종료)
... 이후 곧바로 NavigateRecovery IDLE -> RUNNING, ComputePathToPose RUNNING 재시작
```
같은 관측 구간 `/cmd_vel`은 계속 `linear.x=0, angular.z=0`였다(로봇은 물리적으로 정지 상태).
즉 BT는 "정지해서 멈춰있는" 게 아니라 **네비게이션 사이클을 극도로 빠르게(수 마이크로초 단위)
성공/재시도로 처리하면서 실제로는 아무 이동도 만들어내지 못하는 상태**다. 이는 controller_server가
매 사이클 유효한 지역 코스트맵을 못 받아 즉시 실패 → replanning → 즉시 재실패를 반복하는 패턴과
정확히 일치한다(A-5).

#### A-5. use_sim_time이 모든 관련 노드에 일관되게 설정돼 있는가 — **[확인됨: 가장 유력한 근본 원인]**
`grep -rn use_sim_time`으로 전체 launch 파일을 확인한 결과, robot_state_publisher/slam_toolbox
(slam.launch.py)/nav2 전체 서버(nav2.launch.py→navigation_launch.py)/explore_lite(explore.launch.py)
모두 `use_sim_time: true`로 **일관되게** 설정돼 있었다(이 부분 자체는 문제 없음). 그런데 워크스페이스
전체(src/ 하위 전부, m-explore-ros2 포함)를 `grep -rni clock`으로 뒤져도 **`/clock` 토픽을 실제로
브릿지하는 `ros_gz_bridge` 설정이 단 한 곳도 없었다** (`spawn_gazebo.launch.py`의 6개 `parameter_bridge`
노드 목록에 `/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock`이 빠져 있음). `ros_gz_sim`의
`gz_sim.launch.py`도 자체적으로 clock을 브릿지하지 않음(패키지 launch 파일에 clock 관련 코드 없음).

라이브로 직접 확인:
```
$ ros2 topic info /clock -v
Type: rosgraph_msgs/msg/Clock
Publisher count: 0
Subscription count: 15   # robot_state_publisher, controller_manager, slam_toolbox,
                          # controller_server, planner_server, behavior_server, bt_navigator,
                          # local_costmap, global_costmap, lifecycle_manager_navigation 등

$ timeout 6 ros2 topic hz /clock
(6초간 단 1개의 메시지도 없이 타임아웃)
```
공식적으로 알려진 동작이다: `use_sim_time=true`인데 `/clock` 퍼블리셔가 없으면 해당 노드의
`now()`는 0에서 멈춘다 ([ros2/rclcpp#940 "Sim time always zero"](https://github.com/ros2/rclcpp/issues/940)).
그리고 nav2 costmap의 내부 갱신 루프(`Costmap2DROS::mapUpdateLoop`)는 시뮬레이션 시간 기준
타이머로 동작하도록 설계돼 있다 ([ros-navigation/navigation2#3325](https://github.com/ros-navigation/navigation2/issues/3325)).
`/clock`이 없으면 이 루프가 다시 돌지 않는다는 뜻이다. 실측으로 정확히 이 현상이 재현됐다:

```
$ timeout 8 ros2 topic hz /local_costmap/costmap     → 8초간 0개 메시지
$ timeout 8 ros2 topic hz /global_costmap/costmap    → 8초간 0개 메시지
$ timeout 6 ros2 topic hz /local_costmap/costmap_raw → 6초간 0개 메시지
$ timeout 6 ros2 topic hz /global_costmap/costmap_raw→ 6초간 0개 메시지
```

launch 로그에서도 같은 이야기가 나온다. 세션 시작 t+20~29초 구간에만
`Robot is out of bounds of the costmap! Sensor origin at (-0.00, -0.00) is out of map bounds ...`
경고가 10회 찍히고, **그 이후로는 세션이 6분 넘게 지속되는 동안 이 경고가 단 한 번도 다시 뜨지
않았다** — 로그를 봤을 때는 "문제가 해결된 것"처럼 보이지만, 같은 시간 동안 `/odom`을 찍어보면
로봇은 `x=0.2612`(bringup.launch.py의 20초 priming nudge로 이동한 위치)에서 완전히 멈춰 있었다.
즉 코스트맵이 "로봇이 이동해서 경고가 사라진 것"이 아니라 **update 루프 자체가 통째로 멈춰서
경고조차 다시 못 내는 상태**로 굳은 것이다.

**인과 사슬 정리**: `/clock` 브릿지 누락 → use_sim_time 노드들의 ROS 시계가 0에 고정 →
costmap 갱신 루프(시뮬레이션 시간 타이머 기반)가 최초 진입 이후 재실행되지 않음 → local/global
costmap이 로봇의 실제 이동을 전혀 반영하지 못함 → controller_server/planner_server가 유효한
지역 코스트맵을 못 얻어 유의미한 경로/속도를 생성 못 함 → `/cmd_vel`이 관측 구간 내내 0 →
로봇이 물리적으로 전혀 움직이지 못함(explore_lite 기동 후 6분 넘게 정지). `docs/troubleshooting_p1-p4.md`
12번 항목에서 "`/global_costmap/costmap`이 원인 불명으로 메시지를 한 번도 안 준다"고 미해결로
남겼던 문제, 그리고 `docs/troubleshooting_p1-p4.md` 16번 항목의 "간헐적으로 멈춤, 근본 원인
못 잡음"도 이 `/clock` 미브릿지로 상당 부분 설명 가능하다(단, 이전 세션들에서 이 정도로 완전한
정지가 아니라 "간헐적" 정지로 보고된 점은 완전히 일치하진 않으며, 세션마다 우연히 초기 몇 틱이
더 돌았는지에 따라 체감 증상이 달라졌을 가능성으로 추정 — 단정은 아님).

**가장 유력한 원인**: `spawn_gazebo.launch.py`의 `ros_gz_bridge` 목록에 `/clock` 브릿지가 빠져
있어, `use_sim_time:=true`로 설정된 Nav2 costmap의 갱신 루프가 사실상 영구 정지 상태다.

---

### B. HDF5 0 frames 문제

#### B-1. subscriber 콜백이 실제로 타는가 — **[배제됨 — 현재 코드에서 정상 작동]**
`hdf5_collector_node`를 A번 진단용 explore 스택이 떠 있는 상태에서 그대로 라이브 실행(`duration_sec:=15.0`):
```
[INFO] [hdf5_collector_node]: Collecting synchronized data for 15.0s from [...] ...
[INFO] [hdf5_collector_node]: Collected 10 synchronized frames so far...
...
[INFO] [hdf5_collector_node]: Collected 90 synchronized frames so far...
[INFO] [hdf5_collector_node]: Duration reached (15.0s). Saving and shutting down.
[INFO] [hdf5_collector_node]: Saved 95 synchronized frames to: /home/jangjunseo/semih_datasets/semih_session_20260811_113425.h5
```
콜백이 정상적으로 타고 10프레임 단위 로그가 그대로 찍힘. **"0 frames"는 오늘 세션에서 재현되지 않았다.**

#### B-2. message_filters sync / slop vs 실제 타임스탬프 차 — **[배제됨 — 정상]**
저장된 파일을 `verify_dataset.py`로 검증:
```
Actual synchronized frames stored: 95
Sync frame interval: mean=0.0987s std=0.0213s (min 0.032 / max 0.166)
camera drift: mean=0.0ms max=0.0ms
lidar   drift: mean=22.6ms max=84.0ms
joint_states drift: mean=33.5ms max=96.0ms
odom    drift: mean=18.5ms max=86.0ms
No suspiciously large gaps detected (all intervals within 2x average).
```
`sync_slop=0.1s` 기준 모든 drift가 그 안에 들어옴. `docs/troubleshooting_p1-p4.md` #6-2에서
확정한 "joint_states(850Hz대)를 동기화 대상에서 빼고 최신값 캐싱"이 현재 코드에 그대로 반영돼
있고(`hdf5_collector_node.py` 85~88행), 오늘 실측으로도 joint_states가 여전히 734~757Hz로
발행됨(A-3)에도 문제없이 동작함 — 이 우회책이 원인 재발과 무관하게 견고함을 재확인.

#### B-3. HDF5 write가 콜백마다 즉시 쓰는지 / 종료 시 버퍼링, SIGINT 처리 — **[배제됨 — 설계대로 동작]**
코드 확인 결과 수집 중에는 Python 리스트에만 append(`self.images.append` 등)하고, 실제 `h5py.File`
쓰기는 `save_and_shutdown()` 한 곳에서만 발생한다. 호출 경로는 두 가지: ①`duration_sec` 타이머
콜백(`check_duration`)에서 시간 초과 시, ②`main()`의 `except KeyboardInterrupt`에서 Ctrl-C 시.
데이터셋별로 개별 `try/except`(카메라 이미지, joint_states 각각)로 감싸 하나가 실패해도 나머지는
저장됨(`docs/senior_review_notes.md` 4번 원칙과 일치). 라이브 실행에서 저장까지 정상 확인됨.

#### B-4. executor 종류 / HDF5 write가 콜백 스레드를 블로킹하는가 — **[배제됨 — 블로킹은 있으나 무해]**
`main()`은 `rclpy.spin(node)`만 호출 → 기본 `SingleThreadedExecutor`. 라이브 로그의 타임스탬프로
실측한 쓰기 소요시간: `Duration reached` 1786415665.296873521 → `Saved` 1786415666.133249652,
약 **837ms**(gzip 압축 포함 95프레임). 이 시점엔 이미 `self.timer.cancel()`이 먼저 호출돼 새
메시지를 받을 필요가 없는 상태이므로, 단일 스레드가 037ms 블로킹돼도 실질적 손실이 없다. 설계상
의도된 동작이며 문제 없음.

#### B-5. CPU 실측 / WSL2 리소스 제한(.wslconfig) — **[배제됨 — 현재 코드에서 병목 아님]**
Windows 쪽 `.wslconfig`(`/mnt/c/Users/USER/.wslconfig`)는 **존재하지 않음** → WSL2가 기본값(호스트
RAM의 절반, 전체 논리 코어)으로 동작 중, 인위적 제한 없음. A-3에서 측정한 대로 `ign gazebo`가
426~448% CPU, load average 9.05인 상황에서도 `hdf5_collector_node`는 15초에 95프레임을 문제없이
수집했다(B-1, B-2 참고). `semiH_troubleshooting_log.md` #16이 "CPU 과부하"로 추정했던 최초 진단은
`docs/troubleshooting_p1-p4.md` #6에서 이미 "CPU와 거의 무관한 3가지 독립 버그였다"로 정정됐고,
오늘 실측(무거운 explore 스택 + 라이다/뎁스카메라 렌더링 동시 구동 중에도 정상 수집)이 그 정정을
다시 한번 뒷받침한다.

**가장 유력한 원인**: 없음 — **현재 코드베이스에서 "HDF5 0 frames"는 재현되지 않았다.**
`docs/troubleshooting_p1-p4.md` #6에서 규명·수정한 3가지 근본 원인(odom gz 토픽명 불일치,
joint_states 850Hz대 동기화 큐 고갈, `/joint_states` 이중 발행자로 인한 배열 길이 불일치)이
현재 `hdf5_collector_node.py`·`joint_state_bridge.yaml`·`semih.urdf.xacro`에 모두 반영돼 있고,
오늘 nav2/explore 풀스택이 동시에 도는 무거운 조건에서 재검증까지 마쳤다. 만약 실제로 "0 frames"가
재발했다면 (a) 이 워크스페이스가 아닌 다른 체크아웃/브랜치에서 실행했거나, (b) 이번 조사 이전
시점(수정 전 코드)을 가리키는 보고이거나, (c) 오늘 재현 못 한 별도 트리거(장시간 수집 시 누적
메모리, 카메라 해상도 변경 등)일 가능성이 있다 — 아래 다음 단계 참고.

---

### 다음 단계 제안 (수정은 하지 않음, 제안만)

**A(자율주행 정지) 관련**
1. `spawn_gazebo.launch.py`의 `parameter_bridge` 목록에 `/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock`
   브릿지 노드를 추가하는 것을 최우선으로 검토. 추가 후 동일한 라이브 절차(`ros2 topic hz /clock`,
   `/local_costmap/costmap` hz, 6분 이상 explore 세션에서 odom 위치 변화)로 반드시 재검증할 것.
2. `semih.urdf.xacro` 283~286행의 "`update_rate:30`이 878Hz 플러딩을 고쳤다"는 주석은 실측과
   어긋나므로(오늘도 734~757Hz 실측) 오해를 막기 위해 정정 필요 — 실제 완화책은
   `hdf5_collector_node.py`의 캐싱 우회임을 명시.
3. A-1에서 못 다한 검증(로봇이 실제로 수 미터 이동하는 동안 TF 드롭 여부)은 /clock 수정 후,
   진짜 주행이 가능해진 다음에 재시도.
4. 센서 실측 publish rate가 스펙 대비 30~45% 낮은 문제(A-3)는 별도로 GPU 패스스루
   (`semiH_troubleshooting_log.md` P1 #2, `/dev/dri` 부재) 재조사 여지가 있음 — 우선순위는 낮음.

**B(HDF5 0 frames) 관련**
1. 사용자가 보고한 "0 frames" 재현 조건(정확한 실행 명령, 브랜치/커밋, 실행 시간)을 먼저 확인해
   이번 조사가 같은 상황을 재현한 게 맞는지 교차 확인.
2. 재현된다면 장시간(수 분 이상) 수집 시나리오로 다시 테스트 — 오늘은 15초만 검증했으므로 메모리
   누적이나 장시간 드리프트 확대 가능성은 배제하지 못함.
3. A번 수정(/clock 브릿지) 이후 `use_sim_time`이 걸린 다른 노드들의 시계가 정상화되면, 현재
   `use_sim_time`을 선언하지 않은 `hdf5_collector_node`/perception 노드들과의 시간 기준 불일치
   (한쪽은 sim time, 한쪽은 system time)가 새로운 문제로 드러날 수 있으니 함께 점검.

---

## 2. launch 구조 인벤토리 (2026-08-17)

**목적**: `semiH_web`(웹 개발 키트, 아직 미생성) 설계를 위해 기존 `.launch.py` 파일들이 SLAM/Nav2/
YOLO/IK/HDF5/explore를 어떤 단위로 나누고 있는지 정확히 기록. 이번 조사에서는 **코드를 전혀 수정하지
않았고**, 파일 내용과 `docs/usage_guide.md`에 문서화된 실제 사용법만 근거로 작성. 애매하거나 소스로
확인 못 한 부분은 "확인 안 됨"으로 남김.

### 2-1. 전체 `.launch.py` 파일 목록 (경로 포함)

```
src/semih_description/launch/bringup.launch.py
src/semih_description/launch/spawn_gazebo.launch.py
src/semih_description/launch/slam.launch.py
src/semih_description/launch/nav2.launch.py
src/semih_description/launch/explore.launch.py
src/semih_description/launch/display.launch.py
src/m-explore-ros2/explore/launch/explore.launch.py        # 벤더링된 explore_lite 자체 launch, 이 프로젝트는 안 씀
src/m-explore-ros2/map_merge/launch/map_merge.launch.py    # 멀티로봇용, 이 프로젝트 launch 체인에서 미사용
src/m-explore-ros2/map_merge/launch/from_map_server.launch.py  # 위와 동일, 미사용
```

`semih_perception`, `semih_data_collection` 두 패키지에는 **launch/ 디렉터리 자체가 없음**
(`find`로 확인됨, `setup.py`의 `data_files`에도 launch 관련 항목 없음) — YOLO/3D좌표추출/IK/HDF5는
전부 `ros2 run <pkg> <executable>`로만 실행되는 순수 노드이며, 이 워크스페이스 어디에도 이들을 감싸는
launch 파일이 없다.

### 2-2. launch 파일 include 트리

```
bringup.launch.py  (인자: mode=sim|slam|nav|explore [기본 sim], headless [기본 true])
├── spawn_gazebo.launch.py                         [조건 없음 — 모든 mode에서 항상 include]
│   └── (ros_gz_sim 패키지) launch/gz_sim.launch.py   [외부 패키지 include]
├── slam.launch.py                                 [조건: mode in (slam, nav, explore)]
│   └── (slam_toolbox 패키지) launch/online_async_launch.py   [외부 패키지 include]
├── nav2.launch.py                                 [조건: mode in (nav, explore)]
│   └── (nav2_bringup 패키지) launch/navigation_launch.py   [외부 패키지 include]
├── priming_spin                                   [조건: mode==explore, TimerAction period=20.0s]
│   └── ExecuteProcess(`bash -c 'ros2 topic pub ...'`)  — launch include 아님, bash로 직접 cmd_vel 퍼블리시
└── explore.launch.py (TimerAction으로 감쌈, period=33.0s)  [조건: mode==explore]
    └── (직접 include 없음) explore_lite 패키지의 `explore` 실행파일을 Node로 직접 기동
        — explore_lite 자체 launch/explore.launch.py는 의도적으로 안 씀 (아래 2-3 참고)

display.launch.py   — 다른 launch를 전혀 include하지 않는 독립 파일 (robot_state_publisher +
                       joint_state_publisher_gui + rviz2로 Gazebo 없이 URDF만 확인하는 용도)
```

### 2-3. launch 파일별 상세 (노드 / yaml / include / 인자)

| launch 파일 | 실행 노드 (패키지::실행파일) | 로드 yaml (기본값 → override 인자) | include하는 launch | 선언된 인자 |
|---|---|---|---|---|
| `bringup.launch.py` | 없음(직접 Node 없음, 전부 하위 launch에 위임) + `priming_spin`은 Node가 아니라 `ExecuteProcess` | 없음 | spawn_gazebo(항상), slam.launch.py(조건부), nav2.launch.py(조건부), explore.launch.py(조건부+33s 지연) | `mode`(기본 `sim`), `headless`(기본 `true`) |
| `spawn_gazebo.launch.py` | `robot_state_publisher::robot_state_publisher`, `ros_gz_sim::create`(엔티티 스폰), `ros_gz_bridge::parameter_bridge` × 6(개별 Node: cmd_vel, clock, camera/image_raw, camera/camera_info, scan, depth_camera) + 1개(`config_file`로 joint_state_bridge.yaml 로드 — joint_states/odom/tf 3개 토픽), `tf2_ros::static_transform_publisher`(lidar_link→semih/base_link/lidar 고정 변환), `controller_manager::spawner` × 2(joint_state_broadcaster, arm_controller) | `config/joint_state_bridge.yaml` (인자로 override 불가 — 코드에 경로 고정) | (ros_gz_sim) `gz_sim.launch.py` | `headless`(기본 `true`) |
| `slam.launch.py` | 없음(직접 Node 없음) | `config/slam_params.yaml` (기본값 → `slam_params_file` 인자로 override 가능) | (slam_toolbox) `online_async_launch.py` | `slam_params_file` |
| `nav2.launch.py` | 없음(직접 Node 없음) | `config/nav2_params.yaml` (기본값 → `nav2_params_file` 인자로 override 가능) | (nav2_bringup) `navigation_launch.py` | `nav2_params_file` |
| `explore.launch.py` (semih_description) | `explore_lite::explore` (name=`explore_node`) | `config/explore_params.yaml` (기본값 → `explore_params_file` 인자로 override 가능) | 없음 | `explore_params_file` |
| `display.launch.py` | `robot_state_publisher::robot_state_publisher`, `joint_state_publisher_gui::joint_state_publisher_gui`, `rviz2::rviz2` | 없음 | 없음 | 없음 |

`nav2.launch.py`가 include하는 `navigation_launch.py`는 nav2_bringup의 "navigation-only" 진입점으로,
**amcl/map_server(localization_launch.py 계열)를 포함하지 않는다** — 소스에 amcl/map_server 관련
언급이 전혀 없고(grep 확인), 실제 라이브 실행 중 `ps aux`에서도 `amcl`/`map_server` 프로세스가 뜬
적이 없었다(이전 세션 A항목 진단 과정에서 반복 확인). 즉 map→odom TF는 전적으로 slam_toolbox가
공급하며, Nav2만 단독으로는 map 프레임을 만들 수 없다.

### 2-4. 기능 단위별 "독립 실행 가능 여부" (소스 기준)

| 기능 단위 | 자체 launch 파일 존재? | 독립 실행 가능? | 근거 / 제약 |
|---|---|---|---|
| SLAM (slam_toolbox) | O (`slam.launch.py`) | **가능** (파일 구조상 다른 launch를 include하지 않음) | `docs/usage_guide.md` 69행에 `ros2 launch semih_description slam.launch.py` 단독 실행 예시 명시. 단, 기능적으로 `/scan` + `odom→base_link` TF(Gazebo 등 소스)가 이미 있어야 의미 있게 동작 — 이건 launch 하드코딩이 아니라 topic/TF 런타임 의존성. |
| Nav2 bringup | O (`nav2.launch.py`) | **가능** (파일 구조상 다른 launch를 include하지 않음) | `docs/usage_guide.md` 70행에 단독 실행 예시 명시. 단, `navigation_launch.py`가 localization을 포함하지 않으므로(2-3 참고) slam_toolbox(또는 다른 map→odom TF 소스)가 같이 떠 있어야 실제 내비게이션이 성립 — 역시 코드 하드코딩이 아니라 기능적 전제조건. |
| YOLO 인식 노드 (`yolo_detection_node`) | **없음** | **완전 독립** | `semih_perception`에 launch/ 디렉터리 자체가 없음. `ros2 run semih_perception yolo_detection_node`로 단독 기동(`docs/usage_guide.md` 117행). `/camera/image_raw`가 없어도 노드는 기동되고 구독 대기만 함(콜백이 안 탈 뿐 크래시 안 함, `yolo_detection_node.py` 구조상 확인). |
| depth camera 3D 좌표 추출 + IK (`target_3d_node` + `arm_reach_node`) | **없음** | **완전 독립** (2개 노드 각각) | `semih_perception`에 launch/ 없음, 각각 `ros2 run`으로 개별 기동(`docs/usage_guide.md` 121, 124행). `arm_reach_node`는 `target_3d_node` 없이도 `/target_point`를 수동 publish하면 단독 동작(`docs/usage_guide.md` 127~134행에 방법 명시). 기능적으로 `arm_controller` 액션서버(`spawn_gazebo.launch.py`가 spawn)가 떠 있어야 실제 팔이 움직임. |
| HDF5 데이터 수집 노드 (`hdf5_collector_node`) | **없음** | **완전 독립** | `semih_data_collection`에 launch/ 없음, `ros2 run`으로 단독 기동(`docs/usage_guide.md` 141행). `arm_reach_node` 없이도 동작하지만 팔 관련 필드는 NaN으로 기록됨(`docs/usage_guide.md` 154행에 명시, 코드상 `hdf5_collector_node.py`의 개별 try/except 구조와 일치). |
| explore_lite | O (`semih_description/launch/explore.launch.py`, explore_lite 자체 launch는 의도적으로 미사용) | **가능하지만 조건부** | 파일 자체(`explore.launch.py`)는 다른 launch를 include하지 않아 단독 실행 가능(`docs/usage_guide.md` 96행: `ros2 launch semih_description explore.launch.py`). 다만 `bringup.launch.py`의 `mode:=explore` 경로에서는 그 앞에 하드코딩된 20초 지연(`priming_spin`, `ros2 topic pub`으로 로봇을 강제 전진)과 33초 지연(TimerAction)이 붙어 있음 — 이건 `explore.launch.py` 파일의 제약이 아니라 `bringup.launch.py`가 "완전자율모드"를 만들기 위해 추가한 조건부 시퀀싱. `explore.launch.py`를 단독으로 띄우면 이 프라이밍이 없으므로, `docs/usage_guide.md` 82~97행은 "사람이 먼저 조금 돌아다니며 지도를 넓힌 뒤" 단독 실행하라고 명시적으로 권장하고 있다. |

### 2-5. `/clock` 브릿지 및 이번 세션 수정 파일이 걸리는 launch 흐름 (재확인)

- **`/clock` 브릿지**: `spawn_gazebo.launch.py`의 `clock_bridge` Node. `spawn_gazebo.launch.py`는
  `bringup.launch.py`의 모든 mode(sim/slam/nav/explore)에서 조건 없이 항상 include되므로, 어떤
  mode로 띄우든 `/clock`은 항상 브릿지된다.
- **`explore.cpp`** (m-explore-ros2/explore 패키지 소스, 빌드 산출물은
  `install/explore_lite/lib/explore_lite/explore`): `semih_description/launch/explore.launch.py`가
  `package='explore_lite', executable='explore'`로 이 바이너리를 직접 실행한다. 이 launch 파일은
  `bringup.launch.py mode:=explore`의 TimerAction(33초 지연) 경로, 또는 `ros2 launch
  semih_description explore.launch.py` 단독 실행 경로 두 군데에서 로드된다.
- **`slam_params.yaml`**: `slam.launch.py`가 기본값으로 로드 → `bringup.launch.py`의 `mode in
  (slam, nav, explore)` 조건에서 include되거나, `ros2 launch semih_description slam.launch.py`
  단독 실행 시 로드.
- **`nav2_params.yaml`**: `nav2.launch.py`가 기본값으로 로드 → `bringup.launch.py`의 `mode in
  (nav, explore)` 조건에서 include되거나, `ros2 launch semih_description nav2.launch.py` 단독
  실행 시 로드.

### 2-6. 하드코딩된 절대경로 / 포트 / 디바이스 (grep 기준)

- `hdf5_collector_node.py:43` — `output_dir` 파라미터 기본값이 `os.path.expanduser('~/semih_datasets')`.
  리터럴 절대경로는 아니지만 실행 사용자의 `$HOME`에 의존 — 웹 래퍼에서 설정화 대상 1순위.
- `yolo_detection_node.py:22`, `target_3d_node.py:46` — `model_name` 파라미터 기본값이 `'yolov8n.pt'`
  (상대 경로). 실제 파일은 워크스페이스 루트 `/home/jangjunseo/semiH_ws/yolov8n.pt`에 존재함을
  `find`로 확인. ultralytics `YOLO()`가 이 상대경로를 어떤 기준 디렉터리로 resolve하는지(실행 시
  CWD 기준인지, 별도 캐시 디렉터리를 먼저 보는지)는 **확인 안 됨** — CWD가 워크스페이스 루트가
  아닌 상태로 `ros2 run`하면 재다운로드를 시도할 가능성이 있으나 이번 조사에서 실제로 재현/검증하지
  않음.
- `spawn_gazebo.launch.py:123-127` (`lidar_frame_bridge`) — `static_transform_publisher` 인자에
  프레임 이름 `lidar_link`, `semih/base_link/lidar`가 하드코딩. 로봇 엔티티명 `semih`도
  `spawn_entity`의 `-name semih`(50행)에 하드코딩.
- `config/joint_state_bridge.yaml` — gz 토픽명에 `/world/semih_world/model/semih/joint_state`가
  하드코딩. 월드 이름(`semih_world`)과 모델명(`semih`) 둘 다에 의존적이며, 둘 중 하나라도 바뀌면
  이 브릿지가 조용히 데이터를 못 받는 방식으로 깨진다(에러 없이 그냥 토픽이 안 들어옴).
- 네트워크 주소(IP)·포트 번호 하드코딩: grep(`192\.168`, `localhost:[0-9]`, `0\.0\.0\.0:`) 결과
  **확인 안 됨**.
- USB/시리얼 디바이스 경로(`/dev/tty*`, `/dev/video*`) 하드코딩: grep(`/dev/tty`, `/dev/video`) 결과
  **확인 안 됨** — 이 프로젝트는 전부 Gazebo 시뮬레이션 기반이라 실물 하드웨어 디바이스 경로 자체가
  코드에 없음(`/dev/dri` 관련 언급은 존재하지만 이는 코드가 아니라 `CLAUDE.md`/트러블슈팅 문서 내
  GPU 패스스루 미해결 이슈에 대한 서술일 뿐, launch/config 파일 안의 하드코딩이 아님).
- `ROS_DOMAIN_ID`, rosbridge/websocket 등 웹 연동 관련 설정: grep(`rosbridge`, `websocket`, `8080`,
  `9090`) 결과 **확인 안 됨** — `semiH_web`을 설계한다면 이런 종류의 ROS↔웹 브릿지 계층이 현재
  아무것도 구성되어 있지 않은 상태에서 처음부터 추가해야 함을 의미.
- `package.xml` 의존성 기준으로는 `semih_perception`/`semih_data_collection` 어느 쪽도
  `semih_description`을 `depend`/`exec_depend`로 선언하지 않음 — 세 패키지는 launch 파일이나
  빌드 의존성으로 서로 묶여있지 않고, 오직 런타임 토픽/액션(`/camera/image_raw`, `/target_point`,
  `arm_controller` 액션서버 등)으로만 연결된다.

---

## 3. mode 구조 및 맵 재사용 방식 확인 (2026-08-17)

**목적**: `semiH_web` 설계를 위해 "한 번 매핑 → 저장된 맵으로 반복 주행" 경로가 현재 코드에 있는지
확인. 이번 조사에서도 **코드를 전혀 수정하지 않았고**, 파일 내용과 grep 결과만 근거로 작성. 애매한
부분은 "확인 안 됨"으로 남김.

### 3-1. `mode` 인자 값별 include 매트릭스

`bringup.launch.py`에서 조건 분기에 쓰이는 인자는 `mode` 하나뿐이다(다른 launch 파일에는 이런
모드 분기 인자가 없음 — `slam.launch.py`/`nav2.launch.py`/`explore.launch.py`는 각각 파라미터
파일 경로 인자만 가짐, grep으로 `localization`/`Localization` 검색 결과 전무 확인). 소스
(`bringup.launch.py` 35~100행)에 나타난 조건은 다음 3개뿐이다:

```python
want_slam    = mode in ('slam', 'nav', 'explore')
want_nav     = mode in ('nav', 'explore')
want_explore = mode == 'explore'
```

`spawn_gazebo.launch.py`는 `mode` 값과 무관하게 **항상** include된다(조건 없음).

| `mode` 값 | spawn_gazebo | slam.launch.py | nav2.launch.py | priming_spin (20s 지연) | explore.launch.py (33s 지연) |
|---|---|---|---|---|---|
| `sim` (기본값, 미지정 시) | O | X | X | X | X |
| `slam` | O | O | X | X | X |
| `nav` | O | O | O | X | X |
| `explore` | O | O | O | O | O |

`mode`에 위 4개 외의 값(예: 오타나 `mapping` 같은 값)을 주면 어떤 `IfCondition`도 참이 되지 않아
`spawn_gazebo`만 뜨는 `mode=sim`과 동일하게 동작한다 — 코드에 값 검증(validation)이나 에러 처리가
없음(확인됨, `bringup.launch.py`에 `mode` 값을 화이트리스트와 대조하는 로직이 없다).

**참고**: 사용자가 예시로 든 "`mode=mapping`" 이라는 값은 이 코드베이스에 존재하지 않는다 — 매핑
전용 모드는 없고, `mode=slam`이 곧 "매핑하면서 동시에 raw map만 갱신"하는 모드다(3-2 참고). 별도의
"매핑 전용"과 "내비게이션 전용"을 가르는 인자는 없다.

### 3-2. slam_toolbox: mapping 모드인가 localization 모드인가

`config/slam_params.yaml` 17행: `mode: mapping` — **명시적으로 mapping 모드로 고정**되어 있다.
slam_toolbox가 지원하는 `localization` 모드(저장된 맵을 불러와 그 안에서 로컬라이제이션만 수행하는
모드, 보통 `map_file_name`/`map_start_pose` 같은 파라미터와 함께 씀)를 가리키는 값이나 관련
파라미터는 이 파일에 **없음**.

이 워크스페이스 전체(`src/` 하위, m-explore-ros2 포함)를 대상으로 다음을 grep했으며 전부 **매치
없음(확인 안 됨이 아니라 명확히 "없음")**:
- `mode: localization` 또는 유사 문자열
- `map_file_name` (slam_toolbox의 localization 모드용 파라미터)
- `localization_launch.py`, `localization.launch.py` 파일 include
- `nav2_map_server`/`map_server` 패키지에 대한 어떤 참조도 `semih_description` 소스 안에 없음
  (`package.xml`의 `exec_depend`에도 없음, launch 파일에도 없음)

따라서 slam_toolbox의 localization 모드를 쓰는 launch/파라미터 조합은 **이 코드베이스에 없다**.

### 3-3. 저장된 맵 파일 존재 여부 및 재사용 코드

디스크에서 SLAM 맵 관련 확장자(`.pgm`, `.posegraph`, `.data`, 그리고 이름이 `map*`인 파일/디렉터리)를
전수 검색한 결과:

- `.posegraph`/`.data`(slam_toolbox 자체 직렬화 포맷) 파일: **워크스페이스 전체에 0개**.
- `.pgm` 파일: `build/multirobot_map_merge/` 아래 4개(`2011-08-09-12-22-52.pgm`, `map00.pgm`,
  `2012-01-28-11-12-01.pgm`, `map05.pgm`)가 존재하지만, 이건 **이 프로젝트가 만든 맵이 아니다** —
  `src/m-explore-ros2/map_merge/test/download.sh`, `test/download_data.sh`가 map_merge 패키지
  자체 유닛테스트(`test_merging_pipeline.cpp`)용으로 외부에서 내려받는 테스트 픽스처이며,
  `build/`에 있는 이유도 colcon 빌드 시 그 테스트 데이터가 복사된 것일 뿐이다. semih 로봇이 자체
  주행으로 만든 맵과는 무관.
- 워크스페이스 최상위 또는 `src/semih_description/` 하위에 `map*` 이름의 파일/디렉터리:
  **0개**.

맵을 저장하거나 다시 불러오는 코드 흔적을 grep했으며 결과는 다음과 같다:
- `map_saver_cli` 문자열: 워크스페이스 어디에도 **없음** (소스 코드, launch, docs 전부 포함해서
  0건).
- `use_map_saver: true` — `config/slam_params.yaml` 16행에 이 파라미터가 켜져 있다. 이건
  slam_toolbox의 `/slam_toolbox/save_map` 서비스를 **활성화**만 할 뿐, 이 자체가 맵을 자동으로
  저장하지는 않는다(사용자가 직접 서비스를 호출해야 함). 이 서비스를 실제로 호출하는 코드/스크립트/
  문서(`docs/usage_guide.md` 등)는 **없음** — 즉 이 프로젝트 안에 "맵을 저장하는 실행 경로" 자체가
  기록되어 있지 않다.
- `docs/usage_guide.md` 전체를 대상으로 "맵 저장", "저장된 맵", "map.yaml", `save_map` 관련 문자열
  검색 결과 **매치 없음** — 사용법 문서에도 맵 저장/재사용 워크플로우가 설명되어 있지 않다.
- `nav2.launch.py`/`bringup.launch.py` 어디에도 nav2_bringup의 `localization_launch.py`나
  `map_server` Node를 include/실행하는 코드가 **없음**(3-2 grep 결과와 동일).

### 3-4. 결론: "한 번 매핑 → 저장된 맵으로 반복 주행" 경로가 있는가

**(c) 전혀 없고, 항상 SLAM+Nav2를 동시 실행하는 경로만 존재한다.**

근거:
1. `slam_params.yaml`이 `mode: mapping`으로 고정돼 있고, localization 모드로 전환하는 파라미터
   조합이나 그걸 쓰는 launch가 없다(3-2).
2. 지금까지 이 로봇이 자체적으로 저장한 맵 파일이 디스크에 전혀 없다(3-3) — `build/`에 있는 pgm은
   다른 패키지(map_merge)의 외부 테스트 픽스처.
3. 맵을 저장하는 실행 경로(`map_saver_cli` 호출, `save_map` 서비스 호출) 자체가 코드/문서 어디에도
   기록돼 있지 않다 — `use_map_saver: true`로 서비스만 열려 있을 뿐 실제로 쓰인 흔적이 없다(3-3).
4. `nav2.launch.py`가 include하는 `navigation_launch.py`는 amcl/map_server를 포함하지 않으므로
   (섹션 2-3에서 이미 확인), 설령 저장된 맵이 있었다 해도 그걸 불러와 로컬라이제이션할 launch 경로가
   현재 `semih_description` 안에 준비돼 있지 않다.
5. `bringup.launch.py`의 `mode` 값 중 "매핑 전용" 또는 "저장된 맵으로 주행 전용"을 가리키는 값은
   없다(3-1) — `nav`/`explore` 모드는 항상 `slam.launch.py`(즉 매 실행마다 새로 매핑)를 함께
   켠다.

즉 현재 코드 기준으로 Nav2를 쓰려면 매 실행마다 slam_toolbox가 처음부터 다시 매핑하며 동시에
로컬라이제이션을 제공하는 것 외의 경로가 없다 — "이전에 만든 맵을 불러와서 매핑 없이 반복 주행"하는
기능은 설계도 구현도 되어 있지 않다.
