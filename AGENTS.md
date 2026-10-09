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
