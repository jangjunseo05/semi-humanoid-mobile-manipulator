# semiH_web — 프로젝트 메모 (SSOT)

`semiH_web`은 `semiH_ws`(ROS 2 워크스페이스, 같은 부모 디렉터리에 위치)를 감싸는
웹 개발 키트다. 이 파일은 `semiH_web` 자체의 설계 근거를 기록하는 단일 진실
소스(SSOT)이며, `semiH_ws/CLAUDE.md`(그 워크스페이스 자체의 진단/구조 기록)와는
별개 문서다.

---

## 1. 설계 논의 요약 (2026-08-17, 뼈대 생성 시점)

### 1-1. 원칙: 기존 코드 비수정

`semiH_web`은 `semiH_ws` 내부의 어떤 파일도 수정/이동/복사하지 않는다.
`core/launch_runner.py`/`core/node_runner.py`가 `subprocess`로 `ros2 launch`/
`ros2 run`을 바깥에서 실행하는 것만 허용되며, `semiH_ws`를 참조하는 유일한
경로는 `config/settings.yaml`의 `workspace.path`(및 관련 setup.bash 경로)다.
이 원칙은 이번 뼈대 생성 세션에서도 그대로 지켜졌다 — 아래 "검증" 절 참고.

### 1-2. mode 구조: 4개 값으로 고정된 Enum

`semiH_ws/CLAUDE.md` 섹션 3-1에서 확인된 사실: `bringup.launch.py`의 `mode`
인자는 코드상 `sim`/`slam`/`nav`/`explore` 4개 누적 값만 의미 있게 처리하고
(각각 spawn_gazebo만 / +slam / +slam+nav2 / +slam+nav2+explore), 그 외의
값(오타 포함)을 줘도 아무 조건도 참이 안 돼 **조용히 `mode=sim`으로
fallback**된다 — 원본 코드에 값 검증이 없다.

`core/launch_runner.py`의 `BringupMode`는 이 4개 값만 갖는 `enum.Enum`으로
고정했다. 웹 레이어 어디에도 mode를 자유 텍스트로 입력받는 경로를 두지
않는다 — `pages/main.py`의 `render_bringup_panel()`도 select 계열 위젯만
쓰도록 docstring에 명시했다. 목적은 "잘못된 값이 조용히 다른 동작으로
새는" 원본의 버그 패턴을 웹 레이어에서 반복하지 않는 것.

**How to apply**: 앞으로 `semih_description`에 새 mode 값이 추가되거나
기존 값의 의미가 바뀌면, `semiH_ws/CLAUDE.md` 섹션 2·3을 먼저 재조사한
뒤에만 `BringupMode`를 갱신할 것 — 추측으로 값을 늘리지 않는다.

### 1-3. 락 설계: 자원은 `gazebo_instance` 하나뿐

`semiH_ws/CLAUDE.md` 섹션 2-4에서 확인된 사실: `mode` 값이 무엇이든
`spawn_gazebo.launch.py`는 `bringup.launch.py` 안에서 항상 include되므로,
4개 mode 전부가 결국 같은 Gazebo 프로세스 트리(gzserver + 브릿지 + 컨트롤러
스포너 등)를 하나씩만 가질 수 있다. 즉 **"mode가 다르면 동시 실행 가능"이
아니라, mode가 무엇이든 bringup 프로세스 트리는 시스템에 하나만 존재할 수
있다.**

이걸 반영해 `core/process_manager.STAGE_RESOURCES`는 `"bringup":
["gazebo_instance"]` 단 하나의 자원 태그만 쓴다 — so101_web처럼 stage마다
다른 자원 조합(`mujoco_viewer`/`serial_port`/`gpu`)을 갖는 구조가 아니라,
"bringup을 시작하려는데 이미 어떤 mode로든 bringup이 떠 있으면 무조건
충돌"이라는 단순한 규칙 하나로 충분하다고 판단했다.

YOLO/3D좌표+IK/HDF5(4개 독립 `ros2 run` 노드, `semiH_ws/CLAUDE.md` 섹션
2-1·2-4에서 launch 파일 자체가 없음을 확인)는 `gazebo_instance`와 자원이
겹치지 않고, 이 4개 노드끼리도 이번 스코프에서는 자원 경합이 없다고
간주해 `STAGE_RESOURCES`에서 전부 빈 리스트로 선언했다. 다만 같은 노드를
두 번 중복 실행하는 것은 여전히 `process_manager`가 stage 단위 lock으로
막는다(자원 충돌이 아니라 "이미 실행 중" 충돌).

**GPU 락은 이번 스코프에서 명시적으로 제외한다.** so101_web은 `training`/
`inference` stage가 `gpu` 자원을 선언했지만, semiH_web은 아직 학습/추론
단계를 다루지 않으므로 GPU 자원 태그를 두지 않았다. 나중에 필요해지면
`STAGE_RESOURCES`에 항목을 추가하는 것만으로 확장 가능하도록 so101_web과
동일한 구조(`dict[str, list[str]]`)를 그대로 유지했다.

**Why**: gazebo_instance 두 개가 동시에 뜨면 포트/토픽 이름이 겹쳐서
`ros_gz_bridge`가 어느 쪽에 붙는지 불확정해지는 문제가 `semiH_ws` 쪽 진단
세션 중 실측으로도 확인된 바 있다(중복 실행된 두 스택이 뒤섞여 결과를
오염시킨 사례, `semiH_ws/CLAUDE.md` 섹션 1 진단 작업 로그 참고). 웹
레이어에서 이걸 구조적으로 막는 것이 목적.

### 1-4. 페이지 통합 결정: 단일 `pages/main.py`

`so101_web`(참고 패턴 출처, `C:\Users\USER\Desktop\so101_web`)은 파이프라인
단계가 5개(데이터 수집/QA/학습/추론/전처리 구조)라 Streamlit 다중 페이지
(`dashboard/pages/1_...py` ~ `5_...py`) + `st.navigation()` 라우터 구조를
썼다. `semiH_web`은 다루는 기능 단위가 "bringup 제어(mode 4종) + 독립 노드
4개 + 상태 모니터링" 정도로 상대적으로 적고, 이 전부를 한 화면에서 동시에
보는 편이 사용성 면에서 자연스럽다고 보고 **단일 페이지(`pages/main.py`
하나)로 통합**했다. 이는 요청받은 폴더 구조 자체에도 반영돼 있다
(`pages/main.py` 단수, so101_web의 `dashboard/pages/1_...py` 같은 번호
매김 다중 파일 패턴이 아님).

**How to apply**: 나중에 기능이 늘어나(예: 학습/추론 단계 추가) 한 화면에
다 담기 어려워지면, 그때 so101_web의 `st.navigation()` 다중 페이지 패턴으로
전환하는 것을 재검토할 것 — 지금 단계에서 미리 다중 페이지 뼈대를 만들지는
않는다(YAGNI).

### 1-5. 참고 패턴 출처

`core/process_manager.py`의 lock 파일 설계(디스크 JSON lock, `O_CREAT |
O_EXCL` 원자적 생성, graceful-then-force 종료, stale lock 정리)는
`so101_web/dashboard/lib/process_manager.py`를 실제로 찾아서 읽고 참고했다
(추측 아님 — 파일 경로: `C:\Users\USER\Desktop\so101_web\dashboard\lib\process_manager.py`).
semiH_web 쪽은 자원 태그가 `gazebo_instance` 하나뿐이라는 점에서 구조를
단순화했지만, lock 파일 원자성·graceful 종료 원칙은 그대로 가져왔다.

---

## 2. process_manager 구현 및 검증 (2026-08-17)

`core/process_manager.py`(gazebo_instance 락)와 `core/launch_runner.py`
(bringup 실행/종료 결선)를 실제 로직으로 구현하고 라이브로 검증했다.
`core/node_runner.py`, `monitoring/topic_watcher.py`, `pages/main.py`는
여전히 1절 시점의 뼈대(`NotImplementedError`) 상태 그대로다 — 이번 작업
범위 밖.

### 2-0. 사전 확인: target_3d_node / arm_reach_node가 정말 별도 노드인가

**둘 다 실제로 존재하는 별도 `ros2 run` 실행 파일이다** —
`semih_perception/setup.py`의 `entry_points.console_scripts`에
`yolo_detection_node`/`target_3d_node`/`arm_reach_node` 3개가 각각 독립
항목으로 등록돼 있고, 빌드된 `install/semih_perception/lib/semih_perception/`
안에도 세 실행 파일이 개별로 존재함을 직접 확인했다(한 노드를 잘못 둘로
나눠 이해한 게 아님).

### 2-1. Stale lock 판단: timeout → PID 생존 여부로 변경

so101_web은 pid가 채워진 lock도 오래되면(관찰상) `psutil.pid_exists`로만
판단했지만, 요청대로 semiH_web은 **pid가 기록된 lock에는 timeout을 아예
적용하지 않는다** — `_all_locks()`가 매번 `os.kill(pid, 0)`으로 생존을
확인하고, 죽어있을 때만 lock을 지운다(`core/process_manager._pid_alive`).
timeout(`STARTING_TIMEOUT_S`, 60초)은 lock이 원자적으로 생성된 직후 ~
실제 subprocess pid로 채워지기 전인 극히 짧은 "starting" placeholder
구간에만 적용된다 — `mode=explore`처럼 사용자가 수 시간 띄워둬도 stale로
오판되지 않는다.

### 2-2. SIGINT 전파 범위 라이브 검증: 단일 PID로는 부족함을 확인

**소스 확인**: `ros2 launch`(`osrf_pycommon/process_utils/
async_execute_process_asyncio/impl.py`)는 각 자식 프로세스를
`loop.subprocess_exec()`로 띄우며 `start_new_session`/`preexec_fn=os.setsid`
같은 프로세스 그룹 분리 옵션을 전혀 쓰지 않는다 — 즉 launch가 직접 띄우는
자식들은 기본적으로 launch 프로세스와 같은 프로세스 그룹에 속한다.
`launch_service.py`는 SIGINT에 대해 자체 핸들러를 등록해(`manager.handle
(signal.SIGINT, _on_sigint)`) 내부적으로 각 자식에 개별적으로 종료를
요청하는 구조이지, OS 프로세스 그룹 브로드캐스트에 의존하는 구조가
아니다 — 그래서 "launch 부모 하나에만 신호를 보내도 될지"는 소스만으로는
단정할 수 없었고, 실측이 필요했다.

**라이브 테스트 방법**: `bringup.launch.py mode:=explore headless:=true`를
`subprocess.Popen(..., start_new_session=True)`로 띄운 뒤(그 pid가 곧 그
프로세스 그룹의 pgid), `ros2 node list`로 40개 안팎의 노드(explore_node
포함)가 전부 뜬 것을 확인하고 두 가지를 비교했다:

| 테스트 | 방법 | 결과 |
|---|---|---|
| 1차 | launch 부모 pid **하나에만** `SIGINT` | launch 자신과 직계 ROS 노드(robot_state_publisher, 브릿지 7개, slam_toolbox, nav2 서버 전체, explore_lite)는 전부 정상 종료됨. **그러나 `ign gazebo` 실제 프로세스(PID 20266)가 고아로 남았다** — `/bin/sh -c 'ruby /usr/bin/ign gazebo ...'` 셸 래퍼(launch가 직접 추적하는 자식)는 죽었지만, 그 손자인 실제 `ign gazebo`에는 신호가 전달되지 않았기 때문. `ps -eo pid,ppid,pgid`로 재확인. |
| 2차 (같은 조건 재현) | 같은 프로세스 그룹 전체에 `kill -INT -<pgid>` (`os.killpg`와 동일 효과) | 21개 프로세스(gzserver, 브릿지, slam, nav2, explore_lite 전부) 및 `ign gazebo`까지 잔여물 없이 전부 종료. `ps aux` 전수 재확인 — 좀비/고아 0개. |

**결론(채택)**: `process_manager.start_job()`은 모든 job을
`start_new_session=True`로 띄우고, `stop_job()`은 항상 leader pid가 아니라
**프로세스 그룹 전체**(`os.killpg`)에 신호를 보낸다. 단일 PID 방식은
채택하지 않았다 — 실측으로 `ign gazebo` 고아 프로세스가 실제로 재현됐기
때문(추측이 아니라 두 번의 라이브 실행으로 원인까지 확인: 셸 래퍼가
자기 자식에게 신호를 전달하지 않는 것이 근본 원인).

### 2-3. 부가 발견: graceful timeout 기본값(5초)이 bringup에는 부족함

2-2 검증 중 `stop_job()`의 기존 기본값 `graceful_timeout_s=5.0`으로
`stop_bringup()`을 실측했더니, 매번 5.01초를 다 채우고 SIGKILL로
escalate됐다 — "graceful"이라는 이름과 달리 사실상 매번 강제종료였다는
뜻이었다. 원인을 확인하려고 `graceful_timeout_s` 제한 없이 그룹 SIGINT
이후 launch 부모가 스스로 죽기까지 걸리는 시간을 직접 재봤더니 **8.43초**
였다(nav2 lifecycle manager가 여러 서버를 순서대로 deactivate/cleanup하고
gzserver까지 내려가는 시퀀스라 오래 걸리는 것으로 추정 — 정확한 내부
단계별 소요는 미조사).

`core/launch_runner.py`에 `BRINGUP_GRACEFUL_TIMEOUT_S = 15.0`을 따로 두고
`stop_bringup()`이 이 값을 `process_manager.stop_job()`에 넘기도록
수정했다(`process_manager`의 전역 기본값 5.0초는 그대로 둠 — 가벼운 단일
`ros2 run` 노드 stage에는 맞는 값이라고 판단, 다만 이 판단 자체는 아직
node_runner.py 쪽에서 실측 검증하지 않았음, "확인 안 됨"으로 남김). 수정
후 재측정: 5.21초 만에 스스로 종료(15초 한도 안에서 자연 종료, SIGKILL
escalate 없음) — 매 실행마다 걸리는 시간에 변동이 있음을 확인했다(8.43초
→ 5.21초, 이 워크스페이스가 WSL2 CPU 경합에 민감하다는 기존
semiH_ws/CLAUDE.md 섹션 1 A-3 기록과 일치하는 것으로 보이나, 이번 조사에서
원인까지 확정하지는 않음).

### 2-4. 검증 결과 (요청된 5단계 전부 실행, 웹 레이어 거치지 않고
`core/launch_runner.py` 직접 호출)

1. **`mode=explore` 직접 실행** → `launch_runner.start_bringup(BringupMode.EXPLORE,
   headless=True)` 성공, `{"running": True, "pid": <pid>, ...}` 반환 확인.
2. **락 파일 생성 확인** → `state/locks/bringup.json`을 `cat`으로 직접 열어
   `"pid"` 필드에 실제 `ros2 launch` PID가 들어있음을 확인.
3. **중복 실행 거부 확인** → 위 job이 도는 상태에서 `start_bringup(SIM)`을
   또 호출 → `process_manager.JobConflictError` 발생 확인(`이미 실행
   중이거나 지금 막 시작되고 있습니다`).
4. **graceful 종료 확인** → `stop_bringup(graceful=True)` 호출 →
   `ros2 node list` 및 `ps aux` 양쪽에서 관련 프로세스 전부 사라짐을 확인
   (일시적으로 `ros2 node list`에 죽은 노드 이름이 몇 초간 남아있었는데,
   `ps aux`로 이미 실제 프로세스가 없음을 먼저 확인했고 10초 후 재조회
   시 `ros2 node list`도 비워짐 — ros2cli 데몬의 그래프 캐시 지연으로
   판단, 좀비 프로세스 아님).
5. **락 파일 정리 확인** → `state/locks/` 디렉터리가 종료 후 빈 디렉터리로
   돌아옴을 확인.
6. **semiH_ws 무변경 재확인** → 이번 구현·검증 작업 전체(코드 작성 + 4회의
   라이브 launch/kill 사이클) 동안 `find /home/jangjunseo/semiH_ws -newer
   CLAUDE.md`가 매번 빈 결과 — 소스 파일 변경 없음.

### 2-5. 부산물: 이번 세션 중 발견한 기존 정리 누락

라이브 테스트 시작 전 `ps aux`에서 이전 세션(이 대화 안의 더 앞선 작업)에서
남긴 `ros2 launch` 잔여 프로세스 1개와, `tf2_ros/static_transform_publisher`
좀비 프로세스 7개(각기 다른 이전 실행에서 하나씩 누적됨 — `spawn_gazebo.
launch.py`의 `lidar_frame_bridge` 노드)를 발견해 정리했다. 원인은 그동안의
수동 정리 시 `ps aux | grep` 패턴에 `static_transform`/`tf2_ros`를 포함하지
않았던 것으로 보인다(semiH_ws 코드 문제 아님, 이 조사 세션의 수동 운영
실수). `process_manager`가 구현됐으니 앞으로 bringup을 이 코드로 시작/
종료하면 이런 수동 누락이 구조적으로 발생하지 않는다(항상 프로세스 그룹
전체를 추적·종료하므로).

---

## 3. node_runner 구현 및 검증 (2026-08-17)

`core/node_runner.py`(YOLO/target_3d/arm_reach/HDF5 4개 독립 `ros2 run`
노드)를 실제 로직으로 구현하고 라이브로 검증했다. 구현 전에 "락이 정말
필요한가"부터 4개 노드 각각 실측으로 확인한 뒤 정책을 갈랐다 — 추측으로
전부 잠그거나 전부 안 잠그지 않았다.

### 3-1. HDF5 중복 실행 조사: 카테고리 (c) — 에러내고 죽음(데이터는 유실)

**소스 확인**: `hdf5_collector_node.py`의 저장 경로는
`{output_dir}/semih_session_{datetime.now().strftime('%Y%m%d_%H%M%S')}.h5`
(203행 근처) — **초 단위 타임스탬프뿐, PID/세션ID 등 구분자가 전혀 없고**,
이 문자열은 `duration_sec` 경과 시점(저장 시점)에야 계산된다. 시작 시각이
아니라 저장 시각 기준이므로, 같은 `output_dir` + 비슷한 `duration_sec`로
얼추 동시에 띄운 두 인스턴스가 저장 시점까지 같은 초 안에 들어오면 파일명이
그대로 충돌한다.

**라이브 재현** (mode=sim 브링업 아래, `duration_sec:=3.0`, 같은
`output_dir`로 두 인스턴스를 동시에 `subprocess.Popen`): 먼저 저장을 마친
프로세스는 정상 종료(15프레임, 544KB 파일 정상 생성). 나중에
`h5py.File(filepath, 'w')`를 연 프로세스는:

```
BlockingIOError: [Errno 11] Unable to synchronously create file
(unable to lock file, errno = 11, error message = 'Resource temporarily unavailable')
[ros2run]: Process exited with failure 1
```

HDF5 라이브러리 자체의 파일 락(POSIX `flock`류) 때문에 "덮어쓰며 조용히
망가짐"이 아니라 **두 번째 프로세스가 예외를 던지고 그대로 크래시**했다 —
사용자가 요청한 3가지 카테고리 중 (c). 다만 (c)라고 "안전"은 아니다: 크래시한
쪽이 그동안 수집하던 세션 데이터 전체를 저장 한 번 못 해보고 통째로 잃는다
(먼저 연 쪽만 살아남음, 어느 쪽이 "먼저"일지는 보장 안 됨). 그래서
`hdf5_collector`는 process_manager의 **고정 stage 이름**(원자적 락)으로
묶어 애초에 두 번째 시작 시도 자체를 막기로 했다.

**부수 발견**: `save_and_shutdown()`이 정상적으로 파일을 저장하고
`rclpy.shutdown()`까지 호출해도, **프로세스 자체는 종료되지 않고 그대로
떠 있다**(라이브 확인: "Saved N frames..." 로그 이후 93초가 지나도록 살아
있음, CPU 0.3~7%). `main()`의 `rclpy.spin(node)`가 `KeyboardInterrupt`만
잡고 있고, 타이머 콜백 안에서의 `rclpy.shutdown()` 자체는 spin 루프를
바로 못 빠져나오게 하는 것으로 보인다(정확한 rclpy 내부 원인까지는
미조사 — "확인 안 됨"). 실무적으로 중요한 결론: **`duration_sec`이 지나
"저장 완료" 로그가 찍혀도 웹 UI는 이 job을 여전히 "실행 중"으로 표시해야
하고, 사용자가 명시적으로 정지(`stop_node`)해야 프로세스가 실제로
사라진다** — `node_runner.stop_node()`가 이미 그 경로다.

### 3-2. YOLO / target_3d / arm_reach 중복 실행 조사: 전부 안전

같은 mode=sim 브링업 아래 세 노드를 각각 두 인스턴스씩 동시에 띄워 확인:

| 노드 | 확인 방법 | 결과 |
|---|---|---|
| `yolo_detection_node` | 2개 동시 실행, `ros2 topic info /yolo/image_annotated -v` | 에러 없음, `Publisher count: 2`, 둘 다 정상적으로 프레임마다 detection 로그 출력 |
| `target_3d_node` | 2개 동시 실행, `ros2 topic info /target_point` | 에러 없음, `Publisher count: 2`, 둘 다 같은 입력(같은 카메라/뎁스 프레임)에서 사실상 동일한 좌표를 각자 독립적으로 계산·발행(낭비지만 위험은 아님) |
| `arm_reach_node` | 2개 동시 실행 + `/target_point`에 같은 좌표(x=0.3,y=0,z=0.35) 3회 publish | 에러/크래시 없음. 둘 다 `arm_controller` 액션 서버에 목표를 보냈고 거부 로그(`Trajectory goal was rejected`) 없음. 최종 `/joint_states`가 의도한 값(joint2≈43°, joint3≈-90°)으로 정상 수렴 |

**arm_reach_node에 대한 한계 고지**: 이 테스트는 두 인스턴스가 **같은**
목표를 보낸 경우만 재현했다(둘 다 같은 카메라 입력을 보므로). 서로 다른
목표를 경쟁적으로 보낼 때 액션 서버가 새 목표로 이전 목표를 프리엠프트하며
팔이 흔들리는 상황 자체는 라이브로 재현하지 않았다 — 이건 ros2_control
(`JointTrajectoryController`)의 표준 동작(단일 실행 액션 서버는 새 목표가
이전 목표를 프리엠프트)이라 크래시로 이어지지는 않을 것으로 판단했지만,
이 판단은 실측이 아니라 표준 동작에 대한 근거 추론임을 구분해서 기록한다
("확인 안 됨" 아님 — 크래시 여부는 표준 동작상 확정적이라고 보되, 실제
흔들림의 "정도"까지는 검증하지 않았다는 뜻).

**셸 래퍼 여부(소스 + 라이브 둘 다 확인)**: `ros2run/api.py`의
`run_executable()`은 대상 실행 파일을 `subprocess.Popen([path] + argv)`로
직접 띄운다 — `/bin/sh -c`류 래퍼가 전혀 없다(gazebo 사례와 다름). 단
`ros2 run` 자체(부모)와 실제 노드(자식)는 별개의 OS 프로세스이고,
`run_executable()`의 주석("the subprocess will also receive the signal
and should shut down")은 신호가 **프로세스 그룹을 통해** 자식에게도
전달되는 것을 전제로 할 뿐, 코드가 명시적으로 자식에게 신호를 릴레이하지
않는다. `ps -eo pid,ppid,pgid`로 4개 노드 전부 부모/자식이 같은 pgid를
공유함을 라이브로 확인했다 — 즉 셸 래퍼는 없지만, bringup과 마찬가지로
**단일 PID가 아니라 프로세스 그룹 단위로 종료해야** 자식(진짜 노드)까지
확실히 죽는다는 결론은 동일하다. `core/process_manager.stop_job()`이 이미
모든 stage에 대해 `os.killpg()`를 쓰므로(2절), 별도 코드 없이 4개 노드
전부 이 메커니즘을 그대로 물려받는다.

### 3-3. 구현: 락 정책을 노드별로 분리

- **`hdf5_collector`**: 고정 stage 이름(`"hdf5_collector"`)으로
  `process_manager`를 호출 — O_CREAT|O_EXCL 원자적 락 그대로 재사용,
  두 번째 시작 시도는 `JobConflictError`.
- **`yolo_detection`/`target_3d`/`arm_reach`**: 시작할 때마다
  `f"{node.value}__{uuid4().hex[:8]}"` 형태의 고유 stage 키를 새로
  발급 — `process_manager` 레벨에서 서로 절대 부딪히지 않는다(락 없음).
  `get_node_status()`는 이 경우 `state/locks/`에서 해당 접두어로 시작하는
  stage를 전부 스캔해 **리스트**를 반환한다(몇 개가 떠 있는지 보여주는
  "상태 추적" 요구사항 — `process_manager.py` 자체는 건드리지 않고
  `LOCKS_DIR`을 node_runner에서 직접 glob).
- `build_node_command()`는 `config/settings.yaml`의
  `perception.yolo_model_path`(yolo_detection/target_3d의 `model_name`)와
  `dataset.output_path`(hdf5_collector의 `output_dir`)를 기본
  `--ros-args -p`로 자동 주입한다 — semiH_ws/CLAUDE.md 섹션 2-6에 기록된
  `yolov8n.pt` 상대경로/CWD 의존 문제를 웹 레이어에서 없앰. 사용자가
  `start_node(..., ros_args={...})`로 넘긴 값이 있으면 그게 우선한다.

### 3-4. 검증 결과

1. **hdf5_collector 락 확인**: `start_node(HDF5_COLLECTOR)` 성공 후 같은
   호출 재시도 → `process_manager.JobConflictError` 발생 확인(웹 레이어
   거치지 않고 `core/node_runner.py` 직접 호출).
2. **안전 노드 다중 인스턴스 확인** (YOLO로 대표 검증): `start_node()`를
   연속 2번 호출 → 서로 다른 `stage`/`pid` 2개 확인 → `get_node_status()`
   가 길이 2 리스트 반환 → `stop_node()`(stage 미지정, "전부 끄기") 호출 →
   두 인스턴스 모두 정상 종료, `ps aux`에 잔여 프로세스 0개, 이후
   `get_node_status()`가 빈 리스트 반환.
3. **4개 노드 동시 기동 → ps aux 확인 → 종료 → 좀비 없음 확인**: 4개
   노드를 각각 `start_node()`로 동시에 띄우고 `ps -eo pid,ppid,pgid,cmd`로
   전부 확인(부모=`ros2 run`, 자식=실제 노드, 부모/자식 같은 pgid, 설정값
   자동 주입 확인) → 4개 전부 `stop_node()`로 종료(0.40~4.57초, 전부 기본
   5초 타임아웃 안에 자연 종료 — 강제 SIGKILL escalate 없었음) → `ps aux`
   재확인 결과 4개 노드 프로세스 전부 0개, `state/locks/`도 빈 상태로
   복귀(테스트용으로 같이 띄워둔 `bringup`(mode=sim)의 lock만 남아 있었고,
   이것도 이어서 `stop_bringup()`으로 정리·재확인).
4. **semiH_ws 무변경 확인**: 이번 조사·구현·검증 전체(소스 3개 노드 파일
   재확인 + `ros2run` 소스 확인 + 라이브 실행 10회 이상) 동안
   `find /home/jangjunseo/semiH_ws -newer CLAUDE.md`가 매번 빈 결과.

---

## 4. topic_watcher 구현 및 검증 (2026-08-17)

`monitoring/topic_watcher.py`에 `get_system_status()` 하나를 구현했다.
**새 프로세스 관리 로직은 만들지 않았다** — `core/launch_runner.
get_bringup_status()`와 `core/node_runner.get_node_status()`(각 4개
노드)를 그대로 호출한 결과를 재구성만 한다. lock 파일을 직접 읽거나
`ps`/`psutil`을 새로 부르는 코드는 이 모듈에 없다.

### 4-1. 설계: 두 가지 반환 모양을 하나로 통일

`node_runner.get_node_status(node)`는 노드 종류에 따라 반환 타입이
다르다(3절에서 이미 그렇게 설계함) — `hdf5_collector`(락 노드)는 dict
하나, 나머지 3개(비-락 노드)는 list. `pages/main.py`가 노드 종류별로
분기하지 않고 4개를 동일하게 순회할 수 있도록, `_normalize_node_status()`
가 이 둘을 `{"locked": bool, "instance_count": int, "instances": [...]}`
로 통일한다. `locked` 여부는 `node_runner`의 비공개 구현(`_SINGLE_
INSTANCE_NODES` 등)을 들여다보지 않고, **반환 타입(dict vs list) 자체를
보고 판단**한다 — 공개 API의 모양만으로 정책을 읽어내는 방식이라
`node_runner` 내부가 바뀌어도(락 정책 자체가 바뀌지 않는 한) 깨지지
않는다.

`get_system_status()`의 반환 형태:
```
{
  "bringup": {"running": bool, "mode": str|None, "headless": bool|None, "pid": int|None},
  "nodes": {
    "yolo_detection": {"locked": False, "instance_count": int, "instances": [...]},
    "target_3d":      {"locked": False, "instance_count": int, "instances": [...]},
    "arm_reach":       {"locked": False, "instance_count": int, "instances": [...]},
    "hdf5_collector": {"locked": True,  "instance_count": 0|1, "instances": [...]},
  },
}
```
`instances`의 각 원소는 `process_manager`가 lock 파일에 적어둔 필드
(`pid`, `started_at`, `cmd`, `log_path`, 비-락 노드는 `stage`까지)를 가공
없이 그대로 통과시킨다.

토픽 hz 체크/`/clock` 상태 확인은 이번 스코프에서 제외했다 — 기존
스켈레톤에 있던 `is_topic_publishing`/`get_topic_hz`/`get_active_topics`/
`get_bringup_health`/`get_node_health`는 손대지 않고 TODO로 그대로
남겨뒀다(`get_system_status()`는 이것들을 전혀 호출하지 않음).

### 4-2. 검증 결과

1. **아무것도 안 뜬 상태에서 조회**: `get_system_status()` →
   `bringup.running=False`(mode/headless/pid 전부 null), 4개 노드 전부
   `instance_count=0, instances=[]` (단, `hdf5_collector`는 `locked=True`
   로 정확히 표시 — 개수가 0이어도 락 정책 자체는 계속 보여야 하므로 의도한
   동작).
2. **`mode=nav` 브링업 + YOLO 2개 인스턴스 상태에서 조회, `ps aux`와 대조**:
   - `ps -eo pid,ppid,pgid,cmd`로 전체 프로세스 트리 캡처: bringup 부모
     pid `26893`(mode=nav, gzserver+브릿지 7개+slam_toolbox+nav2 서버
     전체), YOLO 두 `ros2 run` 프로세스 pid `26896`/`26904`.
   - 같은 시점 `get_system_status()` 결과: `bringup.running=True,
     mode="nav", pid=26893` — **정확히 일치**. `nodes.yolo_detection.
     instance_count=2`, 두 인스턴스의 `pid`가 각각 `26896`/`26904`로
     **정확히 일치**. 나머지 3개 노드는 `instance_count=0` — 실제로 안 띄웠으니
     맞음.
3. **전부 종료 후 재조회**: `node_runner.stop_node()` + `launch_runner.
   stop_bringup()` 호출 → `ps aux`에 관련 프로세스 0개 확인 →
   `get_system_status()`가 1번과 완전히 동일한 "아무것도 없음" 형태로
   복귀.
4. **semiH_ws 무변경 확인**: 이번 구현·검증 전체 동안
   `find /home/jangjunseo/semiH_ws -newer CLAUDE.md`가 매번 빈 결과.

---

## 5. 알려진 제약/미검증 사항 (pages/main.py 구현 시 UI에 반영할 것)

`pages/main.py`를 구현할 때 아래 두 가지를 사용자에게 드러나게 표시해야
한다 — 둘 다 3절 실측에서 나온 결과이며, `topic_watcher`/`node_runner`
수준에서는 프로세스가 "살아있다"는 것만 보여줄 뿐 이 두 함정 자체를
자동으로 걸러주지 않는다.

1. **HDF5 수집 노드는 저장 완료 후에도 프로세스가 스스로 안 죽는다**
   (3-1절 "부수 발견"). `duration_sec`이 지나 로그에 "Saved N frames..."
   가 찍혀도 `get_system_status()`의 `hdf5_collector.instance_count`는
   계속 1이다 — 이건 버그가 아니라 실제로 프로세스가 살아있는 그대로를
   보여주는 것이다. **UI는 "duration 경과 + 저장 로그 확인됨"과 "완전히
   종료됨"을 구분해서 보여줘야 하고(예: 저장 로그를 `tail_log()`로 확인해
   "저장 완료, 아직 실행 중 — 정지 버튼을 눌러 종료하세요" 같은 안내),
   사용자가 명시적으로 정지 버튼을 눌러야 한다는 것을 명확히 알려야 한다**
   — 안 그러면 "다 됐는데 왜 계속 떠 있지?"로 혼란을 준다.
2. **`arm_reach_node`를 2개 이상 동시에 띄웠을 때, 서로 다른 목표를
   두고 경쟁하는 상황은 검증하지 않았다**(3-2절 한계 고지). 두 인스턴스가
   같은 목표를 보낸 경우만 라이브로 확인했고, 크래시가 안 난다는 것까지는
   확인했지만 실제 궤적이 얼마나 흔들리는지는 안 봤다. **UI는
   `arm_reach`가 이미 1개 이상 떠 있는 상태에서 사용자가 하나 더 시작하려
   할 때 "여러 개를 동시에 켜면 팔이 서로 다른 목표를 두고 경합할 수
   있습니다" 같은 경고를 보여주는 것을 권장** — `node_runner`가 막지는
   않으므로(3절에서 안전하다고 판단해 락을 안 걸었음) UI 레벨의 안내로
   보완해야 한다.

---

## 6. pages/main.py 구현 및 검증 (2026-08-17)

`pages/main.py`를 Streamlit 앱으로 구현했다. **새 프로세스 관리/상태 판단
로직은 만들지 않았다** — 모든 상태는 `topic_watcher.get_system_status()`
하나로만 조회하고, 모든 조작은 `launch_runner`/`node_runner`의 기존 함수만
호출한다(`_hdf5_save_detected()`만 예외 — `process_manager.tail_log()`가
주는 텍스트를 읽어 UI 문구를 고르는 페이지 레벨 해석이며, so101_web
`pm.looks_like_crash(tail)`를 페이지 코드에서 직접 부르는 것과 동일한
선례를 따름).

### 6-1. 참고 패턴 (so101_web 실제 확인, 추측 아님)

`C:\Users\USER\Desktop\so101_web\dashboard\lib\ui_components.py`,
`dashboard/lib/next_action.py`, `dashboard/pages/1_데이터_수집.py`를 실제로
읽고 그대로 재사용했다:
- `render_card_header(icon, title, role)` 색상표(action=파랑 `#2a78d6`,
  settings=회색 `#52514e`, success=초록 `#1baf7a`, warning=노랑
  `#eda100`) — `pages/main.py`의 `_render_card_header()`가 동일한 값을
  그대로 씀.
- 차단 사유를 헤더 색 전환(`"warning" if blocked else "action"`) +
  버튼 위 캡션으로 보여주는 패턴(`1_데이터_수집.py` 66~100행) — 환경
  카드/인식·수집 카드 전부 동일하게 적용.
- `next_action.get_next_action()`의 "첫 매치 채택" 우선순위 방식 —
  `get_next_action(status)`가 동일 구조(5단계 우선순위, semiH_web은
  다중 페이지가 아니라 단일 페이지라 `target_page` 링크는 없음).
- `streamlit-autorefresh` 사용 패턴(실행 중일 때만 2~3초 간격 자동
  새로고침).

### 6-2. 페이지 구성 (요청된 4개 섹션 그대로 구현)

1. **상단 동적 CTA** (`render_top_cta`/`get_next_action`): HDF5 저장완료
   미종료 감지(최우선, warning) → 환경 꺼짐(action) → mode nav 미만
   (action, 이유 안내) → 노드 하나도 안 뜸(action) → 그 외 정상 동작
   (success, 초록).
2. **환경 카드**: `BringupMode` 4개 버튼(sim/slam/nav/explore). 실행
   중이면 버튼 대신 mode/PID/headless 메트릭 + 종료 버튼, 헤더는
   warning(그때는 "다른 mode 쓰려면 먼저 종료" 캡션이 버튼보다 위).
3. **인식·수집 카드**: mode가 nav 미만이면(또는 환경 자체가 꺼져
   있으면) 4개 카드 전부 헤더 warning + 차단 사유 캡션 + 버튼
   `disabled=True`. YOLO/target_3d/arm_reach는 인스턴스 여러 개 허용
   (각각 PID+경과시간+개별 종료 버튼), arm_reach는 이미 1개 이상이면
   시작 버튼 위에 경고 캡션(5절 "알려진 제약 2" 반영). HDF5는 이미
   1개 실행 중이면 시작 버튼 자체가 `disabled=True`(경고가 아니라
   원천 차단, 5절 "알려진 제약 1" 반영 — 실행 중이면 저장완료 감지 시
   `st.warning()`까지 추가로 표시).
4. **하단 접힘**(`st.expander`): 실행 중인 job들의 로그 경로 목록 +
   `config/settings.yaml` 전체를 `st.json()`으로 표시.

### 6-3. 부수 발견 및 수정: `process_manager._pid_alive()`가 좀비를 못 걸렀음

라이브 검증 중(6-4절의 "sim 정지" 단계) `ps aux`로 우연히 발견 —
`⏹ 환경 종료` 버튼을 눌러 bringup을 정지시켰는데, 프로세스가 실제로는
종료됐지만 `ps`에 `[ros2] <defunct>`(좀비)로 남아 있었다.

**원인**: `_pid_alive()`가 `os.kill(pid, 0)`만으로 생존을 판단했는데,
**좀비 프로세스도 `os.kill(pid, 0)`에 예외 없이 성공한다**(직접 실측
확인: `os.kill(zombie_pid, 0)` → 성공). 좀비는 "이미 종료했지만 부모가
아직 `wait()`로 거두지 않은" 상태이므로, `_pid_alive()`가 이걸
"살아있음"으로 잘못 판단하면 — `stop_job()`의 목적 자체는 lock 파일을
무조건 지우므로(2절에서 이미 그렇게 구현) 이번 케이스처럼 사용자가 보는
결과에는 문제가 없었지만 — **`_all_locks()`의 stale lock 정리(2-1절,
"timeout이 아니라 PID 생존 여부로 판단")가 원래 막으려 했던 정확히 그
문제(좀비가 되도록 방치된 채 lock이 영원히 안 지워지는 경우)를 못 걸러낼
수 있는 구조적 결함이었다.

**수정**: `_pid_alive()`가 `os.kill()` 전에 먼저
`os.waitpid(pid, os.WNOHANG)`로 거두기를 시도하도록 고쳤다 — 우리
프로세스의 직계 자식(`start_job()`이 항상 그렇게 띄움)이고 좀비 상태면
이 호출이 그 자리에서 확정적으로 거둬서 죽음을 확인하고, 아직 살아있으면
`(0, 0)`을 즉시 반환(비차단)해 기존 `os.kill()` 체크로 자연스럽게
넘어간다. 우리 자식이 아니면(`ChildProcessError`) 기존 방식으로 폴백.

**검증**: streamlit을 재기동해 고친 코드로 sim을 다시 시작→종료 →
`ps -p <pid>` 결과 아무것도 안 뜸(좀비 없이 완전히 사라짐) 확인.

**추가로 발견한 한계(정직하게 기록)**: 이 수정은 `_pid_alive()`가 **그
자식을 실제로 소유한 프로세스 안에서** 호출될 때만 거두기가 된다.
검증 중 정리용으로 별도 스크립트(`/tmp/cleanup_nodes.py`)에서
`stop_node()`를 호출했더니, 신호는 정상적으로 전달돼 프로세스는 죽었지만
(그 스크립트의 자식이 아니므로) 거두기는 안 돼 좀비 5개가 남았다 — 이건
버그가 아니라 예상된 동작(그 스크립트 입장에서 그 pid는 자기 자식이
아니므로 거둘 권한/자격이 없음). 실제 서비스에서는 `process_manager`
호출이 항상 streamlit이라는 하나의 지속 프로세스 안에서만 일어나므로 이
경로를 안 탄다. 그래도 안전망이 있는지 확인했다: 그 5개 좀비를 남긴 채
streamlit 프로세스 안에서 **새 job을 하나 시작**(`start_bringup`)했더니
그 좀비 5개가 전부 사라졌다 — CPython `subprocess` 모듈이 새
`Popen()`을 만들 때마다 이전에 자신이 추적하던, 아직 안 거둬진 자식들을
내부적으로 정리(`_cleanup()`)하는 동작 덕분(라이브로 재현·확인, 문서
추측 아님). 즉 최악의 경우에도 다음 job 시작 시점에 저절로 청소된다.

### 6-4. 라이브 검증 결과 (실제 웹 UI 클릭, Playwright로 조작 + 스크린샷)

스크린샷 도구: `so101_web/.claude/skills/dashboard-screenshot`(Node +
Playwright 1.62.1, 전역 chromium 캐시 재사용)의 패턴을 그대로 따라
스크래치 디렉터리에 별도로 만들어 사용(`interact.js`: 지정한 텍스트의
버튼을 클릭 후 스크린샷, 클릭 없이 로드만도 가능). Streamlit은 WSL2 안에서
`streamlit run pages/main.py --server.port 8503`으로 띄웠고, **WSL2의
localhost 포워딩 덕분에 Windows 쪽 Playwright가 `localhost:8503`으로
그대로 접근 가능함을 직접 확인**(추측 아님, `Invoke-WebRequest`로 200
확인 후 진행). 스크린샷 파일 자체는 검증 후 삭제(스크래치 산출물, 저장소에
남기지 않음 — so101_web 스킬의 동일 관례를 따름).

| 단계 | 조작 | 확인 결과 |
|---|---|---|
| sim 시작 | `🟢 sim` 클릭 | UI: mode=sim, PID 표시. `ps -p <pid>` 실제 프로세스와 **정확히 일치**. |
| sim 종료 | `⏹ 환경 종료` 클릭 | 프로세스 사라짐(좀비 버그는 여기서 처음 발견 → 6-3절 수정 → 재검증) |
| slam 시작/종료 | 각각 클릭 | mode=slam, PID 일치. 종료 후 좀비 없음(수정 반영 후). |
| nav 시작 | `🧭 nav` 클릭 | mode=nav, PID 일치. **상단 CTA가 파랑→"환경이 준비됐습니다..."로 전환, 인식·수집 카드 4개 전부 헤더가 주황(warning)→파랑(action)으로 전환** — mode 게이팅이 실제로 동작함을 확인. |
| YOLO 2개 동시 시작 | `▶ YOLO 인식 새로 시작` 2회 클릭 | "현재 2개 실행 중" + PID 2개(30116, 30019) 각각 표시 → `ps -p 30116,30019`로 **둘 다 실제 프로세스임을 확인**. |
| arm_reach 경고 | `▶ 팔 리치 새로 시작` 1회 클릭 | 1개 실행 중이 되자마자 "⚠️ 이미 1개가 실행 중입니다 — arm_reach를 여러 개 동시에 켜면 팔이 서로 다른 목표를 두고 경합할 수 있습니다..." 경고가 시작 버튼 **위에** 표시됨(요청사항 그대로 재현). |
| HDF5 중복 차단 | `▶ HDF5 수집 시작` 1회 클릭 후 재확인 | 1개 실행 중이 되자 시작 버튼이 `disabled`로 바뀜. Playwright로 `isDisabled()` → **true**, 강제 클릭 시도 → **actionability 타임아웃으로 클릭 자체가 안 됨**(진짜로 막혀 있음을 프로그램적으로 증명). `ps aux \| grep hdf5_collector_node \| wc -l`로 프로세스 수(2, `ros2 run`+실제 노드) 클릭 전후 **불변** 확인. |
| explore 시작/종료 | 각각 클릭 | mode=explore, PID 일치. 정상 종료. |
| 전체 정리 | 4개 노드 + bringup 종료 | `ps aux`에 관련 프로세스 0개, `state/locks/` 빈 디렉터리, HDF5 테스트로 인한 `~/semih_datasets/` 잔여 파일 없음(정지 시점에 저장할 데이터가 없었음) 확인. |

### 6-5. semiH_ws 무변경 확인

이번 세션 전체(6-3절 버그 수정 포함, 라이브 사이클 10회 이상) 동안
`find /home/jangjunseo/semiH_ws -newer CLAUDE.md`가 매번 빈 결과 —
semiH_ws 소스는 한 번도 건드리지 않았다.

---

## 7. 브랜딩 및 스플래시 화면 (2026-08-18)

`logo.png`(`semiH_web/logo.png`, 다색 꽃 로고 + "AI CAMPUS" 텍스트, 150x150,
남색 배경 — 존재 확인 후 사용, 플레이스홀더 생성 안 함)를 이용해 공통
헤더와 스플래시 화면을 추가했다. **새 프로세스 관리/상태 판단 로직은
전혀 건드리지 않았다** — `core/`, `monitoring/`은 이번 세션에서 아예 열지
않았고, 수정한 건 새 파일 `lib/ui_components.py`와 `pages/main.py`의
`main()` 함수 앞부분뿐이다.

### 7-1. "매번 vs 세션당 1회" 판단 (구현 전 제안)

**제안한 대로 "매번"(재방문/새로고침마다 다시 표시)으로 구현했다.**
이유: semiH_web은 데이터 수집/학습처럼 "이어서 하던 작업으로 복귀"하는
개념이 없는 제어 대시보드다 — 새로고침은 대부분 "새로 세션을 시작한다"는
사용자 의도에 가깝고, 스플래시가 매번 짧게(~1초) 뜨는 게 브랜딩 노출
측면에서도 자연스럽다고 판단했다. 반대 근거(세션당 1회)는 "환경이 이미
떠 있는 도중에 실수로 새로고침했을 때 스플래시가 다시 뜨는 게 거슬릴 수
있다"인데, 스플래시는 배경 상태와 무관하게 항상 동일하게(1초) 뜨고 그
뒤엔 원래 상태 그대로 이어지므로 실질적 방해는 적다고 봤다.

구현은 so101_web과 동일하게 `st.session_state`로 게이팅한다
(`"splash_shown" not in st.session_state`) — 페이지 안에서 버튼을 눌러
`st.rerun()`이 여러 번 일어나도(같은 세션 유지) 매 클릭마다 다시 뜨지는
않고, 실제 새로고침/재접속(새 세션)일 때만 다시 뜬다. 이 구분이 실제로
그렇게 동작하는지는 추측하지 않고 7-3절에서 라이브로 확인했다.

### 7-2. 구현

- **`lib/ui_components.py`**(신규): so101_web `dashboard/lib/
  ui_components.py`의 `render_brand_header()`/`render_splash_screen()`을
  실제로 읽고 그대로 재사용. semiH_web은 페이지가 `pages/main.py` 하나뿐
  이지만 so101_web처럼 페이지 파일 밖(`lib/`)에 둬서 페이지가 늘어나도
  재사용 가능한 구조로 만들었다.
  - `render_brand_header()`: 로고 36px + "DAPIER", 카드 role 색상표
    (action/settings/success/warning)와 분리된 영역 — so101_web과 동일한
    이유(로고가 다색이라 배경색 블록으로 만들면 "카드 색은 의미 고정"
    원칙과 충돌).
  - `render_splash_screen()`: `position:fixed` 전체 화면 덮개, 배경색
    `#051644`(로고 파일 네 모서리 픽셀 PIL로 직접 실측 — 임의 추정
    아님, so101_web과 같은 값이 나온 건 같은 브랜드 남색을 공유해서로
    보임). 로고 72px → "DAPIER" 19px → 태그라인 "SLAM, NAV, YOLO까지
    한 번에" 36px bold, 세로 중앙 정렬 — 요청 범위(60~80/18~20/32~40px)
    안에 이미 들어맞는 so101_web 크기를 그대로 썼다.
- **`pages/main.py`**: `main()` 맨 앞에 `st.set_page_config(...,
  page_icon=ui_components.get_logo_icon())` + 스플래시 게이팅 블록 +
  `render_brand_header()` 호출 추가. 그 외 기존 카드 렌더링 로직은 전혀
  손대지 않음.

### 7-3. 라이브 검증

Playwright로 실제 브라우저 스크린샷 확인(스크린샷 파일 자체는 검증 후
삭제 — so101_web 스킬과 동일한 스크래치 관례).

- **버스트 캡처**(새 브라우저 컨텍스트 = 새 세션으로 접속 직후 250ms
  간격 연속 스크린샷): 1098~1407ms 구간에 스플래시(남색 배경 + 로고 +
  DAPIER + 태그라인)가 처음 나타났고, 2618~3106ms 구간에 본 페이지로
  전환됨을 확인 — **스플래시가 실제로 화면에 떠 있던 시간은 대략
  1.7~2초로, 코드의 `time.sleep(1.0)` 값보다 길었다.** 원인은 코드
  버그가 아니라 Streamlit 자체의 재실행 왕복 지연(서버가 `st.rerun()`을
  호출한 뒤 클라이언트가 그 신호를 받아 두 번째 스크립트 실행 결과를
  받아 그리기까지 걸리는 시간) + 이 WSL2 환경의 기존에 문서화된 CPU
  스케줄링 지연(semiH_ws/CLAUDE.md 섹션 1 A-3)으로 추정된다 — so101_web도
  동일한 메커니즘(`time.sleep(1.0)` + `st.rerun()`)을 쓰므로 같은 지연이
  있을 것으로 보이나 so101_web 쪽에서 별도로 실측한 기록은 없다. **이
  차이를 감추지 않고 정직하게 기록한다** — "1초 근처에서 자연스럽게
  전환"이라는 요청 기준으로 보면 허용 범위로 판단했지만, 정확히 1.0초는
  아니다.
- **실제 새로고침(F5) 시 재표시 확인**: 본 페이지가 뜬 상태에서
  `page.reload()`(Playwright의 진짜 브라우저 새로고침, 새 컨텍스트가
  아니라 같은 탭)를 실행 → 900ms 시점 스크린샷에 스플래시 텍스트가 다시
  나타남을 확인(`getByText(...).count()` → 1). **"매번 새로고침마다
  다시 뜬다"는 7-1절 설계가 실제로 동작함을 라이브로 증명** — 추측이
  아니라 진짜 리로드로 확인.
- **레이아웃 확인**: 헤더(로고+DAPIER+구분선)가 페이지 최상단, 제목보다
  위에 페이지 폭 전체로 걸쳐 있고, 그 아래 환경 카드(4개 mode 버튼)/
  인식·수집 카드(YOLO/target_3d/arm_reach/HDF5) 전부 이전과 동일하게
  렌더링됨을 전체 페이지 스크린샷으로 확인 — 헤더 추가로 인한 레이아웃
  깨짐 없음.

### 7-4. semiH_ws 무변경 확인

이번 세션 동안(Streamlit 라이브 실행 여러 회) `find /home/jangjunseo/
semiH_ws -newer CLAUDE.md`가 매번 빈 결과. `core/`, `monitoring/`
디렉터리는 이번 세션에서 아예 읽지도 않았다.

### 7-5. 참고: 검증 중 발견한 관련 없는 프로세스

라이브 검증 시작 전 `ps aux`에서 이번 세션이 띄우지 않은 `ign gazebo`
프로세스 2개(PID 32120, 32382, 부모가 `-bash`)를 발견했다 — lock 파일이
없어 `process_manager`가 관리하는 게 아니고, 별도 터미널에서 수동으로
띄운 것으로 보인다. 이번 작업과 무관하다고 판단해 건드리지 않았고
(작업 시작 전/후 PID·상태 그대로임을 확인), semiH_web 자체 검증은
Streamlit만 띄워서(브링업 없이) 진행했다 — 이 작업이 UI 레이어뿐이라
백엔드 상태가 필요 없었기 때문이기도 하다.

### 7-6. 스플래시 노출 시간 조정: 1.0초 → 1.5초 (2026-08-18)

`pages/main.py`의 `time.sleep(1.0)` → `time.sleep(1.5)` 한 줄만 수정.
`lib/ui_components.py`(헤더/스플래시 렌더링 자체), `session_state` 게이팅
로직, 그 외 어떤 것도 건드리지 않았다.

**재검증 시 발견한 사실**: 이번 세션 시작 시 `ps aux`에 이 작업과 무관한
Streamlit 프로세스가 이미 하나 떠 있었다(PID 33622, `pts/2`에 붙은
포그라운드 프로세스, 기본 포트 8501 사용 중 — 사용자가 직접 실행해둔
것으로 보임). 이 프로세스는 전혀 건드리지 않았고, 검증용 인스턴스는
포트 8503으로 별도로 띄워서 겹치지 않게 했다. 종료할 때도 8503 PID만
정확히 골라 껐고, 8501 인스턴스는 작업 전후 계속 그대로 살아있음을
확인했다.

**재측정 결과** (버스트 캡처, 새 세션 접속 직후 250ms 간격 연속 스크린샷):

| 시점 | 상태 |
|---|---|
| 253ms | 백지(아직 최초 페인트 전) |
| 1284ms | 스플래시 표시 중 |
| 2787ms | 스플래시 여전히 표시 중 |
| 3091ms | 본 페이지로 전환 완료 |

즉 스플래시가 실제로 화면에 떠 있던 구간은 대략 1~1.3초 지점부터
2.8~3.1초 지점까지, **약 1.8~2.1초**로 측정됐다 — 코드 값 1.5초보다
여전히 더 길다. 1.0초였을 때(7-3절)의 실측 노출 시간(~1.7~2초)과
비교하면 **+0.5초 조정이 실측 노출 시간에는 대략 +0.1~0.4초 정도만
반영**된 것으로 보인다(오차 범위가 넓어 정확한 비례관계는 확정하지
못함) — 이는 앞서 7-3절에서 이미 지목한 대로 Streamlit 재실행 왕복
지연과 이 WSL2 환경의 CPU 스케줄링 지연이 매 측정마다 변동성 있게
얹히기 때문으로 추정된다. **목표치(1.5초)에 맞춰 억지로 보정하지 않고
실측값을 그대로 기록한다.**

**재확인**: `page.reload()`(실제 브라우저 새로고침) → 1300ms 시점
스크린샷에 스플래시 텍스트 여전히 존재 확인 — 1.5초로 늘어난 뒤에도
"매번 새로고침마다 다시 뜬다"는 7-1절 동작은 그대로 유지됨. 전체 페이지
스크린샷으로 헤더/환경 카드/인식·수집 카드 레이아웃도 여전히 정상임을
재확인.

**semiH_ws 무변경**: 이번 수정·재검증 전체 동안 `find /home/jangjunseo/
semiH_ws -newer CLAUDE.md`가 매번 빈 결과.

---

## 8. RViz 버튼 · 기능 설명 토글 · 처음 사용 가이드 (2026-08-28)

사용자가 웹 UI로 `nav` mode를 켰는데 로봇이 안 움직여 문의한 것을 계기로
발견한 실제 UX 결함 3가지에 대한 응답이다: (1) Nav2는 목표를 받아야
움직이는데 웹에 목표를 줄 방법(RViz)이 없었음, (2) `explore` 같은 mode
이름이나 `arm_reach` 같은 노드 이름만으로는 기능을 추측하기 어려움, (3)
처음 쓰는 사람이 최종적으로 "자율주행 + 객체 인식"을 경험하기까지 어떤
순서로 버튼을 눌러야 하는지 안내가 없었음.

### 8-1. RViz는 왜 필요한가 (근거 재확인)

`nav2.launch.py`에 rviz 노드가 없고(`grep -rn "rviz"` 결과 없음),
`bringup.launch.py`가 부르는 include 목록(`sim`/`slam`/`nav2`) 어디에도
RViz가 없음을 소스로 재확인했다. `display.launch.py`(RViz 포함)는 있지만
`robot_state_publisher`/`joint_state_publisher_gui`를 자체적으로 새로
띄우는 **독립 URDF 뷰어**라 bringup과 같이 켜면 `robot_state_publisher`가
중복돼 충돌한다 — 그래서 이걸 재사용하지 않고, `rviz2` 단독 프로세스를
새로 감싸는 쪽을 택했다.

### 8-2. 구현: `core/launch_runner.py`에 RViz 3종 함수 추가

`start_rviz()`/`stop_rviz()`/`get_rviz_status()`를 `build_bringup_command()`
와 동일한 패턴(setup.bash 2개 source 후 `exec`)으로 추가했다. **새 자원
태그는 만들지 않았다** — `process_manager.STAGE_RESOURCES["rviz"] = []`
(빈 리스트, `gazebo_instance`와 무관 — RViz와 Gazebo가 동시에 떠 있는 게
정상적인 사용 형태이므로 자원 충돌 개념 자체가 없음). 고정 stage 이름
하나만 쓰므로 두 번째 시작 시도는 `hdf5_collector`와 동일하게
`JobConflictError`로 막힌다(여러 개 띄울 이유가 없다고 판단).

`config/nav_view.rviz`(신규 파일, semiH_ws 무관)를 만들어 `-d`로
넘긴다 — Map/LaserScan/RobotModel/TF 디스플레이와 Fixed Frame=map,
"Set Goal" 툴을 미리 채워서 빈 화면으로 열리지 않게 했다. **라이브 검증**:
`timeout 6 rviz2 -d config/nav_view.rviz`로 직접 로드 — OpenGL 초기화
성공, YAML/디스플레이 클래스 관련 오류 없이 6초간 정상 실행 후 SIGTERM으로
정상 종료 확인. 이어서 `core/launch_runner.start_rviz()`를 웹 레이어 안
거치지 않고 직접 호출해 재검증: `ps -o pid,ppid,pgid`로 pid==pgid(자체
세션 리더) 확인 → `stop_rviz()` 호출 → 1초 안에 프로세스 완전히 사라짐
(그룹 SIGINT만으로 충분, bringup처럼 긴 graceful timeout이 불필요함을
실측 확인 — 셸 래퍼도 없고 단일 GUI 프로세스라 nav2 lifecycle 같은 복잡한
종료 시퀀스가 없기 때문으로 보임) → `state/locks/`도 빈 상태로 복귀,
로그(`logs/jobs/rviz_*.log`)에 에러 없음.

`monitoring/topic_watcher.get_system_status()`가 반환하는 dict에
`"rviz": {"running": bool, "pid": int|None}` 키를 추가했다 — 기존
`bringup`/`nodes` 조회 방식과 동일하게 `launch_runner.get_rviz_status()`를
그대로 통과시킬 뿐, 이 모듈이 직접 lock을 읽는 코드는 추가하지 않았다
(4절 원칙 유지).

### 8-3. 기능 설명 토글: `st.popover`로 구현

"토글을 누르면 텍스트 설명이 나온다"는 요청을 `st.popover("❓ ...")`로
그대로 구현했다 — 클릭 전엔 안 보이다가 클릭하면 설명이 뜨는 UX가
정확히 popover의 동작이라, 별도 `st.session_state` 토글 로직을 새로
만들지 않았다. mode 버튼 4개(`_MODE_DESCRIPTIONS`)와 노드 카드 4개
(`_NODE_DESCRIPTIONS`), RViz 카드(`_RVIZ_DESCRIPTION`) 전부에 달았다.
설명 문구는 각 mode/노드가 **파이프라인에서 서로 어떤 순서·의존관계**에
있는지("YOLO가 먼저 켜져 있어야 target_3d가 의미 있는 값을 냄" 등)까지
포함해, 개별 카드 하나만 봐도 왜 이 순서로 켜야 하는지 알 수 있게 했다.

`AppTest`(streamlit.testing.v1)로 헤드리스 실행해 9개 버튼(mode 4 +
RViz 1 + 노드 4)이 전부 렌더링되고 예외가 없음을 확인했다 — 실제 브라우저
Playwright 검증은 이번엔 생략했다(chromium 캐시가 이 머신에 없어 새로
받아야 하는데, AppTest가 이미 스크립트 실행 단계의 오류(잘못된 변수 참조,
`st.popover` 호출 오류 등)는 전부 잡아내므로 이번 변경 규모에는 충분하다고
판단 — 픽셀 단위 레이아웃 검증까지는 하지 않았다는 뜻, "확인 안 됨"으로
정직하게 남긴다).

### 8-4. "팔 리치" → "팔 뻗기 (인식한 물체로 이동)"로 개명

`_NODE_DISPLAY[NodeName.ARM_REACH]`의 표시 라벨만 바꿨다 — `NodeName.
ARM_REACH`라는 내부 enum 값, `ros2 run` 실행 파일 이름(`arm_reach_node`)
등 semiH_ws/코드 레벨 식별자는 전혀 건드리지 않았다(표시 텍스트 하나만
바뀐 것). 나머지 3개 노드명("YOLO 인식", "3D 좌표 추출", "HDF5 데이터
수집")은 이미 기능이 이름에서 드러난다고 판단해 그대로 뒀다.

### 8-5. "처음 사용 가이드" 탭: 다중 페이지가 아니라 `st.tabs()`

1-4절에서 확정한 "단일 페이지" 결정을 깨지 않기 위해, `st.navigation()`
다중 페이지 파일(`pages/2_가이드.py` 등)이 아니라 **같은 `pages/main.py`
안에서 `st.tabs(["🎛 제어판", "📖 처음 사용 가이드"])`** 로 구현했다 — 엔트리
포인트/URL은 여전히 하나뿐이고, 기존 카드 렌더링 함수들은 전부 그대로
`render_control_tab()`으로 옮겨 담기만 했다(내부 로직 변경 없음).

가이드 탭은 새 프로세스 조작을 전혀 하지 않는 **정적 안내문**이다 — 실제
버튼은 전부 제어판 탭에 있고, 가이드 탭은 "환경(nav/explore 켜기) →
(nav라면) RViz로 목표 주기 → YOLO → 3D 좌표 추출 → 팔 뻗기 순서로 인식
켜기 → (선택) HDF5 수집 → 최종 확인 방법" 5단계로 정리했다. 각 단계
설명에서 8-1~8-2절의 "RViz가 왜 필요한가", 8-3절의 "왜 이 순서로 켜야
하는가"를 다시 요약해, 가이드 탭만 봐도(또는 제어판 탭 각 카드의 ❓만
봐도) 같은 결론에 도달하도록 이중화했다.

### 8-6. semiH_ws 무변경 확인

이번 세션 전체(rviz2 라이브 실행 2회, 헤드리스 streamlit 기동 1회, AppTest
실행 다수) 동안 `find /home/jangjunseo/semiH_ws -newer CLAUDE.md`가 매번
빈 결과. `core/`, `monitoring/`, `pages/`, `config/` 전부 semiH_web 자체
파일만 수정했다.

### 8-7. 배포 직후 사용자가 실제로 겪은 문제 2건과 수정 (2026-08-28, 같은 날 후속)

8절 배포 직후 사용자가 실제 화면에서 `KeyError: 'rviz'`와 popover UI가
크고 어색하다는 것 두 가지를 보고했다.

**`KeyError: 'rviz'` 원인**: 사용자가 이미 띄워둔 streamlit 프로세스는
Streamlit의 파일 감시가 **엔트리 스크립트(`pages/main.py`) 자체**는 변경
시 항상 다시 읽어 실행하지만, 그 스크립트가 `import`하는 **하위 모듈**
(`monitoring/topic_watcher.py` 등)은 `sys.modules`에 이미 캐시돼 있어서
프로세스를 완전히 재시작하지 않는 한 이전 버전이 계속 쓰인다 — 그 결과
`pages/main.py`는 (새 코드라) `status["rviz"]`를 참조하는데, 정작 그
`status`를 만드는 `topic_watcher.get_system_status()`는 (구 코드가 캐시된
채라) 아직 `"rviz"` 키를 안 넣고 있어서 `KeyError`가 났다 — 코드 버그가
아니라 **개발 중 핫리로드의 구조적 한계**(하위 모듈 변경은 프로세스 전체
재시작이 필요)였다. 사용자에게는 `streamlit run`을 완전히 껐다 다시
켜도록 안내했다.

이 클래스의 실패가 다시 나도 화면 전체가 죽지 않도록, `render_rviz_card()`
/ `render_meta_footer()` / `main()`의 `status["rviz"]` 접근을 전부
`status.get("rviz", ...)` 기본값 fallback으로 바꿨다(방어적 코드 — 근본
해결책은 여전히 "모듈 수정 후엔 프로세스 재시작"이라는 점은 그대로).

**popover UI 축소**: "❓ 기호 없애고, 사각형 테두리를 버튼 정도 크기로
줄여달라"는 요청 그대로 두 가지를 고쳤다:
- 모든 popover 라벨에서 `❓` 이모지를 뗐다("❓ 설명"/"❓ 이 기능은
  무엇인가요?" → 전부 "설명"으로 통일).
- mode 버튼 옆 popover에 걸려 있던 `use_container_width=True`를
  제거했다 — 이게 mode 버튼과 같은 너비로 popover 트리거를 늘려서 버튼
  아래 큰 사각형 두 개가 겹쳐 보이던 원인이었다. 제거하면 popover
  트리거가 라벨 텍스트("설명")만큼만 차지하는 기본 크기로 줄어든다(노드
  카드 쪽 popover는 원래 `use_container_width`를 안 썼으므로 라벨만
  "설명"으로 줄였다).

`streamlit.testing.v1.AppTest`로 재실행해 예외 없음과 9개 버튼이 그대로
렌더링됨을 재확인했다(Playwright 실브라우저 검증은 8-3절과 동일한 이유로
이번에도 생략).

### 8-8. 같은 원인 재발: `AttributeError: ... has no attribute 'start_rviz'`

8-7절과 **완전히 동일한 근본 원인**(하위 모듈 `sys.modules` 캐시)이 이번엔
`core.launch_runner`에 대해 재현됐다 — `KeyError: 'rviz'`는 데이터(dict
키)가 없어서 난 에러였고, 이번엔 함수 자체가 없어서 `AttributeError`가
난 것뿐, 원인 구조는 같다(사용자가 재시작을 놓쳤거나, 재시작한 프로세스가
아닌 다른 프로세스/탭을 보고 있었을 가능성).

**근본 해결책(재시작)은 코드로 대신할 수 없다** — Python의 import 캐싱
자체가 원인이라 애플리케이션 코드 수준에서 "항상 최신 모듈을 쓰게" 만들
수는 없다. 대신 `main()` 맨 앞에 `_check_stale_module_cache()`를 추가해,
`hasattr(launch_runner, "start_rviz")`로 이번 세션에서 새로 추가된
속성이 실제로 있는지 확인하고 없으면 원인·조치를 명확한 한국어 문장으로
안내한 뒤 `st.stop()`으로 멈춘다 — 똑같은 원인의 다음 에러(예: 앞으로
또 새 함수를 추가했는데 재시작을 안 한 경우)를 알아보기 힘든 파이썬
트레이스백 대신 사용자가 바로 조치할 수 있는 문구로 먼저 잡아내기 위함.

**라이브 검증**: `del core.launch_runner.start_rviz`로 이 상황을 직접
재현한 뒤 `AppTest`로 실행 — 예외 없이 `st.error()` 메시지 1개만
렌더링되고 이후 카드들은 렌더링되지 않음(`st.stop()`이 정상 작동)을
확인했다. 정상 상태(속성 있음)에서는 이 체크가 아무것도 안 하고
통과함도 재확인.

**사용자에게 전달한 조치**: 지금 떠 있는 streamlit 프로세스를 다시 한번
완전히 종료(Ctrl+C) 후 재실행할 것 — 브라우저 새로고침/재접속이 아니라
**터미널의 프로세스 자체**를 재시작해야 한다는 점을 강조.

---

## 9. Gazebo GUI 창이 WSLg에서 안 뜨는 문제 — 조치 (2026-08-29)

사용자가 웹 UI의 환경 버튼(mode)을 눌렀을 때 Gazebo 창이 화면에 안 뜨고
Windows 작업표시줄 아이콘만 남으며, 그 아이콘/창 영역에 마우스를 올리면
"copy mode" 커서가 나온다고 보고했다. RViz도 같은 증상을 낼 수 있다.

### 9-1. 진단 (라이브 확인)

- 버튼·프로세스 자체는 정상이었다. `ros2 launch ... bringup.launch.py
  headless:=false`가 뜨고 `ign gazebo gui`(pid 확인)도 살아 있었으며,
  `DISPLAY=:0 xwininfo -root -tree`에 `0x600012 "Gazebo"
  ("ign-gazebo-gui" "Gazebo GUI") 1200x1000` 창이 정상 크기로 존재했다 —
  **top-level 창은 만들어지는데 그 안에 실제 렌더 child가 없다**(RViz가
  정상일 때는 `rviz2` top-level 밑에 `933x796` 렌더 패널 child가 생기는
  것과 대조).
- `/mnt/wslg/weston.log`: `appId: Gazebo GUI WindowId:0x2` /
  `Client: ClientGetAppidReq: pid:<gui> appId:Gazebo GUI` 로 창이
  **아이콘 전용으로만 등록**되고, 바로 다음 줄에
  `!!!cursor role is added after creation - WindowId:0x1`. 사용자가 본
  "copy mode" 커서 = 이 cursor-role 충돌로 weston-rdprail이 그 창 위에서
  Windows drag-copy 커서를 표시하는 것.
- 표면 증상: WSLg에서 GL이 llvmpipe(소프트웨어)로만 동작한다. **RViz
  (Ogre1/GLX)는 소프트웨어 렌더로도 창이 뜬다**(8/28 로그에서 GL 4.5로
  맵 렌더 확인, 이번에도 `1400x900` 창 + `933x796` 렌더 패널 + 라이다
  프레임 수신까지 재확인). **Gazebo GUI(Ogre2)는 소프트웨어 렌더 + WSLg
  조합에서 창 표시 자체가 깨진다**(작업표시줄 아이콘만). `gui_env`를 gui
  프로세스에 정상 주입해도(pid environ 확인) 소용없다.

### 9-1b. 근본 원인 재조사 — Mesa 문제가 아니라 GPU-PV 채널 붕괴

이전 세션은 "mesa 23.2.1 d3d12 초기화 실패 → `ppa:kisak/kisak-mesa`로
Mesa 업그레이드"로 결론냈으나, 2026-08-29 재조사에서 **둘 다 틀렸음**을
확인했다:

- **kisak-mesa PPA(jammy)는 폐기된 빈 저장소다.** `add-apt-repository
  ppa:kisak/kisak-mesa` + `apt update`를 해도 `apt-cache policy
  libgl1-mesa-dri`에 새 버전이 안 뜨고 "All packages are up to date"만
  나온다. 확인: 그 저장소 `InRelease`의 해시 목록에서
  `main/binary-amd64/Packages` 크기가 **0바이트**
  (`d41d8cd98f00b204e9800998ecf8427e` = 빈 파일 MD5). apt는 InRelease만
  받고 인덱스 타깃을 0개 만든다(`apt-get indextargets | grep kisak` →
  없음). 서명·키·네트워크는 전부 정상(`gpgv: Good signature`) — 저장소가
  비어 있을 뿐.
- **진짜 원인은 Mesa(유저스페이스)보다 아래, WSL2 ↔ Windows GPU
  가상화(GPU-PV) 채널이다:**
  - `/dev/dri` 렌더 노드가 **아예 없다**(`/dev/dri/renderD128` 부재) →
    Mesa d3d12 gallium 드라이버가 GBM 디바이스를 못 만든다
    (`eglinfo -p gbm` → `eglInitialize failed`).
  - `dmesg`: `misc dxg: dxgk: dxgkio_query_adapter_info: Ioctl failed:
    -22`(EINVAL) / `-2`(ENOENT), `dxgkio_reserve_gpu_va: -75`(EOVERFLOW)
    — `/dev/dxg` 커널 드라이버가 Windows 호스트 DirectX 커널과 어댑터
    질의 단계부터 실패.
  - `/mnt/wslg/stderr.log`: `Xwayland glamor: GBM Wayland interfaces not
    available` → `Failed to initialize glamor, falling back to sw`.
    즉 Xwayland 자체가 소프트웨어 모드라, 모든 X11 앱이 llvmpipe.
  - GPU는 **NVIDIA RTX 5050(Blackwell)**. `nvidia-smi`: `NVIDIA-SMI
    595.68` / `Driver Version: 596.13` — WSL 쪽 런타임과 Windows
    드라이버 버전 불일치. 결정적으로 `/usr/lib/wsl/lib/libd3d12core.so`
    /`libd3d12.so`가 **2023-10-20 빌드**(`libdxcore.so`는 2024-03-31) —
    Blackwell + 596.13 드라이버에는 너무 오래된 D3D12 매핑 레이어.
  - `/mnt/c/Users/USER/.wslconfig` 없음, 커스텀 커널 핀 없음. 커널은
    `6.18.33.2-microsoft-standard-WSL2`(2026-06-18 빌드, MS 정식). 즉
    WSL 런타임 라이브러리(`/usr/lib/wsl/lib/*`)가 ~2년 방치돼
    `wsl --update`가 오래 안 됐을 가능성이 크다.
- **결론**: 이 계층은 `apt`/Mesa로 못 고친다. Windows 쪽에서
  `wsl --update`(커널 + `/usr/lib/wsl/lib/*` 런타임 갱신) + NVIDIA
  드라이버 클린 재설치 + `wsl --shutdown`이 필요하다. 그 뒤 GPU-PV가
  살아나면(`/dev/dri` 생성, `glxinfo -B` renderer가 `D3D12 (NVIDIA
  GeForce RTX 5050)`) `qtwayland5` 설치 + `gui_env`를
  `QT_QPA_PLATFORM: "wayland;xcb"`로 교체하면 Gazebo GUI가 뜬다.

### 9-2. 취한 조치

1. **`pages/main.py` 환경 카드에 "Gazebo GUI 창도 띄우기 (headless 끄기)"
   체크박스 추가.** 기존에는 `start_bringup(mode, headless=False)`로
   **항상** Gazebo GUI를 강제했다. 이제 기본은 headless(체크 해제)이고,
   `headless=not show_gui`로 넘긴다. 기본값은
   `config/settings.yaml`의 `bringup.default_headless`(→ `true`로 변경)를
   읽어 그 반대를 초기 체크 상태로 쓴다. 체크를 켜면 "창이 안 뜨면 끄고
   RViz로 보라"는 캡션이 함께 표시된다. `AppTest`로 예외 없음 + 체크박스
   렌더 + 토글 동작 확인.
2. **`config/settings.yaml`**: `bringup.default_headless: false → true`
   (근거 주석 포함). `gui_env` 위 주석을 재조사 결과(GPU-PV 채널 붕괴,
   kisak PPA 폐기)로 교체하고, 블록 아래에 "GPU-PV 복구 후
   `QT_QPA_PLATFORM: wayland;xcb` 한 줄로 교체" 주석 블록을 둠. 기존
   소프트웨어 렌더 gui_env 5줄은 그대로 둠(복구 전까지의 rviz2 안정화용
   이고, 지우면 xcb 폴백 로그 노이즈만 늘어남).
3. **`scripts/upgrade_mesa_wslg.sh`를 진단 스크립트로 재작성**(원래는
   kisak PPA 업그레이드 스크립트였으나 그 PPA가 빈 저장소로 확인돼 폐기).
   이제 sudo 없이 실행하는 순수 진단: `/dev/dri` 유무 / `dmesg`의 dxgk
   ioctl 실패 / Xwayland glamor sw 폴백 / `glxinfo -B` 렌더러 /
   `libd3d12core.so` 빌드 연도 5개를 `[OK]`/`[FAIL]`로 출력하고, 파일
   상단 주석에 Windows 쪽 조치 순서(`wsl --update` → NVIDIA 드라이버
   클린 설치 → `wsl --shutdown` → 재진단 → `qtwayland5` + `gui_env` 교체)를
   적었다. 이 세션에서 실행 → 5개 중 4개 `[FAIL]`(dmesg 항목은 root 없이
   못 읽어 `[..]`), 예상과 일치.

### 9-3. 검증 결과

1. 막혀 있던 explore bringup(이전 세션 잔여, pid 554) → `stop_bringup()`
   으로 정리, `ps`에 관련 프로세스 0개 확인.
2. `start_bringup(SIM, headless=True)` → `ign gazebo ... -s -r`(server만,
   `-s` 플래그) 확인, `ign gazebo gui` 프로세스 **안 뜸** — 깨지는 GUI
   경로를 안 타는 것 확인.
3. 그 위에서 `start_rviz()` → `xwininfo`에 `1400x900` RViz 창 + `933x796`
   렌더 패널 child 확인, 로그에 GL 4.5 + 라이다 프레임 수신(헤드리스 sim
   에서). "아이콘만 뜸/copy mode" 증상 없음 — headless + RViz가 현재
   시각화 경로로 정상 동작함을 라이브 확인.
4. `stop_rviz()` + `stop_bringup()` → 수 초 내 관련 프로세스 전부 소멸,
   `state/locks/` 빈 디렉터리로 복귀.

### 9-4. 사용자에게 전달한 조치 / 남은 것

- **지금 당장**: streamlit 프로세스를 완전히 재시작(터미널 Ctrl+C 후
  재실행 — 하위 모듈 캐시 때문, 8-8절과 동일). 그러면 환경 카드가 기본
  headless로 동작하고, 시각화는 "RViz 열기"로 한다. nav 목표도 RViz의
  "2D Nav Goal"로 준다.
- **Gazebo 창까지 살리려면 (Windows 쪽, 이 세션에서 불가):**
  1. Windows PowerShell(관리자): `wsl --update` (Store 버전 아니면
     `wsl --update --web-download`), `wsl --version`로 갱신 확인.
  2. Windows NVIDIA 드라이버를 최신으로 **클린 설치**(RTX 5050 = Blackwell).
  3. Windows PowerShell: `wsl --shutdown` → WSL 터미널 새로 열기.
  4. `bash scripts/upgrade_mesa_wslg.sh` 재실행 → 전부 `[OK]`인지 확인
     (특히 `/dev/dri/renderD128` 생성, `glxinfo -B` renderer가
     `D3D12 (NVIDIA GeForce RTX 5050)`).
  5. `sudo apt install qtwayland5` → `config/settings.yaml`의 `gui_env`를
     주석 블록대로 `QT_QPA_PLATFORM: "wayland;xcb"` 한 줄로 교체 →
     streamlit 완전 재시작 → 환경 카드 "Gazebo GUI 창도 띄우기" 체크 후
     mode 시작.

### 9-5. semiH_ws 무변경 확인

이번 세션 전체(bringup/rviz 라이브 사이클, AppTest 다수) 동안
`find /home/jangjunseo/semiH_ws -newer CLAUDE.md`가 매번 빈 결과.
수정한 파일은 `pages/main.py`, `config/settings.yaml`,
`scripts/upgrade_mesa_wslg.sh`(신규), 이 `CLAUDE.md`뿐이다.

---

## 10. YOLO 인식 결과 창 자동 열기 (rqt_image_view) + WSLg 업데이트 후속 (2026-08-31)

### 10-0. 배경: WSLg 업데이트로 GUI 창 표시 자체는 해결됨 (GPU 가속은 여전히 X)

9절의 Windows 쪽 조치를 사용자가 실제로 수행했다: `wsl --update`
(2.7.10 → 2.7.12 → `--pre-release` 2.9.9.0, 커널 6.18.33 → 6.18.40,
WSLg 1.0.73 → 1.0.79), NVIDIA 드라이버 클린 재설치, Windows Update
(빌드 26200.9168 → 26200.9278), 완전 재부팅 수 회.

**결과 (라이브 재확인):**
- **연산(CUDA) 스택은 갱신됨**: `nvidia-smi` 595.68/CUDA 13.2 →
  **615.65 / KMD 616.56 / CUDA 13.4**. `C:\Windows\System32\lxss\lib\`의
  `libcuda*`/`libnvidia-*`가 2026-08-21 자로 새로 깔림.
- **그래픽(D3D12) 경로는 여전히 안 됨**: `/dev/dri` 없음, `glxinfo -B`
  렌더러 `llvmpipe`(소프트웨어) 그대로, `dmesg`의
  `dxgk: dxgkio_query_adapter_info: Ioctl failed: -22` 부팅·런타임 모두
  그대로. `libd3d12core.so`/`libd3d12.so`는 여전히 **2023-10-20** 빌드
  (이건 NVIDIA가 아니라 MS가 WSLg 시스템 이미지 `/gpu_lib_packaged:
  /gpu_lib_inbox` 오버레이로 제공하는 D3D12-on-Linux 매핑 레이어이고,
  `wsl --update`로도, `--pre-release`로도, `wsl --version`의
  `Direct3D 버전: 1.611.1-81528511`이 한 번도 안 바뀜). WSLg
  `versions.txt`: `DirectX-Headers v1.608.0`, 내부 mesa 23.1.0.
  → **현재 배포된 가장 최신 WSLg조차 이 조합(RTX 5050 Blackwell +
  AMD 780M 하이브리드)에서 D3D12 어댑터 열거에 실패**한다. `apt`/Mesa로
  못 고치는 계층이라는 9-1b절 결론 유지. 아직 안 해본 것: HAGS 강제 ON,
  하이브리드에서 NVIDIA dGPU를 WSL VM에 강제(NVIDIA 제어판 전역 설정 /
  Windows 그래픽 기본값 고성능 / BIOS dGPU-only) — 안내는 했으나 사용자가
  "이미 Gazebo·RViz 둘 다 뜬다"고 해서 중단.
- **그러나 창 표시 버그(9절의 "작업표시줄 아이콘만", cursor role 충돌)는
  사라졌다**: 사용자가 웹 UI에서 mode를 켜니 **Gazebo GUI 창이
  1908x1107 전체 크기로 정상 생성**됨(`xwininfo`에
  `0x600012 "Gazebo" ("ign-gazebo-gui" "Gazebo GUI") 1908x1107` 확인).
  `ign gazebo gui`는 소프트웨어 렌더라 CPU ~560%(server ~285%)로 무겁지만
  창은 뜨고 렌더된다 — WSLg 1.0.79가 9절의 Ogre2 창-미표시 버그를 고친
  것으로 보인다(정확한 수정 커밋까지는 미확인, "확인 안 됨"). **9절의
  headless 기본값(`bringup.default_headless: true`)과 "Gazebo GUI 창도
  띄우기" 체크박스는 그대로 둔다** — 소프트웨어 렌더 부하가 커서 기본
  headless가 여전히 맞고, 창이 필요하면 체크 한 번으로 뜬다.

### 10-1. 요청: "YOLO 버튼 누르면 인식 결과 창도 자동으로 떠야"

YOLO 노드는 `/yolo/image_annotated`(박스·라벨 그려진 BGR 이미지,
`results[0].plot()`)로만 결과를 내보내고 창을 스스로 안 띄운다. 웹에서
"YOLO 인식 새로 시작"을 누르면 그 영상 창도 같이 뜨게 해달라는 요청.

**모델 한계 (사용자에게 고지함, 코드로 못 바꿈)**: `yolov8n.pt`는
COCO 80종 학습 모델(`confidence_threshold: 0.4`)이라 sim 월드의 물체가
그 80종처럼 생겨야 박스가 뜬다. 박스 색은 Ultralytics 기본값이 클래스별
자동 배정이라 "빨간 박스" 고정이 아니다. "가까이 가면"이 트리거가 아니라
신뢰도 0.4를 넘으면 그려지는 것(가까울수록 신뢰도가 올라 결과적으로 잘
잡히긴 함).

### 10-2. 구현: RViz 래퍼와 완전히 같은 패턴으로 image_view 3종 추가

`rqt_image_view`가 이미 설치돼 있고(`/opt/ros/humble/lib/rqt_image_view/`),
`--help`로 **positional `topic` 인자**(`topic  The topic name to subscribe
to`)를 받아 그 토픽을 미리 물린 채 여는 것을 확인함.

- **`core/launch_runner.py`**: `start_image_view(topic="/yolo/image_annotated")`
  / `stop_image_view()` / `get_image_view_status()` + `build_image_view_command()`
  추가. `build_rviz_command()`와 동일하게 setup.bash 2개 source 후
  `exec ros2 run rqt_image_view rqt_image_view <topic>`. 고정 stage
  `"image_view"`, `extra_meta={"topic": topic}`, `env_overlay=_gui_env()`.
  두 번째 시작 시도는 `JobConflictError`(rviz·hdf5_collector와 동일한
  "고정 stage = 하드 락").
- **`core/process_manager.py`**: `STAGE_RESOURCES["image_view"] = []`
  (bringup/rviz와 자원 안 겹침 — 셋 다 동시에 떠 있는 게 정상). 이게
  유일한 process_manager 변경이고, `.get(stage, [])`라 사실 없어도
  동작하지만 rviz 선례대로 명시적으로 등록.
- **`monitoring/topic_watcher.py`**: `get_system_status()` 반환 dict에
  `"image_view": {"running": bool, "pid": int|None, "topic": str|None}`
  키 추가 — `launch_runner.get_image_view_status()`를 그대로 통과시킬
  뿐(rviz와 동일, 직접 lock 안 읽음).
- **`pages/main.py`**:
  - YOLO 카드에만 시작 버튼 위 체크박스 **"인식 결과 창(rqt_image_view)도
    함께 열기"**(기본 켜짐, blocked면 disabled). 켜져 있으면
    `node_runner.start_node(YOLO)` 성공 직후 `launch_runner.start_image_view()`
    를 호출하되 `JobConflictError`는 삼킨다(이미 창이 있으면 그대로 둠).
  - YOLO 카드 하단에 `_render_yolo_viewer_row()`: 창이 떠 있으면
    `🖼 인식 결과 창 실행 중 · PID · topic` + `⏹ 창 닫기`, 없으면
    `🖼 인식 결과 창 열기`(수동). 창은 고정 stage라 YOLO 인스턴스가
    여러 개여도 뷰어는 1개.
  - `_check_stale_module_cache()`가 `start_rviz`뿐 아니라
    `start_image_view`까지 확인하도록 확장(8-8절과 같은 하위 모듈 캐시
    문제 대비 — 이 변경도 사용자가 streamlit 프로세스를 **완전히
    재시작**해야 반영됨).
  - `render_meta_footer()` 로그 경로 목록 + `main()` 자동 새로고침 조건에
    `image_view` 추가. 가이드 탭 3단계와 YOLO 설명 popover에 창 자동
    열림 문구 추가.

### 10-3. 검증

1. **단위**: `build_image_view_command()`/커스텀 토픽/`STAGE_RESOURCES`/
   `get_system_status()` 키 — 파이썬으로 직접 확인.
2. **AppTest**(headless): 예외 0개, 체크박스 "인식 결과 창... 함께 열기"
   + 버튼 렌더 확인(사용자 세션이 살아있어 bringup=explore·YOLO 인스턴스가
   떠 있는 상태로도 정상 렌더).
3. **라이브** (웹 안 거치고 `core/launch_runner` 직접 호출, 사용자의
   기존 bringup/rviz/YOLO 락은 건드리지 않음): `start_image_view()` →
   `ros2 run`(pid 5546) + 실제 `rqt_image_view`(pid 5563) 같은 pgid,
   `xwininfo`에 `0x1400006 "rqt_image_view__ImageView - rqt" 1908x1107`
   창 생성 확인(child 43% CPU — 소프트웨어 렌더지만 2D 블릿이라 Gazebo
   560%보다 훨씬 가벼움) → 두 번째 `start_image_view()`는
   `JobConflictError` → `stop_image_view()` → 두 프로세스 전부 소멸,
   `state/locks/image_view.json` 정리, 좀비 0개, 로그에 에러 없음(SIGINT
   핸들러 줄만).

### 10-4. semiH_ws 무변경 확인

이번 세션 전체(9절 Windows 조치 재진단 + rqt_image_view 라이브 사이클 +
AppTest) 동안 `find /home/jangjunseo/semiH_ws -newer CLAUDE.md`가 매번
빈 결과. 수정 파일: `core/launch_runner.py`, `core/process_manager.py`,
`monitoring/topic_watcher.py`, `pages/main.py`, 이 `CLAUDE.md`.
