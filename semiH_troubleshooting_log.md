# semiH 프로젝트 — Gazebo 시뮬레이션 환경 구축 트러블슈팅 로그

**대상 범위**: WSL2 환경 구축 → URDF 제작 → Gazebo 스폰 → 차동구동 → ros2_control → 카메라/라이다 센서 → SLAM(slam_toolbox)까지
**최종 환경**: WSL2 (Ubuntu 22.04) + ROS2 Humble + Gazebo Fortress

---

## 1. 환경 선정: Humble + Harmonic → Humble + Fortress로 전환

<details>
<summary>요약: Gazebo Harmonic으로 시작했으나, ros2_control 연동 패키지가 Humble을 공식 지원하지 않아 Fortress로 전면 전환</summary>

**증상**
```
Library [...] does not export any plugins. The symbol [GzPluginHook] is missing
Failed to load system plugin: (Reason: No plugins detected in library)
Requested plugin name: [gz_ros2_control::GazeboSimROS2ControlPlugin]
```

**원인 규명 과정**
1. 처음엔 `filename` 오타로 의심 → 실제 설치된 `.so` 파일명(`libgz_ros2_control-system.so`)과 대조해 오타 아님을 확인
2. `gz_ros2_control` 공식 리포지토리의 지원 조합표 확인 결과:
   - **Humble → 공식 지원은 Fortress만** (`ros-humble-ign-ros2-control` / `ros-humble-gz-ros2-control` 패키지 모두 Fortress 대상 빌드)
   - Harmonic 지원은 Iron/Jazzy/Rolling부터
3. `iron` 브랜치를 소스에서 직접 빌드 시도 → 컴파일 에러 발생:
   ```
   'HardwareInfo' has no member named 'hardware_plugin_name'
   ```
   → Humble의 `ros2_control` 코어는 해당 필드명이 `hardware_class_type` (Iron 이후 개명됨). 필드명 패치 후 재빌드 성공.
4. 그러나 재실행 시 **동일한 GzPluginHook 에러 재발** → 원인 재분석 결과, `iron` 브랜치 자체가 여전히 옛 Ignition API(`ignition::gazebo::v6`, `IGNITION_ADD_PLUGIN` 매크로)를 대상으로 작성되어 있어 Harmonic(`gz::sim`, `GZ_ADD_PLUGIN`)과 근본적으로 호환 불가능함을 확인.

**최종 결정**: 브랜치 패치를 반복하는 대신, **Gazebo를 Fortress로 다운그레이드**하여 공식 지원 조합으로 전환. `sudo apt install ros-humble-ros-gz ros-humble-gz-ros2-control`로 바이너리 설치, 소스 빌드 불필요.

**교훈**: 비공식 조합(Humble+Harmonic)은 개별 패키지 단위로 지원 여부가 다르다 — `ros_gz_sim`/`ros_gz_bridge`는 비공식 조합도 웬만큼 동작하지만, `ros2_control` 연동처럼 저수준 API에 직접 의존하는 패키지는 버전 간 API 변경(필드명, 네임스페이스, 플러그인 매크로)에 훨씬 취약함.

</details>

---

## 2. Fortress 전환에 따른 네이밍 규칙 변경

<details>
<summary>요약: Harmonic(gz-sim)과 Fortress(Ignition) 사이의 플러그인 파일명/네임스페이스 표기 규칙 차이</summary>

| 구분 | Harmonic | Fortress |
|---|---|---|
| 실행 명령 | `gz sim` | `ign gazebo` |
| DiffDrive 플러그인 | `gz-sim-diff-drive-system` / `gz::sim::systems::DiffDrive` | `libignition-gazebo-diff-drive-system.so` / `ignition::gazebo::systems::DiffDrive` |
| JointStatePublisher | `gz-sim-joint-state-publisher-system` | `libignition-gazebo-joint-state-publisher-system.so` |
| `gz_ros2_control` 플러그인 | 동일 (`gz_ros2_control-system`) | **버전 무관 — 변경 없음** |
| 라이다 센서 하위 태그 | `<lidar>` (신버전 별칭) | `<ray>` (원래 표기) |

**교훈**: 튜토리얼/문서를 참고할 때 반드시 해당 문서가 어느 Gazebo 버전 기준인지 확인. Harmonic 문서의 코드를 Fortress에 그대로 쓰면 `filename` 인식 실패로 이어짐.

</details>

---

## 3. ROS2 ↔ Gazebo 트랜스포트는 별개 — 브릿지 없이는 아무것도 안 통함

<details>
<summary>요약: /cmd_vel을 ros2 topic pub으로 보내도 Gazebo가 못 받는 문제 → ros_gz_bridge 필요성 학습</summary>

**증상**: 로봇은 정상 스폰되고 플러그인도 정상 로드됐는데, `ros2 topic pub /cmd_vel ...`을 보내도 로봇이 안 움직임.

**원인**: ROS2 토픽과 Gazebo 내부 트랜스포트(gz transport)는 완전히 별개의 pub/sub 시스템. `ros_gz_bridge`의 `parameter_bridge` 노드가 둘 사이를 명시적으로 이어줘야 함.

**적용한 해결**: launch 파일에 `parameter_bridge` 노드 추가, 방향 표기 학습:
- `/topic@ROS_TYPE]GZ_TYPE` → ROS2 → Gazebo (예: cmd_vel)
- `/topic@ROS_TYPE[GZ_TYPE` → Gazebo → ROS2 (예: camera, scan)

같은 이유로 카메라(`/camera/image_raw`), 라이다(`/scan`) 모두 센서 자체는 Gazebo 쪽에서 정상 동작해도 브릿지 노드 없이는 `ros2 topic list`에 아예 안 나타남.

</details>

---

## 4. `robot_description` 파라미터 YAML 파싱 에러

<details>
<summary>요약: URDF 문자열에 포함된 특수문자(대시 등)를 launch_ros가 YAML로 오인 파싱</summary>

**증상**
```
Unable to parse the value of parameter robot_description as yaml.
```

**원인**: `Command(['xacro ', xacro_path])`로 생성한 robot_description 문자열이 특정 문자(콜론, 대시 등 — 플러그인 추가 이후 발생)를 만나면 launch_ros가 YAML로 해석 시도하다 실패.

**해결**: `ParameterValue(Command(...), value_type=str)`로 명시적으로 문자열 타입임을 지정.

</details>

---

## 5. 카메라 센서: 토픽은 생기는데 Gazebo가 실제로 렌더링을 안 함

<details>
<summary>요약: 월드에 Sensors 시스템 플러그인이 없어서 카메라/라이다 자체가 동작하지 않았음</summary>

**증상**: `/camera/image_raw` 토픽은 생성되지만 `ign topic -l`에서 조회 시 아예 목록에 없거나 데이터가 안 흐름.

**원인**: 물리엔진(Physics)과 별개로, 카메라·라이다처럼 렌더링이 필요한 센서는 **월드 레벨에 `ignition-gazebo-sensors-system` 플러그인이 명시적으로 로드**되어 있어야 함. 기본 제공 `empty.sdf`에는 이 플러그인이 빠져 있음.

**해결**: 커스텀 월드 SDF(`semih_world.sdf`) 작성, 아래 플러그인들을 명시적으로 포함:
```xml
<plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```
Physics, UserCommands, SceneBroadcaster, Contact 플러그인도 함께 필요.

**추가로 겪은 오해**: 위 조치 후 데이터는 정상 흘렀지만, `rqt_image_view`에서 아무 특징 없는 단색 그라데이션만 보여서 "또 문제가 생겼나" 오인. 실제로는 **월드에 무늬 없는 회색 바닥 하나뿐이라 진짜로 볼 게 없어서** 생긴 정상적인 결과였음. 빨간 테스트 박스를 추가하고 나서야 정상 렌더링임을 시각적으로 확정.

**미해결로 남긴 것**: Gazebo 내장 "Image Display" GUI 플러그인이 메뉴에 안 보이는 문제 — 우선순위상 보류, 팀원 인수인계 예정.

</details>

---

## 6. 라이다: 메시지 타입은 표준인데 프레임 이름이 안 맞음

<details>
<summary>요약: Gazebo가 자체 명명 규칙으로 센서 프레임 이름을 붙여서 URDF의 링크 이름과 불일치</summary>

**증상**: `/scan` 데이터는 정상 수신되는데 RViz에 아무것도 안 보임. RobotModel 디스플레이에는 `No transform from [X] to [lidar_link]` 에러.

**원인**: `/scan`의 실제 `frame_id`가 `semih/base_link/lidar` (Gazebo 내부 명명 규칙 — 모델명/링크명/센서명 조합)인데, URDF/TF 트리에는 `lidar_link`라는 이름만 존재.

**해결**: 물리적으로 동일 위치이므로, `tf2_ros static_transform_publisher`로 `lidar_link` → `semih/base_link/lidar` identity TF 발행하여 우회.

</details>

---

## 7. 바퀴 관절 TF 누락

<details>
<summary>요약: 팔 관절은 ros2_control이 TF를 커버하지만, 차동구동 플러그인이 제어하는 바퀴는 별도 처리 필요</summary>

**증상**: RViz에서 `No transform from [left_wheel_link] to [lidar_link]` 등 에러.

**원인**: 팔 관절(`arm_joint1~3`)은 `joint_state_broadcaster`가 `/joint_states`를 발행해 TF가 자동 생성되지만, 바퀴는 `ros2_control`에 안 걸려있고 Gazebo의 DiffDrive 플러그인이 직접 제어 — 바퀴 각도 정보가 ROS2로 넘어온 적이 없었음.

**해결 과정**:
1. `ign topic -l | grep joint` → `/world/semih_world/model/semih/joint_state` 토픽 발견 (타입: `ignition.msgs.Model`)
2. 이 타입은 `ros_gz_bridge`의 **CLI 간단 문법으로는 브릿지 불가** (ROS/Gazebo 양쪽 토픽 이름이 다르기 때문) → YAML 설정 파일 방식으로 전환
3. `ignition::msgs::Model` ↔ `sensor_msgs/msg/JointState`가 공식 지원 매핑임을 확인, YAML로 브릿지 구성

</details>

---

## 8. SLAM이 지도를 전혀 안 그림 (가장 근본적인 문제)

<details>
<summary>요약: odom 토픽은 있었지만 실제 TF(odom→base_link)가 발행된 적이 없었음 — DiffDrive 플러그인 옵션 누락</summary>

**증상 진행 과정**
1. `slam_toolbox` 실행은 되는데 RViz에서 `Frame [map] does not exist`, `No map received`
2. 1차 의심: `base_frame` 기본값이 `base_footprint`인데 우리 URDF엔 `base_link`만 존재 → 커스텀 파라미터 YAML로 `base_frame: base_link` 지정
3. `slam_params_file`(정확한 launch 인자명 — 처음엔 `params_file`로 잘못 시도했다가 무시됨) 적용 후에도 **동일 문제 재발**
4. 근본 원인 확인: `ros2 run tf2_ros tf2_echo odom base_link` → **`odom` 프레임 자체가 TF 트리에 없음**. `/odom` 토픽(nav_msgs/Odometry)은 정상 발행 중이었지만, 이건 TF가 아니라 메시지일 뿐 — SLAM은 TF를 봄.
5. DiffDrive 플러그인 공식 문서 확인 결과, `<tf_topic>`, `<frame_id>`, `<child_frame_id>` 옵션이 있고, 이걸 안 넣으면 TF를 아예 안 냄.

**해결**:
```xml
<odom_topic>odom</odom_topic>
<tf_topic>/model/semih/tf</tf_topic>
<frame_id>odom</frame_id>
<child_frame_id>base_link</child_frame_id>
```
추가 후, `ignition.msgs.Pose_V` ↔ `tf2_msgs/msg/TFMessage` 브릿지를 YAML에 추가하여 `/tf`로 연결.

**교훈**: "토픽이 존재한다"와 "TF가 존재한다"는 완전히 다른 확인 대상. Nav2/SLAM 계열 스택은 대부분 메시지가 아니라 TF 트리를 기준으로 동작하므로, odom 관련 문제 진단 시 `ros2 topic echo /odom`이 아니라 **`ros2 run tf2_ros tf2_echo <parent> <child>`로 먼저 확인**하는 게 훨씬 빠른 진단 경로였음.

</details>

---

## 9. Nav2 플러그인 pluginlib 네임스페이스 표기 불일치 (반복 패턴)

<details>
<summary>요약: 여러 Nav2 서버에서 콜론(::) 표기를 슬래시(/) 표기로 하나씩 고쳐야 했음</summary>

**증상**: `planner_server`, `behavior_server`가 각각 다른 시점에 `FATAL: ... does not exist. Declared types are ...`로 종료.

**원인**: `nav2_params.yaml`에 플러그인 이름을 C++ 네임스페이스 표기(`nav2_navfn_planner::NavfnPlanner`, `nav2_behaviors::Spin`)로 적었는데, Nav2의 pluginlib 조회는 **슬래시 표기**(`nav2_navfn_planner/NavfnPlanner`, `nav2_behaviors/Spin`)를 사용. 에러 로그 마지막 줄에 실제 등록된 이름 목록이 항상 그대로 나와 있어서, 로그만 끝까지 읽으면 바로 고칠 수 있었음.

**교훈**: 하나 고치고 재시도하면 다음 서버에서 같은 유형의 에러가 또 나는 패턴이 반복됨 — Nav2는 lifecycle_manager가 노드를 순차적으로 configure하기 때문에, 앞 노드의 에러부터 순서대로 드러남. 한 번에 전체 파일을 다 검토하기보다 순차적으로 잡아나가는 게 오히려 각 에러 메시지가 주는 정답(Declared types 목록)을 활용하기 좋음.

</details>

---

## 10. bt_navigator: 빈 문자열로 지정한 BT XML 경로

<details>
<summary>요약: default_nav_to_pose_bt_xml을 빈 문자열로 잘못 지정해서 "Empty Tree" 에러 발생</summary>

**증상**: `Behavior tree threw exception: Empty Tree. Exiting with failure.`

**원인**: 파라미터 파일에 `default_nav_to_pose_bt_xml: ""`로 명시적으로 빈 문자열을 넣어둠. 이 값이 비어있지 않고 "빈 문자열"이라는 값 자체로 들어가버려서, bt_navigator가 실제 존재하는 기본 BT XML을 못 찾음.

**해결**: 해당 파라미터 자체를 파일에서 삭제 → bt_navigator가 자체 내장 기본 경로를 자동으로 사용.

**교훈**: "값을 안 정하고 싶다"는 의도라면 파라미터 자체를 빼야지, 빈 문자열/None 같은 placeholder를 넣으면 안 됨 — 대부분의 ROS2 노드는 "파라미터 없음"과 "파라미터가 빈 값"을 다르게 처리함.

</details>

---

## 11. "off the global costmap" — 버그가 아니라 미탐색 영역이었던 경우

<details>
<summary>요약: SLAM 지도가 아직 좁을 때 미탐색 방향으로 목표를 보내서 생긴 정상적인 실패</summary>

**증상**: `compute_path_to_pose` 액션이 `ABORTED`, `poses: []`. 로그: `The goal sent to the planner is off the global costmap.`

**진단 과정**: Claude Code로 `~/.ros/log`의 실제 노드 로그를 분석한 결과, 실패한 목표가 로봇 시작 지점에서 겨우 몇 cm 떨어진, 아직 SLAM이 한 번도 스캔하지 못한 방향의 좌표였음이 확인됨. teleop으로 로봇을 충분히 이동시켜 지도를 넓힌 뒤 동일한 시도에서는 에러가 재발하지 않음.

**교훈**: 에러 메시지만 보고 "설정이 잘못됐다"고 단정하기 전에, 애초에 그 시점에 지도/코스트맵이 그 좌표를 포함할 만큼 충분히 탐색됐는지부터 확인해야 함. RViz에서 회색(탐색됨) 영역 안쪽만 클릭하는 습관이 필요.

</details>

---

## 12. WSLg 자체 다운 — 개별 앱 문제로 오인하기 쉬움

<details>
<summary>요약: Gazebo, RViz 등 여러 GUI 프로그램이 동시에 창을 못 띄우면 WSLg 자체 문제</summary>

**증상**: Gazebo, RViz 둘 다 실행은 되는데(터미널 로그는 정상) 창 자체가 안 뜸.

**진단**: `xeyes` 같은 최소 테스트 앱으로도 창이 안 뜨는지 확인 → 안 뜨면 개별 앱이 아니라 WSLg(디스플레이 서버) 자체가 죽은 것.

**추정 원인**: `pkill -9`로 Gazebo 관련 프로세스를 여러 차례 강제 종료한 이력과 관련 있을 가능성 (그래픽 세션이 비정상 종료되며 꼬였을 것으로 추정, 확정은 아님).

**해결**: PowerShell에서 `wsl --shutdown` 후 재시작 — 껐다 켜면 해결됨.

**교훈**: 여러 프로그램이 동시에 같은 증상(창 안 뜸)을 보이면, 각 프로그램을 따로 디버깅하기 전에 `xeyes` 같은 최소 테스트로 공통 원인(디스플레이 서버 자체)부터 배제하는 게 시간을 아낀다.

</details>

---

## 13. cv_bridge의 numpy 호환성 버그 (KeyError: 16)

<details>
<summary>요약: cv_bridge.cv2_to_imgmsg()가 특정 numpy/opencv 버전 조합에서 내부 타입 매핑에 실패</summary>

**증상**:
```
KeyError: 16
File ".../cv_bridge/core.py", line 276, in cv2_to_imgmsg
    if self.cvtype_to_name[self.encoding_to_cvtype2(encoding)] != cv_type:
```

**시도 1 (부분 해결)**: YOLO의 `results[0].plot()`이 반환하는 배열이 메모리상 비연속(non-contiguous)일 수 있어 `np.ascontiguousarray(..., dtype=np.uint8)`로 강제 — 이것만으로는 에러가 완전히 사라지지 않음.

**최종 해결**: `cv_bridge.cv2_to_imgmsg()` 자체를 우회. `sensor_msgs/msg/Image` 메시지를 직접 생성해서 `height`, `width`, `encoding`, `step`, `data` 필드를 numpy 배열로부터 수동으로 채워 발행:
```python
out_msg = Image()
out_msg.header = msg.header
out_msg.height = annotated.shape[0]
out_msg.width = annotated.shape[1]
out_msg.encoding = 'bgr8'
out_msg.is_bigendian = 0
out_msg.step = annotated.shape[1] * annotated.shape[2]
out_msg.data = annotated.tobytes()
```

**교훈**: `cv_bridge`는 사실 매우 얇은 변환 유틸일 뿐이라, 문제가 생기면 라이브러리 내부를 고치려 하기보다 메시지를 직접 구성하는 우회로가 더 빠르고 확실한 경우가 많음.

</details>

---

## 14. 팔 사거리 부족 — "버그처럼 보이는 물리적 한계"

<details>
<summary>요약: IK 계산은 정상인데, 팔이 너무 작아서 매번 "한계까지 뻗은 자세"만 나옴</summary>

**증상**: `arm_reach_node` 로그에 매번 `joint2≈89°, joint3≈-90°`(관절 한계값)만 반복되고, Gazebo 화면에서 팔이 움직이는 걸 체감하기 어려움.

**원인**: 팔의 최대 사거리(0.25m, 이후 0.40m로 확장)에 비해, 월드에 배치한 인식 대상 물체들이 로봇에서 1~3m씩 떨어져 있었음. 목표가 항상 사거리 밖이라 "클램핑된 최대 뻗은 자세"만 계산된 것 — IK 로직 자체의 결함이 아니라 배치 문제.

**진단 방법**: 실제 시뮬레이션으로 반복 확인하는 대신, **IK 계산 함수만 떼어내서 가상의 좌표(예: "30cm 앞, 15cm 높이")로 단위 테스트**를 돌려봄 → 이 경우 89/90 고정값이 아니라 상황에 맞는 중간값이 나오는 것으로 "로직 자체는 정상"임을 빠르게 확정.

**교훈**: 통합 테스트(전체 파이프라인을 실제로 돌려보기)에서 이상한 결과가 반복될 때, 원인이 "로직 버그"인지 "환경/배치 문제"인지 구분하려면, **문제되는 함수만 떼어내서 알고 있는 입력값으로 단위 테스트**하는 게 훨씬 빠르다. 시뮬레이션 전체를 매번 다시 돌리며 눈으로 확인하는 것보다 훨씬 저렴한 진단 비용.

</details>

---

## 15. 깊이 카메라 센서 존재 여부 — 실행해서 확인하는 게 검색보다 빠름

<details>
<summary>요약: Fortress에 depth_camera 센서 지원 여부를 문서로 확정 못 했지만, 실행해서 바로 확인됨</summary>

문서(웹 검색)로는 Fortress의 `depth_camera` 센서 타입 지원 여부를 100% 확정하지 못했음 (Harmonic 자료만 명확히 있었음). 대신 **URDF에 추가하고 바로 실행해서 `ign topic -l | grep depth`로 확인**하는 방법으로 몇 분 만에 "지원됨"을 확정.

**교훈**: 오늘 세션 내내 반복된 패턴이지만, Gazebo 버전별 기능 지원 여부는 문서 검색보다 **직접 실행 후 `ign topic -l`로 확인**하는 게 훨씬 빠르고 확실한 경우가 많다.

</details>

---

## 16. HDF5 동기화 수집 실패 — CPU 과부하 재발 (완전히 해결되지 않은 채로 넘어감)

<details>
<summary>요약: 여러 프로세스가 동시에 돌면서 특정 토픽(/odom, /scan)만 발행이 멈추는 현상 재발, 오늘 두 번째로 겪은 CPU 병목</summary>

**증상**: `ApproximateTimeSynchronizer`로 4개 토픽(카메라, 라이다, 관절, odom)을 동기화하는 노드에서 10초간 프레임이 0개 수집됨. 진단 중 `/camera/image_raw`, `/joint_states`는 정상 응답했지만 `/odom`, `/scan`은 `ros2 topic echo`에 아무 응답도 없었음.

**진단 과정**: `ps aux`로 Gazebo 프로세스의 CPU 사용률을 보니 서버/GUI 각각 300~400%대로 비정상적으로 높음. 오늘 Nav2 때(`Behavior Tree tick rate exceeded`)와 같은 유형의 문제로 판단.

**미해결로 남긴 부분**: 이번에도 정확한 근본 조치(headless 실행, 불필요 프로세스 정리 등)는 실제로 적용/검증하지 않고, "CPU 문제일 가능성이 높다"는 진단만 내린 채 다음 세션으로 넘김. 수집 노드/검증 스크립트 코드 자체는 완성했지만, 실제 라이브 데이터로 검증하는 것은 숙제로 남음.

**교훈**: WSL2 환경에서 여러 무거운 프로세스(Gazebo 렌더링 + 여러 브릿지 + SLAM/Nav2/데이터수집 노드)를 동시에 돌리는 건 이 프로젝트 전체에서 반복적으로 병목 지점이 됨. 다음에는 애초에 **Gazebo GUI를 headless(`-s` 옵션)로 띄우는 걸 기본값으로 삼는 것**을 고려할 가치가 있음.

</details>

---

## 전체 교훈 요약 (최종 업데이트)

11번 요약에 이어:

12. **통합 테스트에서 이상 현상이 반복되면, 의심되는 함수만 떼어내 알고 있는 입력으로 단위 테스트할 것.** 전체 시스템을 매번 재현하며 눈으로 확인하는 것보다 압도적으로 빠르다.
13. **Gazebo 버전별 센서/기능 지원 여부는 검색보다 직접 실행 후 `ign topic -l`로 확인하는 게 더 빠르고 확실하다.**
14. **WSL2에서 여러 무거운 ROS2/Gazebo 프로세스를 동시에 돌리면 CPU 병목이 반복적으로 발생한다.** 특정 토픽만 선택적으로 응답이 없어지는 현상은 버그보다 리소스 경합의 신호일 가능성이 높다 — 의심되면 headless 실행이나 불필요 프로세스 정리부터 시도.
15. **"검증까지 끝내야 완료"와 "코드/파이프라인 자체는 완성"은 구분해서 기록해두는 게 좋다.** 오늘 세션 다수의 항목(Nav2 목표도달, 팔 통합 데모, HDF5 실측 검증)이 후자 상태로 남았고, 이는 명확한 다음 세션 시작점이 된다.

