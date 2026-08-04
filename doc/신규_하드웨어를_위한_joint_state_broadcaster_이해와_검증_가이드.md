# 신규 하드웨어를 위한 joint_state_broadcaster 이해와 검증 가이드

## 1. 문서 목적

이 문서는 새로운 로봇 하드웨어를 제작하는 교육생이
`joint_state_broadcaster`를 단순히 "`/joint_states`를 발행하는 노드"로만
외우지 않고, 실제 센서값이 ROS 2 메시지가 되기까지의 전체 경로를 이해하도록 돕는다.

학습 목표는 다음과 같다.

1. `joint_state_broadcaster`와 hardware interface의 역할을 구분한다.
2. URDF, `<ros2_control>`, controller YAML이 각각 무엇을 정의하는지 설명한다.
3. 실제 장치의 센서값이 `/joint_states`로 나오는 경로를 추적한다.
4. 새로운 관절과 state interface를 안전하게 추가한다.
5. 이름, 단위, 부호, 배열 순서 오류를 단계별로 검증한다.
6. `/joint_states`가 발행된다는 사실만으로 하드웨어가 정상이라고 판단하지 않는다.

기준 환경은 ROS 2 Jazzy와 `ros2_control`이다.

---

## 2. 가장 중요한 결론

`joint_state_broadcaster`는 모터나 센서를 직접 읽지 않는다.

실제 장치와 통신하는 것은 hardware plugin이고,
`joint_state_broadcaster`는 hardware plugin이 갱신한 state interface를 읽어
ROS 메시지로 변환한다.

```text
실제 모터·엔코더·센서
  │
  │ 장치 프로토콜
  ▼
hardware plugin의 read()
  │
  │ state interface 갱신
  ▼
joint_state_broadcaster
  │
  ├─ /joint_states
  └─ /dynamic_joint_states
```

따라서 `/joint_states`가 이상할 때 broadcaster만 확인해서는 안 된다.
값이 만들어지는 출발점인 hardware plugin의 `read()`와 단위 변환부터 확인해야 한다.

---

## 3. 구성 요소별 역할

| 구성 요소 | 역할 | 실제 장치와 직접 통신 |
|---|---|---|
| 일반 URDF joint | 링크 연결, joint 종류, 축, 제한값 정의 | 아니요 |
| `<ros2_control>` joint | 사용 가능한 state/command interface 선언 | 아니요 |
| hardware plugin | 장치 상태를 읽고 명령을 전송 | 예 |
| controller manager | hardware와 controller의 lifecycle 및 주기 관리 | 간접 관리 |
| joint_state_broadcaster | state interface를 ROS 상태 토픽으로 발행 | 아니요 |
| robot_state_publisher | URDF와 `/joint_states`로 TF 계산 | 아니요 |
| RViz | TF와 모델을 시각화 | 아니요 |

### 3.1 broadcaster라는 이름의 의미

여기서 broadcaster는 DDS broadcast나 TF broadcaster라는 뜻이 아니다.

`joint_state_broadcaster`는 `ros2_control` controller plugin의 한 종류이며,
하드웨어 state interface를 ROS 토픽으로 공개하는 역할을 한다.

TF를 발행하는 주체는 `robot_state_publisher`다.

```text
joint_state_broadcaster → /joint_states
robot_state_publisher   → /tf, /tf_static
```

---

## 4. OpenManipulator-X의 실제 연결 구조

OpenManipulator-X의 controller 설정에는 다음 항목이 있다.

```yaml
controller_manager:
  ros__parameters:
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
```

파일:

```text
open_manipulator_bringup/
└─ config/open_manipulator_x/hardware_controller_manager.yaml
```

이 설정의 의미는 다음과 같다.

```text
controller 인스턴스 이름:
  joint_state_broadcaster

로드할 C++ plugin 타입:
  joint_state_broadcaster/JointStateBroadcaster
```

YAML은 broadcaster 코드를 생성하지 않는다.
이미 설치되고 pluginlib에 등록된 C++ 구현을 선택한다.

launch 파일의 spawner가 실제 로드를 요청한다.

```python
robot_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=[
        'arm_controller',
        'gripper_controller',
        'joint_state_broadcaster',
    ],
)
```

전체 과정은 다음과 같다.

```text
ros2_control_node 실행
  │
  ├─ robot_description 읽기
  ├─ hardware plugin 로드
  └─ hardware state/command interface 등록
       │
       ▼
spawner가 joint_state_broadcaster 로드 요청
       │
       ▼
controller manager가 broadcaster plugin 생성
       │
       ├─ configure
       └─ activate
       │
       ▼
state interface 읽기 및 토픽 발행
```

---

## 5. 같은 URDF를 어떻게 사용하는가

`ros2_control_node`와 `robot_state_publisher`는 같은 `robot_description`을 받지만
사용 목적은 다르다.

```text
robot_description
  ├─ 일반 URDF
  │    ├─ link
  │    ├─ joint
  │    ├─ parent/child
  │    ├─ axis
  │    └─ limit
  │
  └─ <ros2_control>
       ├─ hardware plugin
       ├─ state interface
       └─ command interface
```

### 5.1 hardware plugin이 사용하는 정보

hardware plugin은 `<ros2_control>`의 다음 정보를 사용한다.

- hardware plugin 타입
- 장치 포트와 통신 파라미터
- joint 이름
- state interface 이름
- command interface 이름
- 장치별 추가 파라미터

### 5.2 joint_state_broadcaster가 사용하는 정보

broadcaster는 controller manager에 등록된 state interface를 사용한다.

Jazzy의 기본 `use_urdf_to_filter` 값은 `true`이므로
일반 URDF에 존재하는 joint를 기준으로 `/joint_states`를 필터링하고
메시지의 joint 순서를 정할 때도 URDF 순서를 사용할 수 있다.

그러나 실제 position, velocity, effort 숫자를 URDF에서 가져오는 것은 아니다.

```text
URDF
  → 어떤 joint가 존재하는지, 순서를 어떻게 정할지 도움

hardware state interface
  → 실제 position, velocity, effort 숫자의 출처
```

---

## 6. OpenManipulator-X state interface

OpenManipulator-X의 ros2_control Xacro에는 다음과 같은 선언이 있다.

```xml
<joint name="${prefix}joint1">
  <command_interface name="position"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>
</joint>
```

파일:

```text
open_manipulator_description/
└─ ros2_control/open_manipulator_x_position.ros2_control.xacro
```

이 선언만으로 값이 자동 측정되는 것은 아니다.
`DynamixelHardware.read()`가 실제 Dynamixel 데이터를 읽고
다음과 같이 매핑해야 한다.

```text
Present Position → joint1/position
Present Velocity → joint1/velocity
Present Current  → joint1/effort
```

그 후 broadcaster가 이 interface를 읽는다.

```text
joint1/position
joint1/velocity
joint1/effort
       │
       ▼
sensor_msgs/msg/JointState
```

---

## 7. `/joint_states` 메시지 해석

메시지 타입은 다음과 같다.

```text
sensor_msgs/msg/JointState
```

주요 필드는 다음과 같다.

```yaml
header:
  stamp: ...
name:
  - joint1
  - joint2
position:
  - 0.10
  - -0.50
velocity:
  - 0.01
  - 0.00
effort:
  - 15.0
  - -23.0
```

배열은 같은 인덱스로 대응한다.

```text
name[0]     = joint1
position[0] = joint1 위치
velocity[0] = joint1 속도
effort[0]   = joint1 effort

name[1]     = joint2
position[1] = joint2 위치
velocity[1] = joint2 속도
effort[1]   = joint2 effort
```

### 주의: 배열 순서를 고정값으로 가정하지 않는다

사용자 코드에서 다음처럼 인덱스를 고정하면 위험하다.

```python
joint1_position = msg.position[0]
```

namespace, URDF 순서, broadcaster 파라미터 또는 하드웨어 등록 순서가 바뀌면
배열 순서도 바뀔 수 있다.

이름으로 인덱스를 찾아야 한다.

```python
index = msg.name.index('joint1')
joint1_position = msg.position[index]
```

또한 `position`, `velocity`, `effort` 배열은 지원하지 않는 필드일 경우
비어 있을 수 있으므로 길이를 확인해야 한다.

---

## 8. `/dynamic_joint_states`와의 차이

Jazzy의 `JointStateBroadcaster`는 기본적으로 다음 토픽도 발행한다.

```text
/dynamic_joint_states
```

타입은 다음과 같다.

```text
control_msgs/msg/DynamicJointState
```

`/joint_states`는 표준 운동 interface만 표현한다.

- position
- velocity
- effort

`/dynamic_joint_states`는 사용자 정의 interface도 표현할 수 있다.

- temperature
- voltage
- current
- motor_error
- calibration_state
- 기타 hardware plugin이 제공하는 interface

새로운 하드웨어가 온도나 전압을 제공한다면
이를 무조건 `effort` 같은 표준 필드에 억지로 넣지 말고,
명확한 이름의 custom state interface로 설계하는 것이 좋다.

---

## 9. joint_state_broadcaster의 주요 파라미터

필요한 경우 다음과 같이 broadcaster 범위를 제한할 수 있다.

```yaml
joint_state_broadcaster:
  ros__parameters:
    joints:
      - joint1
      - joint2

    interfaces:
      - position
      - velocity
      - effort

    use_local_topics: false
    use_urdf_to_filter: true
    publish_dynamic_joint_states: true
```

| 파라미터 | 의미 |
|---|---|
| `joints` | 발행 대상으로 요청할 joint 목록 |
| `interfaces` | 발행 대상으로 요청할 state interface 목록 |
| `use_local_topics` | controller namespace 아래에 토픽을 발행할지 결정 |
| `use_urdf_to_filter` | URDF에 존재하는 joint만 `/joint_states`에 포함할지 결정 |
| `publish_dynamic_joint_states` | `/dynamic_joint_states` 발행 여부 |
| `extra_joints` | 실제 interface 없이 값 0으로 추가할 joint |

기본적으로 `joints`와 `interfaces`가 비어 있으면
사용 가능한 모든 joint state interface를 요청한다.

### 주의: `extra_joints`는 실제 센서값이 아니다

`extra_joints`로 추가된 joint는 상태가 0으로 들어간다.
이 값을 실제 센서 측정값으로 오해하면 안 된다.

---

## 10. 새로운 하드웨어가 제공해야 하는 것

새 hardware plugin을 제작할 때 broadcaster가 정상 동작하려면
최소한 다음 조건을 만족해야 한다.

### 10.1 joint 이름 일치

다음 이름이 모두 일치해야 한다.

```text
일반 URDF joint name
<ros2_control> joint name
hardware plugin 내부 joint name
controller YAML joint name
사용자 메시지 joint name
```

예:

```text
joint1 ≠ Joint1 ≠ joint_1
```

ROS 이름은 대소문자를 구분하므로 서로 다른 이름이다.

### 10.2 state interface 등록

예:

```xml
<joint name="joint1">
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>
</joint>
```

hardware plugin은 이 interface에 대응하는 storage를 제공해야 한다.

### 10.3 `read()`에서 매 주기 값을 갱신

개념적인 구현은 다음과 같다.

```cpp
return_type MyHardware::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  joint_position_[0] = read_encoder_in_radians();
  joint_velocity_[0] = calculate_velocity_rad_per_sec();
  joint_effort_[0] = read_or_estimate_effort();

  return return_type::OK;
}
```

값을 한 번만 초기화하고 갱신하지 않으면
토픽은 계속 발행되어도 오래된 값만 반복될 수 있다.

### 10.4 표준 단위 사용

ROS joint state의 표준 단위는 다음과 같다.

| joint 종류 | position | velocity | effort |
|---|---|---|---|
| revolute/continuous | rad | rad/s | N·m |
| prismatic | m | m/s | N |

엔코더 tick, degree, rpm, ADC count, 전류 raw 값을
그대로 표준 필드에 넣지 않는 것이 원칙이다.

반드시 hardware plugin 경계에서 단위를 변환한다.

```text
degree → rad
rpm → rad/s
encoder tick → rad 또는 m
motor current → 가능한 경우 관절 토크 N·m
```

정확한 물리량으로 변환할 수 없다면:

1. custom interface로 raw 값을 별도 제공하고,
2. 표준 `effort`에는 검증된 값만 넣거나 비워 두며,
3. 단위와 보정 방법을 문서화한다.

### 10.5 부호와 joint axis 일치

URDF axis가 다음과 같다고 가정한다.

```xml
<axis xyz="0 0 1"/>
```

양의 모터 회전이 URDF의 양의 joint 방향과 반대라면
hardware plugin 또는 transmission에서 부호를 변환해야 한다.

부호가 틀리면 다음 문제가 생긴다.

- 실제 로봇과 RViz가 반대 방향으로 움직임
- velocity 부호가 잘못됨
- effort 방향 해석이 틀림
- controller 피드백이 양의 피드백처럼 동작할 수 있음

---

## 11. 새로운 joint 추가 절차

`joint3`를 새로 추가한다고 가정한다.

### 11.1 일반 URDF에 joint 추가

```xml
<joint name="joint3" type="revolute">
  <parent link="link2"/>
  <child link="link3"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="5.0" velocity="1.0"/>
</joint>
```

### 11.2 `<ros2_control>`에 interface 추가

```xml
<joint name="joint3">
  <command_interface name="position"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>
</joint>
```

### 11.3 hardware plugin에 장치 매핑 추가

예:

```text
joint3
  ↔ motor ID 3
  ↔ encoder channel 2
  ↔ current sensor channel 2
```

### 11.4 controller YAML 갱신

명령 controller가 joint3도 제어해야 한다면 해당 목록에 추가한다.

```yaml
arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
```

`joint_state_broadcaster`가 기본 설정으로 모든 state interface를 사용한다면
별도 joint 목록을 추가하지 않아도 발견할 수 있다.
하지만 교육 과정에서는 의도한 목록을 명시해 검증하는 방식도 유용하다.

---

## 12. 검증 절차

검증은 아래 순서로 진행한다.

### 12.1 최종 URDF 확인

```bash
xacro robot.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

확인:

- 일반 joint가 있는가?
- `<ros2_control>` joint가 있는가?
- 이름과 interface가 정확한가?

### 12.2 hardware component 확인

```bash
ros2 control list_hardware_components
```

확인:

- 의도한 hardware plugin이 로드되었는가?
- lifecycle 상태가 정상인가?

### 12.3 hardware interface 확인

```bash
ros2 control list_hardware_interfaces
```

예:

```text
state interfaces
  joint1/position
  joint1/velocity
  joint1/effort
  joint2/position
  joint2/velocity
  joint2/effort
```

interface가 여기에서 보이지 않으면 broadcaster 문제가 아니다.
URDF 또는 hardware plugin 등록 문제다.

### 12.4 broadcaster 상태 확인

```bash
ros2 control list_controllers
```

정상:

```text
joint_state_broadcaster  joint_state_broadcaster/JointStateBroadcaster  active
```

### 12.5 publisher 확인

```bash
ros2 topic info /joint_states --verbose
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
```

확인:

- publisher가 예상한 broadcaster인가?
- 메시지가 주기적으로 들어오는가?
- timestamp가 갱신되는가?
- joint 이름과 배열 길이가 맞는가?

### 12.6 정지 상태 검증

로봇을 움직이지 않은 상태에서:

- position이 튀지 않는가?
- velocity가 합리적으로 0 근처인가?
- effort가 심하게 변하지 않는가?
- timestamp가 현재 시각으로 갱신되는가?

### 12.7 한 관절씩 방향 검증

한 관절만 작은 양의 방향으로 움직인다.

확인:

- 실제 joint가 URDF의 양의 방향으로 움직이는가?
- 해당 position만 증가하는가?
- 다른 joint 값이 잘못 바뀌지 않는가?
- velocity 부호가 position 변화 방향과 일치하는가?

### 12.8 TF 검증

```bash
ros2 topic echo /tf
```

RViz에서:

- 실제 로봇과 같은 방향으로 움직이는가?
- link 길이와 회전축이 맞는가?
- 움직이지 않은 다른 링크가 흔들리지 않는가?

---

## 13. 자주 발생하는 오류

### 13.1 `/joint_states`가 없을 때

확인 순서:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic info /joint_states --verbose
```

가능한 원인:

- broadcaster가 로드되지 않음
- broadcaster가 inactive
- state interface가 하나도 없음
- controller manager namespace가 다름
- `use_local_topics: true`라서 토픽 경로가 달라짐
- ROS_DOMAIN_ID 불일치

### 13.2 값이 항상 0일 때

가능한 원인:

- mock hardware를 사용 중
- hardware `read()`가 값을 갱신하지 않음
- 장치 통신 실패를 0으로 숨기고 있음
- 센서 변환식 오류
- 잘못된 장치 ID 또는 채널 매핑
- `extra_joints`로 만든 가상 값

통신 실패를 정상적인 0으로 표현하면 위험하다.
가능하면 오류 상태를 명확히 보고하고 lifecycle을 error로 전환한다.

### 13.3 position이 degree처럼 보일 때

예:

```text
0, 30, 60, 90
```

ROS 표준 position은 radian이어야 한다.

```text
0°, 30°, 60°, 90°
→ 0, 0.524, 1.047, 1.571 rad
```

### 13.4 토픽은 정상인데 RViz가 이상할 때

가능한 원인:

- URDF joint axis 오류
- parent/child 연결 오류
- origin 오류
- joint 이름 불일치
- robot_state_publisher가 다른 URDF를 사용
- `/joint_states` publisher가 두 개 이상

### 13.5 `/joint_states` publisher가 여러 개일 때

실기기 bringup에서 `joint_state_publisher_gui`를 함께 실행하면
가상값과 실제값이 같은 토픽에 섞일 수 있다.

```bash
ros2 topic info /joint_states --verbose
```

publisher 수와 노드 이름을 반드시 확인한다.

---

## 14. 신규 하드웨어 제작 시 주의사항

### 주의 1. 토픽이 보인다는 것과 측정값이 정확하다는 것은 다르다

토픽 발행은 통신 경로가 존재한다는 뜻일 뿐이다.
센서 보정, 단위, 부호, offset이 정확하다는 증거는 아니다.

### 주의 2. raw 값을 표준 필드에 넣지 않는다

다음 값을 변환 없이 넣지 않는다.

- encoder tick
- degree
- rpm
- ADC count
- motor current raw

### 주의 3. 통신 실패를 0으로 숨기지 않는다

0은 정상적인 위치, 속도, effort가 될 수 있으므로
통신 오류와 구분할 수 없다.

오류 로그, diagnostic, hardware lifecycle 또는 별도 상태 interface를 사용한다.

### 주의 4. timestamp를 실제 측정 시각과 일치시킨다

오래된 센서값을 현재 값처럼 발행하면
제어기와 상태 추정기가 잘못된 판단을 할 수 있다.

### 주의 5. 여러 센서 읽기의 동기화를 고려한다

여러 관절을 순차적으로 느리게 읽으면
한 `/joint_states` 메시지 안의 값들이 서로 다른 시각의 상태일 수 있다.

고속 동작에서는 bulk read, sync read 또는 timestamp 전략을 검토한다.

### 주의 6. 안전 제어를 broadcaster에 맡기지 않는다

`joint_state_broadcaster`는 상태 공개용이다.
다음 기능은 hardware 또는 별도 안전 계층에서 구현해야 한다.

- joint limit
- current limit
- 온도 보호
- 통신 timeout
- emergency stop
- watchdog
- torque disable

---

## 15. 교육생 제출 체크리스트

### 구조

- [ ] 일반 URDF joint와 ros2_control joint의 차이를 설명할 수 있다.
- [ ] hardware plugin과 broadcaster의 역할을 구분할 수 있다.
- [ ] robot_state_publisher가 TF를 담당한다는 것을 설명할 수 있다.

### 이름과 interface

- [ ] URDF, ros2_control, hardware, controller의 joint 이름이 일치한다.
- [ ] 필요한 state interface가 `list_hardware_interfaces`에 보인다.
- [ ] 사용자 코드가 배열 인덱스를 고정하지 않고 joint 이름을 사용한다.

### 단위와 부호

- [ ] 회전 위치가 rad 단위다.
- [ ] 회전 속도가 rad/s 단위다.
- [ ] effort의 단위와 산출 방식을 문서화했다.
- [ ] 양의 joint 방향이 URDF axis와 일치한다.

### 실행

- [ ] hardware component가 정상 lifecycle 상태다.
- [ ] joint_state_broadcaster가 active다.
- [ ] `/joint_states` publisher가 의도한 하나의 경로다.
- [ ] timestamp가 계속 갱신된다.
- [ ] 정지 상태와 한 관절 동작 시험을 통과했다.
- [ ] 실제 로봇과 RViz의 방향이 일치한다.

---

## 16. 핵심 정리

1. `joint_state_broadcaster`는 실제 장치를 직접 읽지 않는다.
2. hardware plugin의 `read()`가 state interface를 갱신한다.
3. broadcaster는 state interface를 `/joint_states`와
   `/dynamic_joint_states`로 변환한다.
4. URDF는 joint 존재 여부와 순서 필터링에 사용될 수 있지만,
   실제 측정값의 출처는 아니다.
5. 새로운 하드웨어에서는 이름, 단위, 부호, timestamp, 통신 오류 처리가 핵심이다.
6. `/joint_states`가 발행된다는 이유만으로 하드웨어가 정상이라고 판단하면 안 된다.
7. `list_hardware_interfaces → list_controllers → topic echo → 실제 방향 시험`
   순서로 검증해야 한다.

## 17. 참고 자료

- ROS 2 Jazzy Joint State Broadcaster:
  https://control.ros.org/jazzy/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html
- ros2_control 문서:
  https://control.ros.org/jazzy/
- OpenManipulator-X ros2_control Xacro:
  `open_manipulator_description/ros2_control/open_manipulator_x_position.ros2_control.xacro`
- OpenManipulator-X controller YAML:
  `open_manipulator_bringup/config/open_manipulator_x/hardware_controller_manager.yaml`

