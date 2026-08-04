# joint_state_publisher와 joint_state_broadcaster의 차이

## 1. 이 문서의 비교 대상

이 문서는 다음 두 수업 환경을 비교한다.

1. 이전 URDF 실습  
   `tf2_basic/launch/urdf_display.launch.py`와 ROS 2의 `display.launch.py`를 사용해
   가상 관절값으로 로봇 모델을 움직였다.
2. OpenManipulator-X 실습  
   `open_manipulator_bringup/launch/open_manipulator_x.launch.py`가
   `ros2_control`과 실제 Dynamixel 하드웨어를 실행하고,
   `joint_state_broadcaster`가 측정된 관절 상태를 전달한다.

두 환경 모두 최종적으로 `/joint_states`를 사용한다. 가장 큰 차이는
그 관절값이 **사람이 만든 가상 값인지**, **하드웨어에서 읽은 실제 값인지**이다.

> `joint_state_broadcaster`의 broadcaster는 TF broadcaster라는 뜻이 아니다.
> 이 controller는 ros2_control의 관절 상태를 ROS 토픽으로 전달한다.
> TF는 두 환경 모두 `robot_state_publisher`가 발행한다.

## 2. 이전 수업: URDF 모델 표시

### 2.1 실제 실행 구조

이전 수업의 파일은 다음과 같다.

```text
kong_ws/src/tf2_basic/launch/urdf_display.launch.py
```

이 파일은 설치된 `urdf_launch/launch/display.launch.py`를 포함한다.
실제 노드 구성은 다음과 같다.

```text
urdf_display.launch.py
  └─ urdf_launch/display.launch.py
      ├─ joint_state_publisher
      │   또는 joint_state_publisher_gui
      ├─ robot_state_publisher
      └─ rviz2
```

`tf2_basic`의 launch 파일은 `gui` 기본값이 `false`이므로
기본 실행 시 `joint_state_publisher`를 사용한다. `gui:=true`이면
`joint_state_publisher_gui`를 사용한다.

### 2.2 데이터 흐름

```text
사용자가 입력한 값 또는 기본 관절값
  → joint_state_publisher(_gui)
  → /joint_states
  → robot_state_publisher + URDF
  → /tf, /tf_static
  → RViz2
```

이 환경에는 다음 요소가 없다.

- 실제 모터와 엔코더
- Dynamixel 통신
- ros2_control hardware interface
- controller_manager
- trajectory controller
- 실제 관절 상태 피드백

따라서 RViz에서 로봇이 움직이는 것은 실제 모터가 움직였다는 뜻이 아니다.
`joint_state_publisher_gui`의 슬라이더를 움직이면 화면 속 모델만 움직인다.

### 2.3 이전 실습 실행 예

```bash
cd /home/ju/kong_manipulator2026/kong_ws
source install/setup.bash

ros2 launch tf2_basic urdf_display.launch.py \
  model:=urdf/04_pysics.urdf.xacro \
  gui:=true
```

OpenManipulator-X 패키지에도 모델만 확인하는 별도 launch가 있다.

```bash
cd /home/ju/kong_manipulator2026/open_manipulator_ws
source install/setup.bash

ros2 launch open_manipulator_description open_manipulator_x.launch.py
```

이 `open_manipulator_description` launch도
`joint_state_publisher_gui`, `robot_state_publisher`, RViz2만 실행한다.
실제 OpenManipulator-X를 제어하는 bringup launch와 혼동하면 안 된다.

## 3. OpenManipulator-X: 실제 하드웨어 상태 전달

### 3.1 실제 실행 구조

실기기용 파일은 다음과 같다.

```text
open_manipulator_ws/src/open_manipulator/
└─ open_manipulator_bringup/
   ├─ launch/open_manipulator_x.launch.py
   └─ config/open_manipulator_x/hardware_controller_manager.yaml
```

`open_manipulator_x.launch.py`는 다음 구성 요소를 실행한다.

```text
open_manipulator_x.launch.py
  ├─ ros2_control_node (= controller_manager)
  │   ├─ DynamixelHardware
  │   ├─ joint_state_broadcaster
  │   ├─ arm_controller
  │   └─ gripper_controller
  ├─ robot_state_publisher
  ├─ joint_trajectory_executor
  └─ rviz2 (start_rviz:=true일 때)
```

저장소의 controller 설정은 다음과 같다.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    gripper_controller:
      type: position_controllers/GripperActionController
```

`joint_state_broadcaster`는 일반 Python 노드를 직접 실행하는 방식이 아니다.
controller plugin을 `controller_manager` 안에 로드하고 활성화한다.
launch 파일의 `controller_manager spawner`가 다음 세 controller를 생성한다.

- `arm_controller`
- `gripper_controller`
- `joint_state_broadcaster`

spawner 프로세스는 controller를 로드하고 활성화한 뒤 종료될 수 있다.
그러나 로드된 controller는 `controller_manager` 안에서 계속 동작한다.

### 3.2 실제 데이터 흐름

```text
Dynamixel 엔코더
  → dynamixel_hardware_interface/DynamixelHardware
  → ros2_control state_interface
      ├─ position
      ├─ velocity
      └─ effort
  → joint_state_broadcaster
  → /joint_states
  → robot_state_publisher + OpenManipulator-X URDF
  → /tf, /tf_static
  → RViz2 / MoveIt 2
```

OpenManipulator-X의 ros2_control Xacro에는 다음 관절의 상태 인터페이스가 정의되어 있다.

- `joint1`
- `joint2`
- `joint3`
- `joint4`
- `gripper_left_joint`

각 관절은 `position`, `velocity`, `effort` 상태를 제공한다.
실기기 기본 포트는 `/dev/ttyUSB0`, 통신 속도는 1,000,000 baud로 설정되어 있다.

### 3.3 joint_state_broadcaster가 보이는 이유

다음 명령에서 `/joint_state_broadcaster`가 노드처럼 보일 수 있다.

```bash
ros2 node list
```

하지만 수업에서는 다음과 같이 이해하는 것이 정확하다.

- 실행 단위: ros2_control controller plugin
- 관리 주체: `controller_manager`
- 상태 원본: 하드웨어의 state interface
- ROS 측 출력: `/joint_states`
- TF 발행 여부: 직접 발행하지 않음

controller 상태는 `ros2 node list`보다 다음 명령으로 확인하는 것이 더 정확하다.

```bash
ros2 control list_controllers
```

정상적인 실기기 bringup이라면 적어도 다음 항목들이 `active`여야 한다.

```text
joint_state_broadcaster
arm_controller
gripper_controller
```

## 4. 핵심 차이 비교

| 구분 | joint_state_publisher | joint_state_broadcaster |
|---|---|---|
| 소속 | 독립 ROS 2 패키지/노드 | ros2_control controller plugin |
| 값의 출처 | 기본값, 파라미터, GUI 슬라이더 | 하드웨어 또는 시뮬레이터의 state interface |
| 주 사용 목적 | URDF 모델 확인, 교육, 시각화 | 실제 로봇·시뮬레이터의 관절 상태 전달 |
| `/joint_states` 발행 | 발행함 | 발행함 |
| TF 직접 발행 | 하지 않음 | 하지 않음 |
| TF 담당 | `robot_state_publisher` | `robot_state_publisher` |
| controller_manager 필요 | 필요 없음 | 반드시 필요 |
| 실제 엔코더 값 | 읽지 않음 | hardware interface를 통해 읽음 |
| GUI 슬라이더 | GUI 버전에서 사용 | 사용하지 않음 |
| 명령 기능 | 실제 모터 명령 기능 없음 | 상태 전달용이며 모터 명령 controller는 별도 |
| 잘못된 동시 사용의 결과 | 실제 값과 가상 값이 `/joint_states`에서 충돌할 수 있음 | 실기기 bringup에서는 이쪽을 사용 |

## 5. 같은 점과 다른 점

### 5.1 같은 점

두 구성 모두 `/joint_states`에 `sensor_msgs/msg/JointState` 메시지를 발행한다.
메시지에는 다음 정보가 포함된다.

- 관절 이름 `name`
- 관절 위치 `position`
- 관절 속도 `velocity`
- 관절 힘 또는 토크 `effort`

두 구성 모두 TF를 직접 발행하지 않는다. 다음 단계의
`robot_state_publisher`가 URDF와 `/joint_states`를 이용해 link 좌표를 계산한다.

### 5.2 가장 중요한 다른 점

```text
joint_state_publisher
  = “로봇 관절이 이 위치라고 가정하자.”

joint_state_broadcaster
  = “하드웨어가 보고한 현재 관절 상태를 ROS에 전달하자.”
```

즉, 전자는 모델 시각화를 위한 가상 상태 생성기이고,
후자는 제어 시스템이 읽은 상태를 외부 ROS 노드에 공개하는 전달자이다.

## 6. TF와의 관계

두 경우 모두 TF 처리 구조는 동일하다.

```text
/joint_states + robot_description(URDF)
  → robot_state_publisher
  → /tf, /tf_static
```

- 움직이는 joint의 link 관계는 `/tf`로 발행된다.
- fixed joint의 link 관계는 `/tf_static`으로 발행된다.
- `joint_state_broadcaster`의 이름에 있는 broadcaster는 TF 방송을 뜻하지 않는다.

따라서 OpenManipulator-X에서 TF가 보인다고 해서
`joint_state_broadcaster`가 TF를 만든 것으로 해석하면 안 된다.
TF를 실제로 계산하고 발행하는 노드는 이전 실습과 동일하게
`robot_state_publisher`이다.

## 7. 상태와 명령을 구분하기

`joint_state_broadcaster`는 관절 **상태**를 내보내는 controller이다.
로봇을 움직이는 **명령**은 다른 controller가 담당한다.

```text
명령 방향
사용자 프로그램 / MoveIt 2
  → arm_controller 또는 gripper_controller
  → ros2_control command_interface
  → Dynamixel 모터

피드백 방향
Dynamixel 엔코더
  → ros2_control state_interface
  → joint_state_broadcaster
  → /joint_states
```

`/joint_states`에 값을 임의로 발행해도 실제 OpenManipulator-X 모터가 움직이지 않는다.
실제 동작 명령은 `arm_controller`의 trajectory 인터페이스나
`gripper_controller`의 action 인터페이스로 보내야 한다.

## 8. 저장소의 이전 사용자 코드와 차이

이전 수업의 `tf2_basic/move_u2d2.py`도 Python 코드에서
`JointState` 메시지를 만들어 `/joint_states`에 직접 발행한다.

```python
self.publisher = self.create_publisher(
    JointState,
    "/joint_states",
    10,
)
```

하지만 이 코드는 실제 U2D2나 Dynamixel에서 값을 읽지 않는다.
프로그램 내부에서 임의의 목표와 보간값을 만들며, 사용하는 관절 이름도
OpenManipulator-X의 `joint1`~`joint4`, `gripper_left_joint`와 다르다.

따라서 다음과 같이 구분해야 한다.

| 이전 `move_u2d2.py` | OpenManipulator-X bringup |
|---|---|
| Python이 임의 관절값 계산 | Dynamixel 엔코더 상태를 읽음 |
| `/joint_states` 직접 발행 | `joint_state_broadcaster`가 발행 |
| 화면 속 사용자 URDF용 | 실제 OpenManipulator-X 상태용 |
| 모터 제어 기능 없음 | 별도 arm/gripper controller로 제어 |
| ros2_control 사용 안 함 | ros2_control 사용 |

파일 이름에 `u2d2`가 포함되어 있어도 실제 U2D2 통신 코드로 해석하면 안 된다.

## 9. 실행 방법 비교

### 9.1 이전 URDF 시각화 실습

```bash
cd /home/ju/kong_manipulator2026/kong_ws
source install/setup.bash

ros2 launch tf2_basic urdf_display.launch.py \
  model:=urdf/04_pysics.urdf.xacro \
  gui:=true
```

확인할 내용:

```bash
ros2 topic echo /joint_states --once
ros2 node list
```

슬라이더를 변경하면 `/joint_states`와 RViz 모델이 바뀌지만 실제 모터는 움직이지 않는다.

### 9.2 OpenManipulator-X 모델만 표시

```bash
cd /home/ju/kong_manipulator2026/open_manipulator_ws
source install/setup.bash

ros2 launch open_manipulator_description open_manipulator_x.launch.py
```

이 명령도 GUI용 `joint_state_publisher` 방식이며 실기기 bringup이 아니다.

### 9.3 실제 OpenManipulator-X bringup

실행 전에 로봇 자세, 전원, U2D2 연결과 장치 포트를 확인한다.

```bash
ls -l /dev/ttyUSB*
```

그다음 실제 하드웨어용 launch를 실행한다.

```bash
cd /home/ju/kong_manipulator2026/open_manipulator_ws
source install/setup.bash

ros2 launch open_manipulator_bringup open_manipulator_x.launch.py \
  port_name:=/dev/ttyUSB0 \
  start_rviz:=true
```

다른 포트가 확인되었다면 `port_name`을 실제 장치명으로 바꾼다.
이 launch의 기본값은 실제 하드웨어 모드인 `use_mock_hardware:=false`이다.

## 10. 실습 확인 명령

### 10.1 controller 상태

```bash
ros2 control list_controllers
```

`joint_state_broadcaster`, `arm_controller`, `gripper_controller`가
`active`인지 확인한다.

### 10.2 하드웨어와 인터페이스

```bash
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
```

관절의 state interface와 command interface가 준비되어 있는지 확인한다.

### 10.3 `/joint_states` 발행자

```bash
ros2 topic info /joint_states --verbose
ros2 topic echo /joint_states --once
```

실기기 bringup에서 `/joint_states`의 발행자가
`joint_state_broadcaster` 계통인지 확인한다.

### 10.4 TF

```bash
ros2 run tf2_ros tf2_echo world end_effector_link
```

이 결과는 `robot_state_publisher`가 만든 TF이다.
로봇을 움직였을 때 `/joint_states`와 TF가 함께 바뀌는지 확인한다.

## 11. 학생들에게 설명할 핵심 문장

1. `joint_state_publisher`는 URDF 실습을 위해 가상의 관절 상태를 만든다.
2. `joint_state_broadcaster`는 ros2_control이 하드웨어에서 읽은 관절 상태를 전달한다.
3. 두 구성 모두 `/joint_states`를 발행하지만 데이터의 출처와 신뢰성이 다르다.
4. 두 구성 모두 TF를 직접 발행하지 않으며 TF는 `robot_state_publisher`가 만든다.
5. `joint_state_broadcaster`는 `controller_manager`가 관리하는 controller plugin이다.
6. 상태를 읽는 controller와 모터에 명령하는 controller는 서로 다르다.
7. 실제 로봇에서 `joint_state_publisher`를 동시에 실행하면 가상 상태와 실제 상태가
   충돌할 수 있으므로 사용하지 않는다.
