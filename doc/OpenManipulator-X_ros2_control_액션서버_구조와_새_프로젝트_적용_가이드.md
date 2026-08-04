# OpenManipulator-X ros2_control 액션 서버 구조와 새 프로젝트 적용 가이드

## 1. 문서 목적

이 문서는 다음 질문에 답한다.

1. OpenManipulator-X bringup에서 `/arm_controller/follow_joint_trajectory`
   액션 서버는 어디에서 생성되는가?
2. `hardware_controller_manager.yaml`에 controller 설정을 적으면
   왜 ROS 2 노드, 토픽, 액션이 나타나는가?
3. `joint_trajectory_executor`와 `arm_controller`는 각각 어떤 역할인가?
4. 새로운 로봇 프로젝트를 만들거나 관절을 추가할 때
   `ros2_control`을 어떻게 구성해야 하는가?
5. 설정이 정상적으로 연결되었는지 어떤 순서로 검증해야 하는가?

이 문서에서 사용하는 정식 ROS 2 명칭은 `ros2_control`이다.
ROS 1의 프레임워크는 `ros_control`이므로 둘을 구분해야 한다.

기준 환경은 다음과 같다.

- Ubuntu 24.04
- ROS 2 Jazzy
- OpenManipulator-X
- 작업공간:
  `/home/ju/kong_manipulator2026/open_manipulator_ws`

---

## 2. 먼저 구분해야 할 구성 요소

OpenManipulator-X 제어 구조를 해석할 때는 다음 네 계층을 분리해야 한다.

| 계층 | OpenManipulator-X 구성 요소 | 역할 |
|---|---|---|
| 사용자 명령 | `joint_trajectory_executor`, MoveIt, teleop | 목표 궤적이나 위치를 controller에 전달 |
| controller | `arm_controller`, `gripper_controller` | 목표와 현재 상태를 비교하고 command interface에 명령 기록 |
| ros2_control 관리 | `controller_manager`, `ros2_control_node` | 하드웨어와 controller plugin을 로드하고 주기적으로 `read-update-write` 실행 |
| hardware interface | `dynamixel_hardware_interface/DynamixelHardware` | Dynamixel 상태를 읽고 controller 명령을 실제 모터로 전달 |

전체 데이터 흐름은 다음과 같다.

```text
joint_trajectory_executor / MoveIt / 사용자 노드
  │
  │ FollowJointTrajectory goal
  ▼
/arm_controller/follow_joint_trajectory
  │
  │ JointTrajectoryController
  ▼
joint1~joint4 position command interface
  │
  │ ros2_control write()
  ▼
DynamixelHardware
  │
  ▼
실제 Dynamixel 모터

실제 Dynamixel 엔코더
  │
  │ ros2_control read()
  ▼
position / velocity / effort state interface
  │
  ├─ arm_controller 피드백 및 오차 계산
  └─ joint_state_broadcaster → /joint_states
```

중요한 점은 `joint_trajectory_executor`가 모터 드라이버가 아니라는 것이다.
이 노드는 `arm_controller`의 액션 서버에 목표를 보내는 액션 클라이언트이다.

---

## 3. 현재 저장소에서 bringup이 시작되는 위치

실기기 bringup 파일은 다음과 같다.

```text
open_manipulator_ws/src/open_manipulator/
└─ open_manipulator_bringup/
   └─ launch/open_manipulator_x.launch.py
```

이 launch 파일은 `ros2_control_node`를 다음과 같이 실행한다.

```python
control_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[
        {'robot_description': urdf_file},
        controller_manager_config,
    ],
    output='both',
)
```

`ros2_control_node`가 받는 입력은 크게 두 가지다.

1. `robot_description`
   - URDF/Xacro로 만든 로봇 설명
   - `<ros2_control>`의 hardware plugin과 joint interface 정보 포함
2. `controller_manager_config`
   - 어떤 controller plugin을 사용할지 지정하는 YAML
   - controller가 제어할 관절과 인터페이스 지정

즉 URDF와 controller YAML은 서로 다른 정보를 제공한다.

```text
URDF/Xacro
  └─ 하드웨어가 어떤 joint interface를 제공하는가?

controller YAML
  └─ controller가 어떤 joint interface를 사용할 것인가?
```

두 설정의 관절 이름과 interface 이름이 정확히 일치해야 한다.

---

## 4. hardware_controller_manager.yaml 해석

OpenManipulator-X의 설정 파일은 다음과 같다.

```text
open_manipulator_bringup/
└─ config/open_manipulator_x/hardware_controller_manager.yaml
```

핵심 설정은 다음과 같다.

```yaml
/**:
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

각 항목의 의미는 다음과 같다.

| 설정 | 의미 |
|---|---|
| `update_rate: 100` | controller manager의 제어 루프를 초당 100회 실행 |
| `arm_controller` | 로드할 controller 인스턴스 이름 |
| `type` | pluginlib에서 찾을 controller plugin 타입 |
| `JointTrajectoryController` | 여러 관절의 시간 기반 trajectory를 실행하는 C++ controller |

`arm_controller`의 세부 파라미터는 다음과 같다.

```yaml
/**:
  arm_controller:
    ros__parameters:
      joints:
        - joint1
        - joint2
        - joint3
        - joint4

      command_interfaces:
        - position

      state_interfaces:
        - position
        - velocity

      allow_partial_joints_goal: true
```

이 설정은 다음을 뜻한다.

- `joint1`부터 `joint4`까지 제어한다.
- 하드웨어의 `position` command interface에 목표를 쓴다.
- 하드웨어의 `position`, `velocity` state interface를 읽는다.
- 일부 관절만 포함한 trajectory goal도 허용한다.

### 4.1 YAML이 액션 서버 코드를 생성하는 것은 아니다

다음 문자열은 코드가 아니라 plugin의 조회 키이다.

```yaml
type: joint_trajectory_controller/JointTrajectoryController
```

설치된 plugin 등록 파일은 다음과 같다.

```text
/opt/ros/jazzy/share/joint_trajectory_controller/
└─ joint_trajectory_plugin.xml
```

등록 정보의 핵심 구조는 다음과 같다.

```xml
<class
  name="joint_trajectory_controller/JointTrajectoryController"
  type="joint_trajectory_controller::JointTrajectoryController"
  base_class_type="controller_interface::ControllerInterface">
</class>
```

연결 관계는 다음과 같다.

```text
YAML의 type 문자열
  joint_trajectory_controller/JointTrajectoryController
        │
        │ pluginlib 조회
        ▼
C++ 클래스
  joint_trajectory_controller::JointTrajectoryController
        │
        │ 동적 라이브러리 로드
        ▼
/opt/ros/jazzy/lib/libjoint_trajectory_controller.so
```

따라서 YAML만으로 새로운 기능이 생성되는 것이 아니다.
이미 설치되고 pluginlib에 등록된 C++ 구현을 선택하는 것이다.

---

## 5. controller를 실제로 로드하는 spawner

YAML에 controller를 선언하는 것만으로는 충분하지 않다.
controller manager에 controller를 로드하고 configure/activate하라는 요청이 필요하다.

현재 launch 파일에서는 `spawner`가 이 작업을 수행한다.

```python
robot_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=[
        'arm_controller',
        'gripper_controller',
        'joint_state_broadcaster',
    ],
    output='both',
)
```

실행 과정은 다음과 같다.

```text
controller_manager 시작
  │
  ▼
spawner가 arm_controller 로드 요청
  │
  ▼
controller_manager가 YAML에서 arm_controller.type 조회
  │
  ▼
pluginlib가 JointTrajectoryController C++ plugin 로드
  │
  ├─ init
  ├─ configure
  └─ activate
  │
  ▼
arm_controller가 active 상태로 제어 루프에 참여
```

spawner는 controller를 로드하고 활성화한 뒤 종료될 수 있다.
spawner 프로세스가 종료되어도 controller는 `ros2_control_node` 내부에서 계속 동작한다.

controller 상태는 다음 명령으로 확인한다.

```bash
ros2 control list_controllers
```

정상 예시는 다음과 같다.

```text
arm_controller           joint_trajectory_controller/JointTrajectoryController  active
gripper_controller       position_controllers/GripperActionController           active
joint_state_broadcaster  joint_state_broadcaster/JointStateBroadcaster           active
```

---

## 6. 액션 서버를 만드는 실제 C++ 코드

Jazzy의 실제 소스는 `ros-controls/ros2_controllers` 저장소에 있다.

```text
ros2_controllers/
└─ joint_trajectory_controller/
   ├─ include/joint_trajectory_controller/
   │  └─ joint_trajectory_controller.hpp
   ├─ src/
   │  └─ joint_trajectory_controller.cpp
   └─ joint_trajectory_plugin.xml
```

공식 Jazzy 소스:

- https://github.com/ros-controls/ros2_controllers/blob/jazzy/joint_trajectory_controller/src/joint_trajectory_controller.cpp
- https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html

설치된 헤더에는 다음 액션 서버 멤버가 선언되어 있다.

```cpp
rclcpp_action::Server<FollowJTrajAction>::SharedPtr action_server_;
```

`JointTrajectoryController`가 configure될 때 실행되는 핵심 코드는 다음 구조다.

```cpp
action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
  get_node()->get_node_base_interface(),
  get_node()->get_node_clock_interface(),
  get_node()->get_node_logging_interface(),
  get_node()->get_node_waitables_interface(),
  std::string(get_node()->get_name()) + "/follow_joint_trajectory",
  std::bind(
    &JointTrajectoryController::goal_received_callback,
    this, _1, _2),
  std::bind(
    &JointTrajectoryController::goal_cancelled_callback,
    this, _1),
  std::bind(
    &JointTrajectoryController::goal_accepted_callback,
    this, _1));
```

컨트롤러 이름이 `arm_controller`이므로 액션 이름은 다음 규칙으로 만들어진다.

```text
controller 이름 + /follow_joint_trajectory

arm_controller + /follow_joint_trajectory
  → /arm_controller/follow_joint_trajectory
```

controller 이름을 `robot_arm`으로 바꿔 로드하면 기본 액션 이름도 다음처럼 바뀐다.

```text
/robot_arm/follow_joint_trajectory
```

즉 현재 액션 서버 이름은 OpenManipulator 전용 Python 코드에 고정된 것이 아니다.
공용 `JointTrajectoryController` C++ plugin이 controller 인스턴스 이름을 이용해 만든다.

---

## 7. joint_trajectory_executor는 액션 클라이언트다

OpenManipulator-X의 초기 자세 이동 노드는 다음 파일에 있다.

```text
open_manipulator_bringup/
└─ open_manipulator_bringup/joint_trajectory_executor.py
```

핵심 코드는 다음과 같다.

```python
self.action_client = ActionClient(
    self,
    FollowJointTrajectory,
    self.action_topic,
)
```

기본 액션 주소는 다음과 같다.

```python
'/arm_controller/follow_joint_trajectory'
```

따라서 역할은 다음처럼 구분된다.

| 구성 요소 | Action 역할 |
|---|---|
| `joint_trajectory_executor` | Action Client |
| `arm_controller` | Action Server |

bringup 로그에서도 어느 쪽이 goal을 보내고 받는지 확인할 수 있다.

```text
[joint_trajectory_executor]: Sending goal...
[arm_controller]: Received new action goal
[arm_controller]: Accepted new action goal
```

`joint_trajectory_executor`는 `initial_positions.yaml`의 목표 위치로 이동한 뒤
스스로 종료한다. 그러므로 초기 동작이 끝난 후 `ros2 node list`에는 보이지 않는다.

반면 `arm_controller`는 controller manager 안에서 계속 실행되므로 남아 있다.

실행 중 action 연결 상태는 다음 명령으로 확인할 수 있다.

```bash
ros2 action list -t
ros2 action info /arm_controller/follow_joint_trajectory -t
ros2 node info /arm_controller
```

---

## 8. hardware interface는 어디에서 선택되는가

controller YAML은 controller를 선택한다.
실제 모터와 통신할 hardware plugin은 URDF의 `<ros2_control>`에서 선택한다.

OpenManipulator-X의 관련 파일은 다음과 같다.

```text
open_manipulator_description/
└─ ros2_control/open_manipulator_x_position.ros2_control.xacro
```

실기기에서는 다음 plugin을 사용한다.

```xml
<hardware>
  <plugin>dynamixel_hardware_interface/DynamixelHardware</plugin>
  <param name="port_name">${port_name}</param>
  <param name="baud_rate">1000000</param>
</hardware>
```

mock hardware에서는 다음 plugin을 사용한다.

```xml
<hardware>
  <plugin>mock_components/GenericSystem</plugin>
  <param name="mock_sensor_commands">${mock_sensor_commands}</param>
</hardware>
```

Gazebo에서는 다음 plugin을 사용한다.

```xml
<hardware>
  <plugin>gz_ros2_control/GazeboSimSystem</plugin>
</hardware>
```

controller는 동일한 `position` command interface를 사용하면서
그 아래 hardware plugin만 바꿀 수 있다.

```text
             ┌─ GenericSystem: 하드웨어 없이 테스트
controller ──┼─ GazeboSimSystem: Gazebo 시뮬레이션
             └─ DynamixelHardware: 실제 Dynamixel
```

이 구조가 `ros2_control`을 사용하는 중요한 이유다.
사용자 명령과 controller 코드를 유지하면서 하드웨어 구현을 교체할 수 있다.

---

## 9. URDF joint와 ros2_control joint의 차이

새 관절을 추가할 때 가장 자주 놓치는 부분이다.

### 9.1 일반 URDF joint

일반 URDF joint는 로봇의 기구학적 연결을 설명한다.

```xml
<joint name="joint1" type="revolute">
  <parent link="base_link"/>
  <child link="link1"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
</joint>
```

이 정보만으로는 모터 제어가 생성되지 않는다.
이 joint를 `robot_state_publisher`가 읽으면 TF 계산에는 사용할 수 있지만,
controller가 사용할 command/state interface는 아직 없다.

### 9.2 ros2_control joint

같은 이름의 관절을 `<ros2_control>`에도 등록해야 한다.

```xml
<ros2_control name="MyRobotSystem" type="system">
  <hardware>
    <plugin>mock_components/GenericSystem</plugin>
  </hardware>

  <joint name="joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

역할을 비교하면 다음과 같다.

| 항목 | 일반 URDF joint | `<ros2_control>` joint |
|---|---|---|
| 목적 | 링크 연결과 운동학 표현 | controller와 hardware 사이의 데이터 인터페이스 선언 |
| TF 계산 | 사용됨 | 직접 사용되지 않음 |
| joint limit | 위치·속도·effort 범위 표현 | command/state interface와 하드웨어 파라미터 표현 |
| 모터 명령 | 생성하지 않음 | command interface를 통해 전달 가능 |
| 하드웨어 상태 | 읽지 않음 | state interface를 통해 전달 |

관절 이름은 두 위치에서 정확히 일치해야 한다.

```text
URDF joint name          = joint1
ros2_control joint name = joint1
controller YAML joint   = joint1
trajectory joint name   = joint1
```

---

## 10. 새 프로젝트에서 ros2_control을 사용하는 최소 구성

새 프로젝트는 처음부터 실제 모터를 연결하기보다 다음 단계로 진행하는 것이 안전하다.

```text
1. URDF/Xacro 모델 검증
2. mock_components/GenericSystem 연결
3. joint_state_broadcaster 검증
4. JointTrajectoryController 검증
5. action goal 전송 검증
6. 실제 hardware plugin으로 교체
7. 낮은 속도와 안전 범위에서 실기기 검증
```

예제 패키지 구조는 다음과 같이 구성할 수 있다.

```text
my_robot_bringup/
├─ config/
│  └─ controllers.yaml
├─ launch/
│  └─ control.launch.py
├─ urdf/
│  ├─ my_robot.urdf.xacro
│  └─ my_robot.ros2_control.xacro
├─ CMakeLists.txt
└─ package.xml
```

description과 bringup을 별도 패키지로 분리해도 된다.
교육용 최소 프로젝트에서는 한 패키지에 두고 시작할 수 있다.

### 10.1 URDF에 ros2_control을 포함한다

`my_robot.urdf.xacro`에서 별도 ros2_control Xacro를 포함한다.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  <xacro:include filename="$(find my_robot_bringup)/urdf/my_robot.ros2_control.xacro"/>

  <!-- base_link, link1, joint1 등의 기구학 모델 -->

  <xacro:my_robot_ros2_control
    name="MyRobotSystem"
    use_mock_hardware="true"/>
</robot>
```

### 10.2 mock hardware용 ros2_control Xacro

`my_robot.ros2_control.xacro`의 최소 예시는 다음과 같다.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro
    name="my_robot_ros2_control"
    params="name use_mock_hardware:=true">

    <ros2_control name="${name}" type="system">
      <hardware>
        <plugin>mock_components/GenericSystem</plugin>
        <param name="calculate_dynamics">false</param>
      </hardware>

      <joint name="joint1">
        <command_interface name="position"/>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
      </joint>

      <joint name="joint2">
        <command_interface name="position"/>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
      </joint>
    </ros2_control>
  </xacro:macro>
</robot>
```

mock hardware는 실제 장치 없이 command와 state interface 연결을 먼저 검증할 때 사용한다.

### 10.3 controller YAML 작성

`config/controllers.yaml`의 최소 예시는 다음과 같다.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    allow_partial_joints_goal: false
```

controller 이름을 `arm_controller`로 지정했기 때문에
생성되는 주요 인터페이스는 다음과 같다.

```text
/arm_controller/follow_joint_trajectory
/arm_controller/joint_trajectory
/arm_controller/controller_state
/arm_controller/query_state
```

### 10.4 launch 파일 작성

`launch/control.launch.py`의 기본 구조는 다음과 같다.

```python
from launch import LaunchDescription
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare('my_robot_bringup')

    xacro_file = PathJoinSubstitution([
        description_share,
        'urdf',
        'my_robot.urdf.xacro',
    ])

    controller_file = PathJoinSubstitution([
        description_share,
        'config',
        'controllers.yaml',
    ])

    robot_description = {
        'robot_description': Command(['xacro ', xacro_file])
    }

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controller_file,
        ],
        output='both',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='both',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='both',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output='both',
    )

    return LaunchDescription([
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])
```

실제 프로젝트에서는 controller manager가 준비되기 전 spawner가 실행되는 문제를 막기 위해
spawner의 대기 기능이나 launch event handler를 함께 사용할 수 있다.

### 10.5 package.xml 의존성

최소한 다음 실행 의존성을 준비한다.

```xml
<exec_depend>controller_manager</exec_depend>
<exec_depend>joint_state_broadcaster</exec_depend>
<exec_depend>joint_trajectory_controller</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>ros2_control</exec_depend>
<exec_depend>ros2_controllers</exec_depend>
<exec_depend>xacro</exec_depend>
```

mock hardware를 사용할 경우 해당 ROS 2 배포판의
`ros2_control` mock components가 설치되어 있어야 한다.

### 10.6 설정 파일 설치

ament CMake 패키지라면 `CMakeLists.txt`에서 다음 디렉터리를 설치해야 한다.

```cmake
install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)
```

파일이 소스 트리에는 있지만 `install/share/<package>`에 설치되지 않으면
`FindPackageShare`로 찾을 수 없다.

---

## 11. 빌드와 mock hardware 검증

### 11.1 빌드

```bash
cd ~/my_robot_ws
colcon build --symlink-install
source install/setup.bash
```

### 11.2 Xacro와 URDF 구문 확인

```bash
xacro \
  $(ros2 pkg prefix --share my_robot_bringup)/urdf/my_robot.urdf.xacro \
  > /tmp/my_robot.urdf

check_urdf /tmp/my_robot.urdf
```

확인할 내용:

- 일반 URDF joint가 존재하는가?
- `<ros2_control>` 블록이 최종 URDF에 포함되는가?
- joint 이름이 controller YAML과 일치하는가?
- hardware plugin 이름이 정확한가?

### 11.3 bringup

```bash
ros2 launch my_robot_bringup control.launch.py
```

### 11.4 하드웨어 구성 확인

```bash
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
```

정상이라면 다음과 같은 interface를 확인할 수 있어야 한다.

```text
command interfaces
  joint1/position [claimed]
  joint2/position [claimed]

state interfaces
  joint1/position
  joint1/velocity
  joint2/position
  joint2/velocity
```

`[claimed]`는 활성 controller가 해당 command interface를 점유했다는 뜻이다.

### 11.5 controller 상태 확인

```bash
ros2 control list_controllers
```

두 controller가 모두 `active`인지 확인한다.

```text
joint_state_broadcaster  ...  active
arm_controller           ...  active
```

### 11.6 상태 토픽 확인

```bash
ros2 topic echo /joint_states
```

`name`, `position`, `velocity` 배열의 순서와 길이를 확인한다.

### 11.7 액션 서버 확인

```bash
ros2 action list -t
ros2 action info /arm_controller/follow_joint_trajectory -t
```

액션 타입은 다음과 같아야 한다.

```text
control_msgs/action/FollowJointTrajectory
```

### 11.8 시험 goal 전송

mock hardware에서 다음과 같이 두 관절 목표를 보낼 수 있다.

```bash
ros2 action send_goal \
  /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [joint1, joint2],
      points: [
        {
          positions: [0.5, -0.5],
          velocities: [0.0, 0.0],
          time_from_start: {sec: 3}
        }
      ]
    }
  }" \
  --feedback
```

실기기에 적용하기 전에 mock hardware에서 다음을 확인한다.

- goal이 accepted 되는가?
- feedback이 오는가?
- goal이 success로 끝나는가?
- `/joint_states`가 목표 위치를 따라가는가?

---

## 12. 기존 프로젝트에 새 관절을 추가하는 절차

예를 들어 `joint5`를 추가한다고 가정한다.

### 12.1 링크와 일반 URDF joint 추가

```xml
<link name="link5">
  <!-- visual, collision, inertial -->
</link>

<joint name="joint5" type="revolute">
  <parent link="link4"/>
  <child link="link5"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="5.0" velocity="1.0"/>
</joint>
```

### 12.2 ros2_control joint interface 추가

```xml
<joint name="joint5">
  <command_interface name="position"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>
</joint>
```

### 12.3 controller YAML에 관절 추가

```yaml
arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
```

### 12.4 실제 hardware plugin이 joint5를 처리하도록 수정

mock hardware는 URDF의 interface 선언을 이용해 일반적으로 자동 구성할 수 있다.
그러나 실제 장비에서는 hardware plugin이 `joint5`의 장치 주소와 단위를 알아야 한다.

Dynamixel 기반이라면 다음 항목도 함께 검토한다.

- Dynamixel ID
- 모델명
- baud rate
- operating mode
- position/velocity/current 단위 변환
- joint 방향과 offset
- 최소·최대 위치
- torque enable/disable 정책
- 통신 실패 timeout

OpenManipulator-X 형식에서는 새 Dynamixel에 대응하는 `<gpio>` 정보,
joint와 transmission 매핑, `number_of_joints`, `number_of_transmissions` 같은
하드웨어별 파라미터도 함께 변경해야 할 수 있다.

단순히 controller YAML에 `joint5`만 추가하면 실제 hardware interface가
`joint5/position`을 제공하지 못해 controller 활성화가 실패한다.

### 12.5 사용자 명령과 MoveIt 설정 갱신

다음 위치에 관절 목록이 별도로 존재하는지도 확인한다.

- 초기 자세 YAML
- trajectory 생성 코드
- SRDF planning group
- MoveIt controller 설정
- joint limits 설정
- RViz 설정
- ros2_control mimic/transmission 설정

`FollowJointTrajectory` goal의 `joint_names`와 `positions` 배열 길이도 맞아야 한다.

---

## 13. 실제 하드웨어 plugin을 새로 만드는 경우

지원되는 hardware plugin이 없다면 직접 구현해야 한다.

대표적인 system hardware plugin 구조는 다음과 같다.

```text
my_robot_hardware/
├─ include/my_robot_hardware/
│  └─ my_robot_system.hpp
├─ src/
│  └─ my_robot_system.cpp
├─ my_robot_hardware.xml
├─ CMakeLists.txt
└─ package.xml
```

일반적으로 `hardware_interface::SystemInterface`를 상속하고
다음 lifecycle과 입출력 기능을 구현한다.

| 기능 | 역할 |
|---|---|
| 초기화 | URDF hardware parameter와 joint 정보를 읽음 |
| configure | 장치 포트와 내부 버퍼 준비 |
| activate | 모터 활성화 또는 현재 위치로 command 초기화 |
| deactivate | 안전 정지와 torque 처리 |
| state interface export | position, velocity, effort 등의 상태 제공 |
| command interface export | position, velocity, effort 등의 명령 입력 제공 |
| `read()` | 장치 상태를 읽어 state buffer 갱신 |
| `write()` | command buffer를 실제 장치로 전송 |

개념적인 제어 루프는 다음과 같다.

```text
controller_manager 주기 시작
  │
  ├─ hardware.read()
  │    └─ 엔코더 → state interface
  │
  ├─ controller.update()
  │    └─ 목표와 상태 계산 → command interface
  │
  └─ hardware.write()
       └─ command interface → 모터 드라이버
```

pluginlib 등록도 필요하다.

```cpp
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  my_robot_hardware::MyRobotSystem,
  hardware_interface::SystemInterface)
```

plugin XML의 개념적인 예는 다음과 같다.

```xml
<library path="my_robot_hardware">
  <class
    name="my_robot_hardware/MyRobotSystem"
    type="my_robot_hardware::MyRobotSystem"
    base_class_type="hardware_interface::SystemInterface">
    <description>Hardware interface for My Robot</description>
  </class>
</library>
```

URDF에서는 등록한 plugin 이름을 사용한다.

```xml
<hardware>
  <plugin>my_robot_hardware/MyRobotSystem</plugin>
  <param name="port">/dev/ttyUSB0</param>
  <param name="baud_rate">1000000</param>
</hardware>
```

controller plugin과 hardware plugin은 서로 다른 종류다.

| 종류 | base class | YAML/URDF 위치 | 예 |
|---|---|---|---|
| controller plugin | `controller_interface::ControllerInterface` | controller YAML의 `type` | `JointTrajectoryController` |
| hardware plugin | `hardware_interface::SystemInterface` 등 | URDF `<hardware><plugin>` | `DynamixelHardware` |

---

## 14. controller를 직접 새로 만들어야 하는 경우

일반적인 다관절 위치 trajectory라면
`joint_trajectory_controller/JointTrajectoryController`를 재사용하면 된다.

다음과 같은 경우에는 custom controller plugin을 검토할 수 있다.

- 특수한 힘/토크 제어 법칙이 필요함
- 여러 센서와 command interface를 동시에 결합해야 함
- 일반 trajectory controller로 표현할 수 없는 실시간 제어가 필요함
- 특수한 안전 제약과 상태 머신이 controller 주기 안에 필요함

단순히 새로운 로봇이나 새로운 관절을 만들었다는 이유만으로
controller를 새로 구현할 필요는 없다.

대부분은 다음 세 부분만 로봇에 맞게 변경하면 된다.

1. URDF/Xacro의 링크와 관절
2. `<ros2_control>`의 hardware 및 joint interface
3. controller YAML의 관절 목록과 controller 설정

---

## 15. 자주 발생하는 오류와 원인

### 15.1 controller가 `unconfigured` 또는 `inactive` 상태

확인:

```bash
ros2 control list_controllers
```

가능한 원인:

- controller 파라미터 누락
- YAML 들여쓰기 또는 노드 이름 오류
- 존재하지 않는 joint 이름
- 필요한 state/command interface가 없음
- 다른 controller가 command interface를 이미 점유

### 15.2 `Not acceptable command interfaces combination`

controller가 요청한 interface 조합을 hardware가 제공하지 않는 경우다.

예:

```text
controller 요청: joint1/position
hardware 제공:  joint1/velocity
```

확인:

```bash
ros2 control list_hardware_interfaces
```

URDF `<ros2_control>`과 controller YAML을 함께 비교해야 한다.

### 15.3 액션 서버가 보이지 않음

확인 순서:

```bash
ros2 control list_controllers
ros2 action list -t
ros2 node info /arm_controller
```

가능한 원인:

- `joint_trajectory_controller` plugin 미설치
- spawner가 실행되지 않음
- controller load/configure 실패
- controller 이름이 예상과 다름
- namespace 또는 `ROS_DOMAIN_ID` 불일치

액션 이름은 controller 이름에서 만들어지므로 실제 controller 이름부터 확인한다.

### 15.4 goal이 rejected 됨

가능한 원인:

- controller가 active가 아님
- goal의 joint 이름이 controller 관절과 다름
- `allow_partial_joints_goal: false`인데 일부 관절만 보냄
- `joint_names`와 positions/velocities 배열 길이가 다름
- trajectory 시간이 증가하지 않음
- 마지막 점의 velocity 조건 위반

### 15.5 모터는 움직이지만 `/joint_states`가 이상함

가능한 원인:

- hardware `read()` 단위 변환 오류
- joint 방향 또는 offset 오류
- `joint_state_broadcaster` 비활성
- 동일한 `/joint_states`에 다른 publisher가 함께 발행
- joint 이름과 배열 인덱스 매핑 오류

### 15.6 mock에서는 되지만 실기기에서 실패

mock hardware는 통신, 단위 변환, torque, 장치 ID 오류를 재현하지 않는다.

추가 확인:

- 장치 권한
- `/dev/ttyUSB*` 포트
- baud rate
- 모터 ID 중복
- operating mode
- torque 상태
- 전원과 케이블
- 제어 주기 대비 통신 속도
- 위치 단위와 방향

---

## 16. 새 프로젝트 권장 검증 순서

한 단계가 통과한 후 다음 단계로 이동한다.

### 단계 1. 모델

```bash
xacro ... > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

통과 조건:

- URDF 파싱 성공
- 링크와 joint 연결 정상
- `<ros2_control>` 포함

### 단계 2. hardware plugin

```bash
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
```

통과 조건:

- hardware component가 active 또는 의도한 lifecycle 상태
- 모든 joint state/command interface 존재

### 단계 3. broadcaster

```bash
ros2 control list_controllers
ros2 topic echo /joint_states
```

통과 조건:

- `joint_state_broadcaster` active
- 모든 관절 상태 수신

### 단계 4. controller

```bash
ros2 control list_controllers
```

통과 조건:

- `arm_controller` active
- 필요한 command interface가 claimed

### 단계 5. action

```bash
ros2 action info /arm_controller/follow_joint_trajectory -t
```

통과 조건:

- action server 1개 이상
- 타입이 `control_msgs/action/FollowJointTrajectory`

### 단계 6. mock motion

작은 목표를 보내 goal accepted, feedback, result를 확인한다.

### 단계 7. 실기기 정적 확인

모터 torque를 켜기 전에 다음을 확인한다.

- 현재 위치가 정상 범위인가?
- radian 변환과 방향이 맞는가?
- zero offset이 맞는가?
- joint limit이 실제 장치와 맞는가?

### 단계 8. 실기기 저속 동작

- 한 관절씩
- 작은 각도
- 낮은 profile velocity
- 즉시 전원을 차단할 수 있는 환경
- 충돌하지 않는 자세

이 조건에서 먼저 확인한 뒤 다관절 trajectory로 확대한다.

---

## 17. OpenManipulator-X에서 확인할 명령 모음

bringup:

```bash
cd /home/ju/kong_manipulator2026/open_manipulator_ws
source install/setup.bash

ros2 launch open_manipulator_bringup open_manipulator_x.launch.py
```

노드:

```bash
ros2 node list
ros2 node info /controller_manager
ros2 node info /arm_controller
```

controller:

```bash
ros2 control list_controllers
ros2 control list_controller_types
```

hardware:

```bash
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
```

action:

```bash
ros2 action list -t
ros2 action info /arm_controller/follow_joint_trajectory -t
```

상태:

```bash
ros2 topic echo /joint_states
ros2 topic hz /joint_states
```

파라미터:

```bash
ros2 param list /arm_controller
ros2 param get /arm_controller joints
ros2 param get /arm_controller command_interfaces
ros2 param get /arm_controller state_interfaces
```

---

## 18. 핵심 정리

1. `hardware_controller_manager.yaml`은 액션 서버 소스 코드를 생성하지 않는다.
2. YAML의 `type`은 pluginlib에 등록된 C++ controller plugin을 선택한다.
3. `spawner`가 controller manager에 load/configure/activate를 요청해야 한다.
4. `JointTrajectoryController`가 configure되면서
   `<controller_name>/follow_joint_trajectory` 액션 서버를 만든다.
5. 현재 controller 이름이 `arm_controller`이므로
   `/arm_controller/follow_joint_trajectory`가 생성된다.
6. `joint_trajectory_executor`는 해당 서버에 초기 자세 goal을 보내는
   일회성 액션 클라이언트다.
7. 일반 URDF joint만 추가해서는 제어할 수 없다.
8. 새 관절은 URDF, `<ros2_control>`, controller YAML,
   실제 hardware mapping, 사용자 명령 설정에 일관되게 추가해야 한다.
9. 새 프로젝트는 mock hardware에서 controller와 action을 먼저 검증한 뒤
   실제 hardware plugin으로 교체하는 것이 안전하다.
10. 문제가 생기면 노드 목록만 보지 말고 controller 상태,
    hardware interface, action server를 각각 분리해 확인해야 한다.

