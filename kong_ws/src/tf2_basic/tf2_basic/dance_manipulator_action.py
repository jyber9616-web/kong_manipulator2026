"""Dance with OpenMANIPULATOR-X using FollowJointTrajectory actions."""

import random
from pathlib import Path

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
JOINT_LIMITS = [
    (-3.14159, 3.14159),
    (-1.5, 1.5),
    (-1.5, 1.4),
    (-1.7, 1.97),
]


def load_positions(file_path):
    """Load and validate four arm positions from the local text file."""
    positions = {}
    for line_number, source_line in enumerate(
        Path(file_path).read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = source_line.strip()
        if not line or line.startswith("#"):
            continue
        try:
            name, raw_values = line.split(":", maxsplit=1)
            values = [float(value) for value in raw_values.split(",")]
        except ValueError as exc:
            raise ValueError(
                f"{line_number}번째 줄 형식이 잘못됐습니다."
            ) from exc
        if len(values) != len(JOINT_NAMES):
            raise ValueError(f"{name}: 관절 위치가 4개여야 합니다.")
        for joint_name, value, limits in zip(
            JOINT_NAMES, values, JOINT_LIMITS
        ):
            if not limits[0] <= value <= limits[1]:
                raise ValueError(
                    f"{name}의 {joint_name}={value}가 제한 밖입니다."
                )
        positions[name.strip()] = values
    if "home" not in positions or len(positions) < 2:
        raise ValueError("home과 하나 이상의 춤 위치가 필요합니다.")
    return positions


class DanceManipulatorAction(Node):
    """Send the next random pose after each action finishes."""

    def __init__(self):
        super().__init__("dance_manipulator_action")
        self.declare_parameter("moves", 20)
        self.declare_parameter("seed", -1)

        data_file = Path(__file__).with_name("dance_positions.txt")
        self.positions = load_positions(data_file)
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )

        seed = self.get_parameter("seed").value
        self.random = random.Random(None if seed < 0 else seed)
        self.moves = self.get_parameter("moves").value
        self.count = 0
        self.last_pose = None
        self.returning_home = False

        self.start_timer = self.create_timer(1.0, self.start_dance)
        self.get_logger().info(
            f"{len(self.positions)}개 위치를 읽었습니다. 액션 서버를 확인합니다."
        )

    def start_dance(self):
        """Wait for the controller action server and start once."""
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning(
                "/arm_controller/follow_joint_trajectory 서버를 기다리는 중입니다."
            )
            return

        self.start_timer.cancel()
        self.get_logger().info("액션 서버 연결 완료: 춤을 시작합니다.")
        self.send_next_goal()

    def send_next_goal(self):
        """Choose the next random pose, or return home when finished."""
        if self.count >= self.moves:
            pose_name = "home"
            duration = 2.5
            self.returning_home = True
        else:
            candidates = [
                name
                for name in self.positions
                if name != "home" and name != self.last_pose
            ]
            pose_name = self.random.choice(candidates)
            duration = self.random.uniform(1.8, 2.4)
            self.last_pose = pose_name
            self.count += 1

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = self.positions[pose_name]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int(duration % 1 * 1_000_000_000)
        goal.trajectory.points = [point]

        self.get_logger().info(
            f"목표 {self.count}/{self.moves}: {pose_name}, {duration:.1f}초"
        )
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Check whether the controller accepted the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("컨트롤러가 액션 목표를 거부했습니다.")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Continue only after the current motion succeeds."""
        response = future.result()
        result = response.result
        succeeded = (
            response.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        )
        if not succeeded:
            self.get_logger().error(
                f"동작 실패: status={response.status}, "
                f"error_code={result.error_code}, {result.error_string}"
            )
            return

        if self.returning_home:
            self.get_logger().info("춤 종료: home 위치 복귀 완료")
            rclpy.shutdown()
            return

        self.send_next_goal()


def main(args=None):
    """Run the action-based dance node."""
    rclpy.init(args=args)
    node = DanceManipulatorAction()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
