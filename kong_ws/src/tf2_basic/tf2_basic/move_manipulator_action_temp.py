import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class ManipulatorActionClient(Node):
    """팔과 그리퍼를 ROS 2 Action으로 제어하는 예제 노드."""

    def __init__(self):
        super().__init__("manipulator_action_client")

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            "/gripper_controller/gripper_cmd",
        )

        self.joint_state_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10,
        )

        self.joint_names = ["joint1", "joint2", "joint3", "joint4"]
        self.arm_positions = [
            [
                0.8995922516973869,
                -0.5832234352774157,
                -0.26300971181849175,
                -0.6258641614575486,
            ],
            [
                0.4795922516973865,
                -0.883223435277416,
                0.3969902881815083,
                0.8941358385424522,
            ],
        ]
        self.gripper_positions = [0.019, -0.01]

        self.current_joint_positions = {}
        self.duration_sec = 2.0
        self.pose_index = 0
        self.busy = False
        self.pending_actions = set()
        self.server_warning_printed = False

        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        """이전 팔·그리퍼 동작이 모두 끝났을 때 다음 동작을 시작한다."""
        if self.busy:
            return

        arm_ready = self.arm_client.server_is_ready()
        gripper_ready = self.gripper_client.server_is_ready()
        if not arm_ready or not gripper_ready:
            if not self.server_warning_printed:
                self.get_logger().warning(
                    f"Action 서버를 기다리는 중입니다: arm={arm_ready}, gripper={gripper_ready}"
                )
                self.server_warning_printed = True
            return

        self.server_warning_printed = False
        self.busy = True
        self.pending_actions = {"arm", "gripper"}

        index = self.pose_index
        self.pose_index = (self.pose_index + 1) % len(self.arm_positions)

        self.get_logger().info(f"동작 {index + 1}을 시작합니다.")
        self.send_arm_goal(self.arm_positions[index])
        self.send_gripper_goal(self.gripper_positions[index])

    def send_arm_goal(self, positions):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(position) for position in positions]
        point.time_from_start.sec = int(self.duration_sec)
        point.time_from_start.nanosec = int(
            (self.duration_sec - int(self.duration_sec)) * 1_000_000_000
        )
        goal.trajectory.points.append(point)

        future = self.arm_client.send_goal_async(
            goal,
            feedback_callback=self.arm_feedback_callback,
        )
        future.add_done_callback(self.arm_goal_callback)

    def send_gripper_goal(self, position, max_effort=10.0):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)

        future = self.gripper_client.send_goal_async(
            goal,
            feedback_callback=self.gripper_feedback_callback,
        )
        future.add_done_callback(self.gripper_goal_callback)

    def arm_goal_callback(self, future: Future):
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Arm Goal 전송 실패: {error}")
            self.action_finished("arm")
            return

        if not goal_handle.accepted:
            self.get_logger().error("Arm Goal이 거부되었습니다.")
            self.action_finished("arm")
            return

        self.get_logger().info("Arm Goal이 수락되었습니다.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.arm_result_callback)

    def gripper_goal_callback(self, future: Future):
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Gripper Goal 전송 실패: {error}")
            self.action_finished("gripper")
            return

        if not goal_handle.accepted:
            self.get_logger().error("Gripper Goal이 거부되었습니다.")
            self.action_finished("gripper")
            return

        self.get_logger().info("Gripper Goal이 수락되었습니다.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_result_callback)

    def arm_feedback_callback(self, feedback_message):
        feedback = feedback_message.feedback
        self.get_logger().debug(
            "Arm feedback: "
            f"actual={list(feedback.actual.positions)}, "
            f"error={list(feedback.error.positions)}"
        )

    def gripper_feedback_callback(self, feedback_message):
        feedback = feedback_message.feedback
        self.get_logger().debug(
            f"Gripper feedback: position={feedback.position:.4f}, effort={feedback.effort:.4f}"
        )

    def arm_result_callback(self, future: Future):
        try:
            wrapped_result = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Arm 결과 수신 실패: {error}")
            self.action_finished("arm")
            return

        self.log_action_result("Arm", wrapped_result.status)
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"Arm error_code={wrapped_result.result.error_code}, "
                f"error_string={wrapped_result.result.error_string}"
            )
        self.action_finished("arm")

    def gripper_result_callback(self, future: Future):
        try:
            wrapped_result = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Gripper 결과 수신 실패: {error}")
            self.action_finished("gripper")
            return

        self.log_action_result("Gripper", wrapped_result.status)
        result = wrapped_result.result
        self.get_logger().info(
            "Gripper result: "
            f"position={result.position:.4f}, "
            f"effort={result.effort:.4f}, "
            f"stalled={result.stalled}, "
            f"reached_goal={result.reached_goal}"
        )
        self.action_finished("gripper")

    def log_action_result(self, name, status):
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"{name} 동작이 완료되었습니다.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f"{name} 동작이 중단되었습니다.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning(f"{name} 동작이 취소되었습니다.")
        else:
            self.get_logger().warning(f"{name} 결과 상태: {status}")

    def action_finished(self, name):
        self.pending_actions.discard(name)
        if not self.pending_actions:
            self.busy = False
            self.get_logger().info("Arm과 Gripper 동작 처리가 모두 끝났습니다.")

    def joint_callback(self, msg: JointState):
        self.current_joint_positions = dict(zip(msg.name, msg.position))


def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorActionClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트로 종료합니다.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
