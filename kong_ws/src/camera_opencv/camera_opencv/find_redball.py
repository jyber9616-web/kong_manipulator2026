import math
import time

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import (
    FollowJointTrajectory,
    FollowJointTrajectory_GetResult_Response,
    GripperCommand,
    GripperCommand_GetResult_Response,
)
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class Manipulator_pub(Node):
    def __init__(self):
        super().__init__("manipulator_pub")  # 노드 이름
        self.joint_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.gripper_client = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd")
        self.arm_joint_names = ["joint1", "joint2", "joint3", "joint4"]
        self.joint_state_subscription = self.create_subscription(
            JointState, "joint_states", self.joint_callback, 10
        )
        self.current_joint_position = [0.0] * len(self.arm_joint_names)
        self.current_gripper_position = 0.0
        self.joint_state_received = False
        self.brige = CvBridge()
        self.joint_goal_in_flight = False
        self.last_command_time = 0.0
        self.command_interval_sec = 1.0
        self.last_detection_log_time = 0.0
        self.center_deadband_px = 20
        self.gripper_closed = False
        self.stable_detection_count = 0
        self.ball_radius_m = 0.02
        self.focal_length_px = 640.0 / (2.0 * math.tan(1.0472 / 2.0))
        self.create_subscription(Image, "/gripper_camera/image_raw", self.image_callback, 10)

    def image_callback(self, msg: Image):
        try:
            img_sub = self.brige.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"카메라 변환 실패: {exc}")
            return

        hsv = cv2.cvtColor(img_sub, cv2.COLOR_BGR2HSV)
        # 빨강은 HSV 색상환의 양 끝(0도와 180도)에 걸쳐 있으므로 두 구간을 사용합니다.
        lower_red_1 = np.array([0, 40, 40], dtype=np.uint8)
        upper_red_1 = np.array([10, 255, 255], dtype=np.uint8)
        lower_red_2 = np.array([170, 40, 40], dtype=np.uint8)
        upper_red_2 = np.array([179, 255, 255], dtype=np.uint8)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_red_1, upper_red_1),
            cv2.inRange(hsv, lower_red_2, upper_red_2),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)
            if area > 20:
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2
                pixel_radius = math.sqrt(area / math.pi)
                distance_m = (self.focal_length_px * self.ball_radius_m) / pixel_radius
                cv2.rectangle(img_sub, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(img_sub, (center_x, center_y), 5, (0, 255, 0), -1)
                now = time.monotonic()
                if now - self.last_detection_log_time >= 0.75:
                    self.get_logger().info(
                        f"중심 좌표: x={center_x}, y={center_y}, "
                        f"area={area:.1f}, 추정 거리={distance_m:.2f} m"
                    )
                    self.last_detection_log_time = now
                self.control_from_detection(
                    center_x, center_y, img_sub.shape[1], img_sub.shape[0], distance_m
                )
            else:
                self.stable_detection_count = 0
        else:
            self.stable_detection_count = 0
        cv2.imshow("img", img_sub)
        cv2.imshow("mask", mask)
        cv2.waitKey(1)

    @staticmethod
    def clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def control_from_detection(
        self, center_x: int, center_y: int, image_width: int, image_height: int, distance_m: float
    ):
        """검출된 공의 영상 오차를 관절 명령으로 변환합니다."""
        if not self.joint_state_received:
            return

        now = time.monotonic()
        if self.joint_goal_in_flight or now - self.last_command_time < self.command_interval_sec:
            return

        error_x = center_x - image_width / 2.0
        error_y = center_y - image_height / 2.0
        if abs(error_x) <= self.center_deadband_px and abs(error_y) <= self.center_deadband_px:
            self.stable_detection_count += 1
            if distance_m < 0.35 and self.stable_detection_count >= 5 and not self.gripper_closed:
                self.get_logger().info("공이 가까워졌습니다. 그리퍼를 닫습니다.")
                self.gripper_closed = self.move_gripper(-0.01)
            return

        self.stable_detection_count = 0
        target = list(self.current_joint_position)

        # x 오차는 joint1의 yaw로 보정합니다. 화면 오른쪽에 있으면 joint1을 음의 방향으로 돌립니다.
        if abs(error_x) > self.center_deadband_px:
            target[0] = self.clamp(target[0] - 0.0025 * error_x, -math.pi, math.pi)

        # y 오차는 joint2~4의 pitch를 나누어 보정합니다.
        if abs(error_y) > self.center_deadband_px:
            pitch_step = self.clamp(0.0005 * error_y, -0.06, 0.06)
            target[1] = self.clamp(target[1] + pitch_step * 0.5, -1.5, 1.5)
            target[2] = self.clamp(target[2] + pitch_step * 0.3, -1.5, 1.4)
            target[3] = self.clamp(target[3] + pitch_step * 0.2, -1.7, 1.97)

        if all(abs(a - b) < 1e-5 for a, b in zip(target, self.current_joint_position)):
            return

        point = JointTrajectoryPoint()
        point.positions = target
        point.time_from_start.sec = 1
        if self.move_joint(point):
            self.joint_goal_in_flight = True
            self.last_command_time = now
            self.get_logger().info(f"관절 명령: {[round(value, 3) for value in target]}")

    def joint_callback(self, msg: JointState):
        for index, joint_name in enumerate(self.arm_joint_names):
            if joint_name in msg.name:
                self.current_joint_position[index] = msg.position[msg.name.index(joint_name)]
        self.joint_state_received = all(
            joint_name in msg.name for joint_name in self.arm_joint_names
        )

    def move_gripper(self, position: float, max_effort=10.0, timeout_sec=5.0):
        if not self.gripper_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().warning("gripper_controller Action 서버를 찾지 못했습니다.")
            return False
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        send_goal_future = self.gripper_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_callback)
        return True

    def goal_callback(self, future: Future):
        goal_handle = future.result()  # type: ignore
        if not goal_handle.accepted:
            self.get_logger().warning("그리퍼 명령이 거부되었습니다.")
            return
        self.gripper_goal_handle = goal_handle
        self.gripper_result_future = goal_handle.get_result_async()
        self.gripper_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(
        self,
        msg: GripperCommand.Impl.FeedbackMessage,
    ):
        feedback: GripperCommand.Feedback = msg.feedback
        self.get_logger().info(f"{feedback.position}")

    def get_result_callback(self, future: Future):
        result: GripperCommand_GetResult_Response = (
            future.result()  # type: ignore
        )
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"succeeded result: {result.result.position}")
        elif result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("aborted!!")
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("canceled!!")

    def move_joint(self, point: JointTrajectoryPoint):
        if not self.joint_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning("joint_controller Action 서버를 찾지 못했습니다.")
            return False
        goal = FollowJointTrajectory.Goal()
        # 이 노드는 기본적으로 wall clock을 사용하고 Gazebo는 sim time을 사용하므로,
        # wall clock을 trajectory stamp에 넣으면 목표가 영원히 미래로 예약될 수 있습니다.
        # stamp=0은 컨트롤러가 즉시 시작하도록 합니다.
        goal.trajectory.header.stamp.sec = 0
        goal.trajectory.header.stamp.nanosec = 0
        goal.trajectory.header.frame_id = "move_manipulator"
        goal.trajectory.joint_names = self.arm_joint_names
        goal.trajectory.points.append(point)  # type: ignore

        send_goal_future = self.joint_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_joint_callback)
        return True

    def goal_joint_callback(self, future: Future):
        goal_handle = future.result()  # type: ignore
        if not goal_handle.accepted:
            self.joint_goal_in_flight = False
            self.get_logger().warning("관절 명령이 거부되었습니다.")
            return
        self.joint_goal_handle = goal_handle
        self.joint_result_future = goal_handle.get_result_async()
        self.joint_result_future.add_done_callback(self.get_joint_result_callback)
        self.get_logger().info("관절 명령이 수락되었습니다.")

    def feedback_joint_callback(
        self,
        msg: FollowJointTrajectory.Impl.FeedbackMessage,
    ):
        feedback: FollowJointTrajectory.Feedback = msg.feedback
        self.get_logger().info(f"{feedback.actual.positions}")

    def get_joint_result_callback(self, future: Future):
        result: FollowJointTrajectory_GetResult_Response = (
            future.result()  # type: ignore
        )
        self.joint_goal_in_flight = False
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"succeeded result: {result.result.error_string}")
        elif result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("aborted!!")
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("canceled!!")
        else:
            self.get_logger().warning(f"관절 명령 상태 코드: {result.status}")


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = Manipulator_pub()
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
