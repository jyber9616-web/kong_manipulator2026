import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class M_pub(Node):
    def __init__(self):
        super().__init__("move_u2d2")

        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.start_time = self.get_clock().now()
        self.count = 0
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1_000_000_000

        shoulder_angle = 0.8 * math.sin(elapsed)
        elbow_angle = 1.5 * math.sin(elapsed * 1.5)

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = [
            "right_front_wheel_joint",
            "right_back_wheel_joint",
            "left_front_wheel_joint",
            "left_back_wheel_joint",
            "gripper_extension",
            "left_gripper_joint",
            "right_gripper_joint",
            "head_swivel",
            "right_shoulder_joint",
            "elbow_joint",
            "left_shoulder_joint",
            "left_elbow_joint",
        ]
        msg.position = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            shoulder_angle,
            elbow_angle,
            shoulder_angle,
            elbow_angle,
        ]

        self.publisher.publish(msg)

        self.count += 1
        if self.count >= 20:
            self.get_logger().info(
                f"어깨={shoulder_angle:.2f}, 팔꿈치={elbow_angle:.2f}"
            )
            self.count = 0


def main(args=None):
    rclpy.init(args=args)
    node = M_pub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("키보드 인터럽트")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
