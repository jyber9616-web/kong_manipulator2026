import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose


class DriftTurtle(Node):
    def __init__(self):
        super().__init__("drift_turtle")

        self.cmd_pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.create_subscription(Pose, "turtle1/pose", self.pose_callback, 10)
        self.create_timer(0.05, self.timer_callback)

        self.pose = None
        self.time = 0.0

    def pose_callback(self, msg: Pose):
        self.pose = msg

    def timer_callback(self):
        self.time += 0.05
        cmd = Twist()

        if self.pose is not None and self.near_wall():
            # Turn sharply toward the center when approaching a wall.
            target_angle = math.atan2(5.5 - self.pose.y, 5.5 - self.pose.x)
            angle_error = math.atan2(
                math.sin(target_angle - self.pose.theta),
                math.cos(target_angle - self.pose.theta),
            )
            cmd.linear.x = 1.2
            cmd.angular.z = 3.5 * angle_error
        else:
            # Combine waves with different periods for a drifting path.
            cmd.linear.x = 2.2 + 0.9 * math.sin(self.time * 2.7)
            cmd.angular.z = (
                1.8 * math.sin(self.time * 1.4)
                + 0.9 * math.cos(self.time * 3.7)
            )

            # Add a short turbo burst every eight seconds.
            if self.time % 8.0 < 0.7:
                cmd.linear.x = 4.0
                cmd.angular.z += 2.5 * math.sin(self.time * 8.0)

        self.cmd_pub.publish(cmd)

    def near_wall(self):
        margin = 1.2
        return (
            self.pose.x < margin
            or self.pose.x > 11.0 - margin
            or self.pose.y < margin
            or self.pose.y > 11.0 - margin
        )


def main(args=None):
    rclpy.init(args=args)
    node = DriftTurtle()

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
