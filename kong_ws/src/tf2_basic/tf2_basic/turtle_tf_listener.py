# ros2 run turtlesim turtlesim_node
# rviz2
# ros2 run tf2_basic dynamic_turtle_tf2_broadcaster -> turtle1 tf
# ros2 run tf2_basic turtle_tf_listener
# ros2 run turtlesim turtle_teleop_key
# 과제 : turtlesim 을 따라가는 두번째 turtle2 를 생성
# turtle spawn 은 service 코드 쓰세요.
# timer 는 1.0 간격으로 회전과 정지 직진을  tf look-up 정보로 구현
# turtle1 turtle2 tf 모두 발행!!
# ros2 run tf2_basic dynamic_turtle_tf2_broadcaster --ros-args -p turtle_name:=turtle2

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_py import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from turtlesim.srv import Spawn


class M_pub(Node):
    def __init__(self):
        super().__init__("turtle_tf2_frame_listener")

        # Declare and acquire `target_frame` parameter
        self.target_frame = (
            self.declare_parameter("target_frame", "turtle1").get_parameter_value().string_value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a client to spawn a turtle
        self.spawner = self.create_client(Spawn, "spawn")
        # Boolean values to store the information
        # if the service for spawning turtle is available
        self.turtle_spawning_service_ready = False
        # if the turtle was successfully spawned
        self.turtle_spawned = False

        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(Twist, "turtle2/cmd_vel", 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = "turtle2"

        if self.turtle_spawning_service_ready:
            if self.turtle_spawned:
                # Look up for the transformation between target_frame and turtle2 frames
                # and send velocity commands for turtle2 to reach target_frame
                try:
                    t = self.tf_buffer.lookup_transform(
                        to_frame_rel, from_frame_rel, rclpy.time.Time()
                    )
                except TransformException as ex:
                    self.get_logger().info(
                        f"Could not transform {to_frame_rel} to {from_frame_rel}: {ex}"
                    )
                    return

                msg = Twist()
                scale_rotation_rate = 1.0
                msg.angular.z = scale_rotation_rate * math.atan2(
                    t.transform.translation.y, t.transform.translation.x
                )

                scale_forward_speed = 0.5
                msg.linear.x = scale_forward_speed * math.sqrt(
                    t.transform.translation.x**2 + t.transform.translation.y**2
                )

                self.publisher.publish(msg)
            else:
                if self.result.done():
                    self.get_logger().info(f"Successfully spawned {self.result.result().name}")
                    self.turtle_spawned = True
                else:
                    self.get_logger().info("Spawn is not finished")
        else:
            if self.spawner.service_is_ready():
                # Initialize request with turtle name and coordinates
                # Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
                request = Spawn.Request()
                request.name = "turtle2"
                request.x = float(4)
                request.y = float(2)
                request.theta = float(0)
                # Call request
                self.result = self.spawner.call_async(request)
                self.turtle_spawning_service_ready = True
            else:
                # Check if the service is ready
                self.get_logger().info("Service is not ready")


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = M_pub()
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        print("키보드 인터럽트")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
