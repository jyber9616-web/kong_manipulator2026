import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Color, Pose


class Move_tutle(Node):
    def __init__(self):
        super().__init__("move_turtle")  # 노드 이름
        # timer 등록
        self.create_timer(0.1, self.timer_callback)
        self.pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.create_subscription(Pose, "turtle1/pose", self.pose_callback, 10)
        self.create_subscription(Color, "turtle1/color_sensor", self.color_callback, 10)
        self.count = 0.0
        self.pose = Pose()
        self.color = Color()

    def timer_callback(self):
        msg = Twist()  # DDS 에 보낼 객체 초기화
        msg.linear.x = 0.0 + self.count
        msg.angular.z = 1.0
        self.pub.publish(msg)  # DDS 로 보내는 기능 수행
        # AI 활용해서 msg 재미 있게 움직이도록 수정 self.pose, self.color
        # 별표로 움직이기 지그재그로 움직이기 ...
        self.count += 0.01
        if self.count > 3.0:
            self.count = 0.0

    def pose_callback(self, msg: Pose):
        self.pose = msg

    def color_callback(self, msg: Color):
        self.color = msg


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = Move_tutle()
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()