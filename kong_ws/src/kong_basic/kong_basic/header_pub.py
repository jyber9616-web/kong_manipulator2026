import rclpy
from rclpy.node import Node
from std_msgs.msg import Header


class Header_pub(Node):
    def __init__(self):
        super().__init__("header_pub")  # 노드 이름
        # timer 등록
        self.create_timer(1 / 10, self.timer_callback)
        self.pub = self.create_publisher(Header, "time", 10)

    def timer_callback(self):
        msg = Header()  # DDS 에 보낼 객체 초기화
        msg.frame_id = "time test"
        msg.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)  # DDS 로 보내는 기능 수행


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = Header_pub()
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()