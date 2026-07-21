import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class M_pub(Node):
    def __init__(self):
        super().__init__("message_pub")  # 노드 이름
        # timer 등록
        self.create_timer(1, self.timer_callback)
        self.pub = self.create_publisher(String, "message", 10)
        self.count = 0

    def timer_callback(self):
        msg = String()  # DDS 에 보낼 객체 초기화
        msg.data = f"첫번째 프로그램입니다. {self.count}"  # data 를 입력
        self.get_logger().info(msg.data)
        self.pub.publish(msg)  # DDS 로 보내는 기능 수행
        self.count += 1


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = M_pub()
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()