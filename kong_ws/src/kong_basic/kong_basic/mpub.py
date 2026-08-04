import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class M_pub(Node):
    def __init__(self):
        super().__init__("mpub")  # 노드 이름
        # timer 등록
        self.create_timer(1, self.timer_callback)
        self.pub1 = self.create_publisher(String, "message1", 10)
        self.pub2 = self.create_publisher(String, "message2", 10)
        self.count = 0

    def timer_callback(self):
        msg1 = String()  # DDS 에 보낼 객체 초기화
        msg2 = String()  # DDS 에 보낼 객체 초기화
        msg1.data = f"message1 에서 보내는 메세지. {self.count}"  # data 를 입력
        msg2.data = f"message2 에서 보내는 메세지. {self.count}"  # data 를 입력
        self.pub1.publish(msg1)  # DDS 로 보내는 기능 수행
        self.pub2.publish(msg2)  # DDS 로 보내는 기능 수행
        self.count += 1


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
