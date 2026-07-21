import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String


class T_sub(Node):
    def __init__(self):
        super().__init__("message_time_sub")  # 노드 이름
        # timer 등록
        self.create_subscription(String, "message", self.message_callback, 10)
        self.create_subscription(Header, "time", self.time_callback, 10)

    def message_callback(self, msg: String):
        self.get_logger().info(f"메시지 수신: {msg.data}")

    def time_callback(self, msg: Header):
        self.get_logger().info(
            f"시간 수신: {msg.stamp.sec}.{msg.stamp.nanosec:09d}, "
            f"frame_id: {msg.frame_id}"
        )
        


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = T_sub()
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()