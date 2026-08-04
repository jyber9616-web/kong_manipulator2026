import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String


class Mt_sub(Node):
    def __init__(self):
        super().__init__("mtsub")  # 노드 이름
        # subscription callback 등록
        self.create_subscription(String, "message1", self.sub_callback, 10)
        self.create_subscription(Header, "time", self.sub_callback2, 10)

    def sub_callback2(self, msg: Header):
        self.get_logger().info(f"{msg.stamp.sec}")

    def sub_callback(self, msg: String):
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = Mt_sub()
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        print("키보드 인터럽트")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
