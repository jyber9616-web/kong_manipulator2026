import rclpy
from rclpy.node import Node
from user_interface.srv import AddAndOdd


class Service_client(Node):
    def __init__(self):
        super().__init__("service_client")
        self.client = self.create_client(AddAndOdd, "add_server")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available")
        self.request = AddAndOdd.Request()
        self.create_timer(3, self.send_request)
        self.create_timer(1, self.update)
        self.count = 0

    def send_request(self):
        self.get_logger().info(f"서버에 요청함{self.count}")
        self.request.inta = 4
        self.request.intb = 8 + self.count
        self.count += 1
        self.future = self.client.call_async(self.request)  # None-block
        self.future.add_done_callback(self.done_callback)

    def done_callback(self, future):
        response: AddAndOdd.Response = future.result()
        self.get_logger().info(f"{response.sum}")
        self.get_logger().info(f"{response.odd}")

    def update(self):
        self.get_logger().info("updating!!")


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = Service_client()
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        print("키보드 인터럽트")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()