import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from user_interface.srv import AddAndOdd


class Service_server(Node):
    def __init__(self):
        super().__init__("service_server")
        self.lock = threading.Lock()
        self.callback_group = ReentrantCallbackGroup()
        self.create_service(
            AddAndOdd, "add_server", self.add_callback, callback_group=self.callback_group
        )

    def add_callback(self, request: AddAndOdd.Request, response: AddAndOdd.Response):
        with self.lock:  # race condition 이 생길거 같은 부분에 with 로 블럭화
            response.sum = request.inta + request.intb
        time.sleep(10)
        if response.sum % 2:
            response.odd = "Two ints sum is odd"
        else:
            response.odd = "Two ints sum is not odd"
        return response


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = Service_server()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        print("키보드 인터럽트")
    finally:
        executor.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()