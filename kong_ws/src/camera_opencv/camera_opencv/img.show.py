import cv2
import numpy as np
import rclpy
from rclpy.node import Node


class M_pub(Node):
    def __init__(self):
        super().__init__("message_pub")  # 노드 이름
        # timer 등록
        self.create_timer(1/30, self.img_gen_callback)  # 1초에 30번 호출
        cv2.namedWindow("camera")

        
    def img_gen_callback(self):
        cv2.imshow("camera", img)

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