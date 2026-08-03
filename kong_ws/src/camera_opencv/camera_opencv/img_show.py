import cv2
import numpy as np
import rclpy
from rclpy.node import Node


class M_pub(Node):
    def __init__(self):
        super().__init__("image_demo")  # 노드 이름
        # timer 등록
        self.create_timer(1 / 30, self.img_gen_callback)
        cv2.namedWindow("camera")
        self.img = np.zeros((300, 300), dtype=np.uint8)
        self.brightness = 0

    def img_gen_callback(self):
        self.brightness = (self.brightness + 1) % 256
        self.img.fill(self.brightness)  # 채우기 함수
        cv2.imshow("camera", self.img)
        key = cv2.waitKey(30)  # 처리 기간이 필요 milliseconse
        if key == ord("q"):
            raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = M_pub()
    try:
        rclpy.spin(node)  # 블럭 (무한 루프)
    except KeyboardInterrupt:
        print("키보드 인터럽트")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
