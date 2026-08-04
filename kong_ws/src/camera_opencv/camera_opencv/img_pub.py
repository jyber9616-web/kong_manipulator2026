import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class M_pub(Node):
    def __init__(self):
        super().__init__("image_pub")
        self.create_timer(1 / 30, self.img_gen_callback)
        cv2.namedWindow("camera")
        self.img = np.zeros((300, 300), dtype=np.uint8)
        self.brightness = 0
        self.pub = self.create_publisher(Image, "image_raw", 10)
        self.brige = CvBridge()

    def img_gen_callback(self):
        self.brightness += 1
        self.img.fill(self.brightness)  # 채우기 함수
        cv2.imshow("camera", self.img)
        if self.brightness > 255:
            self.brightness = 0
        key = cv2.waitKey(3)  # 처리 기간이 필요 milliseconse
        img = self.brige.cv2_to_imgmsg(self.img, encoding="mono8")
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = "test img"
        self.pub.publish(img)
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
