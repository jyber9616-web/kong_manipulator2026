import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class Image_sub(Node):
    def __init__(self):
        super().__init__("image_sub")
        cv2.namedWindow("camera")
        self.brige = CvBridge()
        self.create_subscription(Image, "image_raw", self.image_callback, 10)

    def image_callback(self, img: Image):
        img_sub = self.brige.imgmsg_to_cv2(img)
        cv2.imshow("camera_sub", img_sub)
        key = cv2.waitKey(3)  # 처리 기간이 필요 milliseconse
        if key == ord("q"):
            raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)  # rmw 활성화
    node = Image_sub()
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
