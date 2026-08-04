"""카메라 영상을 배경으로 마우스로 그림을 그리는 ROS 2 노드."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from . import color


class A09EventDrawCamera(Node):
    """카메라 화면에 마우스 드로잉을 표시하고 결과 영상을 발행한다."""

    def __init__(self):
        super().__init__("a09_event_draw_camera")

        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("image_topic", "camera/image_drawn")

        device = str(self.get_parameter("device").value)
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        fps = int(self.get_parameter("fps").value)
        image_topic = str(self.get_parameter("image_topic").value)

        self.width = width
        self.height = height
        self.fps = fps
        self.window_name = "a09 event draw camera"
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, image_topic, 10)

        self.colors = list(color.COLORS.values())
        self.color_index = 0
        self.last_point = None
        self.overlay = np.zeros((height, width, 3), dtype=np.uint8)

        pipeline = (
            f"v4l2src device={device} ! "
            f"image/jpeg,width={width},height={height},framerate={fps}/1 ! "
            "jpegdec ! videoconvert ! "
            "video/x-raw,format=BGR ! appsink drop=true sync=false"
        )
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError(f"카메라를 열 수 없습니다: {device}")

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.on_mouse)
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

        self.get_logger().info(f"카메라 시작: {device}")
        self.get_logger().info("마우스 드래그: 그리기 | Space: 색상 변경 | C: 지우기 | Q: 종료")

    def on_mouse(self, event, x, y, flags, _param):
        """마우스 이벤트로 카메라 위의 오버레이에 그림을 추가한다."""
        point = (max(0, min(x, self.width - 1)), max(0, min(y, self.height - 1)))

        if event == cv2.EVENT_LBUTTONDOWN:
            self.last_point = point
            cv2.circle(self.overlay, point, 2, self.colors[self.color_index], -1)
        elif event == cv2.EVENT_MOUSEMOVE and flags & cv2.EVENT_FLAG_LBUTTON:
            if self.last_point is not None:
                cv2.line(
                    self.overlay,
                    self.last_point,
                    point,
                    self.colors[self.color_index],
                    2,
                    cv2.LINE_AA,
                )
            self.last_point = point
        elif event == cv2.EVENT_LBUTTONUP:
            self.last_point = None

    def timer_callback(self):
        """프레임을 읽고 오버레이를 합쳐 화면과 ROS 토픽으로 보낸다."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("카메라 프레임을 읽지 못했습니다.")
            return

        if frame.shape[:2] != self.overlay.shape[:2]:
            self.height, self.width = frame.shape[:2]
            self.overlay = np.zeros_like(frame)

        drawn = cv2.add(frame, self.overlay)
        cv2.putText(
            drawn,
            "Drag: draw | Space: color | C: clear | Q: quit",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow(self.window_name, drawn)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            raise KeyboardInterrupt
        if key == ord("c"):
            self.overlay.fill(0)
        elif key == ord(" "):
            self.color_index = (self.color_index + 1) % len(self.colors)

        image_msg = self.bridge.cv2_to_imgmsg(drawn, encoding="bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = "camera_link"
        self.publisher.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = A09EventDrawCamera()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        print(error)
    finally:
        if node is not None:
            node.cap.release()
            node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
