# a09_event_draw 기능을 ros2 에서 카메라 영상을 배경으로 작동 시키기
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

# 같은 ROS 2 Python 패키지 내부의 color.py
from camera_opencv import color


class CameraDrawNode(Node):
    """카메라 영상 위에 마우스로 그림을 그려 발행하는 ROS 2 노드."""

    def __init__(self):
        super().__init__("camera_draw_node")

        self.width = 640
        self.height = 480
        self.fps = 30

        self.bridge = CvBridge()

        # ROS 2 Publisher
        self.image_pub = self.create_publisher(
            Image,
            "camera/image_raw",
            10,
        )

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            "camera/camera_info",
            10,
        )

        # 현재 선택된 색상 번호
        self.color_index = 0
        self.color_values = list(color.COLORS.values())
        self.color_names = list(color.COLORS.keys())

        # 마우스 이전 위치
        self.old_x = 0
        self.old_y = 0
        self.drawing = False

        # 실제 카메라 프레임
        self.frame = np.zeros(
            (self.height, self.width, 3),
            dtype=np.uint8,
        )

        # 그림이 저장되는 레이어
        self.draw_layer = np.zeros(
            (self.height, self.width, 3),
            dtype=np.uint8,
        )

        # 그림이 그려진 위치를 나타내는 마스크
        self.draw_mask = np.zeros(
            (self.height, self.width),
            dtype=np.uint8,
        )

        pipeline = (
            "v4l2src device=/dev/video0 ! "
            f"image/jpeg,width={self.width},height={self.height},"
            f"framerate={self.fps}/1 ! "
            "jpegdec ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true sync=false max-buffers=1"
        )

        self.cap = cv2.VideoCapture(
            pipeline,
            cv2.CAP_GSTREAMER,
        )

        if not self.cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다.")
            raise RuntimeError("카메라 연결 실패")

        self.camera_info = self.create_camera_info()

        # OpenCV 윈도우와 마우스 콜백 등록
        cv2.namedWindow("camera")
        cv2.setMouseCallback(
            "camera",
            self.on_mouse,
        )

        self.timer = self.create_timer(
            1.0 / self.fps,
            self.camera_callback,
        )

    def create_camera_info(self):
        """CameraInfo 메시지를 생성한다."""
        msg = CameraInfo()

        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = "plumb_bob"

        # 실제 캘리브레이션 값으로 변경해야 한다.
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        fx = 600.0
        fy = 600.0
        cx = self.width / 2.0
        cy = self.height / 2.0

        msg.k = [
            fx,
            0.0,
            cx,
            0.0,
            fy,
            cy,
            0.0,
            0.0,
            1.0,
        ]

        msg.r = [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]

        msg.p = [
            fx,
            0.0,
            cx,
            0.0,
            0.0,
            fy,
            cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]

        return msg

    def get_current_color(self):
        """현재 선택된 BGR 색상을 반환한다."""
        return self.color_values[self.color_index]

    def on_mouse(self, event, x, y, flags, param):
        """OpenCV 마우스 이벤트 처리 함수."""
        current_color = self.get_current_color()

        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True

            self.old_x = x
            self.old_y = y

            # 클릭 위치에 점 표시
            cv2.circle(
                self.draw_layer,
                (x, y),
                7,
                current_color,
                -1,
            )

            cv2.circle(
                self.draw_mask,
                (x, y),
                7,
                255,
                -1,
            )

        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            # 색상 레이어에 선 그리기
            cv2.line(
                self.draw_layer,
                (self.old_x, self.old_y),
                (x, y),
                current_color,
                2,
                cv2.LINE_AA,
            )

            # 같은 위치를 마스크에도 표시
            cv2.line(
                self.draw_mask,
                (self.old_x, self.old_y),
                (x, y),
                255,
                2,
                cv2.LINE_AA,
            )

            self.old_x = x
            self.old_y = y

        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False

            # 마지막 위치까지 선을 그린다.
            cv2.line(
                self.draw_layer,
                (self.old_x, self.old_y),
                (x, y),
                current_color,
                2,
                cv2.LINE_AA,
            )

            cv2.line(
                self.draw_mask,
                (self.old_x, self.old_y),
                (x, y),
                255,
                2,
                cv2.LINE_AA,
            )

    def camera_callback(self):
        """카메라 영상을 읽고 그림을 합성하여 발행한다."""
        ret, frame = self.cap.read()
        if not ret:
            return
        self.frame = frame

        # 카메라 영상과 그림 합성
        output = frame.copy()
        drawing_area = self.draw_mask > 0
        output[drawing_area] = self.draw_layer[drawing_area]

        cv2.imshow("camera", output)
        key = cv2.waitKey(1) & 0xFF
        self.process_key(key)

        # OpenCV 이미지를 ROS Image 메시지로 변환
        image_msg = self.bridge.cv2_to_imgmsg(
            output,
            encoding="bgr8",
        )

        now = self.get_clock().now().to_msg()

        image_msg.header.stamp = now
        image_msg.header.frame_id = "camera_link"

        self.camera_info.header.stamp = now
        self.camera_info.header.frame_id = "camera_link"

        self.image_pub.publish(image_msg)
        self.camera_info_pub.publish(self.camera_info)

    def process_key(self, key):
        """키보드 입력을 처리한다."""
        if key == ord("q"):
            raise KeyboardInterrupt
        if key == ord(" "):
            self.color_index += 1
            if self.color_index >= len(self.color_values):
                self.color_index = 0
        elif key == ord("c"):
            self.clear_drawing()

    def clear_drawing(self):
        """그림 레이어와 마스크를 초기화한다."""
        self.draw_layer.fill(0)
        self.draw_mask.fill(0)

    def cleanup(self):
        """카메라와 OpenCV 자원을 해제한다."""
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraDrawNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("프로그램을 종료합니다.")
    except RuntimeError as error:
        print(f"실행 오류: {error}")
    finally:
        if node is not None:
            node.cleanup()
            node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
