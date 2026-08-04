"""카메라 영상 위에서 임의의 위치로 이동하는 원과 이동 경로를 발행한다."""
# 카메라를 이미지를 DDS 에 publish 하고 imshow 로 화면에 표시
# 원을 특정 위치 10곳에 램덤으로 이동 시키는 코드를 작성하세요.
# 원이 이동하면 라인이 그려지는 효과도 추가하세요.
# 10곳을 모두 돌면 그려진 도형은 다 지우고 다시 처음부터 실행하게 하세요.

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CircleFollowCamera(Node):
    """카메라 영상 위에 원의 이동 과정과 경로를 표시하는 ROS 2 노드."""

    WIDTH = 640
    HEIGHT = 480
    FPS = 30

    POINT_COUNT = 10
    CIRCLE_RADIUS = 18

    # 한 지점에서 다음 지점까지 이동하는 시간
    MOVE_DURATION = 1.0

    # 10개 지점을 모두 이동한 뒤 화면을 유지하는 시간
    RESET_DELAY = 1.0

    # OpenCV는 BGR 순서
    LINE_COLORS = (
        (0, 0, 255),  # 빨강
        (0, 165, 255),  # 주황
        (0, 255, 255),  # 노랑
        (0, 255, 0),  # 초록
        (255, 255, 0),  # 청록
        (255, 0, 0),  # 파랑
        (255, 0, 255),  # 자홍
        (128, 0, 255),  # 보라
        (255, 255, 255),  # 흰색
    )

    def __init__(self):
        super().__init__("circle_follow_camera")

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

        self.bridge = CvBridge()
        self.random = np.random.default_rng()

        # 카메라 열기
        pipeline = (
            "v4l2src device=/dev/video0 ! "
            f"image/jpeg,width={self.WIDTH},height={self.HEIGHT},"
            f"framerate={self.FPS}/1 ! "
            "jpegdec ! "
            "videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink drop=true sync=false"
        )

        self.cap = cv2.VideoCapture(
            pipeline,
            cv2.CAP_GSTREAMER,
        )

        if not self.cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다: /dev/video0")
            raise RuntimeError("카메라 열기 실패")

        self.get_logger().info("카메라가 정상적으로 열렸습니다.")

        self.window_name = "Circle Follow Camera"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.camera_info = self.create_camera_info()

        # 애니메이션에 사용할 값
        self.points = []
        self.current_position = np.zeros(2, dtype=np.float64)

        # 현재 이동 시작점의 인덱스
        self.segment_index = 0

        # 현재 구간 진행률
        self.progress = 0.0

        # 모든 지점 이동 완료 여부
        self.finished = False

        # 완료 후 대기 프레임 수
        self.finish_wait_count = 0

        self.reset_animation()

        self.timer_period = 1.0 / self.FPS

        # 한 프레임마다 증가할 진행률
        self.progress_step = self.timer_period / self.MOVE_DURATION

        self.timer = self.create_timer(
            self.timer_period,
            self.timer_callback,
        )

    def create_camera_info(self):
        """카메라 정보 메시지를 생성한다."""
        msg = CameraInfo()

        msg.width = self.WIDTH
        msg.height = self.HEIGHT
        msg.distortion_model = "plumb_bob"

        # 실제 사용 시 카메라 캘리브레이션 값으로 교체해야 한다.
        fx = 600.0
        fy = 600.0
        cx = self.WIDTH / 2.0
        cy = self.HEIGHT / 2.0

        msg.d = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]

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

    def create_random_points(self):
        """원이 화면 밖으로 나가지 않도록 임의의 지점 10개를 생성한다."""
        margin = self.CIRCLE_RADIUS + 20

        x_values = self.random.integers(
            margin,
            self.WIDTH - margin,
            size=self.POINT_COUNT,
        )

        y_values = self.random.integers(
            margin,
            self.HEIGHT - margin,
            size=self.POINT_COUNT,
        )

        return [np.array([x, y], dtype=np.float64) for x, y in zip(x_values, y_values)]

    def reset_animation(self):
        """새로운 임의의 지점을 만들고 처음부터 이동을 시작한다."""
        self.points = self.create_random_points()

        self.segment_index = 0
        self.progress = 0.0
        self.finished = False
        self.finish_wait_count = 0

        self.current_position = self.points[0].copy()

        self.get_logger().info("새로운 이동 경로를 생성했습니다.")

    def update_position(self):
        """현재 원의 위치를 다음 목표 지점 방향으로 조금씩 이동시킨다."""
        if self.finished:
            self.finish_wait_count += 1

            reset_wait_frames = int(self.RESET_DELAY * self.FPS)

            if self.finish_wait_count >= reset_wait_frames:
                self.reset_animation()

            return

        # 마지막 지점까지 도착한 경우
        if self.segment_index >= self.POINT_COUNT - 1:
            self.current_position = self.points[-1].copy()
            self.finished = True
            return

        start_point = self.points[self.segment_index]
        end_point = self.points[self.segment_index + 1]

        self.progress += self.progress_step

        if self.progress >= 1.0:
            # 목표 지점에 정확히 도착
            self.current_position = end_point.copy()

            self.segment_index += 1
            self.progress = 0.0

            if self.segment_index >= self.POINT_COUNT - 1:
                self.finished = True

            return

        # 선형 보간
        self.current_position = start_point * (1.0 - self.progress) + end_point * self.progress

    def draw_completed_path(self, frame):
        """이미 지나온 구간을 그린다."""
        for index in range(self.segment_index):
            start_point = tuple(self.points[index].astype(int))

            end_point = tuple(self.points[index + 1].astype(int))

            color = self.LINE_COLORS[index % len(self.LINE_COLORS)]

            cv2.line(
                frame,
                start_point,
                end_point,
                color,
                4,
                cv2.LINE_AA,
            )

    def draw_current_path(self, frame):
        """현재 이동 중인 구간을 중간 위치까지만 그린다."""
        if self.finished:
            return

        if self.segment_index >= self.POINT_COUNT - 1:
            return

        start_point = tuple(self.points[self.segment_index].astype(int))

        current_point = tuple(self.current_position.astype(int))

        color = self.LINE_COLORS[self.segment_index % len(self.LINE_COLORS)]

        cv2.line(
            frame,
            start_point,
            current_point,
            color,
            4,
            cv2.LINE_AA,
        )

    def draw_target_points(self, frame):
        """이동할 목표 지점을 작은 점으로 표시한다."""
        for index, point in enumerate(self.points):
            point_tuple = tuple(point.astype(int))

            if index <= self.segment_index:
                point_color = (0, 255, 0)
            else:
                point_color = (100, 100, 100)

            cv2.circle(
                frame,
                point_tuple,
                5,
                point_color,
                -1,
                cv2.LINE_AA,
            )

            cv2.putText(
                frame,
                str(index + 1),
                (point_tuple[0] + 7, point_tuple[1] - 7),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

    def draw_circle(self, frame):
        """현재 위치에 움직이는 원을 그린다."""
        current_point = tuple(self.current_position.astype(int))

        cv2.circle(
            frame,
            current_point,
            self.CIRCLE_RADIUS,
            (255, 255, 255),
            -1,
            cv2.LINE_AA,
        )

        cv2.circle(
            frame,
            current_point,
            self.CIRCLE_RADIUS,
            (0, 0, 0),
            2,
            cv2.LINE_AA,
        )

        cv2.circle(
            frame,
            current_point,
            4,
            (0, 0, 255),
            -1,
            cv2.LINE_AA,
        )

    def draw_information(self, frame):
        """현재 이동 상태를 영상에 표시한다."""
        if self.finished:
            text = "Completed - reset soon"
        else:
            # 첫 번째 지점에서 두 번째 지점으로 이동할 때 2/10으로 표시
            current_target = min(
                self.segment_index + 2,
                self.POINT_COUNT,
            )

            text = f"Target: {current_target} / {self.POINT_COUNT}"

        cv2.rectangle(
            frame,
            (10, 10),
            (300, 50),
            (0, 0, 0),
            -1,
        )

        cv2.putText(
            frame,
            text,
            (20, 39),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def publish_image(self, frame):
        """영상을 ROS 2 Image 및 CameraInfo 토픽으로 발행한다."""
        now = self.get_clock().now().to_msg()

        image_msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding="bgr8",
        )

        image_msg.header.stamp = now
        image_msg.header.frame_id = "camera_link"

        self.camera_info.header.stamp = now
        self.camera_info.header.frame_id = "camera_link"

        self.image_pub.publish(image_msg)
        self.camera_info_pub.publish(self.camera_info)

    def timer_callback(self):
        """카메라 영상을 읽고 원의 이동 과정을 그린다."""
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning("카메라 프레임을 읽지 못했습니다.")
            return

        # 카메라 해상도가 다를 경우 지정한 크기로 변경
        frame = cv2.resize(
            frame,
            (self.WIDTH, self.HEIGHT),
        )

        # 원의 위치를 조금씩 이동
        self.update_position()

        # 카메라 영상 위에 이동 경로와 원 표시
        self.draw_completed_path(frame)
        self.draw_current_path(frame)
        self.draw_target_points(frame)
        self.draw_circle(frame)
        self.draw_information(frame)

        cv2.imshow(self.window_name, frame)

        key = cv2.waitKey(1) & 0xFF

        self.publish_image(frame)

        if key == ord("q"):
            raise KeyboardInterrupt


def main(args=None):
    """ROS 2 노드를 실행한다."""
    rclpy.init(args=args)

    node = None

    try:
        node = CircleFollowCamera()
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("키보드 인터럽트로 종료합니다.")

    except RuntimeError as error:
        print(error)

    finally:
        if node is not None:
            if node.cap.isOpened():
                node.cap.release()
            node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
