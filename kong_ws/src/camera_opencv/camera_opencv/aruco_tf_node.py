"""Detect ArUco markers from a ROS 2 camera and publish their dynamic TFs."""

from __future__ import annotations

import math

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


def rotation_matrix_to_quaternion(matrix: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a normalized (x, y, z, w) quaternion."""
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return x / norm, y / norm, z / norm, w / norm


class ArucoTfNode(Node):
    """Subscribe to camera topics, annotate ArUco poses, and broadcast TF."""

    def __init__(self) -> None:
        super().__init__("aruco_tf_node")

        self.declare_parameter("image_topic", "/gripper_camera/image_raw")
        self.declare_parameter("camera_info_topic", "/gripper_camera/camera_info")
        self.declare_parameter("annotated_topic", "/aruco/image_annotated")
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("marker_size", 0.04)
        self.declare_parameter("target_marker_id", 0)
        self.declare_parameter("parent_frame", "camera_optical_frame")
        self.declare_parameter("child_frame_prefix", "aruco_")
        self.declare_parameter("show_image", True)
        self.declare_parameter("fallback_binary_threshold", 100)

        image_topic = str(self.get_parameter("image_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        annotated_topic = str(self.get_parameter("annotated_topic").value)
        dictionary_name = str(self.get_parameter("dictionary").value)
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.target_marker_id = int(self.get_parameter("target_marker_id").value)
        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.child_frame_prefix = str(self.get_parameter("child_frame_prefix").value)
        self.show_image = bool(self.get_parameter("show_image").value)
        self.fallback_binary_threshold = int(
            self.get_parameter("fallback_binary_threshold").value
        )

        dictionary_id = getattr(cv2.aruco, dictionary_name, None)
        if dictionary_id is None:
            raise ValueError(f"지원하지 않는 ArUco dictionary입니다: {dictionary_name}")
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector_parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary,
                self.detector_parameters,
            )
        else:
            self.detector_parameters = cv2.aruco.DetectorParameters_create()
            self.detector = None
        self.detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        half = self.marker_size / 2.0
        self.object_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float32,
        )

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.camera_matrix: np.ndarray | None = None
        self.distortion: np.ndarray | None = None
        self.warned_about_camera_info = False

        self.annotated_publisher = self.create_publisher(Image, annotated_topic, 10)
        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"ArUco TF 시작: image={image_topic}, camera_info={camera_info_topic}, "
            f"marker_size={self.marker_size:.3f} m, target_id={self.target_marker_id}"
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            return
        self.camera_matrix = matrix
        if msg.d:
            self.distortion = np.asarray(msg.d, dtype=np.float64).reshape(-1, 1)
        else:
            self.distortion = np.zeros((5, 1), dtype=np.float64)
        self.warned_about_camera_info = False

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"영상 변환 실패: {exc}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detect_markers(gray)
        if ids is None and self.fallback_binary_threshold >= 0:
            _, binary = cv2.threshold(
                gray,
                self.fallback_binary_threshold,
                255,
                cv2.THRESH_BINARY,
            )
            corners, ids, _ = self.detect_markers(binary)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        if self.camera_matrix is None or self.distortion is None:
            if not self.warned_about_camera_info:
                self.get_logger().warning("CameraInfo를 기다리는 중이라 자세·TF를 계산할 수 없습니다.")
                self.warned_about_camera_info = True
        elif ids is not None:
            for marker_corners, marker_id_array in zip(corners, ids):
                marker_id = int(marker_id_array[0])
                if self.target_marker_id >= 0 and marker_id != self.target_marker_id:
                    continue
                self.process_marker(msg, frame, marker_corners, marker_id)

        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        annotated_msg.header = msg.header
        self.annotated_publisher.publish(annotated_msg)

        if self.show_image:
            try:
                cv2.imshow("ArUco TF", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    self.show_image = False
                    cv2.destroyAllWindows()
            except cv2.error as exc:
                self.get_logger().warning(f"imshow를 비활성화합니다: {exc}")
                self.show_image = False

    def detect_markers(self, image: np.ndarray):
        """Detect markers with the OpenCV API available on this system."""
        if self.detector is not None:
            return self.detector.detectMarkers(image)
        return cv2.aruco.detectMarkers(
            image,
            self.dictionary,
            parameters=self.detector_parameters,
        )

    def process_marker(
        self,
        image_msg: Image,
        frame: np.ndarray,
        marker_corners: np.ndarray,
        marker_id: int,
    ) -> None:
        image_points = np.asarray(marker_corners, dtype=np.float32).reshape(4, 2)
        success, rvec, tvec = cv2.solvePnP(
            self.object_points,
            image_points,
            self.camera_matrix,
            self.distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success or float(tvec[2, 0]) <= 0.0:
            return

        cv2.drawFrameAxes(
            frame,
            self.camera_matrix,
            self.distortion,
            rvec,
            tvec,
            self.marker_size * 0.75,
            2,
        )

        center = np.mean(image_points, axis=0).astype(int)
        x, y, z = (float(value) for value in tvec.reshape(3))
        cv2.putText(
            frame,
            f"id={marker_id} x={x:.3f} y={y:.3f} z={z:.3f} m",
            (max(0, center[0] - 150), max(25, center[1] - 15)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = rotation_matrix_to_quaternion(rotation_matrix)

        transform = TransformStamped()
        transform.header.stamp = image_msg.header.stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = f"{self.child_frame_prefix}{marker_id}"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def destroy_node(self) -> bool:
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoTfNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
