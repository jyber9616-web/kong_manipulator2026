"""여러 벽 사이를 이동하는 OpenManipulator-X MoveItPy 미니 프로젝트."""

import math
import os
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class MoveItMiniProjectNode(Node):
    """사용자 SRDF pose와 다중 충돌 객체를 사용하는 MoveItPy 노드."""

    def __init__(self):
        super().__init__("moveit_mini_project")
        self.moveit = MoveItPy(node_name="moveit_mini_project_py")
        self.arm = self.moveit.get_planning_component("arm")
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()

        self.add_environment()
        self.move_manipulator()

    def add_box(
        self,
        object_id: str,
        dimensions: tuple[float, float, float],
        position: tuple[float, float, float],
        yaw: float = 0.0,
    ) -> bool:
        """BOX 충돌 객체를 Planning Scene에 추가한다."""
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = object_id

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(dimensions)

        box_pose = Pose()
        box_pose.position.x = position[0]
        box_pose.position.y = position[1]
        box_pose.position.z = position[2]
        box_pose.orientation.z = math.sin(yaw / 2.0)
        box_pose.orientation.w = math.cos(yaw / 2.0)

        collision_object.primitives.append(box)  # type: ignore
        collision_object.primitive_poses.append(box_pose)  # type: ignore
        collision_object.operation = CollisionObject.ADD

        success = self.planning_scene_monitor.process_collision_object(
            collision_object
        )
        if success:
            self.get_logger().info(f"{object_id} 추가 완료")
        else:
            self.get_logger().error(f"{object_id} 추가 실패")
        return success

    def add_environment(self) -> None:
        """테이블과 로봇 주위의 대각선 벽 네 개를 추가한다."""
        self.add_box(
            object_id="table",
            dimensions=(0.8, 0.8, 0.05),
            position=(0.25, 0.0, -0.05),
        )

        wall_radius = 0.22
        for index, angle_deg in enumerate((45.0, 135.0, 225.0, 315.0), start=1):
            angle = math.radians(angle_deg)
            position = (
                wall_radius * math.cos(angle),
                wall_radius * math.sin(angle),
                0.07,
            )
            self.add_box(
                object_id=f"wall_{index}",
                dimensions=(0.10, 0.02, 0.14),
                position=position,
                yaw=angle + math.pi / 2.0,
            )

        with self.planning_scene_monitor.read_only() as scene:
            object_ids = [
                collision_object.id
                for collision_object in scene.planning_scene_message.world.collision_objects
            ]
            self.get_logger().info(
                f"planning frame={scene.planning_frame}, objects={object_ids}"
            )

    def move_manipulator(self) -> None:
        """각 통로에 팔을 내렸다 올리며 로봇 주변 360도를 이동한다."""
        counterclockwise_gaps = (
            ("pose_front", "pose_front_low"),
            ("pose_left", "pose_left_low"),
            ("pose_back_ccw", "pose_back_ccw_low"),
        )
        clockwise_gaps = (
            ("pose_right", "pose_right_low"),
            ("pose_back_cw", "pose_back_cw_low"),
        )

        for high_pose, low_pose in counterclockwise_gaps:
            if not self.move_down_and_up(high_pose, low_pose):
                return

        # joint1의 +pi 경계에서 -pi 경계로 바로 넘어가지 않고 정면으로 복귀한다.
        for transit_pose in ("pose_left", "pose_front"):
            if not self.plan_and_execute(transit_pose):
                return

        for high_pose, low_pose in clockwise_gaps:
            if not self.move_down_and_up(high_pose, low_pose):
                return

        for transit_pose in ("pose_right", "pose_front"):
            if not self.plan_and_execute(transit_pose):
                return

    def move_down_and_up(self, high_pose: str, low_pose: str) -> bool:
        """벽 사이 통로에서 높은 자세, 낮은 자세, 높은 자세 순서로 이동한다."""
        for pose_name in (high_pose, low_pose, high_pose):
            if not self.plan_and_execute(pose_name):
                self.get_logger().error(
                    f"{pose_name} 이동 실패로 전체 동작을 중단합니다"
                )
                return False
        return True

    def plan_and_execute(self, configuration: str) -> bool:
        """SRDF named pose까지 충돌 회피 경로를 계획하고 실행한다."""
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=configuration)

        plan_result = self.arm.plan()
        if not plan_result:
            self.get_logger().error(
                f"{configuration} 경로 계획에 실패했습니다"
            )
            return False

        self.get_logger().info(f"{configuration} 경로 실행")
        self.moveit.execute(
            plan_result.trajectory,
            controllers=["arm_controller"],
        )
        return True


def main() -> None:
    rclpy.init()
    node = MoveItMiniProjectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.try_shutdown()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
