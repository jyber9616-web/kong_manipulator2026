#!/usr/bin/env python3

import math
import random
from dataclasses import dataclass, field
from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass
class JointMotion:
    lower: float
    upper: float

    current: float = 0.0
    velocity: float = 0.0

    # 관절별 최대 이동 속도
    max_velocity: float = 0.4

    # continuous 관절 여부
    continuous: bool = False


@dataclass
class MotionSegment:
    """
    하나의 중간 이동 구간.

    start_positions:
        구간 시작 자세

    target_positions:
        구간 목표 자세

    duration:
        해당 구간의 이동 시간
    """

    start_positions: Dict[str, float]
    target_positions: Dict[str, float]
    duration: float


class SmoothRandomJointDance(Node):
    def __init__(self):
        super().__init__("smooth_random_joint_dance")

        # ---------------------------------------------------------
        # 파라미터
        # ---------------------------------------------------------
        self.declare_parameter("publish_rate", 50.0)

        # 최종 목표 자세까지 걸리는 전체 시간
        self.declare_parameter("min_motion_time", 5.0)
        self.declare_parameter("max_motion_time", 9.0)

        # 중간 경유점 개수
        self.declare_parameter("min_waypoints", 2)
        self.declare_parameter("max_waypoints", 4)

        # 자세 도착 후 멈추는 시간
        self.declare_parameter("min_hold_time", 1.0)
        self.declare_parameter("max_hold_time", 2.5)

        self.declare_parameter("wheel_speed", 1.0)
        self.declare_parameter("wheel_acceleration", 0.4)
        self.declare_parameter("enable_wheels", True)

        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self.min_motion_time = float(self.get_parameter("min_motion_time").value)
        self.max_motion_time = float(self.get_parameter("max_motion_time").value)

        self.min_waypoints = int(self.get_parameter("min_waypoints").value)
        self.max_waypoints = int(self.get_parameter("max_waypoints").value)

        self.min_hold_time = float(self.get_parameter("min_hold_time").value)
        self.max_hold_time = float(self.get_parameter("max_hold_time").value)

        self.wheel_speed_limit = float(self.get_parameter("wheel_speed").value)
        self.wheel_acceleration = float(self.get_parameter("wheel_acceleration").value)
        self.enable_wheels = bool(self.get_parameter("enable_wheels").value)

        # ---------------------------------------------------------
        # 관절 정의
        # max_velocity 단위:
        # revolute/continuous: rad/s
        # prismatic: m/s
        # ---------------------------------------------------------
        self.joints: Dict[str, JointMotion] = {
            "right_arm_shoulder_revolute_joint": JointMotion(
                lower=-1.25,
                upper=1.25,
                max_velocity=0.32,
            ),
            "right_arm_elbow_revolute_joint": JointMotion(
                lower=0.05,
                upper=2.35,
                max_velocity=0.38,
            ),
            "left_arm_shoulder_revolute_joint": JointMotion(
                lower=-1.25,
                upper=1.25,
                max_velocity=0.32,
            ),
            "left_arm_elbow_revolute_joint": JointMotion(
                lower=0.05,
                upper=2.35,
                max_velocity=0.38,
            ),
            "head_swivel": JointMotion(
                lower=-math.pi,
                upper=math.pi,
                max_velocity=0.25,
                continuous=True,
            ),
            "gripper_extension": JointMotion(
                lower=-0.35,
                upper=0.0,
                max_velocity=0.035,
            ),
            "left_gripper_joint": JointMotion(
                lower=0.0,
                upper=0.52,
                max_velocity=0.12,
            ),
            "right_gripper_joint": JointMotion(
                lower=0.0,
                upper=0.52,
                max_velocity=0.12,
            ),
        }

        # ---------------------------------------------------------
        # 바퀴 관절
        # ---------------------------------------------------------
        self.wheel_joint_names = [
            "right_front_wheel_joint",
            "right_back_wheel_joint",
            "left_front_wheel_joint",
            "left_back_wheel_joint",
        ]

        self.wheel_positions = {name: 0.0 for name in self.wheel_joint_names}

        self.left_wheel_speed = 0.0
        self.right_wheel_speed = 0.0

        self.target_left_wheel_speed = 0.0
        self.target_right_wheel_speed = 0.0

        # ---------------------------------------------------------
        # 동작 시퀀스 상태
        # ---------------------------------------------------------
        self.motion_segments: List[MotionSegment] = []
        self.current_segment_index = 0

        self.segment_start_time = self.get_clock().now()

        self.is_holding = False
        self.hold_start_time = self.get_clock().now()
        self.hold_duration = 0.0

        self.last_update_time = self.get_clock().now()

        self.motion_modes = [
            "random",
            "wave_left",
            "wave_right",
            "both_hands",
            "surprised",
            "thinking",
            "disco",
            "sleepy",
        ]

        # ---------------------------------------------------------
        # Publisher
        # ---------------------------------------------------------
        self.publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.create_new_motion_sequence()

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(
            timer_period,
            self.timer_callback,
        )

        self.get_logger().info("Smooth random joint dance started")

    # -------------------------------------------------------------
    # 보간 함수
    # -------------------------------------------------------------
    @staticmethod
    def smootherstep(value: float) -> float:
        """
        5차 smootherstep.

        시작과 끝에서 속도뿐만 아니라 가속도도 0이 되므로
        cubic smoothstep보다 더 부드럽다.

        6t^5 - 15t^4 + 10t^3
        """

        value = max(0.0, min(1.0, value))

        return 6.0 * value**5 - 15.0 * value**4 + 10.0 * value**3

    @staticmethod
    def shortest_angle_difference(
        start: float,
        target: float,
    ) -> float:
        return (target - start + math.pi) % (2.0 * math.pi) - math.pi

    def interpolate_joint(
        self,
        joint_name: str,
        start: float,
        target: float,
        ratio: float,
    ) -> float:
        joint = self.joints[joint_name]

        if joint.continuous:
            difference = self.shortest_angle_difference(
                start,
                target,
            )
            return start + difference * ratio

        return start + (target - start) * ratio

    # -------------------------------------------------------------
    # 목표 자세 생성
    # -------------------------------------------------------------
    def clamp_target(
        self,
        joint_name: str,
        value: float,
    ) -> float:
        joint = self.joints[joint_name]

        return max(
            joint.lower,
            min(joint.upper, value),
        )

    def current_positions(self) -> Dict[str, float]:
        return {name: joint.current for name, joint in self.joints.items()}

    def random_pose(self) -> Dict[str, float]:
        pose = {}

        for name, joint in self.joints.items():
            # 전체 범위를 끝까지 사용하지 않고 중앙 위주로 선택
            center = (joint.lower + joint.upper) / 2.0
            half_range = (joint.upper - joint.lower) * 0.42

            pose[name] = random.uniform(
                center - half_range,
                center + half_range,
            )

        gripper_value = random.uniform(0.08, 0.42)

        pose["left_gripper_joint"] = gripper_value
        pose["right_gripper_joint"] = gripper_value

        return pose

    def make_target_pose(
        self,
        mode: str,
    ) -> Dict[str, float]:
        pose = self.random_pose()

        if mode == "wave_left":
            pose.update(
                {
                    "left_arm_shoulder_revolute_joint": -1.0,
                    "left_arm_elbow_revolute_joint": 1.75,
                    "right_arm_shoulder_revolute_joint": 0.2,
                    "right_arm_elbow_revolute_joint": 0.25,
                    "head_swivel": 0.55,
                    "gripper_extension": -0.10,
                    "left_gripper_joint": 0.40,
                    "right_gripper_joint": 0.12,
                }
            )

        elif mode == "wave_right":
            pose.update(
                {
                    "right_arm_shoulder_revolute_joint": -1.0,
                    "right_arm_elbow_revolute_joint": 1.75,
                    "left_arm_shoulder_revolute_joint": 0.2,
                    "left_arm_elbow_revolute_joint": 0.25,
                    "head_swivel": -0.55,
                    "gripper_extension": -0.10,
                    "left_gripper_joint": 0.12,
                    "right_gripper_joint": 0.40,
                }
            )

        elif mode == "both_hands":
            pose.update(
                {
                    "left_arm_shoulder_revolute_joint": -0.95,
                    "right_arm_shoulder_revolute_joint": -0.95,
                    "left_arm_elbow_revolute_joint": 0.75,
                    "right_arm_elbow_revolute_joint": 0.75,
                    "head_swivel": 0.0,
                    "gripper_extension": -0.25,
                    "left_gripper_joint": 0.42,
                    "right_gripper_joint": 0.42,
                }
            )

        elif mode == "surprised":
            pose.update(
                {
                    "left_arm_shoulder_revolute_joint": 1.0,
                    "right_arm_shoulder_revolute_joint": -1.0,
                    "left_arm_elbow_revolute_joint": 0.25,
                    "right_arm_elbow_revolute_joint": 0.25,
                    "head_swivel": random.choice([-0.9, 0.9]),
                    "gripper_extension": -0.30,
                    "left_gripper_joint": 0.48,
                    "right_gripper_joint": 0.48,
                }
            )

        elif mode == "thinking":
            side = random.choice(["left", "right"])
            other = "right" if side == "left" else "left"

            pose.update(
                {
                    f"{side}_arm_shoulder_revolute_joint": -0.45,
                    f"{side}_arm_elbow_revolute_joint": 2.05,
                    f"{other}_arm_shoulder_revolute_joint": 0.15,
                    f"{other}_arm_elbow_revolute_joint": 0.25,
                    "head_swivel": 0.55 if side == "left" else -0.55,
                    "gripper_extension": -0.05,
                    "left_gripper_joint": 0.15,
                    "right_gripper_joint": 0.15,
                }
            )

        elif mode == "disco":
            direction = random.choice([-1.0, 1.0])

            pose.update(
                {
                    "left_arm_shoulder_revolute_joint": 1.0 * direction,
                    "right_arm_shoulder_revolute_joint": -1.0 * direction,
                    "left_arm_elbow_revolute_joint": random.uniform(0.7, 1.7),
                    "right_arm_elbow_revolute_joint": random.uniform(0.7, 1.7),
                    "head_swivel": random.uniform(-1.0, 1.0),
                    "gripper_extension": random.uniform(-0.25, -0.08),
                }
            )

        elif mode == "sleepy":
            pose.update(
                {
                    "left_arm_shoulder_revolute_joint": 0.15,
                    "right_arm_shoulder_revolute_joint": 0.15,
                    "left_arm_elbow_revolute_joint": 0.15,
                    "right_arm_elbow_revolute_joint": 0.15,
                    "head_swivel": random.uniform(-0.25, 0.25),
                    "gripper_extension": -0.02,
                    "left_gripper_joint": 0.08,
                    "right_gripper_joint": 0.08,
                }
            )

        return {name: self.clamp_target(name, value) for name, value in pose.items()}

    # -------------------------------------------------------------
    # 중간 경유점 생성
    # -------------------------------------------------------------
    def create_waypoint(
        self,
        start_pose: Dict[str, float],
        final_pose: Dict[str, float],
        progress: float,
    ) -> Dict[str, float]:
        """
        시작 자세와 최종 자세 사이의 중간 자세를 만든다.

        단순 직선 중간값에 작은 변화만 추가해,
        관절이 갑자기 엉뚱한 방향으로 튀지 않게 한다.
        """

        waypoint = {}

        for name, joint in self.joints.items():
            start = start_pose[name]
            target = final_pose[name]

            if joint.continuous:
                difference = self.shortest_angle_difference(
                    start,
                    target,
                )
                base_value = start + difference * progress
            else:
                base_value = start + (target - start) * progress

            # 중간 지점에만 작은 흔들림 추가
            movement_range = joint.upper - joint.lower

            noise_strength = math.sin(progress * math.pi)

            noise = (
                random.uniform(
                    -0.06,
                    0.06,
                )
                * movement_range
                * noise_strength
            )

            # 그리퍼 extension은 작은 범위이므로 잡음 축소
            if name == "gripper_extension":
                noise *= 0.25

            waypoint[name] = self.clamp_target(
                name,
                base_value + noise,
            )

        return waypoint

    def calculate_segment_duration(
        self,
        start_pose: Dict[str, float],
        target_pose: Dict[str, float],
        preferred_duration: float,
    ) -> float:
        """
        관절별 최대 속도를 초과하지 않도록
        필요한 최소 시간을 계산한다.
        """

        required_duration = 0.0

        for name, joint in self.joints.items():
            start = start_pose[name]
            target = target_pose[name]

            if joint.continuous:
                distance = abs(
                    self.shortest_angle_difference(
                        start,
                        target,
                    )
                )
            else:
                distance = abs(target - start)

            joint_duration = distance / max(joint.max_velocity, 1e-6)

            required_duration = max(
                required_duration,
                joint_duration,
            )

        # smootherstep의 최대 순간 속도를 고려해 여유를 준다.
        required_duration *= 1.9

        return max(
            preferred_duration,
            required_duration,
            0.5,
        )

    def create_new_motion_sequence(self):
        mode = random.choice(self.motion_modes)

        start_pose = self.current_positions()
        final_pose = self.make_target_pose(mode)

        waypoint_count = random.randint(
            self.min_waypoints,
            self.max_waypoints,
        )

        total_motion_time = random.uniform(
            self.min_motion_time,
            self.max_motion_time,
        )

        poses = [start_pose]

        for index in range(1, waypoint_count + 1):
            progress = index / (waypoint_count + 1)

            poses.append(
                self.create_waypoint(
                    start_pose,
                    final_pose,
                    progress,
                )
            )

        poses.append(final_pose)

        segment_count = len(poses) - 1
        preferred_segment_time = total_motion_time / segment_count

        self.motion_segments.clear()

        for index in range(segment_count):
            segment_start = poses[index]
            segment_target = poses[index + 1]

            duration = self.calculate_segment_duration(
                segment_start,
                segment_target,
                preferred_segment_time,
            )

            self.motion_segments.append(
                MotionSegment(
                    start_positions=segment_start,
                    target_positions=segment_target,
                    duration=duration,
                )
            )

        self.current_segment_index = 0
        self.segment_start_time = self.get_clock().now()

        self.is_holding = False

        self.select_wheel_motion()

        actual_duration = sum(segment.duration for segment in self.motion_segments)

        self.get_logger().info(
            f"Motion={mode}, waypoints={waypoint_count}, duration={actual_duration:.1f}s"
        )

    # -------------------------------------------------------------
    # 바퀴
    # -------------------------------------------------------------
    def select_wheel_motion(self):
        if not self.enable_wheels:
            self.target_left_wheel_speed = 0.0
            self.target_right_wheel_speed = 0.0
            return

        mode = random.choice(
            [
                "stop",
                "slow_forward",
                "slow_backward",
                "slow_turn_left",
                "slow_turn_right",
            ]
        )

        speed = random.uniform(
            0.15,
            self.wheel_speed_limit,
        )

        if mode == "slow_forward":
            self.target_left_wheel_speed = speed
            self.target_right_wheel_speed = speed

        elif mode == "slow_backward":
            self.target_left_wheel_speed = -speed
            self.target_right_wheel_speed = -speed

        elif mode == "slow_turn_left":
            self.target_left_wheel_speed = speed * 0.25
            self.target_right_wheel_speed = speed

        elif mode == "slow_turn_right":
            self.target_left_wheel_speed = speed
            self.target_right_wheel_speed = speed * 0.25

        else:
            self.target_left_wheel_speed = 0.0
            self.target_right_wheel_speed = 0.0

    @staticmethod
    def approach(
        current: float,
        target: float,
        maximum_change: float,
    ) -> float:
        difference = target - current

        if abs(difference) <= maximum_change:
            return target

        return current + math.copysign(
            maximum_change,
            difference,
        )

    def update_wheels(self, delta_time: float):
        maximum_speed_change = self.wheel_acceleration * delta_time

        self.left_wheel_speed = self.approach(
            self.left_wheel_speed,
            self.target_left_wheel_speed,
            maximum_speed_change,
        )

        self.right_wheel_speed = self.approach(
            self.right_wheel_speed,
            self.target_right_wheel_speed,
            maximum_speed_change,
        )

        for name in self.wheel_joint_names:
            if name.startswith("left_"):
                speed = self.left_wheel_speed
            else:
                speed = self.right_wheel_speed

            self.wheel_positions[name] += speed * delta_time

            self.wheel_positions[name] %= 2.0 * math.pi

    # -------------------------------------------------------------
    # 동작 업데이트
    # -------------------------------------------------------------
    def update_motion(self, now):
        if self.is_holding:
            hold_elapsed = (now - self.hold_start_time).nanoseconds / 1e9

            if hold_elapsed >= self.hold_duration:
                self.create_new_motion_sequence()

            return

        if not self.motion_segments:
            self.create_new_motion_sequence()
            return

        segment = self.motion_segments[self.current_segment_index]

        elapsed = (now - self.segment_start_time).nanoseconds / 1e9

        raw_ratio = elapsed / segment.duration
        raw_ratio = max(0.0, min(1.0, raw_ratio))

        smooth_ratio = self.smootherstep(raw_ratio)

        for name, joint in self.joints.items():
            previous_position = joint.current

            joint.current = self.interpolate_joint(
                name,
                segment.start_positions[name],
                segment.target_positions[name],
                smooth_ratio,
            )

            joint.velocity = (joint.current - previous_position) * self.publish_rate

        if raw_ratio >= 1.0:
            # 오차가 누적되지 않도록 정확한 목표값 설정
            for name, joint in self.joints.items():
                joint.current = segment.target_positions[name]
                joint.velocity = 0.0

            self.current_segment_index += 1

            if self.current_segment_index >= len(self.motion_segments):
                self.is_holding = True
                self.hold_start_time = now

                self.hold_duration = random.uniform(
                    self.min_hold_time,
                    self.max_hold_time,
                )

                self.target_left_wheel_speed = 0.0
                self.target_right_wheel_speed = 0.0

            else:
                self.segment_start_time = now

    # -------------------------------------------------------------
    # JointState 발행
    # -------------------------------------------------------------
    def publish_joint_state(self, now):
        message = JointState()
        message.header.stamp = now.to_msg()

        movable_names = list(self.joints.keys())

        message.name = movable_names + self.wheel_joint_names

        message.position = [self.joints[name].current for name in movable_names] + [
            self.wheel_positions[name] for name in self.wheel_joint_names
        ]

        message.velocity = [self.joints[name].velocity for name in movable_names] + [
            self.left_wheel_speed if name.startswith("left_") else self.right_wheel_speed
            for name in self.wheel_joint_names
        ]

        message.effort = []

        self.publisher.publish(message)

    def timer_callback(self):
        now = self.get_clock().now()

        delta_time = (now - self.last_update_time).nanoseconds / 1e9

        self.last_update_time = now

        # 시스템 지연 후 큰 값이 들어가는 것 방지
        delta_time = max(
            0.0,
            min(delta_time, 0.1),
        )

        self.update_motion(now)
        self.update_wheels(delta_time)
        self.publish_joint_state(now)


def main(args=None):
    rclpy.init(args=args)

    node = SmoothRandomJointDance()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
