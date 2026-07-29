"""YAML 기반의 랜덤 OpenManipulator-X 춤 동작 노드."""

import random
from collections import deque
from pathlib import Path
from typing import Any

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class DanceManipulator(Node):
    """YAML 자세를 무작위 순서와 오차로 실행한다."""

    def __init__(
        self,
        node_name: str = "dance_manipulator",
        default_dance_file: str = "",
    ) -> None:
        super().__init__(node_name)

        self.declare_parameter("dance_file", default_dance_file)
        self.declare_parameter("repeat_count", -1)
        self.declare_parameter("random_seed", -1)
        self.declare_parameter("variation_ratio", -1.0)
        self.declare_parameter("stop_on_error", True)

        dance_file = self._resolve_dance_file()
        config = self._load_config(dance_file)
        self._configure(config)

        seed = int(self.get_parameter("random_seed").value)
        self._random = random.Random(None if seed < 0 else seed)
        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )
        self._gripper_client = ActionClient(
            self,
            GripperCommand,
            "/gripper_controller/gripper_cmd",
        )
        self._joint_state_subscription = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )

        self._joint_state_received = False
        self._current_positions: list[float] = []
        self._step_queue: deque[tuple[str, int, dict[str, Any]]] = deque()
        self._cycle_count = 0
        self._step_running = False
        self._step_failed = False
        self._pending_results: set[str] = set()
        self._current_pause = 0.0
        self._delay_timer = None
        self._wait_log_counter = 0
        self._finished = False

        self._start_timer = self.create_timer(0.2, self._start_when_ready)
        self.get_logger().info(f"춤 데이터 로드 완료: {dance_file}")
        self.get_logger().info(
            f"패턴 {len(self._patterns)}개, 랜덤 변동 ±{self._variation_ratio * 100:.1f}%"
        )

    def _resolve_dance_file(self) -> Path:
        parameter_path = str(self.get_parameter("dance_file").value).strip()
        if parameter_path:
            path = Path(parameter_path).expanduser()
            if not path.is_absolute():
                path = Path.cwd() / path
            return path.resolve()

        try:
            package_share = Path(get_package_share_directory("tf2_basic"))
            installed_path = package_share / "data" / "dance_positions.yaml"
            if installed_path.is_file():
                return installed_path
        except Exception:  # 패키지 빌드 전 소스 직접 실행도 지원
            pass

        return Path(__file__).resolve().parents[1] / "data" / "dance_positions.yaml"

    @staticmethod
    def _load_config(path: Path) -> dict[str, Any]:
        if not path.is_file():
            raise FileNotFoundError(f"춤 데이터 파일을 찾을 수 없습니다: {path}")
        with path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        if not isinstance(config, dict):
            raise ValueError("춤 YAML의 최상위 데이터는 mapping이어야 합니다.")
        return config

    def _configure(self, config: dict[str, Any]) -> None:
        self._joint_names = self._string_list(config, "joint_names")
        if len(self._joint_names) != 4:
            raise ValueError("OpenManipulator-X arm에는 joint_names 4개가 필요합니다.")

        limits = config.get("joint_limits")
        if not isinstance(limits, dict):
            raise ValueError("joint_limits mapping이 필요합니다.")
        self._joint_limits = [
            self._limit(limits.get(name), f"joint_limits.{name}") for name in self._joint_names
        ]
        self._gripper_limits = self._limit(config.get("gripper_limits"), "gripper_limits")

        randomization = config.get("randomization", {})
        if not isinstance(randomization, dict):
            raise ValueError("randomization은 mapping이어야 합니다.")
        yaml_ratio = float(randomization.get("variation_ratio", 0.05))
        parameter_ratio = float(self.get_parameter("variation_ratio").value)
        self._variation_ratio = parameter_ratio if parameter_ratio >= 0.0 else yaml_ratio
        if not 0.0 <= self._variation_ratio <= 0.05:
            raise ValueError("variation_ratio는 안전을 위해 0.0~0.05여야 합니다.")
        self._minimum_joint_jitter = float(randomization.get("minimum_joint_jitter", 0.01))
        self._minimum_gripper_jitter = float(randomization.get("minimum_gripper_jitter", 0.0002))
        self._shuffle_patterns = bool(randomization.get("shuffle_patterns", True))
        self._shuffle_steps = bool(randomization.get("shuffle_steps", False))

        execution = config.get("execution", {})
        if not isinstance(execution, dict):
            raise ValueError("execution은 mapping이어야 합니다.")
        yaml_repeat = int(execution.get("repeat_count", 0))
        parameter_repeat = int(self.get_parameter("repeat_count").value)
        self._repeat_count = parameter_repeat if parameter_repeat >= 0 else yaml_repeat
        self._default_duration = float(execution.get("default_duration", 1.5))
        self._default_pause = float(execution.get("default_pause", 0.15))
        self._default_max_effort = float(execution.get("gripper_max_effort", 10.0))
        self._stop_on_error = bool(self.get_parameter("stop_on_error").value)
        if self._default_duration <= 0.0:
            raise ValueError("default_duration은 0보다 커야 합니다.")
        if self._default_pause < 0.0 or self._default_max_effort < 0.0:
            raise ValueError("pause와 gripper_max_effort는 0 이상이어야 합니다.")

        patterns = config.get("patterns")
        if not isinstance(patterns, list) or not patterns:
            raise ValueError("patterns에는 하나 이상의 패턴이 필요합니다.")
        self._patterns = []
        names: set[str] = set()
        for pattern_index, pattern in enumerate(patterns):
            validated = self._validate_pattern(pattern, pattern_index)
            if validated["name"] in names:
                raise ValueError(f"중복 패턴 이름: {validated['name']}")
            names.add(validated["name"])
            self._patterns.append(validated)

    @staticmethod
    def _string_list(config: dict[str, Any], key: str) -> list[str]:
        value = config.get(key)
        if (
            not isinstance(value, list)
            or not value
            or not all(isinstance(item, str) and item for item in value)
        ):
            raise ValueError(f"{key}에는 문자열 목록이 필요합니다.")
        return value

    @staticmethod
    def _limit(value: Any, name: str) -> tuple[float, float]:
        if not isinstance(value, list) or len(value) != 2:
            raise ValueError(f"{name}은 [lower, upper] 형식이어야 합니다.")
        lower, upper = float(value[0]), float(value[1])
        if lower >= upper:
            raise ValueError(f"{name}의 lower는 upper보다 작아야 합니다.")
        return lower, upper

    def _validate_pattern(self, pattern: Any, pattern_index: int) -> dict[str, Any]:
        if not isinstance(pattern, dict):
            raise ValueError(f"patterns[{pattern_index}]는 mapping이어야 합니다.")
        name = pattern.get("name")
        steps = pattern.get("steps")
        if not isinstance(name, str) or not name:
            raise ValueError(f"patterns[{pattern_index}].name이 필요합니다.")
        if not isinstance(steps, list) or not steps:
            raise ValueError(f"패턴 '{name}'에는 step이 필요합니다.")
        return {
            "name": name,
            "steps": [self._validate_step(step, name, index) for index, step in enumerate(steps)],
        }

    def _validate_step(self, step: Any, pattern_name: str, step_index: int) -> dict[str, Any]:
        location = f"{pattern_name}.steps[{step_index}]"
        if not isinstance(step, dict):
            raise ValueError(f"{location}는 mapping이어야 합니다.")
        positions = step.get("positions")
        if not isinstance(positions, list) or len(positions) != len(self._joint_names):
            raise ValueError(f"{location}.positions에는 관절 4개의 값이 필요합니다.")
        positions = [float(value) for value in positions]
        for name, position, (lower, upper) in zip(
            self._joint_names, positions, self._joint_limits
        ):
            if not lower <= position <= upper:
                raise ValueError(f"{location}: {name}={position}이 [{lower}, {upper}] 밖입니다.")
        if "gripper" not in step:
            raise ValueError(f"{location}.gripper 값이 필요합니다.")
        gripper = float(step["gripper"])
        if not self._gripper_limits[0] <= gripper <= self._gripper_limits[1]:
            raise ValueError(f"{location}.gripper가 제한 밖입니다.")
        duration = float(step.get("duration", self._default_duration))
        pause = float(step.get("pause", self._default_pause))
        max_effort = float(step.get("max_effort", self._default_max_effort))
        if duration <= 0.0 or pause < 0.0 or max_effort < 0.0:
            raise ValueError(f"{location}의 시간 또는 effort 설정이 잘못되었습니다.")
        return {
            "positions": positions,
            "gripper": gripper,
            "duration": duration,
            "pause": pause,
            "max_effort": max_effort,
        }

    def _joint_state_callback(self, message: JointState) -> None:
        if not set(self._joint_names).issubset(message.name):
            return
        self._current_positions = [
            float(message.position[message.name.index(name)]) for name in self._joint_names
        ]
        self._joint_state_received = True

    def _start_when_ready(self) -> None:
        arm_ready = self._arm_client.server_is_ready()
        gripper_ready = self._gripper_client.server_is_ready()
        if self._joint_state_received and arm_ready and gripper_ready:
            self._start_timer.cancel()
            self.destroy_timer(self._start_timer)
            self.get_logger().info("joint_states와 action server 확인 완료. 춤을 시작합니다.")
            self._dispatch_next_step()
            return
        self._wait_log_counter += 1
        if self._wait_log_counter % 25 == 1:
            self.get_logger().info(
                "시작 대기 중: "
                f"joint_states={self._joint_state_received}, "
                f"arm_action={arm_ready}, gripper_action={gripper_ready}"
            )

    def _prepare_cycle(self) -> bool:
        if self._repeat_count > 0 and self._cycle_count >= self._repeat_count:
            self.get_logger().info(f"{self._repeat_count}회 춤 cycle을 완료했습니다.")
            self._finished = True
            return False
        patterns = list(self._patterns)
        if self._shuffle_patterns:
            self._random.shuffle(patterns)
        self.get_logger().info(
            f"cycle {self._cycle_count + 1} 순서: "
            + " → ".join(pattern["name"] for pattern in patterns)
        )
        for pattern in patterns:
            steps = list(pattern["steps"])
            if self._shuffle_steps:
                self._random.shuffle(steps)
            for index, step in enumerate(steps):
                self._step_queue.append((pattern["name"], index, step))
        self._cycle_count += 1
        return True

    def _dispatch_next_step(self) -> None:
        if self._finished or self._step_running:
            return
        if not self._step_queue and not self._prepare_cycle():
            return
        pattern_name, step_index, step = self._step_queue.popleft()
        positions = [
            self._randomize(value, limit, self._minimum_joint_jitter)
            for value, limit in zip(step["positions"], self._joint_limits)
        ]
        gripper = self._randomize(
            step["gripper"], self._gripper_limits, self._minimum_gripper_jitter
        )

        point = JointTrajectoryPoint()
        point.positions = positions
        seconds = int(step["duration"])
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = int((step["duration"] - seconds) * 1_000_000_000)
        arm_goal = FollowJointTrajectory.Goal()
        arm_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        arm_goal.trajectory.header.frame_id = "dance_manipulator"
        arm_goal.trajectory.joint_names = self._joint_names
        arm_goal.trajectory.points.append(point)
        arm_goal.goal_time_tolerance.sec = 1
        gripper_goal = GripperCommand.Goal()
        gripper_goal.command.position = gripper
        gripper_goal.command.max_effort = step["max_effort"]

        self._step_running = True
        self._step_failed = False
        self._pending_results = {"arm", "gripper"}
        self._current_pause = step["pause"]
        values = ", ".join(f"{value:+.3f}" for value in positions)
        self.get_logger().info(
            f"[{pattern_name} #{step_index + 1}] "
            f"joint=[{values}], gripper={gripper:.5f}, "
            f"이동={step['duration']:.2f}s, 정지={step['pause']:.2f}s"
        )
        arm_future = self._arm_client.send_goal_async(arm_goal)
        arm_future.add_done_callback(self._arm_goal_response)
        gripper_future = self._gripper_client.send_goal_async(gripper_goal)
        gripper_future.add_done_callback(self._gripper_goal_response)

    def _randomize(
        self,
        base_value: float,
        limits: tuple[float, float],
        minimum_jitter: float,
    ) -> float:
        lower, upper = limits
        max_safe_jitter = (upper - lower) * self._variation_ratio
        jitter = min(
            max(abs(base_value) * self._variation_ratio, minimum_jitter),
            max_safe_jitter,
        )
        return min(
            max(base_value + self._random.uniform(-jitter, jitter), lower),
            upper,
        )

    def _arm_goal_response(self, future: Any) -> None:
        try:
            handle = future.result()
        except Exception as error:
            self._finish_channel("arm", False, f"goal 전송 예외: {error}")
            return
        if handle is None or not handle.accepted:
            self._finish_channel("arm", False, "goal이 거부되었습니다.")
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._arm_result)

    def _gripper_goal_response(self, future: Any) -> None:
        try:
            handle = future.result()
        except Exception as error:
            self._finish_channel("gripper", False, f"goal 전송 예외: {error}")
            return
        if handle is None or not handle.accepted:
            self._finish_channel("gripper", False, "goal이 거부되었습니다.")
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._gripper_result)

    def _arm_result(self, future: Any) -> None:
        try:
            wrapped = future.result()
            successful = (
                wrapped.status == GoalStatus.STATUS_SUCCEEDED
                and wrapped.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            )
            detail = wrapped.result.error_string or f"status={wrapped.status}"
        except Exception as error:
            successful, detail = False, f"result 예외: {error}"
        self._finish_channel("arm", successful, detail)

    def _gripper_result(self, future: Any) -> None:
        try:
            wrapped = future.result()
            successful = wrapped.status == GoalStatus.STATUS_SUCCEEDED
            detail = f"position={wrapped.result.position:.5f}, effort={wrapped.result.effort:.3f}"
        except Exception as error:
            successful, detail = False, f"result 예외: {error}"
        self._finish_channel("gripper", successful, detail)

    def _finish_channel(self, channel: str, successful: bool, detail: str) -> None:
        if channel not in self._pending_results:
            return
        self._pending_results.remove(channel)
        if not successful:
            self._step_failed = True
            self.get_logger().error(f"{channel} action 실패: {detail}")
        if self._pending_results:
            return
        self._step_running = False
        if self._step_failed and self._stop_on_error:
            self._finished = True
            self.get_logger().error(
                "안전을 위해 춤을 중단했습니다. controller와 하드웨어를 확인하세요."
            )
            return
        self._delay_timer = self.create_timer(max(self._current_pause, 0.01), self._delay_finished)

    def _delay_finished(self) -> None:
        if self._delay_timer is not None:
            self._delay_timer.cancel()
            self.destroy_timer(self._delay_timer)
            self._delay_timer = None
        self._dispatch_next_step()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DanceManipulator()
        rclpy.spin(node)
    except (FileNotFoundError, ValueError, yaml.YAMLError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"dance_manipulator 설정 오류: {error}")
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
