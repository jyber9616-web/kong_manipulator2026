"""가장 최근에 가르친 OpenManipulator-X 동작을 재생하는 노드."""

from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory

from tf2_basic.dance_manipulator import DanceManipulator


def find_latest_recording() -> Path:
    """홈과 패키지 data 폴더에서 가장 최근 녹화 파일을 찾는다."""
    search_directories = [Path.home(), Path.cwd()]
    try:
        share_data = Path(get_package_share_directory("tf2_basic")) / "data"
        search_directories.append(share_data)
    except Exception:
        pass

    candidates: dict[Path, int] = {}
    for directory in search_directories:
        if not directory.is_dir():
            continue
        for path in directory.glob("recorded_dance*.yaml"):
            resolved = path.resolve()
            candidates[resolved] = resolved.stat().st_mtime_ns
    if candidates:
        return max(candidates, key=candidates.get)
    return Path.home() / "recorded_dance.yaml"


class RecordedDancePlayer(DanceManipulator):
    """녹화 YAML을 기존 action 실행기로 재생한다."""

    def __init__(self) -> None:
        default_file = find_latest_recording()
        super().__init__(
            node_name="play_recorded_dance",
            default_dance_file=str(default_file),
        )
        selected = str(self.get_parameter("dance_file").value)
        self.get_logger().info(f"녹화 동작 재생 파일: {selected}")
        self.get_logger().warning("재생 전에 모터 토크가 ON인지 확인하고 로봇 주변을 비우세요.")


def main(args: list[str] | None = None) -> None:
    """녹화 동작 재생 노드를 실행한다."""
    rclpy.init(args=args)
    node = None
    try:
        node = RecordedDancePlayer()
        rclpy.spin(node)
    except (FileNotFoundError, ValueError, yaml.YAMLError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"play_recorded_dance 설정 오류: {error}")
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
