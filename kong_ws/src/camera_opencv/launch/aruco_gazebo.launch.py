"""Launch OpenManipulator-X Gazebo, an ArUco cube, TF detection, and RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("open_manipulator_bringup"))
    camera_opencv_share = Path(get_package_share_directory("camera_opencv"))
    description_share = Path(get_package_share_directory("open_manipulator_description"))

    marker_size = LaunchConfiguration("marker_size")
    marker_id = LaunchConfiguration("marker_id")
    show_image = LaunchConfiguration("show_image")
    start_rviz = LaunchConfiguration("start_rviz")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_share / "launch" / "open_manipulator_x_gazebo.launch.py")
        ),
        launch_arguments={
            "world": str(camera_opencv_share / "worlds" / "aruco_world")
        }.items(),
    )

    # OpenCV 자세는 optical frame(+Z forward, +X right, +Y down) 기준입니다.
    camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "-1.57079632679",
            "--pitch",
            "0.0",
            "--yaw",
            "-1.57079632679",
            "--frame-id",
            "camera_link",
            "--child-frame-id",
            "camera_optical_frame",
        ],
    )

    aruco_node = Node(
        package="camera_opencv",
        executable="aruco_tf_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "image_topic": "/gripper_camera/image_raw",
                "camera_info_topic": "/gripper_camera/camera_info",
                "dictionary": "DICT_4X4_50",
                "marker_size": ParameterValue(marker_size, value_type=float),
                "target_marker_id": ParameterValue(marker_id, value_type=int),
                "parent_frame": "camera_optical_frame",
                "show_image": ParameterValue(show_image, value_type=bool),
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", str(description_share / "rviz" / "open_manipulator.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("marker_size", default_value="0.04"),
            DeclareLaunchArgument("marker_id", default_value="0"),
            DeclareLaunchArgument("show_image", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            gazebo,
            camera_optical_tf,
            aruco_node,
            rviz,
        ]
    )
