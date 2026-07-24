from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("tf2_basic")
    urdf_path = PathJoinSubstitution(
        [package_share, "urdf", "05_add_arm.urdf"]
    )
    rviz_path = PathJoinSubstitution(
        [package_share, "rviz", "urdf.rviz"]
    )

    robot_description = ParameterValue(
        Command(["xacro ", urdf_path]),
        value_type=str,
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="tf2_basic",
                executable="move_u2d2",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_path],
                output="screen",
            ),
        ]
    )
