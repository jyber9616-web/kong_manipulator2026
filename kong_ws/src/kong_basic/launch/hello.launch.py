from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="kong_basic", executable="class_pub"),
            Node(package="kong_basic", executable="class_sub"),
        ]
    )
