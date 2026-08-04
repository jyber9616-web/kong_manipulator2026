from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="kong_basic", executable="mpub"),
            Node(package="kong_basic", executable="tpub"),
            Node(package="kong_basic", executable="msub"),
            Node(package="kong_basic", executable="m2sub"),
            Node(package="kong_basic", executable="mtsub"),
        ]
    )
