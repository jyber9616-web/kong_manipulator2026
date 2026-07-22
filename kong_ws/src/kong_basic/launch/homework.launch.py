from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="kong_basic", executable="class_pub"),
        Node(package="kong_basic", executable="class_sub", name="message_sub",),
        Node(package="kong_basic", executable="class_sub", name="message_sub_2",),
        Node(package="kong_basic", executable="header_pub", name="time_pub",),
        Node(package="kong_basic", executable="time_sub", name="message_time_sub",),
        ]
    )