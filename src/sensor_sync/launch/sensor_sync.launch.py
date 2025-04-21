import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="sensor_sync",
                executable="sensor_sync_node",
                name="sensor_sync_node",
                output="screen",
                parameters=[],
            )
        ]
    )
