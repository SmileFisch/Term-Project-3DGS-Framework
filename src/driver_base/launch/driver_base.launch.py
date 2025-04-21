import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    return LaunchDescription(
        [
            TimerAction(
                period=0.5,
                actions=[
                    Node(
                        package="driver_base",
                        executable="driver_encodermanager_node",
                        name="driver_encodermanager_node",
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package="driver_base",
                        executable="driver_node",
                        name="driver_node",
                        output="screen",
                        parameters=[
                            {"wheel_diameter": 0.055},
                            {"ticks_per_revolution": 4920},
                            {"gear_ratio": 20.0},
                            {"max_rpm": 400},
                            {"wheel_base": 0.22},
                            {"wheel_track": 0.18},
                        ],
                    )
                ],
            ),
        ]
    )
