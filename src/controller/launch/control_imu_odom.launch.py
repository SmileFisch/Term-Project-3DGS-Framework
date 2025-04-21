from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="imu_publisher",
                executable="imu_orientation_publisher_node",
                name="imu_publisher",
                output="screen",
            ),
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package="odom",
                        executable="odom_node_simple",
                        name="wheelodom_node_simple",
                        output="screen",
                        parameters=[
                            {"wheel_diameter": 0.055},  # Wheel diameter (meters)
                            {"ticks_per_revolution": 4920},  # Pulses per revolution
                            {"gear_ratio": 20.0},  # Gear reduction ratio
                            {"wheel_base": 0.22},
                            # Distance between front and rear wheels (meters)
                            {"wheel_track": 0.18},
                            # Distance between left and right wheels (meters)
                            {"publish_frequency": 10},  # Publishing frequency (Hz)
                        ],
                    )
                ],
            ),
            # TimerAction(
            #     period=1.0,
            #     actions=[
            #         Node(
            #             package='odom',
            #             executable='odom_node',
            #             name='wheelodom_node',
            #             output='screen',
            #             parameters=[
            #                 {'wheel_diameter': 0.055},  # Wheel diameter (meters)
            #                 {'ticks_per_revolution': 4920},  # Pulses per revolution
            #                 {'gear_ratio': 20.0},  # Gear reduction ratio
            #                 {'wheel_base': 0.22},  # Distance between front and rear wheels (meters)
            #                 {'wheel_track': 0.18},  # Distance between left and right wheels (meters)
            #                 {'publish_frequency': 5},  # Publishing frequency (Hz)
            #             ]
            #         ),
            #     ]
            # )
        ]
    )
