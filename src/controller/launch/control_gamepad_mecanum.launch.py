import launch
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # Set ROS domain ID to 30
    # set_domain = SetEnvironmentVariable('ROS_DOMAIN_ID', '10')

    # game_pad_node = Node(
    #     package='controller',
    #     executable='game_pad_mecanum',
    #     name='game_pad_mecanum',
    #     output='screen',
    #     parameters=[
    #         {'spd_rate': 0.010},
    #         {'rot_limit': 0.02},
    #         {'mode': 'j'}  # 'j': joystick, 'k': keyboard, 'l': algorithm-based control
    #     ]
    # )
    game_pad_fixmotion_node = Node(
        package="controller",
        executable="game_pad_fixmotion",
        name="game_pad_fixmotion",
        output="screen",
        parameters=[
            # {'spd_rate': 0.005},    # Speed limit
            # {'rot_limit': 0.002},   # Rotation speed limit
            {"spd_rate": 0.004},  # Speed limit
            {"rot_limit": 0.008},  # Rotation speed limit
            {"mode": "j"},  # 'j': joystick, 'k': keyboard, 'l': algorithm-based control
            {"dist_limit": 0.002},
            {"angle_limit": 0.002},
            {"wait_time": 6.0},
        ],
    )
    # key_board_node = Node(
    #     package='controller',
    #     executable='key_board_mecanum',
    #     name='key_board_node',
    #     output='screen'
    # )

    # mux_node = Node(
    #     package='mux',
    #     executable='mux_node',
    #     name='mux_node',
    #     # output='screen'
    # )

    # driver_base = Node(
    #     package='driver_base',
    #     executable='driver_node',
    #     name='driver_node',
    #     output='screen',
    #     parameters=[
    #         {'wheel_diameter': 0.055},
    #         {'ticks_per_revolution': 4920}, # 246
    #         {'gear_ratio': 20.0},
    #         {'max_rpm': 400},
    #         {'wheel_base': 0.22},
    #         {'wheel_track': 0.18},
    #     ]
    # )

    # encoder_base = Node(
    #     package='driver_base',
    #     executable='driver_encodermanager_node',
    #     name='driver_encodermanager_node',
    #     output='screen',
    # )

    # delay_launch_car_driver = TimerAction(
    #     period=1.0,
    #     actions=[
    #         driver_base,
    #         encoder_base
    #     ]
    # )

    return LaunchDescription(
        [
            # set_domain,
            # game_pad_node,
            game_pad_fixmotion_node,
            # key_board_node,
            # mux_node,
            # insert here for one sec waiting
            # car_driver_node # will cause problem, the driver node will not receive enough message and break out
            # delay_launch_car_driver # delay 1.0s to start driver node
        ]
    )
