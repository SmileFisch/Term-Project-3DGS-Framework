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
    #     name = 'game_pad_mecanum',
    #     output = 'screen'
    # )

    # key_board_node = Node(
    #     package='controller',
    #     executable='key_board_mecanum',
    #     name = 'key_board_node',
    #     output = 'screen'
    # )

    mux_node = Node(
        package="mux",
        executable="mux_node",
        name="mux_node",
        # output='screen'
    )

    driver_base = Node(
        package="driver_base",
        executable="driver_node",
        name="driver_node",
        output="screen",
        parameters=[
            {"wheel_diameter": 0.055},
            {"ticks_per_revolution": 4920},
            # {"ticks_per_revolution": 260},
            {"gear_ratio": 20.0},
            {"max_rpm": 400},
            {"wheel_base": 0.22},
            {"wheel_track": 0.18},
        ],
    )

    encoder_base = Node(
        package="driver_base",
        executable="driver_encodermanager_node",
        name="driver_encodermanager_node",
        output="screen",
    )

    delay_launch_car_driver = TimerAction(period=1.0, actions=[driver_base, encoder_base])

    return LaunchDescription(
        [
            # set_domain,
            # game_pad_node,
            # key_board_node,
            mux_node,
            # insert here for one sec waiting
            # car_driver_node # will cause problem, the driver node will not
            # receive enough message and break out
            delay_launch_car_driver,  # delay 1.0s to start driver node
        ]
    )
