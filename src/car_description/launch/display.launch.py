# import launch
# from launch.substitutions import Command, LaunchConfiguration
# import launch_ros
# import os

# def generate_launch_description():
#     # Find the path of the car_description package
#     pkg_share = launch_ros.substitutions.FindPackageShare(package='car_description').find('car_description')

#     # Define the default paths for the URDF file and the RViz configuration file
#     default_model_path = os.path.join(pkg_share, 'src/description/car_description.urdf')
#     default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

#     # Node to publish the robot state (TF transforms) based on the URDF file
#     robot_state_publisher_node = launch_ros.actions.Node(
#         package='robot_state_publisher',  # Package name
#         executable='robot_state_publisher',  # Executable to run
#         parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]  # Use xacro to process the model
#     )

#     # Node for the non-GUI joint state publisher
#     # joint_state_publisher_node = launch_ros.actions.Node(
#     #     package='joint_state_publisher',  # Package name
#     #     executable='joint_state_publisher',  # Executable to run
#     #     name='joint_state_publisher',  # Node name
#     #     arguments=[default_model_path],  # Path to the URDF file
#     #     condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))  # Only launch if 'gui' is False
#     # )

#     # Node for the GUI-based joint state publisher
#     # joint_state_publisher_gui_node = launch_ros.actions.Node(
#     #     package='joint_state_publisher_gui',  # Package name
#     #     executable='joint_state_publisher_gui',  # Executable to run
#     #     name='joint_state_publisher_gui',  # Node name
#     #     condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))  # Only launch if 'gui' is True
#     # )

#     # Node for launching RViz to visualize the robot
#     # rviz_node = launch_ros.actions.Node(
#     #     package='rviz2',  # Package name
#     #     executable='rviz2',  # Executable to run
#     #     name='rviz2',  # Node name
#     #     output='screen',  # Output logs to the screen
#     #     arguments=['-d', LaunchConfiguration('rvizconfig')],  # Load the specified RViz configuration file
#     # )

#     # Return the launch description with all the declared arguments and nodes
#     return launch.LaunchDescription([
#         # Declare the 'gui' argument (default: True) to enable/disable the GUI joint state publisher
#         launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
#                                              description='Flag to enable joint_state_publisher_gui'),
#         # Declare the 'model' argument to specify the path to the URDF file
#         launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
#                                              description='Absolute path to robot urdf file'),
#         # Declare the 'rvizconfig' argument to specify the path to the RViz configuration file
#         launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
#                                              description='Absolute path to rviz config file'),
#         # Add the nodes to the launch description
#         # joint_state_publisher_node,
#         # joint_state_publisher_gui_node,
#         robot_state_publisher_node,
#         # rviz_node
#     ])
import launch
import launch_ros
import os


def generate_launch_description():
    # 获取 URDF 文件的路径
    pkg_share = launch_ros.substitutions.FindPackageShare(package="car_description").find(
        "car_description"
    )
    urdf_file = os.path.join(pkg_share, "src/description/car_description.urdf")

    # 创建 robot_state_publisher 以发布 TF
    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": launch.substitutions.Command(["cat ", urdf_file])}],
    )

    return launch.LaunchDescription([robot_state_publisher_node])
