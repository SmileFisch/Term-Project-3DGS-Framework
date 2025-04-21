import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "controller"
    imu_odom_launch = os.path.join(
        get_package_share_directory(package_name), "launch", "control_imu_odom.launch.py"
    )
    mecanum_launch = os.path.join(
        get_package_share_directory(package_name), "launch", "control_mecanum.launch.py"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(imu_odom_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(mecanum_launch)),
        ]
    )
