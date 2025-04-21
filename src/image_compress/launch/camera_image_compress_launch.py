import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    rgb_camera_profile = LaunchConfiguration("rgb_camera_profile", default="640,480,15")
    depth_profile = LaunchConfiguration("depth_profile", default="640,480,15")
    gyro_fps = LaunchConfiguration("gyro_fps", default="200")
    accel_fps = LaunchConfiguration("accel_fps", default="200")

    realsense_launch_path = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
    )

    image_compress_launch_path = os.path.join(
        get_package_share_directory("image_compress"), "launch", "image_compress.launch.py"
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            "rgb_camera.color_profile": rgb_camera_profile,
            "depth_module.depth_profile": depth_profile,
            "align_depth.enable": "true",
            "enable_sync": "true",
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "2",
            "pointcloud.enable": "true",
            "pointcloud.stream_filter": "0",
            "pointcloud.ordered_pc": "false",
            "pointcloud.allow_no_texture_points": "true",
            "gyro_fps": gyro_fps,
            "accel_fps": accel_fps,
        }.items(),
    )

    image_compress_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(image_compress_launch_path)
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rgb_camera_profile", default_value="640,480,15", description="RGB camera profile"
            ),
            DeclareLaunchArgument(
                "depth_profile", default_value="640,480,15", description="Depth module profile"
            ),
            DeclareLaunchArgument("gyro_fps", default_value="200", description="Gyroscope FPS"),
            DeclareLaunchArgument(
                "accel_fps", default_value="200", description="Accelerometer FPS"
            ),
            realsense_launch,
            image_compress_launch,
        ]
    )
