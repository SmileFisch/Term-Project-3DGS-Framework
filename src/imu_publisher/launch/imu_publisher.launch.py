from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="imu_publisher",
                executable="imu_orientation_publisher_node",
                name="imu_publisher",
                output="screen",
            ),
            # 静态变换: base_link -> camera_link
            # Node(
            #     package='tf2_ros',
            #     executable='static_transform_publisher',
            #     name='base_to_camera_tf',
            #     arguments=[
            #         '0.1', '0.0', '0.0',  # 位置 (x, y, z)
            #         '0.0', '0.0', '0.0', '1.0',  # 旋转 (x, y, z, w)
            #         'base_link',   # 父坐标系
            #         'camera_link'  # 子坐标系
            #     ]
            # ),
            # 你也可以恢复 camera_link -> imu_frame 变换
            # Node(
            #     package='tf2_ros',
            #     executable='static_transform_publisher',
            #     name='camera_to_imu_tf',
            #     arguments=[
            #         '0.0', '0.0', '0.0',
            #         '0.0', '0.0', '0.0', '1.0',
            #         'camera_imu_optical_frame',  # 父坐标系
            #         'imu_frame'     # 子坐标系
            #     ]
            # )
        ]
    )
