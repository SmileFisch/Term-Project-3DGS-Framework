from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "color_image_topic",
                default_value="/camera/camera/color/image_raw",
                description="Input topic for color image",
            ),
            # DeclareLaunchArgument(
            #     'depth_image_topic', default_value='/camera/camera/depth/image_rect_raw',
            #     description='Input topic for depth image'),
            DeclareLaunchArgument(
                "depth_image_topic",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
                description="Input topic for depth image",
            ),
            DeclareLaunchArgument(
                "compressed_color_topic",
                default_value="/camera/compressed_image",
                description="Output topic for compressed color image",
            ),
            DeclareLaunchArgument(
                "compressed_depth_topic",
                default_value="/camera/compressed_depth_image",
                description="Output topic for compressed depth image",
            ),
            DeclareLaunchArgument(
                "color_compression_format",
                default_value=".jpg",
                description="Compression format for color images",
            ),
            DeclareLaunchArgument(
                "depth_compression_format",
                default_value=".png",
                description="Compression format for depth images",
            ),
            Node(
                package="image_compress",
                executable="image_compress_node",
                name="image_compressor",
                output="screen",
                parameters=[
                    {
                        "color_image_topic": LaunchConfiguration("color_image_topic"),
                        "depth_image_topic": LaunchConfiguration("depth_image_topic"),
                        "compressed_color_topic": LaunchConfiguration("compressed_color_topic"),
                        "compressed_depth_topic": LaunchConfiguration("compressed_depth_topic"),
                        "color_compression_format": LaunchConfiguration("color_compression_format"),
                        "depth_compression_format": LaunchConfiguration("depth_compression_format"),
                    }
                ],
            ),
        ]
    )
