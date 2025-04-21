from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="image_decompress",
                executable="image_decompress_node",
                name="image_decompressor",
                output="screen",
                parameters=[
                    {
                        "compressed_color_topic": "/camera/compressed_image",
                        "compressed_depth_topic": "/camera/compressed_depth_image",
                        "color_image_topic": "/camera/decompressed_color_image",
                        "depth_image_topic": "/camera/decompressed_depth_image",
                        "source_file": "rgbd_dataset_Test",
                    }
                ],
            )
        ]
    )
