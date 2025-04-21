from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="image_decompress",
                executable="image_decompress_rs_node",
                name="image_rs_decompressor",
                output="screen",
                parameters=[
                    {
                        "compressed_color_topic": "/camera/camera/color/image_raw/compressed",
                        "compressed_depth_topic": "/camera/camera/depth/image_rect_raw/compressedDepth",
                        "color_image_topic": "/camera/decompressed_color_image",
                        "depth_image_topic": "/camera/decompressed_depth_image",
                        "camera_info_topic": "/camera/camera/depth/camera_info",
                        "depth_image_width": 640,
                        "depth_image_height": 480,
                    }
                ],
            )
        ]
    )
