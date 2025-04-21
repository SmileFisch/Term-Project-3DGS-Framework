from setuptools import find_packages, setup
from glob import glob

package_name = "image_decompress"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),  # 添加 marker 文件到资源索引
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),  # 添加所有 launch 文件
    ],
    install_requires=[
        "setuptools",
        "opencv-python-headless",
        "numpy",
        # 'rclpy',
        "cv_bridge",
    ],
    zip_safe=True,
    maintainer="Jincheng Pan",
    maintainer_email="jincpan@gmail.com",
    description="A ROS 2 package for decompressing image and depth data.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "image_decompress_node = image_decompress.image_decompress_node:main",
            "image_decompress_rs_node = image_decompress.image_decompress_rs_node:main",
            # 'image_decompress_test = image_decompress.image_decompress_test:main',
        ],
    },
)
