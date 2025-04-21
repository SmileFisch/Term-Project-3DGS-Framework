from setuptools import find_packages, setup
from glob import glob

package_name = "driver_base"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=[
        "setuptools",
        "gpiozero",
        "rclpy",
    ],
    zip_safe=True,
    maintainer="Jincheng Pan",
    maintainer_email="jincpan@gmail.com",
    description="Motor and Encoder Control for Raspberry Pi using ROS 2.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "driver_base = driver_base.driver_base:main",
            "driver_node = driver_base.driver_node:main",
            "driver_encodermanager_node = driver_base.encodermanager_node:main",
        ],
    },
)
