from setuptools import find_packages, setup
from glob import glob
import os

package_name = "sensor_sync"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jincheng Pan",
    maintainer_email="jincpan@gmail.com",
    description="A ROS 2 package to synchronize IMU, Odom, and LaserScan topics.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sensor_sync_node = sensor_sync.sensor_sync_node:main",
        ],
    },
)
