from setuptools import setup
from setuptools import find_packages
from glob import glob

package_name = "odom"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jincheng Pan",
    maintainer_email="jincpan@gamil.com",
    description="Odometry package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odom_node = odom.wheelodom_node:main",
            "odom_node_simple = odom.wheelodom_node_simple:main",
        ],
    },
)
