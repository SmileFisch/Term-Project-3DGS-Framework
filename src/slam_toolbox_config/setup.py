from setuptools import find_packages, setup
import os
from glob import glob

package_name = "slam_toolbox_config"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # 添加 config/ 目录
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        # 添加 launch/ 目录
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jcp",
    maintainer_email="jcp@todo.todo",
    description="Package containing launch and config files for slam_toolbox",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
