from setuptools import find_packages, setup
from glob import glob
import os

package_name = "controller"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob(os.path.join("launch", "*.launch.py"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jincheng Pan",
    maintainer_email="jincpan@gamil.com",
    description="Controller Package",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "game_pad_node = controller.game_pad:main",
            "key_board_node = controller.key_board_fixmotion:main",
            "key_board_mecanum = controller.key_board_mecanum:main",
            "game_pad_mecanum = controller.game_pad_mecanum:main",
            "game_pad_fixmotion = controller.game_pad_fixmotion:main",
        ],
    },
)
