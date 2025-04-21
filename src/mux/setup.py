from setuptools import find_packages, setup

package_name = "mux"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jincheng Pan",
    maintainer_email="jincpan@gmail.com",
    description="A multiplexer node to manage control mode switching and command velocity publishing in ROS 2.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["mux_node = mux.mux_node:main"],
    },
)
