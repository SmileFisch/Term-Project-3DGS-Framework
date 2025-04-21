from setuptools import setup, find_packages

package_name = "imu_publisher"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(include=["imu_publisher", "imu_publisher.*"]),
    # 确保包含子模块
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/imu_publisher.launch.py"],
        ),  # 确保安装 launch 文件
    ],
    install_requires=[
        "setuptools",
        "pyrealsense2",  # 添加第三方依赖 pyrealsense2
        "numpy",  # 添加第三方依赖 numpy
        # 如果需要 MadgwickAHRS 代码，确保它作为依赖
    ],
    zip_safe=True,
    maintainer="Jincheng Pan",  # 替换为实际维护者
    maintainer_email="your_email@example.com",  # 替换为实际邮箱
    description="Publishes IMU data with orientation from RealSense cameras.",  # 更新描述信息
    license="Apache-2.0",  # 确保填写正确的开源协议
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_orientation_publisher_node = imu_publisher.imu_orientation_publisher_node:main",  # 对应新脚本的入口点
            "imu_publisher_node = imu_publisher.imu_publisher_node:main",  # 对应主脚本的入口点
        ],
    },
)
