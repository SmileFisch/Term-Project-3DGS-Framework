cmake_minimum_required(VERSION 3.5)
project(nav2_config)

# 找到 ROS 2 相关的包
find_package(ament_cmake REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)

# 安装 launch 和 params 目录
install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

# 让 ROS 2 识别这个包
ament_package()
