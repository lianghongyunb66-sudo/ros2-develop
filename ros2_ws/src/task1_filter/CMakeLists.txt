cmake_minimum_required(VERSION 3.8)

project(task1_filter)


# 使用 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


# 编译警告
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()


# ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


# ========================================
# signal_generator
# ========================================

add_executable(
  signal_generator
  src/signal_generator.cpp
)

ament_target_dependencies(
  signal_generator
  rclcpp
  std_msgs
)


# ========================================
# signal_filter
# ========================================

add_executable(
  signal_filter
  src/signal_filter.cpp
)

ament_target_dependencies(
  signal_filter
  rclcpp
  std_msgs
)


# 安装可执行程序
install(
  TARGETS
    signal_generator
    signal_filter

  DESTINATION
    lib/${PROJECT_NAME}
)


# 安装launch文件
install(
  DIRECTORY
    launch

  DESTINATION
    share/${PROJECT_NAME}
)


ament_package()

