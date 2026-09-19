cmake_minimum_required(VERSION 3.8)

project(motor_control)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


# ==========================================
# Task 2
# Motor Simulator
# ==========================================

add_executable(
  motor_simulator
  src/motor_simulator.cpp
)

ament_target_dependencies(
  motor_simulator
  rclcpp
  std_msgs
)


# ==========================================
# Task 2
# Torque Test Input
# ==========================================

add_executable(
  torque_test_input
  src/torque_test_input.cpp
)

ament_target_dependencies(
  torque_test_input
  rclcpp
  std_msgs
)


# ==========================================
# Task 3
# Speed Target
# ==========================================

add_executable(
  speed_target
  src/speed_target.cpp
)

ament_target_dependencies(
  speed_target
  rclcpp
  std_msgs
)


# ==========================================
# Task 3
# PID Controller
# ==========================================

add_executable(
  pid_controller
  src/pid_controller.cpp
)

ament_target_dependencies(
  pid_controller
  rclcpp
  std_msgs
)


# ==========================================
# Install
# ==========================================

install(
  TARGETS
    motor_simulator
    torque_test_input
    speed_target
    pid_controller

  DESTINATION
    lib/${PROJECT_NAME}
)


# Install launch files
install(
  DIRECTORY
    launch

  DESTINATION
    share/${PROJECT_NAME}
)


ament_package()
