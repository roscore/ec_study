cmake_minimum_required(VERSION 3.5)
project(ec_study)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ethercat_driver_ros2 REQUIRED)

# Include directories
include_directories(
  include
  ${ethercat_driver_ros2_INCLUDE_DIRS}
)

# Add executable
add_executable(motor_control_node src/motor_control_node.cpp)

# Link libraries
ament_target_dependencies(motor_control_node rclcpp std_msgs ethercat_driver_ros2)

# Install executable
install(TARGETS motor_control_node
  DESTINATION lib/${PROJECT_NAME})

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/)

ament_package()

