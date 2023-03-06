################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(piot_can_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(piot_can_msgs REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
)

add_executable(${PROJECT_NAME}_node src/piot_can_control.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "tf2_ros"
  "nav_msgs"
  "sensor_msgs"
  "piot_can_msgs"
)

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})


################################################################################
# Macro for ament package
################################################################################
ament_package()

