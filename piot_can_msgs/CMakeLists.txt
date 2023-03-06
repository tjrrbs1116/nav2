################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(piot_can_msgs)

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
find_package(rosidl_default_generators REQUIRED)

################################################################################
# Build
################################################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BmsFb.msg" 
  "msg/BmsFlagFb.msg" 
  "msg/CtrlCmd.msg" 
  "msg/CtrlFb.msg" 
  "msg/FrontAngleFb.msg" 
  "msg/FrontAngleFreeCtrlCmd.msg" 
  "msg/FrontVelocityFreeCtrlCmd.msg" 
  "msg/IoCmd.msg" 
  "msg/IoFb.msg" 
  "msg/LfWheelFb.msg" 
  "msg/LrWheelFb.msg" 
  "msg/RearAngleFb.msg" 
  "msg/RearAngleFreeCtrlCmd.msg" 
  "msg/RearVelocityFreeCtrlCmd.msg" 
  "msg/RfWheelFb.msg" 
  "msg/RrWheelFb.msg" 
  "msg/SteeringCtrlCmd.msg" 
  "msg/SteeringCtrlFb.msg" 
)


################################################################################
# Install
################################################################################


################################################################################
# Macro for ament package
################################################################################
ament_package()

