cmake_minimum_required(VERSION 3.8)
project(fork_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # for custom interfaces
find_package(Eigen3 REQUIRED) # for Refelctor.hpp

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fork.action"
  "action/CargoHeight.action"
  "action/CargoRotation.action"
  "msg/BatteryState.msg"
  "srv/BatteryControl.srv"
  "msg/LandMark.msg"
  "msg/QrCode.msg"
  "msg/PathArray.msg"
  "msg/FloatX.msg"
  "msg/FloatY.msg"
  "msg/MyOdom.msg"
  "msg/DifferentialWheel.msg"
  "msg/QrLable.msg"
  "msg/MyPath.msg"
  "msg/Test.msg"
  "msg/Poses.msg"
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()
