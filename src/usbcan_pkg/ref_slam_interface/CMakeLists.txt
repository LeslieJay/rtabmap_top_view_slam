cmake_minimum_required(VERSION 3.5)
project(ref_slam_interface)

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
  "srv/SaveMap.srv"
  "action/CtrlFork.action"
  "msg/BatteryState.msg"
  "srv/BatteryControl.srv"
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()
