cmake_minimum_required(VERSION 3.8)
project(rpp_controller_pgk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# path_publisher executable
add_executable(path_publisher src/path_publisher.cpp)
ament_target_dependencies(path_publisher 
  rclcpp 
  rclcpp_action
  nav_msgs 
  nav2_msgs
  geometry_msgs 
  tf2 
  tf2_geometry_msgs
)

# fake_odom_publisher executable
add_executable(fake_odom_publisher src/fake_odom_publisher.cpp)
ament_target_dependencies(fake_odom_publisher 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  tf2 
  tf2_ros
)

# Install executables
install(TARGETS
  path_publisher
  fake_odom_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files
install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params/
)

# Install maps
install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
