# 保留原有内容，补充注释
cmake_minimum_required(VERSION 3.8)
project(usbcan)

# 查找构建依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ref_slam_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(fork_interfaces REQUIRED)

# 包含头文件目录
include_directories(
	${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
)

# 链接库目录（如有自定义库）
link_directories(
	${PROJECT_SOURCE_DIR}/lib
)

# 可执行文件：usbcan_parser
add_executable(usbcan_parser 
	src/main.cpp
	src/usbcan_parser.cpp
	src/usbcan_battery.cpp
	src/usbcan_utils.cpp)
ament_target_dependencies(usbcan_parser 
	rclcpp 
	nav_msgs 
	tf2_ros 
	geometry_msgs
	ref_slam_interface
	fork_interfaces)
target_link_libraries(usbcan_parser
	usbcan
)

# 可执行文件：fork_cli
add_executable(fork_cli src/client.cpp)
ament_target_dependencies(fork_cli 
	rclcpp 
	rclcpp_action
	ref_slam_interface)

# 可执行文件：keyboard_controller
add_executable(keyboard_controller 
	src/keyboard_controller.cpp
)
ament_target_dependencies(keyboard_controller 
	rclcpp 
	nav_msgs 
	tf2_ros 
	geometry_msgs
	ref_slam_interface
	fork_interfaces)

add_executable(cmd_vel_listener 
	src/cmd_vel_listener.cpp
)
ament_target_dependencies(cmd_vel_listener 
	rclcpp 
	geometry_msgs
	fork_interfaces)
# 安装所有可执行文件
install(TARGETS
	usbcan_parser
	fork_cli
	keyboard_controller
	cmd_vel_listener
	DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
	launch
	params
	DESTINATION share/${PROJECT_NAME}
  )

ament_package()

