cmake_minimum_required(VERSION 3.8)
project(usbcan_jay_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) # 生成自定义接口



# 该命令用于生成自定义ROS2消息接口代码
######位置必须是 pkg/msg/DifferentialWheel.msg, pkg/custom_interfaces/msg/DifferentialWheel.msg 会报错
rosidl_generate_interfaces(${PROJECT_NAME} # ${PROJECT_NAME}：指定当前包名，生成的接口代码会归属于该包
  "msg/DifferentialWheel.msg"  # "msg/DifferentialWheel.msg"：指定要生成的自定义消息文件 
)

# 导出自定义接口依赖
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# 包含头文件目录
include_directories(
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
)


add_executable(usbcan_com_node 
  src/main.cpp
  src/usbcan_parser.cpp
  src/usbcan_utils.cpp
)

# 该命令为usbcan_com_node目标指定头文件搜索路径
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>：在构建时使用源码目录下的include文件夹
# $<INSTALL_INTERFACE:include>：安装后使用安装路径下的include文件夹
target_include_directories(usbcan_com_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(usbcan_com_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(usbcan_com_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 链接usbcan库（在/usr/lib/libusbcan.so）和自定义接口
target_link_libraries(usbcan_com_node usbcan "${cpp_typesupport_target}")


add_executable(keyboard_controller_node 
  src/keyboard_controller.cpp
)
ament_target_dependencies(keyboard_controller_node 
	rclcpp 
	nav_msgs 
	tf2_ros 
	geometry_msgs
	)

target_link_libraries(keyboard_controller_node "${cpp_typesupport_target}")

target_include_directories(keyboard_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(cmd_vel_listener 
    src/cmd_vel_listener.cpp
)
ament_target_dependencies(cmd_vel_listener 
	rclcpp 
	geometry_msgs
	)
target_link_libraries(cmd_vel_listener "${cpp_typesupport_target}")
target_include_directories(cmd_vel_listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 安装可执行文件
install(TARGETS 
  usbcan_com_node
  keyboard_controller_node
  cmd_vel_listener
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
	launch
	params
	DESTINATION share/${PROJECT_NAME}
  )


# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)

ament_package()
