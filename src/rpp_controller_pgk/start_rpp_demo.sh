#!/bin/bash
###
 # @Autor: wei.canming
 # @Version: 1.0
 # @Date: 2025-07-18 17:02:05
 # @LastEditors: wei.canming
 # @LastEditTime: 2025-07-19 15:38:35
 # @Description: 
### 

# 设置ROS2环境
source /opt/ros/humble/setup.bash

# 构建包
# echo "Building package..."
# cd $(dirname "$0")/../..
# colcon build --packages-select rpp_controller_pgk

# 设置环境
source install/setup.bash

# 启动仿真环境
echo "Launching RPP controller demo..."
ros2 launch rpp_controller_pgk rpp_controller_demo.launch.py 