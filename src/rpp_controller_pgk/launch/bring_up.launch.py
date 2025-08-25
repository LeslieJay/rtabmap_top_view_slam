'''
Autor: wei.canming
Version: 1.0
Date: 2025-07-25 16:56:18
LastEditors: wei.canming
LastEditTime: 2025-08-11 14:29:37
Description: 
'''
import os

# ROS2 Launch系统导入
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ( GroupAction,
                            IncludeLaunchDescription, ExecuteProcess)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    
    bringup_dir = get_package_share_directory('rpp_controller_pgk')
    
    # 构建launch文件目录路径
    # 包含导航相关的启动文件
    launch_dir = os.path.join(bringup_dir, 'launch')
    
    bringup_cmd_group = GroupAction([
        ExecuteProcess(
            cmd=[
                'gnome-terminal', '--', 'bash', '-c', 'ros2 run usbcan_jay_pkg cmd_vel_listener'
            ],
            shell=False
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'agv_AF06_robot_localization_launch.py'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'agv_AF06_navigation_launch.py'))
        )
    ])
    
    ld = LaunchDescription()
    
    ld.add_action(bringup_cmd_group)
    
    return ld
"""
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
"""
