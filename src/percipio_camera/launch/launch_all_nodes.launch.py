'''
Autor: wei.canming
Version: 1.0
Date: 2025-04-01 14:03:26
LastEditors: wei.canming
LastEditTime: 2025-04-10 10:29:30
Description: 一次性启动包括相机节点、里程计节点和建图节点在内的多个节点
'''
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from pathlib import Path
import os

def generate_launch_description():
    # 相机节点的启动文件
    percipio_launch_file_path = Path(__file__).parent / os.path.dirname(os.path.abspath(__file__)) / 'percipio_camera.launch.py'
    # rtabmap建图节点的启动文件
    # rtabmap_launch_file_path = Path(__file__).parent / os.path.dirname(os.path.abspath(__file__)) / 'rtabmap.launch.py'
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([percipio_launch_file_path.as_posix()]),
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([rtabmap_launch_file_path.as_posix()]),
        # ),
        # 同时启动USBCAN驱动的CAN解析节点
        Node(
            package='usbcan',
            executable='usbcan_parser',
            name='usbcan_parser',
            output='log',
            # remappings=[
            #     ("/odom", "/rtabmap/odom")
            # ]
        ),
        # 同时启动USBCAN中的键盘控制节点，控制小车运动 （可选）
        # Node(
        #     package='usbcan',
        #     executable='keyboard_controller',
        #     name='keyboard_controller',
        #     output='screen'
        # ),
        # 同时启动重发布节点，重新映射深度图、彩色图以及同步里程计时间戳
        Node(
            package='republish_pkg',
            executable='image_republish_node',
            name='image_republish_node',
            output='log'
        )
    ])