'''
Autor: wei.canming
Version: 1.0
Date: 2025-07-24 16:37:12
LastEditors: wei.canming
LastEditTime: 2025-07-24 17:19:09
Description: 
'''
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_dir = get_package_share_directory('usbcan')
    
    controller_params = os.path.join(pkg_dir, 'params', 'curve_s_params.yaml')
    
    return LaunchDescription([
        Node(
            package='usbcan',
            executable='cmd_vel_listener',
            name='cmd_vel_listener',
            parameters=[controller_params]
        )
    ])
