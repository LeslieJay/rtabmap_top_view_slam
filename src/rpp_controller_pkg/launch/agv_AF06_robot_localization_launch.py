'''
Autor: wei.canming
Version: 1.0
Date: 2025-07-18 16:21:40
LastEditors: wei.canming
LastEditTime: 2025-08-22 13:59:06
Description: 启动轮式里程计+激光里程计，用robot_localization进行融合,输出/odom/filtered话题；
             发布map->odom的静态tf转换 
'''
#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取包的路径
    pkg_dir = get_package_share_directory('rpp_controller_pkg')
    
    # 定义launch参数  
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 声明launch参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    # DeclareLaunchArgument 与 LaunchConfiguration 的区别
    # DeclareLaunchArgument 用于声明一个launch参数，LaunchConfiguration 用于获取launch参数的值
    # DeclareLaunchArgument 可以让用户在启动launch文件时，传递参数给launch文件
    
    # 创建一个静态TF发布器，发布map->odom的转换
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_odom',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
    )
    
    # 创建一个静态TF发布器，发布base_link->scanner_link的转换
    static_tf_base2scan_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base_scanner',
        output='screen',
        arguments=['0.23', '0.0', '0.3', '0.0', '0.0', '0.0', 'base_link', 'scanner_link']
    )

    # static_tf_node2 = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher_map_world',
    #     output='screen',
    #     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'world']
    # )
    
    # 激光雷达节点
    pf_lidar_launch_path = PathJoinSubstitution([
        FindPackageShare('pf_driver'),
        'launch',
        'r2000.launch.py'
    ])
    
    pf_lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pf_lidar_launch_path)
    )
    
    laser_odom_node = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                    'transform_tolerance': 0.5}]  # 增加变换容忍时间
    )
    
    # 轮式里程计节点
    usbcan_parser_node = Node(
        package='usbcan_jay_pkg',
        executable='usbcan_com_node',
        name='usbcan_com_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # robot_localization节点
    robot_localization_launch_path = PathJoinSubstitution([
        FindPackageShare('robot_localization'),
        'launch',
        'ekf.launch.py'
    ])
    
    robot_localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_localization_launch_path)
    )
    
    # 设置tf2缓存时间环境变量
    tf2_cache_time_env = SetEnvironmentVariable(
        'TF2_CACHE_TIME', '5.0'  # 设置tf2缓存时间为5秒
    )
    print("tf2_cache_time_env", tf2_cache_time_env)

    # 返回LaunchDescription对象
    return LaunchDescription([
        tf2_cache_time_env,  # 添加环境变量设置
        declare_use_sim_time_cmd,
        static_tf_node,
        static_tf_base2scan_node,
        # static_tf_node2,
        pf_lidar_node,
        laser_odom_node,
        usbcan_parser_node,
        robot_localization_node,
    ]) 