'''
Autor: wei.canming
Version: 1.0
Date: 2025-07-18 16:21:40
LastEditors: wei.canming
LastEditTime: 2025-08-25 14:19:50
Description: 
'''
#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 声明launch参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    # DeclareLaunchArgument 与 LaunchConfiguration 的区别
    # DeclareLaunchArgument 用于声明一个launch参数，LaunchConfiguration 用于获取launch参数的值
    # DeclareLaunchArgument 可以让用户在启动launch文件时，传递参数给launch文件
    
    # 创建一个静态TF发布器，发布map->odom->base_link的转换
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
    )

    # static_tf_node2 = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher_base_link',
    #     output='screen',
    #     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_link']
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
        parameters=[{'use_sim_time': use_sim_time}]
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

    # 创建一个虚拟地图服务器
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': os.path.join(pkg_dir, 'maps', 'empty_map.yaml'),
            'use_sim_time': use_sim_time,
            'topic_name': 'map',
            'frame_id': 'map'
        }]
    )
    
    # Regulated Pure Pursuit控制器参数
    controller_params = os.path.join(pkg_dir, 'params', 'controller_params.yaml')
    
    # 路径发布器节点
    # 'bash', '-c' 保持终端不关闭
    path_publisher_process = ExecuteProcess(
            cmd=[
                'gnome-terminal', '--', 'bash', '-c', 'ros2 run rpp_controller_pkg path_publisher'
            ],
            shell=False
        )
    # path_publisher_node = Node(
    #     package='rpp_controller_pkg',
    #     executable='path_publisher',
    #     name='path_publisher',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': use_sim_time}
    #     ]
    # )
    
    # 添加Regulated Pure Pursuit控制器节点
    # Regulated Pure Pursuit控制器节点
    controller_node = Node(
        package='nav2_controller',  # 使用nav2_controller包
        executable='controller_server',  # 启动controller_server可执行文件
        name='controller_server',  # 节点名称
        output='screen',  # 输出到终端
        parameters=[
            controller_params,  # 控制器参数文件
            {'use_sim_time': use_sim_time}  # 是否使用仿真时间
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel'),  # 速度指令话题重映射
            ('odom', '/odometry/filtered')         # 里程计话题重映射
        ]
    )
    
    # 生命周期管理节点 - 只管理controller_server和map_server
    lifecycle_nodes = ['map_server', 'controller_server']
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': lifecycle_nodes}
        ]
    )
    
    # 速度话题接收节点，发布底层差速轮速度指令
    # cmd_vel_listener_node = Node(
    #     package='usbcan_jay_pkg',
    #     executable='cmd_vel_listener',
    #     name='cmd_vel_listener',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )
    cmd_vel_listener_process = ExecuteProcess(
            cmd=[
                'gnome-terminal', '--', 'bash', '-c', 'ros2 run usbcan_jay_pkg cmd_vel_listener'
            ],
            shell=False
        )
    
    # 返回LaunchDescription对象
    return LaunchDescription([
        declare_use_sim_time_cmd,
        static_tf_node,
        # static_tf_node2,
        pf_lidar_node,
        laser_odom_node,
        usbcan_parser_node,
        robot_localization_node,
        map_server_node,
        controller_node,
        path_publisher_process,
        lifecycle_manager,
        cmd_vel_listener_process,
    ]) 