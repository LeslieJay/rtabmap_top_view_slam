# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Navigation2 核心导航组件启动文件

本文件负责启动Navigation2的核心导航功能组件，这是整个导航系统的核心部分。
包含路径规划、控制、行为树导航、路径平滑、恢复行为等关键功能模块。

核心组件架构：
============

1. 路径规划层（Planning Layer）：
   - Planner Server: 全局路径规划服务
   - Smoother Server: 路径平滑优化服务

2. 控制执行层（Control Layer）：
   - Controller Server: 路径跟踪控制服务
   - Velocity Smoother: 速度平滑处理

3. 决策协调层（Decision Layer）：
   - BT Navigator: 行为树导航协调器
   - Behavior Server: 恢复行为执行服务

4. 任务管理层（Task Layer）：
   - Waypoint Follower: 航点序列跟随器
   - Lifecycle Manager: 组件生命周期管理器

数据流向：
=========
目标位姿 -> BT Navigator -> Planner Server -> Smoother Server -> 
Controller Server -> Velocity Smoother -> 机器人控制命令

核心特性：
=========
- 模块化设计：每个组件独立可配置
- 插件架构：支持算法插件动态加载
- 行为树控制：灵活的导航逻辑编排
- 生命周期管理：统一的组件状态控制
- 组合式运行：支持容器内高效运行

作者: Intel Corporation
版本: Navigation2 ROS2 版本
"""

import os

# ROS2 Launch系统导入
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    
    bringup_dir = get_package_share_directory('rpp_controller_pgk')

    namespace = LaunchConfiguration('namespace')          # 命名空间配置
    use_sim_time = LaunchConfiguration('use_sim_time')    # 仿真时间开关
    autostart = LaunchConfiguration('autostart')          # 自动启动开关
    params_file = LaunchConfiguration('params_file')      # 参数文件路径
    use_composition = LaunchConfiguration('use_composition')  # 组合模式开关
    use_respawn = LaunchConfiguration('use_respawn')      # 重启开关
    log_level = LaunchConfiguration('log_level')          # 日志级别
    map_yaml_file = LaunchConfiguration('map')
    

    
    lifecycle_nodes = [
                       'map_server',              # 地图服务器
                       'controller_server',      # 路径跟踪控制服务器
                       'smoother_server',         # 路径平滑优化服务器
                       'planner_server',          # 全局路径规划服务器
                       'behavior_server',         # 恢复行为执行服务器
                       'bt_navigator',            # 行为树导航协调器
                       'waypoint_follower',       # 航点序列跟随器
                       'velocity_smoother']       # 速度命令平滑器
    """
    velocity_smoother: 
        - PUB: /cmd_vel: geometry_msgs/msg/Twist
        - SUB: /odometry/filtered: nav_msgs/msg/Odometry
    
    velocity_smoother:
        - PUB:    
        
    """
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,    # 仿真时间设置
        'autostart': autostart,
        'yaml_filename': map_yaml_file}          # 自动启动设置


    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')


    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_map_yaml_file_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'empty_map.yaml'),
        description='Full path to map file')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')


    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')


    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            
            Node(
                package='nav2_map_server',           # 地图服务器包
                executable='map_server',             # 可执行文件名
                name='map_server',                   # 节点名称
                output='screen',                     # 输出到屏幕
                respawn=use_respawn,                 # 崩溃重启设置
                respawn_delay=2.0,                   # 重启延迟（秒）
                parameters=[configured_params],      # 配置参数
                arguments=['--ros-args', '--log-level', log_level],  # 日志级别
                remappings=remappings),              # 话题重映射
            
            Node(
                package='nav2_controller',           # 控制器包
                executable='controller_server',      # 控制器服务器可执行文件
                output='screen',                     # 输出到屏幕
                respawn=use_respawn,                 # 崩溃重启设置
                respawn_delay=2.0,                   # 重启延迟时间
                parameters=[configured_params],      # 参数配置
                arguments=['--ros-args', '--log-level', log_level],  # 日志级别
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav'),  ('odom', '/odometry/filtered') ]),  # 话题重映射
            # 如果有速度平滑器velocity_smoother的话，需要将cmd_vel_nav重映射到cmd_vel，
            # velocity_smoother接收cmd_vel话题，输出cmd_vel话题，
            # 执行机构最终接收cmd_vel话题
            
            Node(
                package='nav2_smoother',             # 平滑器包
                executable='smoother_server',        # 平滑服务器可执行文件
                name='smoother_server',              # 节点名称
                output='screen',                     # 输出到屏幕
                respawn=use_respawn,                 # 崩溃重启设置
                respawn_delay=2.0,                   # 重启延迟时间
                parameters=[configured_params],      # 参数配置
                arguments=['--ros-args', '--log-level', log_level],  # 日志级别
                remappings=remappings),              # 话题重映射

            
            Node(
                package='nav2_planner',              # 规划器包
                executable='planner_server',         # 规划服务器可执行文件
                name='planner_server',               # 节点名称
                output='screen',                     # 输出到屏幕
                respawn=use_respawn,                 # 崩溃重启设置
                respawn_delay=2.0,                   # 重启延迟时间
                parameters=[configured_params],      # 参数配置
                arguments=['--ros-args', '--log-level', log_level],  # 日志级别
                remappings=remappings),              # 话题重映射

            
            Node(
                package='nav2_behaviors',            # 行为包
                executable='behavior_server',        # 行为服务器可执行文件
                name='behavior_server',              # 节点名称
                output='screen',                     # 输出到屏幕
                respawn=use_respawn,                 # 崩溃重启设置
                respawn_delay=2.0,                   # 重启延迟时间
                parameters=[configured_params],      # 参数配置
                arguments=['--ros-args', '--log-level', log_level],  # 日志级别
                remappings=remappings),              # 话题重映射

            
            Node(
                package='nav2_bt_navigator',         # 行为树导航包
                executable='bt_navigator',           # BT导航器可执行文件
                name='bt_navigator',                 # 节点名称
                output='screen',                     # 输出到屏幕
                respawn=use_respawn,                 # 崩溃重启设置
                respawn_delay=2.0,                   # 重启延迟时间
                parameters=[configured_params],      # 参数配置
                arguments=['--ros-args', '--log-level', log_level],  # 日志级别
                remappings=remappings),              # 话题重映射

            
            Node(
                package='nav2_waypoint_follower',    # 航点跟随包
                executable='waypoint_follower',      # 航点跟随器可执行文件
                name='waypoint_follower',            # 节点名称
                output='screen',                     # 输出到屏幕
                respawn=use_respawn,                 # 崩溃重启设置
                respawn_delay=2.0,                   # 重启延迟时间
                parameters=[configured_params],      # 参数配置
                arguments=['--ros-args', '--log-level', log_level],  # 日志级别
                remappings=remappings),              # 话题重映射

            
            Node(
                package='nav2_velocity_smoother',     # 速度平滑器包
                executable='velocity_smoother',       # 速度平滑器可执行文件
                name='velocity_smoother',             # 节点名称
                output='screen',                      # 输出到屏幕
                respawn=use_respawn,                  # 崩溃重启设置
                respawn_delay=2.0,                    # 重启延迟2秒
                parameters=[configured_params],       # 加载配置参数
                arguments=['--ros-args', '--log-level', log_level],  # 设置日志级别
                # 话题重映射：
                # cmd_vel <- cmd_vel_nav: 订阅控制器的原始速度命令
                # cmd_vel_smoothed -> cmd_vel: 发布平滑后的速度命令给机器人
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),

            
            Node(
                package='nav2_lifecycle_manager',     # 生命周期管理器包
                executable='lifecycle_manager',       # 生命周期管理器可执行文件  
                name='lifecycle_manager_navigation',  # 导航生命周期管理器节点名
                output='screen',                      # 输出日志到屏幕
                arguments=['--ros-args', '--log-level', log_level],  # 设置日志级别
                parameters=[{'use_sim_time': use_sim_time},     # 仿真时间配置
                            {'autostart': autostart},           # 自动启动配置
                            {'node_names': lifecycle_nodes}]),  # 管理的节点列表
        ]
    )


    ld = LaunchDescription()

    
    ld.add_action(stdout_linebuf_envvar)

    
    ld.add_action(declare_namespace_cmd)        # 声明命名空间参数
    ld.add_action(declare_use_sim_time_cmd)     # 声明仿真时间参数
    ld.add_action(declare_params_file_cmd)      # 声明参数文件路径参数
    ld.add_action(declare_autostart_cmd)        # 声明自动启动参数
    ld.add_action(declare_use_composition_cmd)  # 声明组合模式参数
    ld.add_action(declare_container_name_cmd)   # 声明容器名称参数
    ld.add_action(declare_use_respawn_cmd)      # 声明重启模式参数
    ld.add_action(declare_log_level_cmd)        # 声明日志级别参数
    ld.add_action(declare_map_yaml_file_cmd)        # 声明日志级别参数
    
    
    ld.add_action(load_nodes)
    


    return ld
