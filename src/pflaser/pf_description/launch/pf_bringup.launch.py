'''
Autor: wei.canming
Version: 1.0
Date: 2025-08-21 16:02:50
LastEditors: wei.canming
LastEditTime: 2025-08-22 13:42:41
Description: 
'''
from ament_index_python.packages import get_package_share_directory
import os
import launch_ros
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
import launch


def launch_setup(context, *args, **kwargs):
    package_name = 'pf_description'
    scanner_arg = LaunchConfiguration('scanner').perform(context)
    publish_tf_arg = LaunchConfiguration('publish_tf').perform(context)
    
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package=package_name).find(package_name)
    scanner_description_path = os.path.join(
        pkg_share, 'urdf', scanner_arg + '_world.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz', scanner_arg + '.rviz')

    launch_actions = []

    # 只有在publish_tf为true时才启动robot_state_publisher
    if publish_tf_arg.lower() == 'true':
        robot_state_publisher_node = launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(
                ['xacro ', scanner_description_path])}]
        )
        launch_actions.append(robot_state_publisher_node)

    rviz2_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d' + rviz_config_path],
    )
    launch_actions.append(rviz2_node)

    return launch_actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "scanner", default_value=TextSubstitution(text="r2000")),
        launch.actions.DeclareLaunchArgument(
            "publish_tf", default_value=TextSubstitution(text="false"),
            description="Whether to publish tf transforms via robot_state_publisher"),
        OpaqueFunction(function=launch_setup)
    ])
