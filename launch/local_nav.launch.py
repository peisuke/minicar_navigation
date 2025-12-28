#!/usr/bin/env python3
"""
Local Navigation Launch File

YAMLファイルからパラメータを読み込んで local_nav_node を起動
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パッケージのパス
    pkg_share = get_package_share_directory('minicar_navigation')
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'controllers.yaml'),
        description='Path to the controller configuration file'
    )
    
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='',
        description='Override controller type (optional, uses YAML if empty)'
    )
    
    # Local navigation node
    local_nav_node = Node(
        package='minicar_navigation',
        executable='local_nav',
        name='local_nav_node',
        parameters=[
            LaunchConfiguration('config_file'),
            # オプション: コマンドラインからcontroller_typeを上書き
            {
                'controller_type_override': LaunchConfiguration('controller_type')
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        controller_type_arg,
        local_nav_node,
    ])