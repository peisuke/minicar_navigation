#!/usr/bin/env python3
"""
Local Navigation Launch File

YAMLファイルからパラメータを読み込んで local_nav_node を起動
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def validate_and_create_nodes(context, *args, **kwargs):
    """Validate input arguments and create nodes"""
    # Get parameter values
    input_sim = context.launch_configurations.get('input_sim', 'true').lower() == 'true'
    input_real = context.launch_configurations.get('input_real', 'false').lower() == 'true'
    output_sim = context.launch_configurations.get('output_sim', 'true').lower() == 'true'
    output_real = context.launch_configurations.get('output_real', 'false').lower() == 'true'
    
    # Validation
    if not (input_sim or input_real):
        raise ValueError("ERROR: At least one of input_sim or input_real must be true")
    
    if input_sim and input_real:
        print("WARNING: Both input_sim and input_real are true. Data will be mixed.")
    
    if not (output_sim or output_real):
        print("WARNING: No output destinations specified. Node will run but not publish commands.")
    
    # Get other configurations
    config_file = context.launch_configurations.get('config_file')
    controller_type = context.launch_configurations.get('controller_type', '')
    sim_ns = context.launch_configurations.get('sim_ns', 'sim_robot')
    real_ns = context.launch_configurations.get('real_ns', 'real_robot')
    
    # Local navigation node
    local_nav_node = Node(
        package='minicar_navigation',
        executable='local_nav',
        name='local_nav_node',
        parameters=[
            config_file,
            {
                'controller_type_override': controller_type,
                'sim_ns': sim_ns,
                'real_ns': real_ns,
                'input_sim': input_sim,
                'input_real': input_real,
                'output_sim': output_sim,
                'output_real': output_real
            }
        ],
        output='screen'
    )
    
    return [local_nav_node]

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
    
    sim_ns_arg = DeclareLaunchArgument(
        'sim_ns',
        default_value='sim_robot',
        description='Simulation robot namespace'
    )
    
    real_ns_arg = DeclareLaunchArgument(
        'real_ns',
        default_value='real_robot',
        description='Real robot namespace'
    )
    
    input_sim_arg = DeclareLaunchArgument(
        'input_sim',
        default_value='true',
        description='Use simulation robot input data (scan, odom, etc.)'
    )
    
    input_real_arg = DeclareLaunchArgument(
        'input_real',
        default_value='false',
        description='Use real robot input data (scan, odom, etc.)'
    )
    
    output_sim_arg = DeclareLaunchArgument(
        'output_sim',
        default_value='true',
        description='Output control commands to simulation robot'
    )
    
    output_real_arg = DeclareLaunchArgument(
        'output_real',
        default_value='false',
        description='Output control commands to real robot'
    )
    
    return LaunchDescription([
        config_file_arg,
        controller_type_arg,
        sim_ns_arg,
        real_ns_arg,
        input_sim_arg,
        input_real_arg,
        output_sim_arg,
        output_real_arg,
        OpaqueFunction(function=validate_and_create_nodes),
    ])