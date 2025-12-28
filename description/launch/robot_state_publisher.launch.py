#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Process xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare('minicar_navigation'),
        'description', 'urdf',
        'minicar_diff_gazebo.xacro'
    ])
    
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    
    return LaunchDescription([
        use_sim_time_arg,
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }]
        ),
        
        # Joint state broadcaster spawner
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),
        
        # Diff drive controller spawner
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen'
        )
    ])