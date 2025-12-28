#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    teleop_arg = DeclareLaunchArgument(
        'teleop',
        default_value='keyboard',
        choices=['keyboard', 'joystick'],
        description='Teleoperation method (keyboard, joystick)'
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
    
    # Launch configurations
    teleop = LaunchConfiguration('teleop')
    sim_ns = LaunchConfiguration('sim_ns')
    real_ns = LaunchConfiguration('real_ns')
    output_sim = LaunchConfiguration('output_sim')
    output_real = LaunchConfiguration('output_real')
    
    teleop_nodes = []
    
    # Simulation robot teleop
    teleop_nodes.append(
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard_sim',
            output='screen',
            prefix='gnome-terminal --',
            remappings=[
                ('cmd_vel', ['/', sim_ns, '/diff_drive_controller/cmd_vel_unstamped'])
            ],
            condition=IfCondition(PythonExpression(['"', teleop, '" == "keyboard" and "', output_sim, '" == "true"']))
        )
    )
    
    # Real robot teleop
    teleop_nodes.append(
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard_real',
            output='screen',
            prefix='gnome-terminal --',
            remappings=[
                ('cmd_vel', ['/', real_ns, '/diff_drive_controller/cmd_vel_unstamped'])
            ],
            condition=IfCondition(PythonExpression(['"', teleop, '" == "keyboard" and "', output_real, '" == "true"']))
        )
    )
    
    # Joystick teleoperation for simulation
    teleop_nodes.extend([
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_sim',
            parameters=[{
                'axis_linear.x': 1,
                'axis_angular.yaw': 0,
                'scale_linear.x': 0.2,
                'scale_angular.yaw': 1.0,
                'enable_button': 0,
                'require_enable_button': True,
            }],
            remappings=[
                ('cmd_vel', ['/', sim_ns, '/diff_drive_controller/cmd_vel_unstamped'])
            ],
            condition=IfCondition(PythonExpression(['"', teleop, '" == "joystick" and "', output_sim, '" == "true"']))
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_real',
            parameters=[{
                'axis_linear.x': 1,
                'axis_angular.yaw': 0,
                'scale_linear.x': 0.2,
                'scale_angular.yaw': 1.0,
                'enable_button': 0,
                'require_enable_button': True,
            }],
            remappings=[
                ('cmd_vel', ['/', real_ns, '/diff_drive_controller/cmd_vel_unstamped'])
            ],
            condition=IfCondition(PythonExpression(['"', teleop, '" == "joystick" and "', output_real, '" == "true"']))
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.12
            }],
            condition=IfCondition(PythonExpression(['"', teleop, '" == "joystick"']))
        )
    ])
    
    return LaunchDescription([
        teleop_arg,
        sim_ns_arg,
        real_ns_arg,
        output_sim_arg,
        output_real_arg,
    ] + teleop_nodes)