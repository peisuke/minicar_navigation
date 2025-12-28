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
    
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='turtlebot3_burger',
        choices=['minicar_diff', 'turtlebot3_burger', 'turtlebot3_waffle'],
        description='Robot model'
    )
    
    # Launch configurations
    teleop = LaunchConfiguration('teleop')
    robot = LaunchConfiguration('robot')
    
    # Keyboard teleoperation for TurtleBot3
    turtlebot3_keyboard_teleop = GroupAction(
        condition=IfCondition(PythonExpression(['"', teleop, '" == "keyboard" and ("', robot, '" == "turtlebot3_burger" or "', robot, '" == "turtlebot3_waffle")'])),
        actions=[
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_keyboard',
                output='screen',
                prefix='gnome-terminal --'
            )
        ]
    )
    
    # Keyboard teleoperation for minicar_diff
    minicar_keyboard_teleop = GroupAction(
        condition=IfCondition(PythonExpression(['"', teleop, '" == "keyboard" and "', robot, '" == "minicar_diff"'])),
        actions=[
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_keyboard',
                output='screen',
                prefix='gnome-terminal --',
                remappings=[
                    ('cmd_vel', 'diff_drive_controller/cmd_vel_unstamped')
                ],
            )
        ]
    )
    
    # Joystick teleoperation 
    joystick_teleop = GroupAction(
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('teleop'), '" == "joystick"'])),
        actions=[
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                parameters=[{
                    'axis_linear.x': 1,
                    'axis_angular.yaw': 0,
                    'scale_linear.x': 0.2,
                    'scale_angular.yaw': 1.0,
                    'enable_button': 0,
                    'require_enable_button': True,
                }],
                remappings=[('/cmd_vel', '/cmd_vel')]
            ),
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                parameters=[{
                    'dev': '/dev/input/js0',
                    'deadzone': 0.12
                }]
            )
        ]
    )
    
    return LaunchDescription([
        teleop_arg,
        robot_arg,
        turtlebot3_keyboard_teleop,
        minicar_keyboard_teleop,
        joystick_teleop,
    ])