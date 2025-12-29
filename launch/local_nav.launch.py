#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 制御パラメータ（動的変更の正本）
    params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.expanduser("~/controllers.yaml"),
        description="Path to params YAML for /local_nav_node (tuning params live).",
    )

    # 起動時に決める運用モード
    sim_ns = DeclareLaunchArgument("sim_ns", default_value="sim_robot")
    real_ns = DeclareLaunchArgument("real_ns", default_value="real_robot")
    input_sim = DeclareLaunchArgument("input_sim", default_value="true")
    input_real = DeclareLaunchArgument("input_real", default_value="false")
    output_sim = DeclareLaunchArgument("output_sim", default_value="true")
    output_real = DeclareLaunchArgument("output_real", default_value="false")
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")

    local_nav_node = Node(
        package="minicar_navigation",
        executable="local_nav",
        name="local_nav_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),  # 制御系（動的にいじる対象もここに含める）
            {
                # 起動時スイッチだけ上書き（ここは動的に変えない想定）
                "sim_ns": LaunchConfiguration("sim_ns"),
                "real_ns": LaunchConfiguration("real_ns"),
                "input_sim": LaunchConfiguration("input_sim"),
                "input_real": LaunchConfiguration("input_real"),
                "output_sim": LaunchConfiguration("output_sim"),
                "output_real": LaunchConfiguration("output_real"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    # Parameter server node for dynamic parameter management
    param_server_node = Node(
        package="minicar_navigation",
        executable="param_server_node.py",
        name="param_server_node",
        output="screen",
        parameters=[{
            "config_path": LaunchConfiguration("params_file"),
            "target_node": "/local_nav_node",
        }],
    )

    return LaunchDescription([
        params_file,
        sim_ns, real_ns,
        input_sim, input_real,
        output_sim, output_real,
        use_sim_time,
        local_nav_node,
        param_server_node,
    ])
