#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Launch args ---
    world = LaunchConfiguration("world")
    entity = LaunchConfiguration("entity")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    use_sim_time = LaunchConfiguration("use_sim_time")
    jsb_name = LaunchConfiguration("joint_state_broadcaster")
    diff_name = LaunchConfiguration("diff_controller")

    # --- Paths ---
    pkg_minicar = FindPackageShare("minicar_navigation")
    default_world = PathJoinSubstitution([pkg_minicar, "minicar_simulation", "worlds", "road_env.world"])
    
    # Gazebo model path setup
    current_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    minicar_model_path = PathJoinSubstitution([pkg_minicar, "minicar_simulation", "models"])
    model_path_env = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', 
        [minicar_model_path, ':', current_model_path] if current_model_path else minicar_model_path
    )

    gazebo_ros_share = FindPackageShare("gazebo_ros")
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"world": world}.items(),
    )

    # 1) robot_description publish（あなたの既存launchを再利用するのが安全）
    #    /robot_description が出てることは確認済みなので、これをそのまま include
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_minicar, "description", "launch", "robot_state_publisher.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # 2) Gazeboへスポーン（/robot_description を読む）
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-entity", entity,
            "-x", x, "-y", y, "-z", z,
            "-Y", yaw,
        ],
    )

    # 3) ros2_control コントローラ起動
    #    Gazebo + spawn の立ち上がりに少し時間が要るので遅延起動（安定重視）
    spawn_after_delay = TimerAction(period=3.0, actions=[spawn])

    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[jsb_name, "--controller-manager", "/controller_manager"],
    )

    spawn_diff = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[diff_name, "--controller-manager", "/controller_manager"],
    )

    controllers_after_delay = TimerAction(
        period=6.0,  # spawn後に立ち上がるまで待つ
        actions=[spawn_jsb, spawn_diff],
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("entity", default_value="minicar"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="1.5"),
        DeclareLaunchArgument("z", default_value="0.05"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        model_path_env,
        gazebo_launch,
        rsp_launch,
        spawn_after_delay,
    ])

