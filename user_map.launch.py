#!/usr/bin/env python3
"""在用户地图 world 里启动两轮扫地机小车（headless gazebo + 30m 激光雷达）"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-19.9')
    y_pose = LaunchConfiguration('y_pose', default='77.86')
    # 用脚本自身位置推导路径，避免硬编码机器路径
    base_dir = os.path.dirname(os.path.abspath(__file__))
    world = os.path.join(base_dir, 'nav2_world', 'world.world')
    model_sdf = os.path.join(base_dir, 'nav2_world', 'robot_sweeper', 'model.sdf')
    urdf_path = os.path.join(base_dir, 'nav2_world', 'robot_sweeper', 'robot_sweeper.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
    )

    spawn_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'sweeper', '-file', model_sdf,
                   '-x', x_pose, '-y', y_pose, '-z', '0.01'],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='-19.9'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='77.86'))
    ld.add_action(gzserver_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_cmd)
    return ld
