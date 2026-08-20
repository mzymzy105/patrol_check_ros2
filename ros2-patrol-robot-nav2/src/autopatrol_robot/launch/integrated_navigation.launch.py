import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取各包路径
    fishbot_description_dir = get_package_share_directory('fishbot_description')
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    autopatrol_robot_dir = get_package_share_directory('autopatrol_robot')

    default_model_path = os.path.join(
        fishbot_description_dir, 'urdf', 'fishbot', 'fishbot.urdf.xacro')
    default_gazebo_world_path = os.path.join(
        fishbot_description_dir, 'world', 'world.world')
    map_yaml_path = os.path.join(
        fishbot_navigation2_dir, 'maps', 'map.yaml')
    nav2_param_path = os.path.join(
        fishbot_navigation2_dir, 'config', 'nav2_params.yaml')
    rviz2_config_dir = os.path.join(
        fishbot_navigation2_dir, 'rviz', 'patrol_view.rviz')
    patrol_config_path = os.path.join(
        autopatrol_robot_dir, 'config', 'patrol_config.yaml')

    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')

    # 声明模型路径参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name="model", default_value=str(default_model_path),
        description="URDF的绝对路径"
    )

    # 获取xacro文件内容生成robot_description
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration("model")]),
        value_type=str
    )

    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}]
    )

    # 启动Gazebo
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'),
             '/launch', '/gazebo.launch.py']
        ),
        launch_arguments=[
            ('world', default_gazebo_world_path),
            ('verbose', 'true')
        ]
    )

    # 生成机器人实体（spawn 在轨迹中点 (-19.9, 77.9)，地图空闲区；原点 (0,0) 是障碍物）
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'fishbot',
                   '-x', '-19.9', '-y', '77.9', '-z', '0.01']
    )

    # 加载并激活控制器（用 spawner 替代 ros2 control CLI，无需 ros2controlcli 包）
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner',
             'fishbot_joint_state_broadcaster'],
        output='screen'
    )

    load_fishbot_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner',
             'fishbot_diff_drive_controller'],
        output='screen'
    )

    # 启动Navigation2 (使用自定义planner和controller，配置在nav2_params.yaml中)
    launch_navigation2 = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_dir, '/launch', '/bringup_launch.py']
        ),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path,
            'use_composition': 'True',
            'autostart': 'true'
        }.items(),
    )

    # 启动RViz2
    rviz2_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 自动巡逻节点
    patrol_node = launch_ros.actions.Node(
        package='autopatrol_robot',
        executable='patrol_node',
        name='patrol_node',
        parameters=[patrol_config_path, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 语音播报节点（本机缺 espeakng 语音库，先注释掉；patrol_node 会自动跳过语音）
    # speaker_node = launch_ros.actions.Node(
    #     package='autopatrol_robot',
    #     executable='speaker',
    #     name='speaker',
    #     output='screen'
    # )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,

        # 当加载机器人结束后执行控制器加载
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[load_joint_state_controller],
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_fishbot_diff_drive_controller],
            )
        ),

        # Navigation2（延迟 15 秒启动，等 Gazebo 加载大地图 world 完成，
        # 避免 CPU 竞争导致 map_server 的 bond 心跳超时）+ RViz2
        launch.actions.TimerAction(
            period=15.0,
            actions=[launch_navigation2],
        ),
        rviz2_node,

        # 自动巡逻
        patrol_node,
        # speaker_node,
    ])