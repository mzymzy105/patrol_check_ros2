import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    """
    一体化启动文件:
    1. 启动 Gazebo 仿真环境 + 加载 fishbot 机器人模型
    2. 启动 Navigation2 导航框架 (AMCL + A*全局规划器 + MPC局部控制器)
    3. 启动 RViz2 可视化
    4. 启动 patrol_node 自动巡逻 (从 patrol_config.yaml 读取初始点和目标点)
    """
    fishbot_description_dir = get_package_share_directory('fishbot_description')
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    autopatrol_robot_dir = get_package_share_directory('autopatrol_robot')

    default_model_path = os.path.join(
        fishbot_description_dir, 'urdf', 'fishbot', 'fishbot.urdf.xacro')
    default_gazebo_world_path = os.path.join(
        fishbot_description_dir, 'world', 'custom_room.world')
    map_yaml_path = os.path.join(
        fishbot_navigation2_dir, 'maps', 'room.yaml')
    nav2_param_path = os.path.join(
        fishbot_navigation2_dir, 'config', 'nav2_params.yaml')
    rviz2_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    patrol_config_path = os.path.join(
        autopatrol_robot_dir, 'config', 'patrol_config.yaml')

    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')

    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name="model", default_value=str(default_model_path),
        description="URDF的绝对路径"
    )

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration("model")]),
        value_type=str
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}]
    )

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

    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'fishbot']
    )

    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'fishbot_joint_state_broadcaster'],
        output='screen'
    )

    load_fishbot_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state',
             'active', 'fishbot_diff_drive_controller'],
        output='screen'
    )

    launch_navigation2 = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_dir, '/launch', '/bringup_launch.py']
        ),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path,
            'use_composition': 'False',
            'autostart': 'true'
        }.items(),
    )

    rviz2_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    patrol_node = launch_ros.actions.Node(
        package='autopatrol_robot',
        executable='patrol_node',
        name='patrol_node',
        parameters=[patrol_config_path, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,

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

        launch_navigation2,
        rviz2_node,
        patrol_node,
    ])