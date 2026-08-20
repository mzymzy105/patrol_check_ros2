import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    #获取默认路径
    urdf_package_path = get_package_share_directory("fishbot_description")
    default_model_path = urdf_package_path + '/urdf/fishbot/fishbot.urdf.xacro'
    default_gazebo_world_path = os.path.join(urdf_package_path,'world','custom_room.world')
    #为launch声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name="model",default_value=str(default_model_path),
        description="URDF的绝对路径"
    )
    #获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ',launch.substitutions.LaunchConfiguration("model")]), #urdf把xacro换成cat
            value_type = str
    )
    #状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package = 'robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description}]
    )
    #启动gazebo
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'),'/launch','/gazebo.launch.py']
        ),
        launch_arguments=[('world',default_gazebo_world_path),('verbose','true')]
    )

    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description','-entity','fishbot']
    )

    #加载并激活 fishbot_joint_state_broadcaster 控制器
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2','control','load_controller','--set-state','active','fishbot_joint_state_broadcaster'],
        output='screen'
    )

    #加载并激活 fishbot_effort_controller 控制器
    load_fishbot_effort_controller = launch.actions.ExecuteProcess(
        cmd=['ros2','control','load_controller','--set-state','active','fishbot_effort_controller'],
        output = 'screen'
    )

    #加载并激活 fishbot_diff_drive_controller 控制器
    load_fishbot_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2','control','load_controller','--set-state','active','fishbot_diff_drive_controller'],
        output = 'screen'
    )

    return launch.LaunchDescription(
        [
            action_declare_arg_mode_path,
            robot_state_publisher_node,
            launch_gazebo,
            spawn_entity_node,
            
            #事件动作，当加载机器人结束后执行
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action = spawn_entity_node,
                    on_exit =[load_joint_state_controller],
                )
            ),

            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action = load_joint_state_controller,
                    on_exit = [load_fishbot_diff_drive_controller],
                )
            )

            
        ]
    )