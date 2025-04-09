
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, LogInfo
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation clock')

    slam_param_file = LaunchConfiguration('slam_param_file')
    declare_slam_param_file = DeclareLaunchArgument('slam_param_file', 
                                                    default_value=os.path.join(get_package_share_directory('taskwhiz_slam'),'config','mapper_params.yaml'),
                                                    description="Full path to the ROS2 parameters file to use for the slam_toolbox node")
    
    map_url = LaunchConfiguration('map_url')
    declare_map_url = DeclareLaunchArgument('map_url',
                                            default_value=os.path.join(get_package_share_directory('taskwhiz_slam'),'maps','maps'),
                                            description='Full path to save the map file.')

    start_async_lifecycle_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
        parameters=[
            slam_param_file,
            {
                'use_sim_time': use_sim_time,
                'use_lifecycle_manager': True,
            }
        ]
    )

    # configure_slam_event = EmitEvent(
    #     event=ChangeState(
    #         lifecycle_node_matcher=matches_action(start_async_lifecycle_node),
    #         transition_id=Transition.TRANSITION_CONFIGURE
    #     )
    # )

    # activate_slam_event = RegisterEventHandler(
    #     event_handler=OnStateTransition(
    #         target_lifecycle_node=start_async_lifecycle_node,
    #         start_state='configuring',
    #         goal_state='inactive',
    #         entities=[
    #             LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
    #             EmitEvent(event=ChangeState(
    #                 lifecycle_node_matcher=matches_action(start_async_lifecycle_node),
    #                 transition_id=Transition.TRANSITION_ACTIVATE
    #             ))
    #         ]
    #     )
    # )

    map_saver_server_node = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        emulate_tty='True',
    )

    map_saver_client_node = Node(
        package='taskwhiz_slam',
        executable='map_saver',
        name='taskwhiz_map_saver_node',
        output='screen',
        emulate_tty='True',
        parameters=[
            {'map_save_path': map_url }
        ]
    )

    start_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['slam_toolbox', 'map_saver', 'taskwhiz_map_saver_node']},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_slam_param_file,
        start_async_lifecycle_node,
        map_saver_server_node,
        map_saver_client_node,
        start_lifecycle_manager
    ])
    