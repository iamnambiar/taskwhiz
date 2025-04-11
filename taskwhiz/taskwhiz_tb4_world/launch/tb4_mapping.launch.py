import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation clock')

    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz_cmd = DeclareLaunchArgument('use_rviz', default_value='True', description='if true, open rviz window')

    slam_param_file = LaunchConfiguration('slam_param_file')
    declare_slam_param_file = DeclareLaunchArgument('slam_param_file', 
                                                    default_value=os.path.join(get_package_share_directory('taskwhiz_tb4_world'),'configs','tb4_mapper_params.yaml'),
                                                    description="Full path to the ROS2 parameters file to use for the slam_toolbox node")
    
    map_url = LaunchConfiguration('map_url')
    declare_map_url = DeclareLaunchArgument('map_url',
                                            default_value=os.path.join(get_package_share_directory('taskwhiz_tb4_world'),'maps','map'),
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

    map_saver_server_node = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        emulate_tty='True',
    )

    map_saver_client_node = Node(
        package='taskwhiz_utils',
        executable='map_saver_client',
        name='taskwhiz_map_saver_client_node',
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
            {'node_names': ['slam_toolbox', 'map_saver', 'taskwhiz_map_saver_client_node']},
        ],
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('taskwhiz_tb4_world'), 'rviz', 'config_map.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_use_rviz_cmd,
        declare_slam_param_file,
        declare_map_url,
        start_async_lifecycle_node,
        map_saver_server_node,
        map_saver_client_node,
        start_lifecycle_manager,
        rviz_node,
    ])
