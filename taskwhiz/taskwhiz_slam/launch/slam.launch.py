from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnExecutionComplete
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    current_dir = FindPackageShare('taskwhiz_slam')
    world_dir = FindPackageShare('taskwhiz_worlds')
    slam_toolbox_dir = FindPackageShare('slam_toolbox')
    map_saver_dir = FindPackageShare('nav2_map_server')
    
    robot_name = LaunchConfiguration('robot_name')
    declare_robot_name_cmd = DeclareLaunchArgument('robot_name', 
                                                   default_value='turtlebot4', 
                                                   description='Specify name of the robot',
                                                   choices=['turtlebot4'])
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation clock if true')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument('rviz_config_file', default_value=PathJoinSubstitution([current_dir, 'rviz', 'config_map.rviz']), description='Path to rviz config file')

    save_map = LaunchConfiguration('save_map')
    declare_save_map_cmd = DeclareLaunchArgument('save_map', default_value='True', description='Whether to save the generated map to a file')

    map_url = LaunchConfiguration('map_url')
    declare_map_url_cmd = DeclareLaunchArgument('map_url',
                                                     default_value=PathJoinSubstitution([current_dir, 'maps', 'my_map']),
                                                     description='Location of the file to save map')

    tb4_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                world_dir,
                'launch',
                'tb4_world.launch.py'
            ])),
        launch_arguments={'use_sim_time': use_sim_time,
                          'rviz_config_file': rviz_config_file}.items(),
        condition=IfCondition(PythonExpression(["'", robot_name, "' == 'turtlebot4'"])),
    )
    
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                slam_toolbox_dir,
                'launch',
                'online_async_launch.py'
            ])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # map_saver_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             map_saver_dir,
    #             'launch',
    #             'map_saver_server'
    #         ])
    #     ),
    #     condition=IfCondition(save_map)
    # )

    map_saver_client = Node(
        package='taskwhiz_slam',
        executable='map_saver',
        name='map_saver',
        output='screen',
        parameters=[{
            'map_url': map_url
        }],
        condition=IfCondition(save_map)
    )

    return LaunchDescription([
        declare_robot_name_cmd,
        declare_use_sim_time_cmd,
        declare_save_map_cmd,
        declare_map_url_cmd,
        declare_rviz_config_file_cmd,
        tb4_world_launch,
        RegisterEventHandler(
            event_handler=OnExecutionComplete(
                target_action=tb4_world_launch,
                on_completion=[slam_toolbox_launch],
            )
        ),
        # slam_toolbox_launch,
        # map_saver_launch,
        # map_saver_client
    ])