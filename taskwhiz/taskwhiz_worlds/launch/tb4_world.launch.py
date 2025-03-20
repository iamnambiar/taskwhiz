
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    sim_directory = FindPackageShare('nav2_minimal_tb4_sim')
    description_dir = FindPackageShare('nav2_minimal_tb4_description')
    bringup_dir = FindPackageShare('taskwhiz_worlds')

    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation clock if true')

    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz_cmd = DeclareLaunchArgument('use_rviz', default_value='True', description='if true, open rviz window')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument('rviz_config_file', default_value=PathJoinSubstitution([bringup_dir, 'rviz', 'config.rviz']), description='Path to rviz config file')

    pose = {
        'x': LaunchConfiguration('x_pose', default='-9.00'),
        'y': LaunchConfiguration('y_pose', default='9.00'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }
    

    robot_urdf = PathJoinSubstitution([description_dir, 'urdf', 'standard', 'turtlebot4.urdf.xacro'])
    world_sdf = PathJoinSubstitution([bringup_dir, 'worlds', 'maze.sdf'])

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    start_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
             'robot_description': Command(['xacro ', robot_urdf])}
        ],
        remappings=remappings
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    gazebo_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])),
        launch_arguments={'gz_args': ['-r ', world_sdf],
                          'on_exit_shutdown': 'True'}.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim_directory, 
                                  'launch', 
                                  'spawn_tb4.launch.py'])),
        launch_arguments={'namespace': namespace,
                          'use_sim_time': use_sim_time,
                          'robot_sdf': robot_urdf,
                          'x_pose': pose['x'],
                          'y_pose': pose['y'],
                          'yaw': pose['Y'],}.items())

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_use_rviz_cmd,
        declare_rviz_config_file_cmd,
        start_robot_state_publisher_node,
        rviz_node,
        spawn_robot,
        gazebo_launcher
    ])