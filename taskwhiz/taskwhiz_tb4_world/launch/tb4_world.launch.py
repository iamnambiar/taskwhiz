from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Locate directories for simulation, description, and bringup packages
    nav2_minimal_tb4_sim_package = FindPackageShare('nav2_minimal_tb4_sim')
    nav2_minimal_tb4_description_package = FindPackageShare('nav2_minimal_tb4_description')
    taskwhiz_tb4_world_package = FindPackageShare('taskwhiz_tb4_world')

    # Declare launch arguments for namespace, simulation time, RViz usage, and headless mode
    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation clock if true')

    # Define robot's initial pose in the simulation
    pose = {
        'x': LaunchConfiguration('x_pose', default='-9.00'),
        'y': LaunchConfiguration('y_pose', default='9.00'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }
    
    # Paths to robot URDF and world SDF files
    robot_urdf = PathJoinSubstitution([nav2_minimal_tb4_description_package, 'urdf', 'standard', 'turtlebot4.urdf.xacro'])
    world_sdf = PathJoinSubstitution([taskwhiz_tb4_world_package, 'worlds', 'maze.sdf'])

    # Remap tf topics for compatibility
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Launch the robot_state_publisher node to publish robot's state
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

    # Include Gazebo simulation launch file
    gazebo_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])),
        launch_arguments={'gz_args': ['-r ', # remove -s to load gazebo gui
                                      world_sdf],
                          'on_exit_shutdown': 'True'}.items(),
    )

    # Include robot spawn launch file
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_minimal_tb4_sim_package, 
                                  'launch', 
                                  'spawn_tb4.launch.py'])),
        launch_arguments={'namespace': namespace,
                          'use_sim_time': use_sim_time,
                          'robot_sdf': robot_urdf,
                          'x_pose': pose['x'],
                          'y_pose': pose['y'],
                          'yaw': pose['Y'],}.items())

    # Return the complete launch description
    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        start_robot_state_publisher_node,
        spawn_robot,
        gazebo_launcher
    ])