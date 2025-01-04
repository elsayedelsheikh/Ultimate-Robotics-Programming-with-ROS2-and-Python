
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='burger',
                          choices=['burger', 'waffle'],
                          description='Turtlebot Model'),
]
def generate_launch_description():
    
    sdf = os.path.join( get_package_share_directory('turtlebot_simulation'), 'models', 'turtlebot3', 'waffle', 'model.sdf')
    doc = xacro.parse(open(sdf))
    xacro.process_doc(doc)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    
    doc = xacro.parse(open(sdf))
    xacro.process_doc(doc)
    
    
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot_simulation'),
            'maps',
            'turtlebot_world.yaml'))

    param_file_name = 'nav.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('turtlebot_simulation'),
            'params',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': doc.toxml()}]),

    ])