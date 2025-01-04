import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    ld = LaunchDescription()

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot_simulation'), 'launch')
    pkg_turtlebot_simulation = get_package_share_directory('turtlebot_simulation')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    ign_gazebo_launch = PathJoinSubstitution([pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])
    model_file_path = PathJoinSubstitution([pkg_turtlebot_simulation, 'models', 'turtlebot3', 'waffle', 'model.sdf'])
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
        
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_turtlebot_simulation, 'worlds') + ':' + 
            os.path.join(pkg_turtlebot_simulation, 'models' )            
        ]
    )
    
    # Nodes
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[('ign_args', ['empty', '.sdf', ' -v 4'])]
    )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'turtlebot',
                   '-file', model_file_path,
                   '-x', '-2.0',
                   '-y', '-0.0',
                   '-z', '0.05'],
    )
    
    ignition_bridge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/ign_bridge.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    
    navigation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/nav.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )

    
    ignition_spawn_world = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-file', PathJoinSubstitution([
                        get_package_share_directory('turtlebot_simulation'),
                        "worlds", "model.sdf"]),
                    '-allow_renaming', 'false'],
        )
    
    ld.add_action( ign_resource_path )
    ld.add_action( ignition_gazebo )
    ld.add_action( ignition_spawn_world )
    ld.add_action( ignition_spawn_entity )
    ld.add_action( ignition_bridge )
    ld.add_action( navigation_launch )

    return ld