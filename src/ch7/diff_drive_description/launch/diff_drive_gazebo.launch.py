
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.descriptions
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def declare_args (context, *args, **kwargs): 
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value=''
    )

    return [robot_type_arg]

def launch_setup( context, *args, **kwargs):    
    robot_type_value = context.perform_substitution(LaunchConfiguration('robot_type'))

    if( robot_type_value == 'base' ):
        xacro_path = 'diff_drive.xacro'
    elif( robot_type_value == 'with_lidar' ):
        xacro_path = 'diff_drive_with_lidar.xacro'
    elif( robot_type_value == 'with_control' ):
        xacro_path = 'diff_drive_with_lidar_control.xacro'
    else:
        raise ValueError('Unknown robot type: ' + robot_type_value)


    robot_description = PathJoinSubstitution([
        get_package_share_directory('diff_drive_description'),	
        xacro_path
    ])
    
    diff_drive_description_package = launch_ros.substitutions.FindPackageShare(package='diff_drive_description').find('diff_drive_description')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',        
        parameters=[{
            'robot_description':Command(['xacro ', robot_description])
        }]
    )


    # Spawn
    spawn_node = Node(package='ros_gz_sim', executable='create',
                 arguments=[
                    '-name', 'diff_drive',
                    '-x', '0',
                    '-y', '0',
                    '-z', '0.1',
                    '-r', '0',
                    '-p', '0',
                    '-Y', '0',
                    '-topic', '/robot_description'],
                 output='screen')

    
    ignition_gazebo_node = IncludeLaunchDescription( PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
                                        launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])])
    
    # Bridge
    # https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge
    if( robot_type_value == 'with_lidar' ):
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
            output='screen'
        )
    elif ( robot_type_value == 'with_control' ):
       
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                    'cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/diff_drive/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
            output='screen'
        )

    if( robot_type_value != 'base' ):
        return [ robot_state_publisher_node, spawn_node, ignition_gazebo_node, bridge]
    else:
        return [ robot_state_publisher_node, spawn_node, ignition_gazebo_node]

def generate_launch_description():

    return LaunchDescription ( [OpaqueFunction(function=declare_args),
                              OpaqueFunction(function=launch_setup) ])


