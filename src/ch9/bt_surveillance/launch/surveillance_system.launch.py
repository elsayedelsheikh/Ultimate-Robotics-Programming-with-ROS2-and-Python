from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
import time

def generate_launch_description():

    conf_file_path = 'surveillance_system.yaml'

    file_conf = PathJoinSubstitution([
        get_package_share_directory('bt_surveillance'),	
        conf_file_path
    ])

    bt_surveillnace_node =    Node(
            package='bt_surveillance',
            executable='surveillance_system',
            name='surveillance_system',
            output='screen',
            parameters=[
                file_conf
            ]
        )
    
    ld = LaunchDescription()
    ld.add_action(bt_surveillnace_node)
    return ld