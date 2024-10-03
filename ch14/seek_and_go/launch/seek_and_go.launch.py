import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    param_file_path = os.path.join(
        get_package_share_directory('seek_and_go'),
        'params.yaml'
    )

    seek_node = Node(
        package='seek_and_go',  # Replace with your package name
        executable='seek_and_go',  # Replace with your executable name
        name='seek_and_go',
        output='screen',
        parameters=[param_file_path]  # Load the YAML file
    )
    
    return LaunchDescription([
        seek_node
    ])
