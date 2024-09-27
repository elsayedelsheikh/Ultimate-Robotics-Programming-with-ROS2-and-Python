import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the default parameters YAML file
    usb_cam_params_file = os.path.join(
        get_package_share_directory('usb_cam_launch'),  # replace with your package name if different
        'config',
        'cam_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='usb_cam',  # The package where the usb_cam_node_exe resides
            executable='usb_cam_node_exe',  # The executable to run
            name='usb_camera',
            output='screen',
            parameters=[usb_cam_params_file],  # YAML file for camera parameters
            remappings=[
                ('/image_raw', '/camera/image_raw'),  # Remap image topic if necessary
                ('/camera_info', '/camera/camera_info'),  # Remap camera info topic if needed
            ]
        ),
    ])