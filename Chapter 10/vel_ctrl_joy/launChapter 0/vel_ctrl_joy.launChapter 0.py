from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='vel_ctrl_joy',
            executable='vel_ctrl',
            name='vel_ctrl',
            output='screen'
        )
    ])