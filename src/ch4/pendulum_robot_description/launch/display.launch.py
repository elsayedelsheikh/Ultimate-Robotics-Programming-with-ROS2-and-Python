from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
import launch_ros.descriptions
def generate_launch_description():
    
    ld = LaunchDescription()
        
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen')
    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen')


    ld.add_action( joint_state_publisher_gui )
    ld.add_action( rviz2_node )

    return ld