from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()
    
    input_topic = "/cmd_vel"
    output_topic = "/pose"
    
    sim_node = Node(
       package='turtlesim',
       executable='turtlesim_node',
       name='sim',
       remappings=[
           ('/turtle1/cmd_vel', input_topic),
           ('/turtle1/pose', output_topic),
       ]
    )
    
    ld.add_action( sim_node )

    
    return ld
