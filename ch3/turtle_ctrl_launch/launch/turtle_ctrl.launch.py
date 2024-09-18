from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()
    

    
    ctrl_node = Node(
       package='turtle_ctrl',
       executable='turtle_control_param',
       name='control_with_param',
       output='screen',
       parameters=[
            {"linear_velocity_scale": 0.1},
            {"angular_velocity_scale": 1.0}
       ],
       remappings=[
           ('/turtle1/cmd_vel', '/cmd_vel'),
       ]
    )
    
    included_launch_path = PathJoinSubstitution([
       get_package_share_directory('turtle_ctrl_launch'),	
       'turtle_sim.launch.py'
    ])
    print(" included_launch_path: ", get_package_share_directory('turtle_ctrl_launch'))
    
    ld.add_action (IncludeLaunchDescription(PythonLaunchDescriptionSource(included_launch_path)  ) )

   
    ld.add_action( ctrl_node )
    return ld
