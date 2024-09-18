from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generates a launch description for the pendulum state publisher and TF nodes.

    This function creates a launch description that includes the robot state publisher,
    the pendulum state publisher and TF node, and the RViz2 node. The robot state publisher
    node is responsible for publishing the state of the robot to the ROS2 network. The
    pendulum state publisher and TF node publishes the state of the pendulum and its
    transforms to the ROS2 network. The RViz2 node is used for visualizing the robot
    and its environment.

    Returns:
        LaunchDescription: The launch description containing the nodes.
    """
    ld = LaunchDescription()

    xacro_path = 'urdf/pendulum_robot.xacro'

    robot_description = PathJoinSubstitution([
        get_package_share_directory('pendulum_robot_description'),	
        xacro_path
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description':Command(['xacro ', robot_description])
        }]
    )

    robost_state_and_tf_node = Node(
        package='pendulum_state',
        executable='pendulum_state',
        name='state_publisher_and_tf',
        output='screen'
    )


    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen')

    ld.add_action( robot_state_publisher_node )
    ld.add_action( robost_state_and_tf_node )
    ld.add_action( rviz2_node )

    return ld