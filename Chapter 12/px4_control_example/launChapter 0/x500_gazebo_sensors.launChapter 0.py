from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import sys
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

def validate_arguments(context, *args, **kwargs):
    # Get the value of the 'my_string' launch argument
    sensor_model_value = context.launch_configurations.get('sensor_model', None)
    
    # Check if the argument has been provided
    if not sensor_model_value:
        print("Error: 'sensor' argument is required but not provided.")
        sys.exit(1)  # Exit the launch process with an error code
    else:
        print(f"Argument 'my_string' provided: {sensor_model_value}")


def generate_launch_description():
    # Declare a launch argument
    declared_arg = DeclareLaunchArgument(
        'sensor_model', 
        description='This string defines the sensor equipped on the UAV'
    )

    validate_arg_action = OpaqueFunction(function=validate_arguments)
    sensor_model = LaunchConfiguration('sensor_model')

    camera_bridge_node = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
			name = 'color_camera_bridge', output='screen',
			arguments = [
				'/camera' + '@sensor_msgs/msg/Image' + '[ignition.msgs.Image'
			],
            condition=IfCondition(PythonExpression(["'", sensor_model, "' == 'camera'"]))
    )
    depth_sensor_bridge_node =  Node(package='ros_gz_bridge', executable='parameter_bridge',
			name = 'depth_camera_bridge',
			output='screen',
			parameters=[{
				'use_sim_time': True
			}],
			arguments = [
				'/depth_camera' + '@sensor_msgs/msg/Image' + '[ignition.msgs.Image',
				'/depth_camera/points' + '@sensor_msgs/msg/PointCloud2' + '[ignition.msgs.PointCloudPacked'
			],
			condition=IfCondition(PythonExpression(["'", sensor_model, "' == 'depth'"]))

            )
    return LaunchDescription([
        declared_arg, 
        camera_bridge_node,
        depth_sensor_bridge_node
    ])