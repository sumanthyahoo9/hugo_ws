import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Get package directory
    pkg_share = FindPackageShare('hugo_robot').find('hugo_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'hugo_arm.urdf.xacro')
    
    # Process xacro to URDF
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Joint State Publisher (non-GUI, publishes zero positions)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher
    ])