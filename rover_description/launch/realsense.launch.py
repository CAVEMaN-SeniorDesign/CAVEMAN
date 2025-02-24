import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    package_description = "gazebo_sensor_ros2"
    
    # Path to realsense.urdf
    urdf_file = os.path.join(get_package_share_directory(package_description), 'urdf', 'realsense.urdf')

    # Robot State Publisher for RealSense
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_realsense',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', urdf_file])}],
        output="screen"
    )
    
    # Joint State Publisher for RealSense
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_realsense',
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node
    ])
