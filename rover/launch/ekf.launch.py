from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_params = os.path.join(get_package_share_directory("rover"), 'config', 'odom_ekf.yaml')

    EKF_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'),
        EKF_node
    ])
