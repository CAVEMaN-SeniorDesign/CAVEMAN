from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_params = os.path.join(get_package_share_directory("rover"), 'config', 'nav2_params.yaml')

    Nav2 = Node(
        package='nav2_bringup',
        executable='bringup_launch.py',
        output='screen',
        parameters=[nav2_params],
        arguments=['--ros-args', '--params-file', nav2_params]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        Nav2
    ])
