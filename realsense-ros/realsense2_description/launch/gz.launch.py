from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    realsense2_description_pkg = FindPackageShare('realsense2_description').find('realsense2_description')
    model_path = realsense2_description_pkg + '/urdf/d435.urdf.xacro'

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'd435',
                '-file', Command(['xacro ', model_path])
            ],
            output='screen'
        )
    ])
