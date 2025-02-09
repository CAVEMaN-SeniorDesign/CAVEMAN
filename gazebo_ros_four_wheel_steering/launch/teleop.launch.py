import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo_launch_path = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )

    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_path]),
        ),

        # Spawn 4WS Vehicle in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'four_wheel_steering_vehicle',
                '-file', os.path.join(
                    get_package_share_directory('gazebo_ros_four_wheel_steering'),
                    'models', 'four_wheel_steering_vehicle', 'model.sdf'
                )
            ],
            output='screen'
        ),

        # Start teleop_twist_keyboard for manual control
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',  # Open in a separate terminal window
            parameters=[{'use_sim_time': True}],
            remappings=[('/cmd_vel', '/cmd_vel_4ws')]  # Rename topic to match conversion node
        ),

        # cmd_vel to cmd_4ws Converter Node
        Node(
            package='four_ws_converter', 
            executable='cmd_vel_to_4ws', 
            name='cmd_vel_to_4ws',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('/cmd_vel', '/cmd_vel_4ws'),
                ('/cmd_4ws', '/cmd_4ws')
            ]
        ),
    ])
