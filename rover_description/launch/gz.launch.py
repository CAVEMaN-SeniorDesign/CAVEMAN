import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import ExecuteProcess



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='rover_description'

    rover = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rover.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'rover'],
                        output='screen')

    steering_ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["steering_controller"],
        output="screen"
    )

    wheel_ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["wheel_controller"],
        output="screen"
    )
    # Launch them all!
    return LaunchDescription([
        rover,
        gazebo,
        spawn_entity,
        steering_ctrl_spawner,
        wheel_ctrl_spawner
    ])
    
    '''
    Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'rover',
        '-topic', 'robot_description',
        '-x', '0', '-y', '0', '-z', '0.2'  # Set an initial spawn height
    ],
    output='screen'
)

    '''