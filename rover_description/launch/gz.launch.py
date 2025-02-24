import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import ExecuteProcess



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    #inspection
    world_file_path = os.path.expanduser('~/GazeboWorlds/gazebo_models_worlds_collection/worlds/office_cpr.world')

    package_name='rover_description'
    
    rover = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rover.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    #Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path, 'extra_gazebo_args': '--verbose'}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rover'
        ],
        output='screen'
    )

    drive_command_translator = Node(
        package="rover",
        executable="joy_to_drive",
        arguments=[],
        output="screen"
    )
    
    launch_teleop_joy = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('rover'), 'launch', 'joy.py')]),
             )
    
    drive_ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["drive_controller", "--controller-manager-timeout", "60"],
        output="screen"
    )
    

    return LaunchDescription([
        rover, 
        #TimerAction(period=2.0, actions=[realsense]),
        TimerAction(period=2.0, actions=[gazebo]), 
        TimerAction(period=3.0, actions=[spawn_entity]),  
        #TimerAction(period=9.0, actions=[spawn_realsense]), 
        #TimerAction(period=12.0, actions=[launch_teleop_joy]), 
        #TimerAction(period=15.0, actions=[drive_ctrl_spawner]),  
        #TimerAction(period=19.0, actions=[drive_command_translator])
    ])
    
    