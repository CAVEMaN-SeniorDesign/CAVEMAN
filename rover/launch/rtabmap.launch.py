import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # RealSense Camera Node
        launch_ros.actions.Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'align_depth': True,
                'enable_depth': True,
                'enable_color': True,
                'enable_infra1': True,
                'enable_infra2': True,
                'enable_gyro': False,
                'enable_accel': False,  # Disabling IMU
                'enable_pointcloud': True
            }],
            on_exit=launch.actions.EmitEvent(event=launch.events.Shutdown())
        ),

        # RTAB-Map Node
        launch_ros.actions.Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'depth_topic': '/camera/aligned_depth_to_color/image_raw',
                'rgb_topic': '/camera/color/image_raw',
                'camera_info_topic': '/camera/color/camera_info',
                'subscribe_odom': False,  # Disables odometry requirement
                'queue_size': 100,
                'approx_sync': True
            }],
            on_exit=launch.actions.EmitEvent(event=launch.events.Shutdown())
        ),

        # RTAB-Map Visualization
        launch_ros.actions.Node(
            package='rtabmap_ros',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen'
        )
    ])
