# Requirements:
#   A realsense D400 series
#   Install realsense2 ros2 package (make sure you have this patch: https://github.com/IntelRealSense/realsense-ros/issues/2564#issuecomment-1336288238)
# Example:
#   $ ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
#
#   $ ros2 launch rtabmap_examples realsense_d400.launch.py
#   OR
#   $ ros2 launch rtabmap_launch rtabmap.launch.py frame_id:=camera_link args:="-d" rgb_topic:=/camera/color/image_raw depth_topic:=/camera/aligned_depth_to_color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=false


#Custom modified in effort to use D435 Stereo
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'camera_link',
          'subscribe_stereo':True,
          'subscribe_odom_info':True,
          'wait_imu_to_init':False,
          'approx_sync':True}]

    remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('left/image_rect', '/camera/infra1/image_rect_raw'),
            ('left/camera_info', '/camera/infra1/camera_info'),
            ('right/image_rect', '/camera/infra2/image_rect_raw'),
            ('right/camera_info', '/camera/infra2/camera_info')]

          #('rgb/camera_info', '/camera/color/camera_info'),
          #('depth/image', '/camera/aligned_depth_to_color/image_raw')

    return LaunchDescription([
        # Nodes to launch
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),
    ])
