# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch realsense2_camera rs_launch.py enable_sync:=true
#
#   $ ros2 launch rtabmap_examples realsense_d435_color.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'camera_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':False,
          'wait_imu_to_init':False}]

    remappings=[
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/realigned_depth_to_color/image_raw')]

    return LaunchDescription([

        # Nodes to launch       
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
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
        
        # Because of this issue: https://github.com/IntelRealSense/realsense-ros/issues/2564
        # Generate point cloud from not aligned depth
        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[{'approx_sync':False}],
            remappings=[('depth/image',       '/camera/depth/image_rect_raw'),
                        ('depth/camera_info', '/camera/depth/camera_info'),
                        ('cloud',             '/camera/cloud_from_depth')]),
        
        # Generate aligned depth to color camera from the point cloud above       
        Node(
            package='rtabmap_util', executable='pointcloud_to_depthimage', output='screen',
            parameters=[{ 'decimation':2,
                          'fixed_frame_id':'camera_link',
                          'fill_holes_size':1}],
            remappings=[('camera_info', '/camera/color/camera_info'),
                        ('cloud',       '/camera/cloud_from_depth'),
                        ('image_raw',   '/camera/realigned_depth_to_color/image_raw')]),
        
    ])
