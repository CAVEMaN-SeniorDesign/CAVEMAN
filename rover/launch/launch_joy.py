import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Joystick Node
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'deadzone': 0.05,   # Small deadzone to prevent accidental movement
                'autorepeat_rate': 1.0,  # Repeat rate when holding down buttons
                'coalesce_interval': 100 # Milliseconds for event coalescing
            }],
            output="screen",
            arguments=['--ros-args', '--log-level', 'info']
        ),

        # Teleop Twist Joy Node
        launch_ros.actions.Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'enable_button': -1,  # No enable button required
                'require_enable_button': False,  # Start without a required enable button

                # Assign joystick axes to movement
                'axis_linear.x': 1,   # Right joystick Y-axis (Forward/Backward)
                'axis_angular.yaw': 2,  # Left joystick X-axis (Steering)

                # Scaling for sensitivity
                'scale_linear.x': 10.0,  # Normal speed
                'scale_linear.x_turbo': 2.0,  # Increased speed when holding turbo button
                'scale_angular.yaw': 0.01,  # Normal turning speed
                'scale_angular.yaw_turbo': 2.0  # Faster turning when holding turbo button
            }],
            output="screen",
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])
