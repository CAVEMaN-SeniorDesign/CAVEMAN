import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from four_wheel_steering_msgs.msg import FourWheelSteeringStamped

class CmdVelTo4WS(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_4ws')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_4ws',  # Receiving cmd_vel from teleop
            self.cmd_vel_callback,
            10
        )
        self.publisher = self.create_publisher(
            FourWheelSteeringStamped,
            '/cmd_4ws',  # Publishing converted command
            10
        )

    def cmd_vel_callback(self, msg):
        four_ws_cmd = FourWheelSteeringStamped()
        four_ws_cmd.header.stamp = self.get_clock().now().to_msg()
        four_ws_cmd.header.frame_id = 'base_link'

        # Convert cmd_vel to four-wheel steering format
        four_ws_cmd.data.front_steering_angle = msg.angular.z * 0.5  # Adjust scaling
        four_ws_cmd.data.rear_steering_angle = msg.angular.z * -0.2  # Adjust scaling
        four_ws_cmd.data.front_steering_angle_velocity = 0.5  # Arbitrary rate
        four_ws_cmd.data.rear_steering_angle_velocity = 0.3  # Arbitrary rate
        four_ws_cmd.data.speed = msg.linear.x
        four_ws_cmd.data.acceleration = 0.5  # Arbitrary acceleration
        four_ws_cmd.data.jerk = 0.1  # Arbitrary jerk

        self.publisher.publish(four_ws_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelTo4WS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
