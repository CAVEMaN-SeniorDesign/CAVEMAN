#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "four_wheel_steering_msgs/msg/four_wheel_steering_stamped.hpp"

class CmdVelTo4WS : public rclcpp::Node {
public:
    CmdVelTo4WS() : Node("cmd_vel_to_4ws") {
        // Subscribe to /cmd_vel
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&CmdVelTo4WS::cmd_vel_callback, this, std::placeholders::_1));

        // Publisher for /cmd_4ws
        cmd_4ws_publisher_ = this->create_publisher<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>(
            "/cmd_4ws", 10);

        RCLCPP_INFO(this->get_logger(), "cmd_vel_to_4ws node started!");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Publisher<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>::SharedPtr cmd_4ws_publisher_;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Create a new /cmd_4ws message
        auto cmd_4ws_msg = four_wheel_steering_msgs::msg::FourWheelSteeringStamped();
        
        rclcpp::Time current_time = this->get_clock()->now();

        if (current_time.seconds() > 1e9) {  // If timestamp is too large, fallback to 0
            RCLCPP_WARN(this->get_logger(), "Invalid sim time detected! Using fallback.");
            cmd_4ws_msg.header.stamp.sec = 0;
            cmd_4ws_msg.header.stamp.nanosec = 0;
        } else {
            cmd_4ws_msg.header.stamp = current_time;
        }

        cmd_4ws_msg.header.frame_id = "base_link";

        // Convert velocity to four-wheel steering
        double wheelbase = 1.5;  // Adjust this based on your robot's actual wheelbase

        cmd_4ws_msg.data.speed = msg->linear.x;
        cmd_4ws_msg.data.front_steering_angle = static_cast<float>(msg->angular.z * wheelbase);
        cmd_4ws_msg.data.rear_steering_angle = static_cast<float>(-msg->angular.z * wheelbase * 0.5);


        // Set velocities and dynamics
        cmd_4ws_msg.data.front_steering_angle_velocity = 0.5;
        cmd_4ws_msg.data.rear_steering_angle_velocity = 0.3;
        cmd_4ws_msg.data.acceleration = 0.5;
        cmd_4ws_msg.data.jerk = 0.1;

        // Publish the transformed command
        cmd_4ws_publisher_->publish(cmd_4ws_msg);

        RCLCPP_INFO(this->get_logger(), "Published /cmd_4ws: Speed %.2f, Front Angle %.2f, Rear Angle %.2f",
                    cmd_4ws_msg.data.speed, cmd_4ws_msg.data.front_steering_angle, cmd_4ws_msg.data.rear_steering_angle);
    }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelTo4WS>());
  rclcpp::shutdown();
  return 0;
}


/*
/*##########################################
|  Message format for commanding /cmd_vel  |
############################################
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -1.0
*/


/*#################################################################
|  Message format for commanding four_ws_steer at topic /cmd_4ws  |
###################################################################

four_wheel_steering_msgs/msg/FourWheelSteeringStamped "{
   header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'base_link' },
   data: {
     front_steering_angle: 0.3,
     rear_steering_angle: -0.15,
     front_steering_angle_velocity: 0.2,
     rear_steering_angle_velocity: 0.1,
     speed: 1.0,
     acceleration: 0.5,
     jerk: 0.1
   }
 }"
*/


/*
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "four_wheel_steering_msgs/msg/four_wheel_steering_stamped.hpp"

using std::placeholders::_1;

class CmdVelTo4WS : public rclcpp::Node
{
  public:
    CmdVelTo4WS()
    : Node("cmd_vel_to_4ws")
    {
      // Subscriber to /cmd_vel_4ws (Twist message)
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_4ws", 10, std::bind(&CmdVelTo4WS::topic_callback, this, _1));

      // Publisher to /cmd_4ws (FourWheelSteeringStamped message)
      publisher_ = this->create_publisher<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>("/cmd_4ws", 10);
    }

  private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      // Convert cmd_vel (Twist) to cmd_4ws (FourWheelSteeringStamped)
      auto four_ws_cmd = four_wheel_steering_msgs::msg::FourWheelSteeringStamped();
      four_ws_cmd.header.stamp = this->get_clock()->now();
      four_ws_cmd.header.frame_id = "base_link";

      // Convert linear velocity
      four_ws_cmd.data.speed = msg->linear.x;
      four_ws_cmd.data.acceleration = 0.5;  // Example constant acceleration
      four_ws_cmd.data.jerk = 0.1;          // Example constant jerk

      // Convert angular velocity to steering angles
      four_ws_cmd.data.front_steering_angle = msg->angular.z * 0.5;  // Scale factor
      four_ws_cmd.data.rear_steering_angle = msg->angular.z * -0.2; // Rear wheels turn opposite
      four_ws_cmd.data.front_steering_angle_velocity = 0.5; // Example rate
      four_ws_cmd.data.rear_steering_angle_velocity = 0.3;  // Example rate

      // Publish converted message
      publisher_->publish(four_ws_cmd);

      RCLCPP_INFO(this->get_logger(), "Converted cmd_vel -> cmd_4ws: speed=%.2f, front_steer=%.2f, rear_steer=%.2f",
                  msg->linear.x, four_ws_cmd.data.front_steering_angle, four_ws_cmd.data.rear_steering_angle);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>::SharedPtr publisher_;
};

// Main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelTo4WS>());
  rclcpp::shutdown();
  return 0;
}

*/