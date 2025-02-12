#include "cmd_vel_to_wheels.hpp"

// Manual clamp function to replace clamp_value (C++17)
double clamp_value(double value, double min_val, double max_val) {

    return std::max(min_val, std::min(value, max_val));
}


CmdVelToWheels::CmdVelToWheels() : Node("cmd_vel_to_wheels")
{
    // Declare and get parameters
    this->declare_parameter("wheel_base", 0.5);  // Distance between front and rear wheels
    this->declare_parameter("track_width", 0.4); // Distance between left and right wheels
    this->declare_parameter("max_steering_angle", 0.785); // Max steering (rad)
    this->declare_parameter("max_wheel_speed", 50.0); // Max wheel velocity (m/s)

    this->get_parameter("wheel_base", wheel_base_);
    this->get_parameter("track_width", track_width_);
    this->get_parameter("max_steering_angle", max_steering_angle_);
    this->get_parameter("max_wheel_speed", max_wheel_speed_);

    // ROS 2 Publishers
    steering_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/steering_controller/joint_trajectory", 10);
    wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_controller/commands", 10);

    // ROS 2 Subscriber
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&CmdVelToWheels::cmdVelCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "CmdVelToWheels node started!");
}

void CmdVelToWheels::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double v = msg->linear.x;  // Forward velocity
    double omega = msg->angular.z; // Angular velocity
    
    trajectory_msgs::msg::JointTrajectory steering_msg;
    steering_msg.joint_names = {"FL_Steer2Servo", "FR_Steer2Servo", "BL_Steer2Servo", "BR_Steer2Servo"};
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    if(omega!=0){
    // Compute steering angles using Ackermann steering model
    double front_steer_angle = atan((wheel_base_ * omega) / v);
    front_steer_angle = clamp_value(front_steer_angle, -max_steering_angle_, max_steering_angle_);
    point.positions = {front_steer_angle, front_steer_angle, -front_steer_angle, -front_steer_angle};
    point.time_from_start = rclcpp::Duration::from_seconds(0.1);
    steering_msg.points.push_back(point);
    steering_pub_->publish(steering_msg); // Publish Steering Commands
    }
    else{
        
        point.positions = {0, 0, -0, -0};
        steering_msg.points.push_back(point);
        steering_pub_->publish(steering_msg);
    }

    // Compute Wheel Velocities
    std_msgs::msg::Float64MultiArray wheel_msg;
    // To prevent the inability to steer when omega = 0
    if(omega==0){
        omega=1; 
    }
    double left_wheel_speed = v - (track_width_ / 2.0) * omega;
    double right_wheel_speed = v + (track_width_ / 2.0) * omega;

    left_wheel_speed = clamp_value(left_wheel_speed, -max_wheel_speed_, max_wheel_speed_);
    right_wheel_speed = clamp_value(right_wheel_speed, -max_wheel_speed_, max_wheel_speed_);

    wheel_msg.data = {left_wheel_speed, right_wheel_speed, left_wheel_speed, right_wheel_speed};
    wheel_pub_->publish(wheel_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelToWheels>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
