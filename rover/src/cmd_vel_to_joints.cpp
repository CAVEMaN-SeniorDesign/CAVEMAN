#include "cmd_vel_to_joints.hpp"

// Super basic, no PID control or anything, just instantaneously commands to a position.

CmdVelToJoints::CmdVelToJoints() : Node("cmd_vel_to_joints"), wheel_speed_(0.0), steering_angle_(0.0)
{
    // Subscriber to listen to /cmd_vel
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&CmdVelToJoints::cmd_vel_callback, this, std::placeholders::_1));

    // Publisher for /joint_states
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Timer to periodically publish joint states
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&CmdVelToJoints::publish_joint_states, this));
}

void CmdVelToJoints::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    wheel_speed_ = msg->linear.x;      // Forward/backward motion
    steering_angle_ = msg->angular.z;  // Turning motion
}

void CmdVelToJoints::publish_joint_states()
{
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->get_clock()->now();

    msg.name = {
        "front_left_steering_joint",
        "front_right_steering_joint",
        "rear_left_steering_joint",
        "rear_right_steering_joint",
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint"
    };

    // Simulate steering movement
    msg.position = {
        steering_angle_, steering_angle_, // Front steering
        -steering_angle_, -steering_angle_, // Rear steering (if applicable)
        wheel_speed_, wheel_speed_, // Left and right wheel speeds
        wheel_speed_, wheel_speed_  // Rear wheels
    };

    joint_state_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing wheel and steering joint states.");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToJoints>());
    rclcpp::shutdown();
    return 0;
}
