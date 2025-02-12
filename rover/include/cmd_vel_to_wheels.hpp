#ifndef CMD_VEL_TO_WHEELS_HPP
#define CMD_VEL_TO_WHEELS_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class CmdVelToWheels : public rclcpp::Node
{
public:
    CmdVelToWheels();

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;

    // Parameters
    double wheel_base_;
    double track_width_;
    double max_steering_angle_;
    double max_wheel_speed_;
};

#endif // CMD_VEL_TO_WHEELS_HPP
