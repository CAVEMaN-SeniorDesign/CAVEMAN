#ifndef CMD_VEL_TO_WHEELS_HPP
#define CMD_VEL_TO_WHEELS_HPP

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <algorithm>

class CmdVelToWheels : public rclcpp::Node
{
public:
    CmdVelToWheels();
private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    double clamp_value(double value, double min_val, double max_val);
    std::string gameControllerType();

    // sub for /cmd_vel_joy topics and publish to joystick topic
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr controller_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // Params
    double wheel_base_, track_width_, max_steering_angle_, max_wheel_speed_;
    std::string game_controller_type_;
};

#endif