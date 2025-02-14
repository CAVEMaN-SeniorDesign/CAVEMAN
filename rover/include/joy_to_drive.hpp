#ifndef JOY_TO_DRIVE_HPP
#define JOY_TO_DRIVE_HPP

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <algorithm>

class JoyToDrive : public rclcpp::Node
{
public:
    JoyToDrive();
private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    double clamp_value(double value, double min_val, double max_val);
    std::string gameControllerType();

    // sub for /cmd_vel_joy topics and publish to joystick topic
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr controller_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Params
    double wheel_base_, track_width_, max_steering_angle_, max_wheel_speed_;
    std::string game_controller_type_;
};

#endif