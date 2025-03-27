#ifndef CMD_VEL_TO_JOINTS_HPP
#define CMD_VEL_TO_JOINTS_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class CmdVelToJoints : public rclcpp::Node
{
public:
    CmdVelToJoints();

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void publish_joint_states();

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double wheel_speed_;
    double steering_angle_;
};

#endif // CMD_VEL_TO_JOINTS_HPP
