// #ifndef ROVER_4WD_CONTROLLER_HPP_
// #define ROVER_4WD_CONTROLLER_HPP_

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/float64.hpp"
// #include "std_msgs/msg/float64_multi_array.hpp"

// class roverCtrl : public rclcpp::Node
// {
//   public:
//     roverCtrl(const double timer_period = 1e-2, const double timeout_duration = 1e9);
//     ~roverCtrl();

//   private:
//     void cmdVelCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const;
//     rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_vel_sub_;
// };