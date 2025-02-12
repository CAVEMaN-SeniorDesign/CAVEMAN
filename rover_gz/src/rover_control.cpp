/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Irstea
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *********************************************************************/

#include <cmath>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <four_wheel_steering_msgs/msg/four_wheel_steering_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "controller_interface/controller_interface.hpp"

namespace four_wheel_steering_controller
{

class FourWheelSteeringController : public controller_interface::ControllerInterface
{
public:
  FourWheelSteeringController()
    : controller_interface::ControllerInterface(),
      publish_rate_(50.0),
      cmd_vel_timeout_(0.5),
      base_frame_id_("base_link"),
      enable_odom_tf_(true)
  {}

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*state*/) override
  {
    // Declare parameters
    publish_rate_ = get_node()->declare_parameter("publish_rate", 50.0);
    cmd_vel_timeout_ = get_node()->declare_parameter("cmd_vel_timeout", 0.5);
    base_frame_id_ = get_node()->declare_parameter("base_frame_id", "base_link");
    enable_odom_tf_ = get_node()->declare_parameter("enable_odom_tf", true);

    // Create publishers and subscribers
    odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&FourWheelSteeringController::cmdVelCallback, this, std::placeholders::_1));
    
    cmd_4ws_sub_ = get_node()->create_subscription<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>(
      "cmd_four_wheel_steering", 10, std::bind(&FourWheelSteeringController::cmdFourWheelSteeringCallback, this, std::placeholders::_1));

    // Timer for publishing odometry
    odom_timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
      std::bind(&FourWheelSteeringController::updateOdometry, this));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    last_cmd_vel_ = *msg;
    last_cmd_time_ = get_node()->now();
  }

  void cmdFourWheelSteeringCallback(const four_wheel_steering_msgs::msg::FourWheelSteeringStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    last_cmd_4ws_ = *msg;
    last_cmd_time_ = get_node()->now();
  }

  void updateOdometry()
  {
    if ((get_node()->now() - last_cmd_time_).seconds() > cmd_vel_timeout_) {
      last_cmd_vel_ = geometry_msgs::msg::Twist();
      last_cmd_4ws_ = four_wheel_steering_msgs::msg::FourWheelSteeringStamped();
    }

    // Compute odometry (simplified)
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = get_node()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = base_frame_id_;

    odom_msg.twist.twist.linear.x = last_cmd_vel_.linear.x;
    odom_msg.twist.twist.angular.z = last_cmd_vel_.angular.z;

    odom_pub_->publish(odom_msg);

    // Publish transform if enabled
    if (enable_odom_tf_) {
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped.header.stamp = odom_msg.header.stamp;
      transformStamped.header.frame_id = "odom";
      transformStamped.child_frame_id = base_frame_id_;
      transformStamped.transform.translation.x = 0.0;
      transformStamped.transform.translation.y = 0.0;
      transformStamped.transform.translation.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, last_cmd_vel_.angular.z);
      transformStamped.transform.rotation = tf2::toMsg(q);

      tf_broadcaster_->sendTransform(transformStamped);
    }
  }

private:
  // Node parameters
  double publish_rate_;
  double cmd_vel_timeout_;
  std::string base_frame_id_;
  bool enable_odom_tf_;

  // ROS 2 interfaces
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>::SharedPtr cmd_4ws_sub_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Command storage
  geometry_msgs::msg::Twist last_cmd_vel_;
  four_wheel_steering_msgs::msg::FourWheelSteeringStamped last_cmd_4ws_;
  rclcpp::Time last_cmd_time_;
  std::mutex cmd_mutex_;
};

}  // namespace four_wheel_steering_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(four_wheel_steering_controller::FourWheelSteeringController, controller_interface::ControllerInterface)
