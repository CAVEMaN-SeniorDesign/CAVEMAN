#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/World.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace gazebo
{
  class RoverPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
      // ROS Node
      rclcpp::init(0, nullptr);
      node_ = std::make_shared<rclcpp::Node>("rover_plugin");

      // Retrieve joints
      front_left_wheel_ = model->GetJoint("fl_wheel_joint");
      front_right_wheel_ = model->GetJoint("fr_wheel_joint");
      rear_left_wheel_ = model->GetJoint("bl_wheel_joint");
      rear_right_wheel_ = model->GetJoint("br_wheel_joint");

      front_left_steer_ = model->GetJoint("fl_steer_joint");
      front_right_steer_ = model->GetJoint("fr_steer_joint");
      rear_left_steer_ = model->GetJoint("bl_steer_joint");
      rear_right_steer_ = model->GetJoint("br_steer_joint");

      if (!front_left_wheel_ || !front_right_wheel_ || !rear_left_wheel_ || !rear_right_wheel_) {
        RCLCPP_ERROR(node_->get_logger(), "One or more wheel joints not found!");
        return;
      }

      // Subscribe to velocity commands
      cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
          "/cmd_rover", 10, std::bind(&RoverPlugin::OnCmdVel, this, std::placeholders::_1));

      // Publish odometry
      odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/rover_odometry", 10);

      update_connection_ = event::Events::ConnectWorldUpdateBegin(
          std::bind(&RoverPlugin::OnUpdate, this));

      RCLCPP_INFO(node_->get_logger(), "Rover Plugin Loaded Successfully.");
    }

    void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;

        // Compute left and right wheel speeds
        double left_speed = linear_x - angular_z * wheel_separation_ / 2.0;
        double right_speed = linear_x + angular_z * wheel_separation_ / 2.0;

        // Set wheel velocity
        front_left_wheel_->SetParam("fmax", 0, 100000.0);  // Increase max force
        front_right_wheel_->SetParam("fmax", 0, 100000.0);
        rear_left_wheel_->SetParam("fmax", 0, 100000.0);
        rear_right_wheel_->SetParam("fmax", 0, 100000.0);

        front_left_wheel_->SetVelocity(0, left_speed);
        front_right_wheel_->SetVelocity(0, right_speed);
        rear_left_wheel_->SetVelocity(0, left_speed);
        rear_right_wheel_->SetVelocity(0, right_speed);
    }

    void OnUpdate()
    {
      auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
      odom_msg->header.stamp = node_->now();
      odom_msg->twist.twist.linear.x = (front_left_wheel_->GetVelocity(0) + front_right_wheel_->GetVelocity(0)) / 2;
      odom_msg->twist.twist.angular.z = (front_right_wheel_->GetVelocity(0) - front_left_wheel_->GetVelocity(0)) / wheel_separation_;
      odom_pub_->publish(*odom_msg);
    }

  private:
    physics::JointPtr front_left_wheel_, front_right_wheel_, rear_left_wheel_, rear_right_wheel_;
    physics::JointPtr front_left_steer_, front_right_steer_, rear_left_steer_, rear_right_steer_;

    double wheel_separation_ = 0.5; // Adjust this based on your rover's real specs
    double wheel_base_ = 0.6;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    event::ConnectionPtr update_connection_;
  };

  GZ_REGISTER_MODEL_PLUGIN(RoverPlugin)
}
