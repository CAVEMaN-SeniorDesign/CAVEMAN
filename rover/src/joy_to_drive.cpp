#include "joy_to_drive.hpp"
#include <fstream>
/*============================================================================================
                            _______               _____        
                            |__   __|             |  __ \       
                                | | ___    ______  | |  | | ___  
                                | |/ _ \  |______| | |  | |/ _ \ 
                                | | (_) |          | |__| | (_) |
                                |_|\___/           |_____/ \___/ 
_____________________________________________________________________________________________

  - Add better PID and natural control
  - Add state accumulation, current wheels are commanded to angle 0 when joy is released

==============================================================================================*/


// Facilitates setting bounds
double JoyToDrive::clamp_value(double value, double min_val, double max_val) {
    const double t = value < min_val ? min_val : value;
    return t > max_val ? max_val : t;
}

std::string JoyToDrive::gameControllerType(){
    std::ifstream file("/proc/bus/input/devices");
    std::string line;

    std::cout << "Checking connected game controllers...\n";

    // Looping through input device folders
    while (std::getline(file, line)) {
        // Search the line with "Name="
        if (line.find("Name=") != std::string::npos) {
            // If found line with "Controller" or "Gamepad" in it.
            bool xbox = line.find("Microsoft Xbox") != std::string::npos;
            bool powerA = line.find("PowerA NSW") != std::string::npos;
            if (xbox) {
                this->game_controller_type_ = "xbox";
                std::cout << game_controller_type_ << " controller detected" << std::endl;
                return std::string("xbox");
            }
            else if(powerA){
                this->game_controller_type_ = "powerA";
                std::cout << game_controller_type_ << " controller detected" << std::endl;
                return std::string("powerA"); //powerA has no analog triggers, will be default case
            }
        }
    }
    return "powerA"; //powerA has no analog triggers, will be default case
}

JoyToDrive::JoyToDrive() : Node("joy_to_drive")
{
    // Declare and get parameters
    this->declare_parameter("wheel_base", 0.5);  // Distance between front and rear wheels
    this->declare_parameter("track_width", 0.4); // Distance between left and right wheels
    this->declare_parameter("max_steering_angle", 0.523599); // Max steering (rad)
    this->declare_parameter("max_wheel_speed", 20.0); // Max wheel velocity (m/s)

    this->get_parameter("wheel_base", wheel_base_);
    this->get_parameter("track_width", track_width_);
    this->get_parameter("max_steering_angle", max_steering_angle_);
    this->get_parameter("max_wheel_speed", max_wheel_speed_);

    controller_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/drive_controller/joint_trajectory", 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoyToDrive::joyCallback, this, std::placeholders::_1));

    //Check if controller is connected: 
    std::string type = this->gameControllerType(); // unused var for now, as it sets private member var

    RCLCPP_INFO(this->get_logger(), "CmdVelToWheels node started!");
}

void JoyToDrive::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{

    // if(this->game_controller_type_=="xbox"){
    //     v = -v + 10; // if detected controller is xbox, commanded velocity should be shifted down so 0 at rest, and inverted.
    // }
    double v = 0.0;
    double omega = 0.0;
    if(this->game_controller_type_=="xbox"){
        double v_right = -msg->axes[5] + 10;  // Forward velocity, right trigger of xbox
        double v_left = -msg->axes[2] + 10;// Back velocity, left trigger of xbox
        v = (v_right - v_left)*max_wheel_speed_;
        omega = msg->axes[0]; // Angular velocity on joy 0
    }
    else{
        v = (msg->axes[1])*max_wheel_speed_; //powerA axis 1 for drive
        omega = msg->axes[2]; //powerA Steering with right joy

    }


    //MARK: VERI. STEER ANGs
    // Ackermann steer calcuations
    double front_steer_angle = omega*max_steering_angle_;//atan((wheel_base_ * omega) / (v + 1e-6)); // tiny val to avoid div-by-zero
    double rear_steer_angle = -front_steer_angle;
    
    // implement some offsets/scalings for xbox.
    // if(this->game_controller_type_ == "xbox"){
    //     return;
    // }

    front_steer_angle = this->clamp_value(front_steer_angle, -max_steering_angle_, max_steering_angle_);
    rear_steer_angle = this->clamp_value(rear_steer_angle, -max_steering_angle_, max_steering_angle_);
    
    //MARK: VERI. WHL SPDs
    // # Ackermann Wheel speeds
    double fl_wheel_speed = v - (track_width_ / 2.0) * omega;
    double fr_wheel_speed = v + (track_width_ / 2.0) * omega;
    double bl_wheel_speed = v - (track_width_ / 2.0) * omega;
    double br_wheel_speed = v + (track_width_ / 2.0) * omega;

    
    fl_wheel_speed = clamp_value(fl_wheel_speed, -max_wheel_speed_, max_wheel_speed_);
    fr_wheel_speed = clamp_value(fr_wheel_speed, -max_wheel_speed_, max_wheel_speed_);
    bl_wheel_speed = clamp_value(bl_wheel_speed, -max_wheel_speed_, max_wheel_speed_);
    br_wheel_speed = clamp_value(br_wheel_speed, -max_wheel_speed_, max_wheel_speed_);

    trajectory_msgs::msg::JointTrajectory joint_traj_msg;
    joint_traj_msg.joint_names = {
        "FL_Steer2Servo", "FR_Steer2Servo", "BL_Steer2Servo", "BR_Steer2Servo", // Steering servos
        "fl_wheel", "fr_wheel", "bl_wheel", "br_wheel" // Wheels
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;

    // Steering angles
    point.positions = {
        front_steer_angle, front_steer_angle, rear_steer_angle, rear_steer_angle, // positions for steer joints
        0.0, 0.0, 0.0, 0.0  // positions for wheels
    };

    // Wheel velocities
    point.velocities = {
        0.0, 0.0, 0.0, 0.0, // velocities for steer
        -fl_wheel_speed, fr_wheel_speed, -bl_wheel_speed, br_wheel_speed // velocities for wheels
    };

    point.time_from_start = rclcpp::Duration::from_seconds(0.1);
    joint_traj_msg.points.push_back(point);

    // Publish the command
    controller_pub_->publish(joint_traj_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyToDrive>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}