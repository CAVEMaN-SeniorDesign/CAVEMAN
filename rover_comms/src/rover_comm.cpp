#include "rover_comm.hpp"
#include <fstream>
#include <iostream>

// Implementation of the virtual destructor for ListenerCallbacks
cave_talk::ListenerCallbacks::~ListenerCallbacks() = default;

// RoverCommsListener implementation
RoverCommsListener::RoverCommsListener(rclcpp::Node* node)
{   
    rover_comm_node = node;
}

void RoverCommsListener::HearOogaBooga(const cave_talk::Say ooga_booga)
{
    RCLCPP_INFO(rover_comm_node->get_logger(), "Ooga Booga");
}

void RoverCommsListener::HearMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate)
{
    RCLCPP_INFO(rover_comm_node->get_logger(), "wagu!?");
}

void RoverCommsListener::HearCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt)
{
    RCLCPP_INFO(rover_comm_node->get_logger(), "ooeoeoe");
}

void RoverCommsListener::HearLights(const bool headlights)
{
    RCLCPP_INFO(rover_comm_node->get_logger(), "ahaha");
}

void RoverCommsListener::HearMode(const bool manual)
{
    RCLCPP_INFO(rover_comm_node->get_logger(), "wokawoka");
}

// RoverComm implementation
std::string RoverComm::gameControllerType(){
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

RoverComm::RoverComm() : Node("rover_comm")
{
    // Create joy subscription
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&RoverComm::joyCallback, this, std::placeholders::_1));

    // Create serial publisher
    serial_read_pub_ = this->create_publisher<rover_interfaces::msg::Serial>(
        "/serial_read/data", 10);

    // Check for connected game controllers
    std::string type = this->gameControllerType();
    
    RCLCPP_INFO(this->get_logger(), "rover_comms up!");
}


void RoverComm::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{   
    if (!port_open) {
        return;
    }

    double v = 0.0;
    double omega = 0.0;
    if(this->game_controller_type_=="xbox"){
        // invert the values
        double r_trig = -msg->axes[5];
        double l_trig = -msg->axes[2];
        omega = msg->axes[0]; // Angular velocity on joy 0
    }
    else{
        double l_joy = msg->axes[1];
        omega = msg->axes[2]; //powerA Steering with right joy
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverComm>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}