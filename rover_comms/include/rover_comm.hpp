#ifndef ROVER_COMM_HPP
#define ROVER_COMM_HPP

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rover_interfaces/msg/serial.hpp"
#include <cmath>
#include <algorithm>

#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

#include "cave_talk.h"
#include "jetson_cave_talk.h"



class RoverCommsListener : public cave_talk::ListenerCallbacks
{
public:
    RoverCommsListener(rclcpp::Node* node); //possibly need "rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr" instead
    void HearOogaBooga(const cave_talk::Say ooga_booga) override;
    void HearMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate) override;
    void HearCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt) override;
    void HearLights(const bool headlights) override;
    void HearMode(const bool manual) override;

private:
    rclcpp::Node* rover_comm_node;
};

/*
cave_talk::ListenerCallbacks::~ListenerCallbacks() = default;

class MockListenerCallbacks : public cave_talk::ListenerCallbacks
{
    public:
        MOCK_METHOD(void, HearOogaBooga, (const cave_talk::Say), (override));
        MOCK_METHOD(void, HearMovement, ((const CaveTalk_MetersPerSecond_t), (const CaveTalk_RadiansPerSecond_t)), (override));
        MOCK_METHOD(void, HearCameraMovement, ((const CaveTalk_Radian_t), (const CaveTalk_Radian_t)), (override));
        MOCK_METHOD(void, HearLights, (const bool), (override));
        MOCK_METHOD(void, HearMode, (const bool), (override));
};
*/


class RoverComm : public rclcpp::Node
{
public:
    RoverComm();

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    std::string gameControllerType();

    // sub for /cmd_vel_joy topics and publish to joystick topic
    rclcpp::Publisher<rover_interfaces::msg::Serial>::SharedPtr serial_read_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    
    // Params
    std::string game_controller_type_;
};

#endif // ROVER_COMM_HPP