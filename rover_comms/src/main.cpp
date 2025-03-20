#include "rover_comms_listener.hpp"


std::shared_ptr<RoverComm> global_rover_node = nullptr; // making global so that sigHandler can access

void signalHandler(int signum){
    std::cout << "\nInterrupt signal (" << signum << ") received. Exiting..." << std::endl;
    global_rover_node->looping = false;

    if(global_rover_node){
        global_rover_node->talker->SpeakOogaBooga(cave_talk::SAY_BOOGA); // close loop by sending BOOGA
        cave_talk::flush(); // flush buffer
        global_rover_node.reset();
        global_rover_node = nullptr;
    }

    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char **argv) {
    std::signal(SIGINT, signalHandler);

    rclcpp::init(argc, argv);
    auto rover_node = std::make_shared<RoverComm>();
    global_rover_node = rover_node;

    std::shared_ptr<RoverCommsListener> listenCallbacks = std::make_shared<RoverCommsListener>(rover_node);

    rover_node->talker = std::make_shared<cave_talk::Talker>(cave_talk::send);
    rover_node->listener = std::make_shared<cave_talk::Listener>(cave_talk::receive, listenCallbacks);


    rclcpp::spin(rover_node);
    //rclcpp::shutdown();
    return 0;
}