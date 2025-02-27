#include "rover_comm.hpp"

static const std::size_t kMaxMessageLength = 255U;
static RingBuffer<uint8_t, kMaxMessageLength> ring_buffer;


bool waiting_booga = true;

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Exiting..." << std::endl;
    waiting_booga = false;
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

    CaveTalk_Error_t init_error = cave_talk::init();

    if (init_error != CAVE_TALK_ERROR_NONE){
        RCLCPP_INFO(this->get_logger(), "UART init error encountered, exiting.");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "rover_comms up!");

    while(waiting_booga){
        RCLCPP_INFO(this->get_logger(), "Sent Ooga, awaiting Booga");
        talker->SpeakOogaBooga(cave_talk::SAY_OOGA);
        sleep(1);
    }
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&RoverComm::listen_callback, this)
    );
}

void RoverComm::listen_callback() {
    if (listener) {
        RCLCPP_INFO(this->get_logger(), "Listening...");
        listener->Listen();
    }
    else{
        RCLCPP_INFO(this->get_logger(), "Waiting for listener to be passed...");
    }
}


void RoverComm::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{   
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


cave_talk::ListenerCallbacks::~ListenerCallbacks() = default;

// ListenerCallbacks implementation
RoverCommsListener::RoverCommsListener(std::shared_ptr<RoverComm> node):rover_comm_node_(node) // assigns to rover_comm_node_
{   

}

void RoverCommsListener::HearOogaBooga(const cave_talk::Say ooga_booga)
{
    std::string output  = "Heard ";

    if (ooga_booga==cave_talk::SAY_OOGA){
        (rover_comm_node_->talker)->SpeakOogaBooga(cave_talk::SAY_BOOGA);
        output = output + "OOGA, said BOOGA";
    }
    else if (ooga_booga==cave_talk::SAY_BOOGA){
        // (rover_comm_node_->talker)->SpeakOogaBooga(cave_talk::SAY_OOGA);
        waiting_booga = false;
        output = output + "Heard Booga, loop closed";
    }

    RCLCPP_INFO(rover_comm_node_->get_logger(), output);
}

void RoverCommsListener::HearMovement(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate)
{
    RCLCPP_INFO(rover_comm_node_->get_logger(), "wagu!?");
}

void RoverCommsListener::HearCameraMovement(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt)
{
    RCLCPP_INFO(rover_comm_node_->get_logger(), "maka-keega");
}

void RoverCommsListener::HearLights(const bool headlights)
{
    RCLCPP_INFO(rover_comm_node_->get_logger(), "unk!");
}

void RoverCommsListener::HearMode(const bool manual)
{
    RCLCPP_INFO(rover_comm_node_->get_logger(), "oosha");
}

int SerialTest(void){
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NONE;

    error = cave_talk::init();

    size_t bytes_available = 0;
    size_t bytes_received = 0;
    char recent_receive;

    while(true){

        error = cave_talk::available(&bytes_available);

         if(bytes_available > 0){
             //std::cout << bytes_available << std::endl;
             error = cave_talk::receive(&recent_receive, 1, &bytes_received);
             std::cout << recent_receive << std::endl;
         }

     }

    return 0;
}



int main(int argc, char **argv) {
    std::signal(SIGINT, signalHandler);


    rclcpp::init(argc, argv);
    auto rover_node = std::make_shared<RoverComm>();
    std::shared_ptr<RoverCommsListener> listenCallbacks = std::make_shared<RoverCommsListener>(rover_node);
    
    rover_node->talker = std::make_shared<cave_talk::Talker>(cave_talk::send);
    rover_node->listener = std::make_shared<cave_talk::Listener>(cave_talk::receive, listenCallbacks);
    
    
    rclcpp::spin(rover_node);
    rclcpp::shutdown();
    return 0;
}
