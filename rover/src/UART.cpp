#include <iostream>
#include <cmath>
#include <fstream>
#include <SDL2/SDL.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#define L_JS_X 0
#define L_JS_Y 1
#define R_JS_X 2
#define R_JS_Y 3
#define L_TRIG 4
#define R_TRIG 5

#define A_BTN 0
#define B_BTN 1
#define X_BTN 3
#define Y_BTN 4
#define L_BUMP_BTN 6
#define R_BUMP_BTN 7
#define SQ_WIN_BTN 10
#define MENU_BTN 11
#define XBOX_BTN 12
#define L_JS_BTN 13
#define R_JS_BTN 14
#define UPLOAD_BTN 15


#define MAX_VELOCITY 4 // meters per second
#define MAX_TURN_RATE 4 // radians per second

#define MIN_SERVO_ANGLE 0 // 0 radians
#define MAX_SERVO_ANGLE M_PI // PI radians

const char port[] = "/dev/ttyUSB0";
const char version = 0x01;

enum packetID {
	
	ooga = 0x01;
	movement = 0x02;
	camera_movement = 0x03;
	lights = 0x04;
	mode = 0x05;
		
};


int main(int argc, char* argv[]){

//	fstream UART_port;
//	UART_port.open(port_name, ios::out);

	
	

	
		

	//int serial_port = open("../../devices70006040.serial/tty/ttyTHS1", O_RDWR | O_NONBLOCK);
	int serial_port = open(port, O_RDWR | O_NONBLOCK);
	
	if(serial_port < 0){
		std::cout << "Couldn't open file" << std::endl;
		return 1;
	}
	
	struct termios tty;
	
	if(tcgetattr(serial_port, &tty) != 0){
		//std::cout << "error from tcgetattr " << tcgetattr(serial_port, &tty) << std::endl;
		
		//printf("Error %i from tcgettattr: %s\n", errno, strerror(errno));
		
		return -1;
	}
	
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CRTSCTS;
	tty.c_cflag |= CREAD | CLOCAL;
	
	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO;
	tty.c_lflag &= ~ECHOE;
	tty.c_lflag &= ~ECHONL;
	tty.c_lflag &= ~ISIG;
	tty.c_lflag &= ~(IXON | IXOFF | IXANY);
	tty.c_lflag &= ~(IGNBRK | BRKINT| PARMRK| ISTRIP| INLCR| IGNCR| ICRNL);
	
	tty.c_oflag &= ~OPOST;
	tty.c_oflag &= ~ONLCR;
	
	tty.c_cc[VTIME] = 10;
	tty.c_cc[VTIME] = 0;
	
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);
	
	if(tcsetattr(serial_port, TCSANOW, &tty) != 0) {
		//std::cout << "Error from tcsetattr " << tcsetattr(serial_port, TCSANOW, &tty) << std::endl;
		
		//printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		
		return -1;
	}
	
	

	std::fstream UARTArduino;
	
	UARTArduino.open(port, std::ios::out);
	
	if(!UARTArduino.is_open()){
		std::cout << "Port not open" << std::endl;
		return -1;
	}
	
	
	

	if(SDL_Init(SDL_INIT_JOYSTICK) < 0){
		std::cout << "SDL could not be initialized" << std::endl;
	}else{
		std::cout << "SDL joystick is ready to go \n";
	}

	SDL_Joystick* jsPlayer = nullptr;

	if(SDL_NumJoysticks() < 1){
		std::cout << "No js avail" << std::endl;
	}else{
		std::cout << SDL_NumJoysticks() << " js avail" << std::endl;
		jsPlayer = SDL_JoystickOpen(0);
	}

	int deadzone = 8000;
	
	int xValLeftJS = 0;
	int yValLeftJS = 0;
	int trigger = 0;
	bool fwdDir = true;
	
	double motorDuty = 0;
	double angleRad = 0;
	
	bool recalcMotor;
	bool recalc
	
	int xValRightJS = 0;
	int yValRightJS = 0;
	
	int8_t encodeXRJS = 0;
	int8_t encodeYRJS = 0;
	
	uint8_t encodeRadius = 0;
	int8_t encodeDeg = 0;
	
	PacketID currID = PacketID::movement;

	while(true){
		SDL_Delay(20);
		SDL_Event event;
		while(SDL_PollEvent(&event)){
			if(event.type == SDL_JOYAXISMOTION && event.jaxis.which == 0){
					
					switch(event.jaxis.axis):
						case L_JS_X:
							xValLeftJS = event.jaxis.value;
							currID = PacketID::movement;
							break;
							
						case L_JS_Y:
							yValLeftJS = event.jaxis.value;
							currID = PacketID::movement;
							break;
							
						case R_JS_X:
							xValRightJS = event.jaxis.value;
							currID = PacketID::camera_movement;
							break;
							
						case R_JS_Y:
							yValRightJS = event.jaxis.value;
							currID = PacketID::camera_movement;
							break;
							
						case L_TRIG:
							trigger = event.jaxis.value;
							fwdDir = true;
							currID = PacketID::movement;
							break;
							
						case R_TRIG:
							trigger = event.jaxis.value;
							fwdDir = false;
							currID = PacketID::movement;
							break;
							
						default:
							break;
							
						
							
					
					if(event.jaxis.axis == 0){ //left JS
						//std::cout << "xAxis: " << event.jaxis.value << std::endl;
						xValLeftJS = event.jaxis.value;
					}
					if(event.jaxis.axis == 1){ //left JS
						//std::cout << "yAxis: " << event.jaxis.value << std::endl;
						yValLeftJS = -event.jaxis.value;
					}
					if(event.jaxis.axis == 2){ //right JS
						//std::cout << "xAxis of 2nd js: " << event.jaxis.value << std::endl;
						xValRightJS = event.jaxis.value;
					}
					if(event.jaxis.axis == 3){ //right JS
						//std::cout << "yAxis of 2nd: " << event.jaxis.value << std::endl;
						yValRightJS = -event.jaxis.value;
					}
					if(event.jaxis.axis == 4){ //right trigger
						//std::cout << "yAxis of 2nd: " << event.jaxis.value << std::endl;
						trigger = event.jaxis.value;
					}
					//if(event.jaxis.axis == 5){ //left trigger
						//std::cout << "yAxis of 2nd: " << event.jaxis.value << std::endl;
					//}
					
					//radius = sqrt(pow(xValLeftJS, 2) + pow(yValLeftJS, 2));
					//std::cout << "Radius = " << radius << std::endl;
					
					angleDeg = 180 / M_PI * atan2(yValLeftJS, xValLeftJS);
					//std::cout << "Angle in Degrees: " << angleDeg << std::endl;
					
					//if(radius < deadzone){
						//encodeRadius = 0;
					//}else if(radius > 32000){
						//encodeRadius = 255;
					//}else{
						//encodeRadius = 255*(radius-deadzone)/(32000-deadzone);
					//}
					
					encodeRadius = (trigger + 32768) * 255 / 65536;
					
					encodeDeg = 127*angleDeg/180;
					
					if(xValRightJS > 0){
						if(xValRightJS < deadzone){
							encodeXRJS = 0;
						}else if(xValRightJS > 32000){
							encodeXRJS = 127;
						}else{
							encodeXRJS = 127*(xValRightJS-deadzone)/(32000-deadzone);
						}
					}else{
						if(xValRightJS > -deadzone){
							encodeXRJS = 0;
						}else if(xValRightJS < -32000){
							encodeXRJS = -128;
						}else{
							encodeXRJS = 128*(xValRightJS+deadzone)/(32000-deadzone);
						}
					}
					
					if(yValRightJS > 0){
						if(yValRightJS < deadzone){
							encodeYRJS = 0;
						}else if(yValRightJS > 32000){
							encodeYRJS = 127;
						}else{
							encodeYRJS = 127*(yValRightJS-deadzone)/(32000-deadzone);
						}
					}else{
						if(yValRightJS > -deadzone){
							encodeYRJS = 0;
						}else if(yValRightJS < -32000){
							encodeYRJS = -128;
						}else{
							encodeYRJS = 128*(yValRightJS+deadzone)/(32000-deadzone);
						}
					}
					
					UARTArduino << "[" << (char)encodeRadius << (char)encodeDeg << (char)encodeXRJS << (char)encodeYRJS << "]" << std::endl; //test using '\n' for noticeable latency diffs
					std::cout << (int)encodeRadius << "," << (int)encodeDeg << "," << (int)encodeXRJS << "," << (int)encodeYRJS << std::endl;
					
					//UARTArduino << "[" << (char)encodeRadius << "]" << std::endl;
					
				}

				//std::cout << event.jaxis.which << " js moved\n";
			
		}
	}

//	SDL_QUIT();

	UARTArduino.close();
		
	close(serial_port);

	return 0;
}



	
	
	

	
	
	
	

