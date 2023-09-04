/** *************************** teleopKey.cpp ***************************
 *
 * This code converts the keyboard inputs from user into different functions
 *
 * *********************************************************************/

#include <iostream>
#include <chrono>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <anafi_autonomy/msg/keyboard_command.hpp>

#define KEYCODE_a 97
#define KEYCODE_d 100
#define KEYCODE_w 119
#define KEYCODE_s 115
#define KEYCODE_h 104
#define KEYCODE_r 114
#define KEYCODE_t 116
#define KEYCODE_l 108
#define KEYCODE_b 98
#define KEYCODE_c 99
#define KEYCODE_H 72
#define KEYCODE_R 82
#define KEYCODE_L 76

#define KEYCODE_LEFT   1792836
#define KEYCODE_RIGHT  1792835
#define KEYCODE_UP     1792833
#define KEYCODE_DOWN   1792834
#define KEYCODE_Esc    27
#define KEYCODE_Insert 458961534
#define KEYCODE_SPACE  32

#define KEYCODE_F1  1789776
#define KEYCODE_F2  1789777
#define KEYCODE_F4  1789779
#define KEYCODE_F5  117494068606
#define KEYCODE_F6  117494069118
#define KEYCODE_F7  117494069374
#define KEYCODE_F8  117494069630
#define KEYCODE_F9  117494132862
#define KEYCODE_F10 117494133118
#define KEYCODE_F11 117494133630
#define KEYCODE_F12 117494133886

#define KEYCODE_ACCENT 96

#define KEYCODE_0     48
#define KEYCODE_2     50
#define KEYCODE_4     52
#define KEYCODE_5     53
#define KEYCODE_6     54
#define KEYCODE_7     55
#define KEYCODE_8     56
#define KEYCODE_9     57
#define KEYCODE_MINUS 45
#define KEYCODE_PLUS  43
#define KEYCODE_Enter 10
#define KEYCODE_DOT   46
#define KEYCODE_SLASH 47

using namespace std;
using namespace std::chrono_literals;

int kfd = 0;
unsigned long int key = 0;
struct termios cooked, raw;

class Teleop : public rclcpp::Node{
	public:
		// Constructor
		Teleop() : Node("keyboard_teleop"){
			RCLCPP_INFO(this->get_logger(), "Keyboard is running...");
			
			cout <<	"Press: \n"
				"Insert \t - arm \n"
				"t \t - take-off \n"
				"l \t - land \n"
				"Esc \t - cut motors \n"
				"UP \t - move forward \n"
				"DOWN \t - move backward \n"
				"LEFT \t - move left \n"
				"RIGHT \t - move right \n"
				"w \t - move up \n"
				"s \t - move down \n"
				"a \t - yaw clockwise \n"
				"d \t - yaw counterclockwise \n"
				"b \t - return to home \n"
				"SPACE \t - halt \n"
				"r \t - reset pose \n";
		
			command_publisher = this->create_publisher<anafi_autonomy::msg::KeyboardCommand>("keyboard/command", rclcpp::SystemDefaultsQoS());
						
			timer = this->create_wall_timer(70ms, std::bind(&Teleop::timer_callback, this));
		}
		
	private:		
		// Timer
		rclcpp::TimerBase::SharedPtr timer;
	
		// Publishers
		rclcpp::Publisher<anafi_autonomy::msg::KeyboardCommand>::SharedPtr command_publisher;

		// Callback
		void timer_callback(){
			anafi_autonomy::msg::KeyboardCommand command_msg;
			command_msg.header.stamp = this->get_clock()->now();
			command_msg.header.frame_id = "body";
		
			if(key != 0)
				RCLCPP_DEBUG_STREAM(this->get_logger(), "key = " << key);

			switch(key){

			/* UAV commands */
			case KEYCODE_Insert: // arm
				command_msg.drone_action = 1;			   
				break;
			case KEYCODE_t: // take-off
				command_msg.drone_action = 2;
				break;
			case KEYCODE_SPACE: // halt
				command_msg.drone_action = 3;
				break;
			case KEYCODE_L: // land
			case KEYCODE_l:
				command_msg.drone_action = 4;
				break;
			case KEYCODE_Esc: // disarm!
				command_msg.drone_action = 5;
				break;
			case KEYCODE_R: // reset pose
			case KEYCODE_r:
				command_msg.drone_action = 6;
				break;
			case KEYCODE_b: // return-to-home
				command_msg.drone_action = 7;
				break;
			case KEYCODE_F1: // remote control!
				command_msg.drone_action = 101;
				break;
			case KEYCODE_F2: // offboard control!
				command_msg.drone_action = 102;
				break;
			case KEYCODE_F4: // reboot!
				command_msg.drone_action = 110;
				break;
			case KEYCODE_F5: // start mission
				command_msg.drone_action = 11;
				break;
			case KEYCODE_F6: // pause mission
				command_msg.drone_action = 12;
				break;
			case KEYCODE_F7: // stop mission
				command_msg.drone_action = 13;
				break;
			case KEYCODE_F9: // initialize VIO
				command_msg.drone_action = 21;
				break;
			case KEYCODE_F11: // calibrate magnetometer!
				command_msg.drone_action = 111;
				break;

			/* UAV movements */
			case KEYCODE_a:
				command_msg.drone_yaw = 1;
				break;
			case KEYCODE_d:
				command_msg.drone_yaw = -1;
				break;
			case KEYCODE_w:
				command_msg.drone_z = 1;
				break;
			case KEYCODE_s:
				command_msg.drone_z = -1;
				break;
			case KEYCODE_RIGHT: // move right
				command_msg.drone_y = -1;
				break;
			case KEYCODE_LEFT: // move left
				command_msg.drone_y = 1;
				break;
			case KEYCODE_UP: // move forward
				command_msg.drone_x = 1;
				break;
			case KEYCODE_DOWN: // move backward
				command_msg.drone_x = -1;
				break;

			/* gimbal commadns */
			case KEYCODE_F12: // calibrate gimbal
				command_msg.camera_action = 111;
				break;
			case KEYCODE_5: // reset to (0, 0)
				command_msg.camera_action = 11;
				break;
			case KEYCODE_7: // pitch left
				command_msg.gimbal_roll = -1;
				break;
			case KEYCODE_9: // pitch right
				command_msg.gimbal_roll = 1;
				break;
			case KEYCODE_8: // roll up
				command_msg.gimbal_pitch = -1;
				break;
			case KEYCODE_2: // roll down
				command_msg.gimbal_pitch = 1;
				break;
			case KEYCODE_4: // yaw left
				command_msg.gimbal_yaw = -1;
				break;
			case KEYCODE_6: // yaw right
				command_msg.gimbal_yaw = 1;
				break;

			/* camera commands */
			case KEYCODE_PLUS: // zoom in
				command_msg.zoom = 1;
				break;
			case KEYCODE_MINUS: // zoom out
				command_msg.zoom = -1;
				break;
			case KEYCODE_Enter: // take picture
				command_msg.camera_action = 1;
				break;
			case KEYCODE_0: // start recording
				command_msg.camera_action = 2;
				break;
			case KEYCODE_DOT: // stop recording
				command_msg.camera_action = 3;
				break;
			case KEYCODE_SLASH: // download media
				command_msg.camera_action = 4;
				break;
			}

			command_publisher->publish(command_msg);

			key = 0;
		}
};

void quit(__attribute__((unused)) int sig){
	tcsetattr(kfd, TCSANOW, &cooked);
}

// This function is called from a thread
void *readKey(void *) {
	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	tcsetattr(kfd, TCSANOW, &raw);

	char c = 0;
	while(true){
		auto t_start = std::chrono::high_resolution_clock::now();
		read(kfd, &c, 1); // get the next event from the keyboard
		auto t_end = std::chrono::high_resolution_clock::now();
		double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
		if(elapsed_time_ms < 1)
			key = (key<<8) + c;
		else
			key = c;
	}
}

int main(int argc, char** argv){
	pthread_t t;
	
	pthread_create(&t, NULL, readKey, NULL); // launch a thread
	signal(SIGINT, quit);
	rclcpp::init(argc, argv);
	
	rclcpp::spin(std::make_shared<Teleop>());
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Keyboard is stopping...");
	tcsetattr(kfd, TCSANOW, &cooked);
	rclcpp::shutdown();
	return 0;
}
