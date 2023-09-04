// ros2 run anafi_autonomy example --ros-args -r __ns:=/anafi

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <anafi_autonomy/msg/reference_command.hpp>

using namespace std;
using namespace std::chrono_literals;
using namespace rclcpp;

class Example : public Node{
	public:
		// Constructor
		Example() : Node("anafi_autonomy_example"){
			RCLCPP_INFO(this->get_logger(), "Example is running...");

			// Subscribers
			state_subscriber = this->create_subscription<std_msgs::msg::String>("drone/state", SystemDefaultsQoS(), std::bind(&Example::stateCallback, this, placeholders::_1));
			gps_subscriber = this->create_subscription<sensor_msgs::msg::NavSatFix>("drone/gps/location", SensorDataQoS(), std::bind(&Example::gpsCallback, this, placeholders::_1));
	
			// Publishers
			action_publisher = this->create_publisher<std_msgs::msg::UInt8>("drone/action", SystemDefaultsQoS());
			command_publisher = this->create_publisher<anafi_autonomy::msg::ReferenceCommand>("drone/reference_command", SystemDefaultsQoS());
		
			// Timer
			timer = this->create_wall_timer(100ms, bind(&Example::timer_callback, this));
		}

	private:		
		// Subscribers
		Subscription<std_msgs::msg::String>::SharedPtr state_subscriber;
		Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber;

		// Publishers
		Publisher<std_msgs::msg::UInt8>::SharedPtr action_publisher;
		Publisher<anafi_autonomy::msg::ReferenceCommand>::SharedPtr command_publisher;

		// Timer
		TimerBase::SharedPtr timer;

		// Variables
		string state = "";
		Time time = Time(0, 0);
		Time initial_time = Time(0, 0);

		// Callback
		void timer_callback(){
			std_msgs::msg::UInt8 action_msg;
			anafi_autonomy::msg::ReferenceCommand command_msg;

			if(time < initial_time + Duration(1, 0))
				RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Waiting for take-off time...");
			else if(time < initial_time + Duration(2, 0) && state == "LANDED"){
				RCLCPP_INFO_ONCE(this->get_logger(), "Arming...");
				action_msg.data = 1; // defined in safeAnafi.h on line 83
				action_publisher->publish(action_msg);
				rclcpp::sleep_for(10ms);
				RCLCPP_INFO_ONCE(this->get_logger(), "Taking-off...");
				action_msg.data = 2; // defined in safeAnafi.h on line 83
				action_publisher->publish(action_msg);
			}
			else if(time < initial_time + Duration(5, 0)){
				RCLCPP_INFO_ONCE(this->get_logger(), "Aligning with north...");
				command_msg.heading_mode = 3; // angle control
				command_msg.yaw = 0.0; // 0deg
				command_publisher->publish(command_msg);
			}
			else if(time < initial_time + Duration(10, 0)){
				RCLCPP_INFO_ONCE(this->get_logger(), "Ascending to 2m...");
				command_msg.vertical_mode = 1; // position control
				command_msg.z = 2.0; // 2m
				command_publisher->publish(command_msg);
			}
			else if(time < initial_time + Duration(12, 0)){
				RCLCPP_INFO_ONCE(this->get_logger(), "Flying forward...");
				command_msg.horizontal_mode = 2; // velocity control
				command_msg.x = 1.0; // 1m/s
				command_publisher->publish(command_msg);
			}
			else if(time < initial_time + Duration(14, 0)){
				RCLCPP_INFO_ONCE(this->get_logger(), "Flying backwards...");
				command_msg.horizontal_mode = 2; // velocity control
				command_msg.x = -1.0; // -1m/s
				command_publisher->publish(command_msg);
			}
			else if(time < initial_time + Duration(15, 0)){
				RCLCPP_INFO_ONCE(this->get_logger(), "Howering...");
				command_msg.horizontal_mode = 2; // velocity control
				command_msg.x = 0.0; // 0m/s
				command_publisher->publish(command_msg);
			}
			else if(state != "LANDED" && state != "LANDING"){
				RCLCPP_INFO_ONCE(this->get_logger(), "Landing...");
				action_msg.data = 4; // defined in safeAnafi.h on line 83
				action_publisher->publish(action_msg);
			}
			else if(state == "LANDED"){
				RCLCPP_INFO_ONCE(this->get_logger(), "The mission is over");
				exit(0);
			}
		}

		// GPS subscriber callback
		void gpsCallback(const sensor_msgs::msg::NavSatFix& gps_msg){
			time = gps_msg.header.stamp;
			if(initial_time.seconds() == 0 && initial_time.nanoseconds() == 0)
				initial_time = time;
		}
	
		// Drone state subscriber callback
		void stateCallback(const std_msgs::msg::String& state_msg){
			state = state_msg.data; // 'LANDED', 'MOTOR_RAMPING', 'USER_TAKEOFF', 'TAKINGOFF', 'HOVERING', 'FLYING', 'LANDING', 'EMERGENCY'
		}
};

int main(int argc, char** argv){	
	init(argc, argv);
	
	spin(std::make_shared<Example>());
	
	RCLCPP_INFO(get_logger("rclcpp"), "Example is stopping...");

	shutdown();
	return 0;
}