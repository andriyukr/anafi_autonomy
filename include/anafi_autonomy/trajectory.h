#include <iostream>
#include <fstream>
#include <chrono>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <anafi_autonomy/msg/reference_command.hpp>
#include <anafi_autonomy/msg/velocity_command.hpp>

#define COMMAND_NONE 		0
#define COMMAND_POSITION 	1
#define COMMAND_VELOCITY 	2
#define COMMAND_ANGLE 		3
#define COMMAND_RATE 		4

using namespace std;
using namespace rclcpp;
using namespace Eigen;

class Trajectory : public rclcpp::Node{
	public:
		// Constructor
		Trajectory();
		
	private:		
		// Timer
		rclcpp::TimerBase::SharedPtr timer;
		
		// Callback
		OnSetParametersCallbackHandle::SharedPtr callback;
		
		// Publishers
		rclcpp::Publisher<anafi_autonomy::msg::ReferenceCommand>::SharedPtr reference_publisher;
		rclcpp::Publisher<anafi_autonomy::msg::VelocityCommand>::SharedPtr derivative_publisher;
		

		// Variables
		Vector4d reference = Vector4d::Zero();
		Vector4d derivative = Vector4d::Zero();
		Vector3d mode = Vector3d::Zero();
		Vector4d pose_d = Vector4d::Zero();
		int trajectory_type = 0;
		
		// Messages
		anafi_autonomy::msg::ReferenceCommand reference_msg;
		anafi_autonomy::msg::VelocityCommand derivative_msg;
		
		
		// Callback
		void timer_callback();
		rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter> &parameters);
};
