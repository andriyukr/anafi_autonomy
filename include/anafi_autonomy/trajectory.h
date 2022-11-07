#include <iostream>
#include <fstream>
#include <chrono>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <anafi_autonomy/msg/pose_command.hpp>
#include <anafi_autonomy/msg/velocity_command.hpp>
#include <anafi_autonomy/msg/axes_command.hpp>

using namespace std;
using namespace std::chrono_literals;
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
		rclcpp::Publisher<anafi_autonomy::msg::PoseCommand>::SharedPtr pose_publisher;
		rclcpp::Publisher<anafi_autonomy::msg::VelocityCommand>::SharedPtr velocity_publisher;
		rclcpp::Publisher<anafi_autonomy::msg::AxesCommand>::SharedPtr axes_publisher;

		// Variables
		Vector4d pose_d = Vector4d::Zero();
		Vector4d pose = Vector4d::Zero();
		Vector4d velocity = Vector4d::Zero();
		Vector3d mode = Vector3d::Zero();
		int trajectory_type = 0;
		
		// Messages
		anafi_autonomy::msg::PoseCommand pose_msg;
		anafi_autonomy::msg::VelocityCommand velocity_msg;
		anafi_autonomy::msg::AxesCommand axes_msg;
		
		// Callback
		void timer_callback();
		rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter> &parameters);
};
