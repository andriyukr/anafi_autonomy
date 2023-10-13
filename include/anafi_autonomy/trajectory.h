#include <iostream>
#include <fstream>
#include <chrono>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <anafi_autonomy/msg/reference_command.hpp>
#include <anafi_autonomy/msg/velocity_command.hpp>

#define ROUND(v) (((v.array()*1e3).round()/1e3).transpose())

#define COMMAND_NONE 		0
#define COMMAND_POSITION 	1
#define COMMAND_VELOCITY 	2
#define COMMAND_ATTITUDE 	3
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
		rclcpp::Publisher<anafi_autonomy::msg::ReferenceCommand>::SharedPtr command_publisher;
		rclcpp::Publisher<anafi_autonomy::msg::VelocityCommand>::SharedPtr derivative_publisher;

		// Variables
		bool changed = false;
		Vector4d command = Vector4d::Zero();
		Vector4d derivative = Vector4d::Zero();
		Vector3d mode = Vector3d::Zero();
		Vector4d pose_d = Vector4d::Zero();
		int trajectory_type = 0;
		double initial_t = 0;
		double speed = 1;
		double scale = 1;
		std::string file_waypoints = "";
		MatrixXd waypoints;
		Vector4d w1;
		Vector4d w2;
		int waypoint = 0;

		// Callback
		void timer_callback();
		rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter> &parameters);

		// Functions
		void readWaypoints(std::string fileName);
		double distance(Vector4d v1, Vector4d v2);
};
