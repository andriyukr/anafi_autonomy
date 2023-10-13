#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#define BOUND3(x, min_x, max_x) (x > max_x ? max_x : (x < min_x ? min_x : x))
#define BOUND2(x, min_max) BOUND3(x, -min_max, min_max)
#define GET_MACRO(_1, _2, _3, NAME, ...) NAME
#define BOUND(...) GET_MACRO(__VA_ARGS__, BOUND3, BOUND2)(__VA_ARGS__)

using namespace std;
using namespace Eigen;

using std::placeholders::_1;

class Controller:public rclcpp::Node{
	public:
		// Constructor
		Controller(rclcpp::NodeOptions options);

		// Functions
		Vector2d controlHorizontalPosition(Vector2d position_desired, Vector2d position_actual, Vector2d velocity_desired, Vector2d velocity_actual, double dt);
		Vector2d controlHorizontalVelocity(Vector2d velocity_desired, Vector2d velocity_actual, Vector2d acceleration_actual);
		double controlVerticalPosition(double position_desired, double position_actual, double velocity_desired, double velocity_actual, double dt);
		double controlYawOrientation(double yaw_desired, double yaw_actual, double velocity_desired, double velocity_actual);
		

	private:
		// Parameter client
		rclcpp::AsyncParametersClient::SharedPtr parameters_client;
		rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_events_subscriber;
				
		// Callback
		OnSetParametersCallbackHandle::SharedPtr parameters_callback;
	
		// Bounds
		MatrixXd bounds = MatrixXd::Zero(3, 2);

		// Callbacks
		void parameter_events_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
		rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters);	
		void parameter_assign(rcl_interfaces::msg::Parameter &parameter);

		// Functions
		double denormalizeAngle(double a1, double a2){
			while(abs(a1 - a2) > M_PI)
				a1 += (a1 < a2) ? 2*M_PI : -2*M_PI;
			return a1;
		}
};
