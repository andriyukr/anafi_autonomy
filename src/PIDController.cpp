/* *************************** PIDController.cpp **********************
*
* 
*
**********************************************************************/

#include "anafi_autonomy/controller.h"

// Gains
double k_position_p = 0;
double k_position_i = 0;
double k_position_d = 0;
double max_position_i = 0;
double k_velocity_p = 0;
double k_velocity_d = 0;
double k_yaw_p = 0;

// Error
Vector3d position_error_i = Vector3d::Zero();

// Constructor
Controller::Controller(rclcpp::NodeOptions options) : Node("pid_controller", options){
	RCLCPP_INFO(this->get_logger(), "PIDController is running...");

	// Parameters
	parameters_callback = this->add_on_set_parameters_callback(std::bind(&Controller::parameter_callback, this, std::placeholders::_1));

	rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
	rcl_interfaces::msg::FloatingPointRange floating_point_range;

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Position proportional gain";
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = 0.0;
	floating_point_range.to_value = 10.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("gains/position/p", 2.0, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Position integral gain";
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = 0.0;
	floating_point_range.to_value = 10.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("gains/position/i", 1.0, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Position derivative gain";
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = 0.0;
	floating_point_range.to_value = 10.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("gains/position/d", 0.5, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Position max integral component";
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = 0.0;
	floating_point_range.to_value = 1.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("gains/position/max_i", 0.1, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Velocity proportional gain";
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = 0.0;
	floating_point_range.to_value = 10.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("gains/velocity/p", 9.1, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Velocity derivative gain";
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = 0.0;
	floating_point_range.to_value = 10.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("gains/velocity/d", 1.3, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Yaw proportional gain";
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = 0.0;
	floating_point_range.to_value = 100.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("gains/yaw/p", 70.0, parameter_descriptor);

	// Parameters client
	//param_events_subscriber = parameters_client->on_parameter_event(std::bind(&Controller::parameter_events_callback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult Controller::parameter_callback(const std::vector<rclcpp::Parameter> &parameters){
	auto result = rcl_interfaces::msg::SetParametersResult();
	result.successful = true;

	for(auto &parameter:parameters){
		if(parameter.get_name() == "gains/position/p"){
			k_position_p = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'k_position_p' set to %.1f", k_position_p);
			return result;
		}
		if(parameter.get_name() == "gains/position/i"){
			k_position_i = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'k_position_i' set to %.1f", k_position_i);
			return result;
		}
		if(parameter.get_name() == "gains/position/d"){
			k_position_d = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'k_position_d' set to %.1f", k_position_d);
			return result;
		}
		if(parameter.get_name() == "gains/position/max_i"){
			max_position_i = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'max_position_i' set to %.1f", max_position_i);
			return result;
		}
		if(parameter.get_name() == "gains/velocity/p"){
			k_velocity_p = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'k_velocity_p' set to %.1f", k_velocity_p);
			return result;
		}
		if(parameter.get_name() == "gains/velocity/d"){
			k_velocity_d = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'k_velocity_d' set to %.1f", k_velocity_d);
			return result;
		}
		if(parameter.get_name() == "gains/yaw/p"){
			k_yaw_p = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'k_yaw_p' set to %.1f", k_yaw_p);
			return result;
		}
	}

	result.successful = false;
	return result;
}

void Controller::parameter_events_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event){
	cout << "OK1" << endl;
	for(rcl_interfaces::msg::Parameter & changed_parameter : event->changed_parameters)
		parameter_assign(changed_parameter);
	for(rcl_interfaces::msg::Parameter & new_parameter : event->new_parameters)
		parameter_assign(new_parameter);
}

void Controller::parameter_assign(rcl_interfaces::msg::Parameter & parameter){
	cout << parameter.name << endl;
	if(parameter.name == "bounds/x/min"){
		bounds(0,0) = parameter.value.double_value;
		RCLCPP_DEBUG(this->get_logger(), "Parameter 'bounds/x/min' set to %.1f", bounds(0,0));
	}
	if(parameter.name == "bounds/x/max"){
		bounds(0,1) = parameter.value.double_value;
		RCLCPP_DEBUG(this->get_logger(), "Parameter 'bounds/x/max' set to %.1f", bounds(0,1));
	}
	if(parameter.name == "bounds/y/min"){
		bounds(1,0) = parameter.value.double_value;
		RCLCPP_DEBUG(this->get_logger(), "Parameter 'bounds/y/min' set to %.1f", bounds(1,0));
	}
	if(parameter.name == "bounds/y/max"){
		bounds(1,1) = parameter.value.double_value;
		RCLCPP_DEBUG(this->get_logger(), "Parameter 'bounds/y/max' set to %.1f", bounds(1,1));
	}
	if(parameter.name == "bounds/z/min"){
		bounds(2,0) = parameter.value.double_value;
		RCLCPP_DEBUG(this->get_logger(), "Parameter 'bounds/z/min' set to %.1f", bounds(2,0));
	}
	if(parameter.name == "bounds/z/max"){
		bounds(2,1) = parameter.value.double_value;
		RCLCPP_DEBUG(this->get_logger(), "Parameter 'bounds/z/max' set to %.1f", bounds(2,1));
	}
}

Vector2d Controller::controlHorizontalPosition(Vector2d position_desired, Vector2d position_actual, Vector2d velocity_desired, Vector2d velocity_actual, double dt){
	Vector2d position_error;
	Vector2d position_error_d;
	Vector2d command;

	position_error = position_desired - position_actual;
	position_error_d = velocity_desired - velocity_actual;
	
	position_error_i(0) = BOUND(position_error_i(0) + position_error(0)*dt, max_position_i);
	position_error_i(1) = BOUND(position_error_i(1) + position_error(1)*dt, max_position_i);

	command(0) = k_position_p*position_error(0) + k_position_i*position_error_i(0) + k_position_d*position_error_d(0);
	command(1) = k_position_p*position_error(1) + k_position_i*position_error_i(1) + k_position_d*position_error_d(1);

	return command;			
}

Vector2d Controller::controlHorizontalVelocity(Vector2d velocity_desired, Vector2d velocity_actual, Vector2d acceleration_actual){
	Vector2d velocity_error;
	Vector2d velocity_error_d;
	Vector2d command;

	velocity_error = velocity_desired - velocity_actual;
	velocity_error_d = -acceleration_actual;

	command(0) = -(k_velocity_p*velocity_error(1) + k_velocity_d*velocity_error_d(1));
	command(1) =   k_velocity_p*velocity_error(0) + k_velocity_d*velocity_error_d(0);

	return command;
}

double Controller::controlVerticalPosition(double position_desired, double position_actual, double velocity_desired, double velocity_actual, double dt){
	double position_error = position_desired - position_actual;
	double position_error_d = velocity_desired - velocity_actual;
	position_error_i(2) = BOUND(position_error_i(2) + position_error*dt, max_position_i);
	
	double command = k_position_p*position_error + k_position_i*position_error_i(2) + k_position_d*position_error_d;

	return command;
}

double Controller::controlYawOrientation(double yaw_desired, double yaw_actual, __attribute__((unused)) double velocity_desired, __attribute__((unused)) double velocity_actual){
	yaw_actual = denormalizeAngle(yaw_actual, yaw_desired);
	
	double command = k_yaw_p*(yaw_desired - yaw_actual);

	return command;
}