/** *************************** trajectory.cpp ***************************
 *
 * This code is the trajectory generator. It generates different trajectories
 * based on user input. The user can also select the speed of trajectory.
 * It publishes the desired position and velocity, and the type of the
 * respective trajectory.
 *
 * **********************************************************************/

#include "anafi_autonomy/trajectory.h"

// ********************** Constructor **********************
Trajectory::Trajectory() : Node("trajectory"){
	RCLCPP_INFO(this->get_logger(), "Trajectory is running...");

    // Publishers
    reference_publisher = this->create_publisher<anafi_autonomy::msg::ReferenceCommand>("drone/reference_command", rclcpp::SystemDefaultsQoS());
	derivative_publisher = this->create_publisher<anafi_autonomy::msg::VelocityCommand>("drone/reference_derivative", rclcpp::SystemDefaultsQoS());

	// Parameters
    callback = this->add_on_set_parameters_callback(std::bind(&Trajectory::parameterCallback, this, std::placeholders::_1));

    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
    rcl_interfaces::msg::IntegerRange integer_range;
    rcl_interfaces::msg::FloatingPointRange floating_point_range;

    parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    parameter_descriptor.description = "Trajectory type: 0 = no trajectory, 1 = hover at (0, 0, 1, yaw_d), 2 = defined by user with (x_d, y_d, z_d, yaw_d)";
    integer_range = rcl_interfaces::msg::IntegerRange();
    integer_range.from_value = 0;
    integer_range.to_value = 2;
    integer_range.step = 1;
    parameter_descriptor.integer_range.push_back(integer_range);
    this->declare_parameter("trajectory_type", 0, parameter_descriptor);

    parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    parameter_descriptor.description = "Desired x position";
    floating_point_range = rcl_interfaces::msg::FloatingPointRange();
    floating_point_range.from_value = -10.0;
    floating_point_range.to_value = 10.0;
    floating_point_range.step = 0.0;
    parameter_descriptor.floating_point_range.push_back(floating_point_range);
    this->declare_parameter("desired/x", 0.0, parameter_descriptor);

    parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    parameter_descriptor.description = "Desired y position";
    floating_point_range = rcl_interfaces::msg::FloatingPointRange();
    floating_point_range.from_value = -10.0;
    floating_point_range.to_value = 10.0;
    floating_point_range.step = 0.0;
    parameter_descriptor.floating_point_range.push_back(floating_point_range);
    this->declare_parameter("desired/y", 0.0, parameter_descriptor);

    parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    parameter_descriptor.description = "Desired z position";
    floating_point_range = rcl_interfaces::msg::FloatingPointRange();
    floating_point_range.from_value = 0.0;
    floating_point_range.to_value = 2.0;
    floating_point_range.step = 0.0;
    parameter_descriptor.floating_point_range.push_back(floating_point_range);
    this->declare_parameter("desired/z", 1.0, parameter_descriptor);

    parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor();
    parameter_descriptor.description = "Desired yaw orientation";
    floating_point_range = rcl_interfaces::msg::FloatingPointRange();
    floating_point_range.from_value = -180.0;
    floating_point_range.to_value = 180.0;
    floating_point_range.step = 0.0;
    parameter_descriptor.floating_point_range.push_back(floating_point_range);
    this->declare_parameter("desired/yaw", 0.0, parameter_descriptor);

	// Timer
	timer = this->create_wall_timer(10ms, std::bind(&Trajectory::timer_callback, this));
}

rcl_interfaces::msg::SetParametersResult Trajectory::parameterCallback(const std::vector<rclcpp::Parameter> &parameters){
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for(auto &parameter:parameters){
        if(parameter.get_name() == "trajectory_type")
            trajectory_type = parameter.as_int();
        if(parameter.get_name() == "desired/x")
            pose_d(0) = parameter.as_double();
        if(parameter.get_name() == "desired/y")
            pose_d(1) = parameter.as_double();
        if(parameter.get_name() == "desired/z")
            pose_d(2) = parameter.as_double();
        if(parameter.get_name() == "desired/yaw")
            pose_d(3) = parameter.as_double();
    }

    return result;
}

void Trajectory::timer_callback(){
	switch(trajectory_type){
	case 0: // no command
		reference << 0, 0, 0, 0;
		derivative << 0, 0, 0, 0;
		mode << COMMAND_NONE, COMMAND_NONE, COMMAND_NONE;
		break;
	case 1: // hover
		reference << 0, 0, 1, pose_d(3);
		derivative << 0, 0, 0, 0;
		mode << COMMAND_VELOCITY, COMMAND_POSITION, COMMAND_ANGLE;
		break;
	case 2: // user
		reference = pose_d;
		derivative << 0, 0, 0, 0;
		mode << COMMAND_POSITION, COMMAND_POSITION, COMMAND_ANGLE;
		break;
	}

	// Publish the corresponding reference for each axis
	reference_msg.header.stamp = this->get_clock()->now();
	reference_msg.horizontal_mode = mode(0);
	reference_msg.x = reference(0);
	reference_msg.y = reference(1);
	reference_msg.vertical_mode = mode(1);
	reference_msg.z = reference(2);
	reference_msg.heading_mode = mode(2);
	reference_msg.yaw = reference(3);
	reference_publisher->publish(reference_msg);

	// Publish the corresponding reference trajectory velocity
	derivative_msg.header.stamp = this->get_clock()->now();
	derivative_msg.vx = derivative(0);
	derivative_msg.vy = derivative(1);
	derivative_msg.vz = derivative(2);
	derivative_msg.yaw_rate = derivative(3);
	derivative_publisher->publish(derivative_msg);
}

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	
	rclcpp::spin(std::make_shared<Trajectory>());
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trajectory is stopping...");
	rclcpp::shutdown();
	return 0;
}
