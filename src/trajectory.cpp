/** *************************** trajectory.cpp ***************************
 *
 * This code is the trajectory generator. It generates different trajectories
 * based on user input. The user can also select the speed of trajectory.
 * It publishes the desired position and velocity, and the type of the
 * respective trajectory.
 *
 * **********************************************************************/

#include "anafi_autonomy/trajectory.h"

// ********************** Callbacks **********************

// Dynamic reconfigure callback function (from GUI)
/*void dynamicReconfigureCallback(anafi_autonomy::setTrajectoryConfig &config, uint32_t level){
	trajectory_type = config.trajectory;
	pose_d << config.x_d, config.y_d, config.z_d, config.yaw_d;
}*/

Trajectory::Trajectory() : Node("trajectory"){
	RCLCPP_INFO(this->get_logger(), "Trajectory is running...");

    // Publishers
	pose_publisher = this->create_publisher<anafi_autonomy::msg::PoseCommand>("drone/reference_pose", rclcpp::SystemDefaultsQoS());
	velocity_publisher = this->create_publisher<anafi_autonomy::msg::VelocityCommand>("drone/reference_velocity", rclcpp::SystemDefaultsQoS());
	axes_publisher = this->create_publisher<anafi_autonomy::msg::AxesCommand>("drone/command_offboard", rclcpp::SystemDefaultsQoS());

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
		pose << 0, 0, 0, 0;
		velocity << 0, 0, 0, 0;
		mode << 0, 0, 0;
		break;
	case 1: // hover
		pose << 0, 0, 1, pose_d(3);
		velocity << 0, 0, 0, 0;
		mode << 0, 0, 3;
		break;
	case 2: // user
		pose = pose_d;
		velocity << 0, 0, 0, 0;
		mode << 1, 1, 3;
		break;
	}

	// Publish the reference trajectory
	pose_msg.header.stamp = this->get_clock()->now();
	pose_msg.x = pose(0);
	pose_msg.y = pose(1);
	pose_msg.z = pose(2);
	pose_msg.yaw = pose(3);
	pose_publisher->publish(pose_msg);

	// Publish the corresponding reference trajectory velocity
	velocity_msg.header.stamp = this->get_clock()->now();
	velocity_msg.vx = velocity(0);
	velocity_msg.vy = velocity(1);
	velocity_msg.vz = velocity(2);
	velocity_msg.yaw_rate = velocity(3);
	velocity_publisher->publish(velocity_msg);

	// Publish the corresponding reference for each axis
	axes_msg.header.stamp = this->get_clock()->now();
	axes_msg.horizontal_mode = mode(0);
	axes_msg.x = pose(0);
	axes_msg.y = pose(1);
	axes_msg.vertical_mode = mode(1);
	axes_msg.z = pose(2);
	axes_msg.heading_mode = mode(2);
	axes_msg.yaw = pose(3);
	axes_publisher->publish(axes_msg);
}

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	
	rclcpp::spin(std::make_shared<Trajectory>());
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trajectory is stopping...");
	rclcpp::shutdown();
	return 0;
}
