/** *************************** safeAnafi.cpp ***************************
 *
 * This node collects all the messages:
 *	  - Feedback from the UAV: state estimates for odometry
 *	  - Reference trajectory: desired position and velocity from the trajectory generator
 *	  - UAV commands: position, velocity and attitude commands from PID or ANN/FNN-PD
 *	  - Keyboard input commands: commands from user by keyboard input
 *	  - Constraints and safety: contains constraints and safety bounds for commands
 *	  - Controller: lets the user choose between position/velocity/attitude commands
 *
 * TODO's:
 *	  - Check velocities frame
 *	  - Tune velocity controller
 *	  - Filter accelerations
 *
 * *********************************************************************/

#include "anafi_autonomy/safeAnafi.h"

// Constructor
SafeAnafi::SafeAnafi() : Node("safe_anafi"){
	RCLCPP_INFO(this->get_logger(), "SafeAnafi is running...");

	// Subscribers
	action_subscriber = this->create_subscription<std_msgs::msg::Int8>("keyboard/action", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::actionCallback, this, _1));
	command_skycontroller_subscriber = this->create_subscription<olympe_bridge_interfaces::msg::SkycontrollerCommand>("skycontroller/command", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::skycontrollerCallback, this, _1));
	command_keyboard_subscriber = this->create_subscription<anafi_autonomy::msg::KeyboardDroneCommand>("keyboard/drone_command", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::keyboardCallback, this, _1));
	command_camera_subscriber = this->create_subscription<anafi_autonomy::msg::KeyboardCameraCommand>("keyboard/camera_command", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::cameraCallback, this, _1));
	reference_pose_subscriber = this->create_subscription<anafi_autonomy::msg::PoseCommand>("drone/reference_pose", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::referencePoseCallback, this, _1));
	reference_velocity_subscriber = this->create_subscription<anafi_autonomy::msg::VelocityCommand>("drone/reference_velocity", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::referenceVelocityCallback, this, _1));
	reference_attitude_subscriber = this->create_subscription<anafi_autonomy::msg::AttitudeCommand>("drone/reference_attitude", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::referenceAttitudeCallback, this, _1));
    reference_command_subscriber = this->create_subscription<anafi_autonomy::msg::ReferenceCommand>("drone/reference_command", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::referenceCommandCallback, this, _1));
	derivative_command_subscriber = this->create_subscription<anafi_autonomy::msg::VelocityCommand>("drone/derivative_command", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::derivativeCommandCallback, this, _1));
	reference_gimbal_subscriber = this->create_subscription<geometry_msgs::msg::Vector3>("drone/reference_gimbal", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::referenceGimbalCallback, this, _1));
	reference_zoom_subscriber = this->create_subscription<std_msgs::msg::Float32>("drone/reference_zoom", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::referenceZoomCallback, this, _1));
	state_subscriber = this->create_subscription<std_msgs::msg::String>("drone/state", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::stateCallback, this, _1));
	gps_subscriber = this->create_subscription<sensor_msgs::msg::NavSatFix>("drone/gps/location", rclcpp::SensorDataQoS(), std::bind(&SafeAnafi::gpsCallback, this, _1));
	altitude_subscriber = this->create_subscription<std_msgs::msg::Float32>("drone/altitude", rclcpp::SensorDataQoS(), std::bind(&SafeAnafi::altitudeCallback, this, _1));
	attitude_subscriber = this->create_subscription<geometry_msgs::msg::QuaternionStamped>("drone/attitude", rclcpp::SensorDataQoS(), std::bind(&SafeAnafi::attitudeCallback, this, _1));
	speed_subscriber = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("drone/speed", rclcpp::SensorDataQoS(), std::bind(&SafeAnafi::speedCallback, this, _1));
	odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("drone/odometry", rclcpp::SensorDataQoS(), std::bind(&SafeAnafi::odometryCallback, this, _1));
	pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("drone/pose", rclcpp::SensorDataQoS(), std::bind(&SafeAnafi::poseCallback, this, _1));

	// Publishers
	rpyg_publisher = this->create_publisher<olympe_bridge_interfaces::msg::PilotingCommand>("drone/command", rclcpp::SystemDefaultsQoS());
	moveto_publisher = this->create_publisher<olympe_bridge_interfaces::msg::MoveToCommand>("drone/moveto", rclcpp::SystemDefaultsQoS());
	moveby_publisher = this->create_publisher<olympe_bridge_interfaces::msg::MoveByCommand>("drone/moveby", rclcpp::SystemDefaultsQoS());
	camera_publisher = this->create_publisher<olympe_bridge_interfaces::msg::CameraCommand>("camera/command", rclcpp::SystemDefaultsQoS());
	gimbal_publisher = this->create_publisher<olympe_bridge_interfaces::msg::GimbalCommand>("gimbal/command", rclcpp::SystemDefaultsQoS());
	odometry_publisher = this->create_publisher<nav_msgs::msg::Odometry>("drone/odometry", rclcpp::SensorDataQoS());
	acceleration_publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("drone/debug/acceleration", rclcpp::SystemDefaultsQoS());
	mode_publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("drone/debug/mode", rclcpp::SystemDefaultsQoS());

	// Services
	arm_client = this->create_client<std_srvs::srv::SetBool>("drone/arm");
	takeoff_client = this->create_client<std_srvs::srv::Trigger>("drone/takeoff");
	land_client = this->create_client<std_srvs::srv::Trigger>("drone/land");
	emergency_client = this->create_client<std_srvs::srv::Trigger>("drone/emergency");
	halt_client = this->create_client<std_srvs::srv::Trigger>("drone/halt");
	rth_client = this->create_client<std_srvs::srv::Trigger>("drone/rth");
	reboot_client = this->create_client<std_srvs::srv::Trigger>("drone/reboot");
	calibrate_magnetometer_client = this->create_client<std_srvs::srv::Trigger>("drone/calibrate");
	offboard_client = this->create_client<std_srvs::srv::SetBool>("skycontroller/offboard");
	flightplan_upload_client = this->create_client<olympe_bridge_interfaces::srv::FlightPlan>("flightplan/upload");
	flightplan_start_client = this->create_client<olympe_bridge_interfaces::srv::FlightPlan>("flightplan/start");
	flightplan_pause_client = this->create_client<std_srvs::srv::Trigger>("flightplan/pause");
	flightplan_stop_client = this->create_client<std_srvs::srv::Trigger>("flightplan/stop");
	followme_start_client = this->create_client<olympe_bridge_interfaces::srv::FollowMe>("followme/start");
	followme_stop_client = this->create_client<std_srvs::srv::Trigger>("followme/stop");
	reset_gimbal_client = this->create_client<std_srvs::srv::Trigger>("gimbal/reset");
	calibrate_gimbal_client = this->create_client<std_srvs::srv::Trigger>("gimbal/calibrate");
	reset_zoom_client = this->create_client<std_srvs::srv::Trigger>("camera/reset");
	take_photo_client = this->create_client<olympe_bridge_interfaces::srv::Photo>("camera/photo/take");
	start_recording_client = this->create_client<olympe_bridge_interfaces::srv::Recording>("camera/recording/start");
	stop_recording_client = this->create_client<olympe_bridge_interfaces::srv::Recording>("camera/recording/stop");
	download_media_client = this->create_client<std_srvs::srv::SetBool>("storage/download");

	// Static requests
	trigger_request = std::make_shared<std_srvs::srv::Trigger::Request>();
	false_request = std::make_shared<std_srvs::srv::SetBool::Request>();
	true_request = std::make_shared<std_srvs::srv::SetBool::Request>();
	photo_request = std::make_shared<olympe_bridge_interfaces::srv::Photo::Request>();
    recording_request = std::make_shared<olympe_bridge_interfaces::srv::Recording::Request>();
	false_request->data = false;
	true_request->data = true;

	// Parameters
	callback = this->add_on_set_parameters_callback(std::bind(&SafeAnafi::parameter_callback, this, std::placeholders::_1));

	rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
	rcl_interfaces::msg::IntegerRange integer_range;
	rcl_interfaces::msg::FloatingPointRange floating_point_range;

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor();
	parameter_descriptor.description = "Enable hand launched takeoff";
	this->declare_parameter("hand_launch", true, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor();
	parameter_descriptor.description = "Enable control during takeoff";
	this->declare_parameter("takingoff_control", false, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Enable control during landing";
	this->declare_parameter("landing_control", false, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Fixed frame for velocity commands";
	this->declare_parameter("fixed_frame", false, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Yaw in world frame";
	this->declare_parameter("world_frame", false, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Mission type: 0 = flight plan, 1 = follow me";
	integer_range = rcl_interfaces::msg::IntegerRange{};
	integer_range.from_value = 0;
	integer_range.to_value = 1;
	integer_range.step = 1;
	parameter_descriptor.integer_range.push_back(integer_range);
	this->declare_parameter("mission_type", 0, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Absolute path to the FlightPlan file";
	std::string package_path = ament_index_cpp::get_package_share_directory("anafi_autonomy");
	this->declare_parameter("flightplan_file", package_path + "/missions/test.mavlink", parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "FollowMe mode: 1 = look at the target without moving automatically, 2 = follow the target keeping the same vector, 3 = follow the target keeping the same orientation to its direction, 4 = follow the target as it was held by a leash";
	integer_range = rcl_interfaces::msg::IntegerRange{};
	integer_range.from_value = 1;
	integer_range.to_value = 4;
	integer_range.step = 1;
	parameter_descriptor.integer_range.push_back(integer_range);
	this->declare_parameter("followme_mode", 2, parameter_descriptor);

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

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Min x bound";
	parameter_descriptor.read_only = true;
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = -4000.0;
	floating_point_range.to_value = 4000.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("bounds/x/min", -1.0, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Max x bound";
	parameter_descriptor.read_only = true;
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = -4000.0;
	floating_point_range.to_value = 4000.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("bounds/x/max", 1.0, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Min y bound";
	parameter_descriptor.read_only = true;
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = -4000.0;
	floating_point_range.to_value = 4000.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("bounds/y/min", -1.0, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Max y bound";
	parameter_descriptor.read_only = true;
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = -4000.0;
	floating_point_range.to_value = 4000.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("bounds/y/max", 1.0, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Min z bound";
	parameter_descriptor.read_only = true;
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = 0.0;
	floating_point_range.to_value = 4000.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("bounds/z/min", 0.0, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Max z bound";
	parameter_descriptor.read_only = true;
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = 0.0;
	floating_point_range.to_value = 4000.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("bounds/z/max", 1.0, parameter_descriptor);

	// Timer
	timer = this->create_wall_timer(10ms, std::bind(&SafeAnafi::timer_callback, this));

	// Initialise the set of velocities
	geometry_msgs::msg::Twist t;
	velocities.clear();
	for(int i = 0; i < FILTER_SIZE; ++i)
		velocities.push_back(t);

	// Parameters client
	auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "anafi");
	param_events_subscriber = parameters_client->on_parameter_event(std::bind(&SafeAnafi::parameter_events_callback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult SafeAnafi::parameter_callback(const std::vector<rclcpp::Parameter> &parameters){
	auto result = rcl_interfaces::msg::SetParametersResult();
	result.successful = true;

	for(auto &parameter:parameters){
		if(parameter.get_name() == "hand_launch"){
			hand_launch = parameter.as_bool();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'hand_launch' set to %s", hand_launch ? "true" : "false");
			return result;
		}
		if(parameter.get_name() == "takingoff_control"){
			takingoff_control = parameter.as_bool();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'takingoff_control' set to %s", takingoff_control ? "true" : "false");
			return result;
		}
		if(parameter.get_name() == "landing_control"){
			landing_control = parameter.as_bool();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'landing_control' set to %s", landing_control ? "true" : "false");
			return result;
		}
		if(parameter.get_name() == "fixed_frame"){
			fixed_frame = parameter.as_bool();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'fixed_frame' set to %s", fixed_frame ? "true" : "false");
			return result;
		}
		if(parameter.get_name() == "world_frame"){
			world_frame = parameter.as_bool();
			if(!world_frame)
				initial_yaw = yaw;
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'world_frame' set to %s", world_frame ? "true" : "false");
			return result;
		}
		if(parameter.get_name() == "mission_type"){
			mission_type = parameter.as_int();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'mission_type' set to %i", mission_type);
			return result;
		}
		if(parameter.get_name() == "flightplan_file"){
			flightplan_file = parameter.as_string();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'flightplan_file' set to '%s'", flightplan_file.c_str());
			return result;
		}
		if(parameter.get_name() == "followme_mode"){
			followme_mode = parameter.as_int();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'followme_mode' set to %i", followme_mode);
			return result;
		}
		if(parameter.get_name() == "gains/position/p"){
			k_p_position = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'k_p_position' set to %.1f", k_p_position);
			return result;
		}
		if(parameter.get_name() == "gains/position/i"){
			k_i_position = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'k_i_position' set to %.1f", k_i_position);
			return result;
		}
		if(parameter.get_name() == "gains/position/d"){
			k_d_position = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'k_d_position' set to %.1f", k_d_position);
			return result;
		}
		if(parameter.get_name() == "gains/position/max_i"){
			max_i_position = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'max_i_position' set to %.1f", max_i_position);
			return result;
		}
		if(parameter.get_name() == "gains/velocity/p"){
			k_p_velocity = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'k_p_velocity' set to %.1f", k_p_velocity);
			return result;
		}
		if(parameter.get_name() == "gains/velocity/d"){
			k_d_velocity = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'k_d_velocity' set to %.1f", k_d_velocity);
			return result;
		}
		if(parameter.get_name() == "gains/yaw/p"){
			k_p_yaw = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'k_p_yaw' set to %.1f", k_p_yaw);
			return result;
		}
		if(parameter.get_name() == "bounds/x/min"){
			bounds(0,0) = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'bounds_x_min' set to %.1f", bounds(0,0));
			return result;
		}
		if(parameter.get_name() == "bounds/x/max"){
			bounds(0,1) = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'bounds_x_max' set to %.1f", bounds(0,1));
			return result;
		}
		if(parameter.get_name() == "bounds/y/min"){
			bounds(1,0) = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'bounds_y_min' set to %.1f", bounds(1,0));
			return result;
		}
		if(parameter.get_name() == "bounds/y/max"){
			bounds(1,1) = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'bounds_y_max' set to %.1f", bounds(1,1));
			return result;
		}
		if(parameter.get_name() == "bounds/z/min"){
			bounds(2,0) = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'bounds_z_min' set to %.1f", bounds(2,0));
			return result;
		}
		if(parameter.get_name() == "bounds/z/max"){
			bounds(2,1) = parameter.as_double();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'bounds_z_max' set to %.1f", bounds(2,1));
			return result;
		}
	}

	result.successful = false;
	return result;
}

void SafeAnafi::parameter_events_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event){
	for(rcl_interfaces::msg::Parameter & changed_parameter : event->changed_parameters)
		parameter_assign(changed_parameter);
	for(rcl_interfaces::msg::Parameter & new_parameter : event->new_parameters)
		parameter_assign(new_parameter);
}

void SafeAnafi::parameter_assign(rcl_interfaces::msg::Parameter & parameter){
	if(parameter.name == "max_vertical_speed"){
		max_vertical_speed = parameter.value.double_value;
		RCLCPP_DEBUG(this->get_logger(), "Parameter 'max_vertical_speed' set to %.1f", max_vertical_speed);
	}
	if(parameter.name == "max_yaw_rate"){
		max_yaw_rate = parameter.value.double_value;
		RCLCPP_DEBUG(this->get_logger(), "Parameter 'max_yaw_rate' set to %.1f", max_yaw_rate);
	}
	if(parameter.name == "max_horizontal_speed"){
		max_horizontal_speed = parameter.value.double_value;
		RCLCPP_DEBUG(this->get_logger(), "Parameter 'max_horizontal_speed' set to %.1f", max_horizontal_speed);
	}
	if(parameter.name == "max_tilt"){
		max_tilt = parameter.value.double_value;
		RCLCPP_DEBUG(this->get_logger(), "Parameter 'max_tilt' set to %.1f", max_tilt);
	}
}

void SafeAnafi::timer_callback(){
	stateMachine();

	// Move gimbal
	gimbal_command << 	(controller_gimbal_command(0) != 0 ?  controller_gimbal_command(0) : (keyboard_gimbal_command(0) != 0 ? keyboard_gimbal_command(0) : offboard_gimbal_command(0))),
	                  	(controller_gimbal_command(1) != 0 ? -controller_gimbal_command(1) : (keyboard_gimbal_command(1) != 0 ? keyboard_gimbal_command(1) : offboard_gimbal_command(1))),
	                  	(controller_gimbal_command(2) != 0 ? -controller_gimbal_command(2) : (keyboard_gimbal_command(2) != 0 ? keyboard_gimbal_command(2) : offboard_gimbal_command(2)));

	olympe_bridge_interfaces::msg::GimbalCommand gimbal_msg;
	gimbal_msg.header.stamp = this->get_clock()->now();
	gimbal_msg.header.frame_id = "body";
	gimbal_msg.mode = 1;
	gimbal_msg.frame = 1;
	if(!gimbal_command.isZero()){
		gimbal_msg.roll = gimbal_command(0);
		gimbal_msg.pitch = gimbal_command(1);
		gimbal_msg.yaw = gimbal_command(2);
		gimbal_publisher->publish(gimbal_msg);
		stop_gimbal = false;
	}
	else
		if(!stop_gimbal){
			gimbal_publisher->publish(gimbal_msg);
			stop_gimbal = true;
		}

	// Zoom
	zoom_command = (controller_zoom_command != 0 ? controller_zoom_command : (keyboard_zoom_command != 0 ? keyboard_zoom_command : offboard_zoom_command));

	olympe_bridge_interfaces::msg::CameraCommand camera_msg;
	camera_msg.header.stamp = this->get_clock()->now();
	camera_msg.header.frame_id = "gimbal";
	camera_msg.mode = 1;
	if(zoom_command != 0){
		camera_msg.zoom = zoom_command;
		camera_publisher->publish(camera_msg);
		stop_zoom = false;
	}
	else
		if(!stop_zoom){
			camera_publisher->publish(camera_msg);
			stop_zoom = true;
		}

	geometry_msgs::msg::Vector3Stamped acceleration_msg; // FOR DEBUG
	acceleration_msg.header.stamp = this->get_clock()->now(); // FOR DEBUG
	acceleration_msg.vector.x = acceleration(0); // FOR DEBUG
	acceleration_msg.vector.y = acceleration(1); // FOR DEBUG
	acceleration_msg.vector.z = acceleration(2); // FOR DEBUG
	acceleration_publisher->publish(acceleration_msg); // FOR DEBUG
		
	geometry_msgs::msg::Vector3Stamped mode_msg; // FOR DEBUG
	mode_msg.header.stamp = this->get_clock()->now(); // FOR DEBUG
	mode_msg.vector.x = mode_move(0); // FOR DEBUG
	mode_msg.vector.y = mode_move(1); // FOR DEBUG
	mode_msg.vector.z = mode_move(2); // FOR DEBUG
	mode_publisher->publish(mode_msg); // FOR DEBUG
}

void SafeAnafi::actionCallback(const std_msgs::msg::Int8& action_msg){
	action = static_cast<Actions>(action_msg.data);
}

void SafeAnafi::skycontrollerCallback(const olympe_bridge_interfaces::msg::SkycontrollerCommand& command_msg){
	// Drone commands
	command_skycontroller << max_horizontal_speed/100*command_msg.x, -max_horizontal_speed/100*command_msg.y, max_vertical_speed/100*command_msg.z, -max_yaw_rate/100*command_msg.yaw;
	mode_skycontroller << ((command_msg.x != 0 || command_msg.y != 0) ? COMMAND_VELOCITY : COMMAND_NONE), (command_msg.z != 0 ? COMMAND_VELOCITY : COMMAND_NONE), (command_msg.yaw != 0 ? COMMAND_RATE : COMMAND_NONE);
	
	// Move
	controller_gimbal_command << 0, (float)command_msg.camera/100, 0;

	// Change zoom
	controller_zoom_command = -(float)command_msg.zoom/100;

	// Switch between manual and offboard
	if(command_msg.reset_camera)
		offboard_client->async_send_request(false_request);
	if(command_msg.reset_zoom)
		offboard_client->async_send_request(true_request);
}

void SafeAnafi::keyboardCallback(const anafi_autonomy::msg::KeyboardDroneCommand& command_msg){
	command_keyboard << max_horizontal_speed/100*command_msg.x, max_horizontal_speed/100*command_msg.y, max_vertical_speed/100*command_msg.z, max_yaw_rate/100*command_msg.yaw;
	mode_keyboard << ((command_msg.x != 0 || command_msg.y != 0) ? COMMAND_VELOCITY : COMMAND_NONE), (command_msg.z != 0 ? COMMAND_VELOCITY : COMMAND_NONE),
			(command_msg.yaw != 0 ? COMMAND_RATE : COMMAND_NONE);
}

void SafeAnafi::cameraCallback(const anafi_autonomy::msg::KeyboardCameraCommand& command_msg){
	keyboard_gimbal_command << command_msg.roll/100, command_msg.pitch/100, command_msg.yaw/100;
	keyboard_zoom_command = command_msg.zoom/100;

	switch(command_msg.action){
	case 1:
		take_photo_client->async_send_request(photo_request);
		break;
	case 2:
		start_recording_client->async_send_request(recording_request);
		break;
	case 3:
		stop_recording_client->async_send_request(recording_request);
		break;
	case 4:
		download_media_client->async_send_request(true_request);
		break;
	case 11:
		reset_zoom_client->async_send_request(trigger_request);
		reset_gimbal_client->async_send_request(trigger_request);
		break;
	case 111:
		calibrate_gimbal_client->async_send_request(trigger_request);
		break;
	}
}

void SafeAnafi::referencePoseCallback(const anafi_autonomy::msg::PoseCommand& command_msg){
	command_offboard << command_msg.x, command_msg.y, command_msg.z, command_msg.yaw*M_PI/180;
	mode_offboard << COMMAND_POSITION, COMMAND_POSITION, COMMAND_ATTITUDE;
	derivative_command << 0, 0, 0, 0;
}

void SafeAnafi::referenceVelocityCallback(const anafi_autonomy::msg::VelocityCommand& command_msg){
	command_offboard << command_msg.vx, command_msg.vy, command_msg.vz, command_msg.yaw_rate;
	mode_offboard << COMMAND_VELOCITY, COMMAND_VELOCITY, COMMAND_RATE;
}

void SafeAnafi::referenceAttitudeCallback(const anafi_autonomy::msg::AttitudeCommand& command_msg){
	command_offboard << command_msg.roll*M_PI/180, command_msg.pitch*M_PI/180, command_msg.trottle, command_msg.yaw*M_PI/180;
	mode_offboard << COMMAND_ATTITUDE, COMMAND_VELOCITY, COMMAND_ATTITUDE;
}

void SafeAnafi::referenceCommandCallback(const anafi_autonomy::msg::ReferenceCommand& command_msg){
	command_offboard <<     (command_msg.horizontal_mode == COMMAND_NONE ? command_offboard(0) : command_msg.x),
	                        (command_msg.horizontal_mode == COMMAND_NONE ? command_offboard(1) : command_msg.y),
                            (command_msg.vertical_mode == COMMAND_NONE ? command_offboard(2) : command_msg.z),
                            (command_msg.heading_mode == COMMAND_NONE ? command_offboard(3) :
                                (command_msg.heading_mode == COMMAND_ATTITUDE ? command_msg.yaw*M_PI/180 : command_msg.yaw));
	mode_offboard << command_msg.horizontal_mode, command_msg.vertical_mode, command_msg.heading_mode;
}

void SafeAnafi::derivativeCommandCallback(const anafi_autonomy::msg::VelocityCommand& derivative_msg){
	derivative_command << cos(yaw)*derivative_msg.vx - sin(yaw)*derivative_msg.vy, sin(yaw)*derivative_msg.vx + cos(yaw)*derivative_msg.vy, derivative_msg.vz, derivative_msg.yaw_rate;
}

void SafeAnafi::referenceGimbalCallback(const geometry_msgs::msg::Vector3& command_msg){
	offboard_gimbal_command << command_msg.x, command_msg.y, command_msg.z;
}

void SafeAnafi::referenceZoomCallback(const std_msgs::msg::Float32& command_msg){
	offboard_zoom_command = command_msg.data;
}

void SafeAnafi::stateCallback(const std_msgs::msg::String& state_msg){ // 'LANDED', 'MOTOR_RAMPING', 'USER_TAKEOFF', 'TAKINGOFF', 'HOVERING', 'FLYING', 'LANDING', 'EMERGENCY'
	state = resolveState(state_msg.data);
}

void SafeAnafi::gpsCallback(__attribute__((unused)) const sensor_msgs::msg::NavSatFix& gps_msg){ //TODO: Implement this function

}

void SafeAnafi::altitudeCallback(const std_msgs::msg::Float32& altitude_msg){
	if(pose_available <= 0 && odometry_available <= 0){
        position(2) = altitude_msg.data;

        altitude_available = 10;
	}
}

void SafeAnafi::attitudeCallback(const geometry_msgs::msg::QuaternionStamped& quaternion_msg){
    if(pose_available <= 0 && odometry_available <= 0){
        time = rclcpp::Time(quaternion_msg.header.stamp.sec, quaternion_msg.header.stamp.nanosec);
        dt = (time - time_old_attitude).nanoseconds()/1e9;

        tf2::Quaternion q(quaternion_msg.quaternion.x, quaternion_msg.quaternion.y, quaternion_msg.quaternion.z, quaternion_msg.quaternion.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, yaw);

        geometry_msgs::msg::Twist t;
        t.angular.x = (roll - orientation(0))/dt;
        t.angular.y = (pitch - orientation(1))/dt;
        t.angular.z = (yaw - orientation(2))/dt;
        t = filter_mean_velocities(t);

        orientation << roll, pitch, yaw;
        rates << t.angular.x, t.angular.y, t.angular.z;

        time_old_attitude = time;

        attitude_available = 10;
    }
}

void SafeAnafi::speedCallback(const geometry_msgs::msg::Vector3Stamped& speed_msg){
    if(odometry_available <= 0){
        time = rclcpp::Time(speed_msg.header.stamp.sec, speed_msg.header.stamp.nanosec);
        dt = (time - time_old_speed).nanoseconds()/1e9;

        velocity << speed_msg.vector.x, speed_msg.vector.y, speed_msg.vector.z;

        acceleration << (velocity - velocity_old)/dt;
        acceleration = filter_mean_acceleration(acceleration);

        velocity_old = velocity;
        time_old_speed = time;

        velocity_available = 10;
	}
}

void SafeAnafi::poseCallback(const geometry_msgs::msg::PoseStamped& pose_msg){
	time = rclcpp::Time(pose_msg.header.stamp.sec, pose_msg.header.stamp.nanosec);
	dt = (time - time_old_pose).nanoseconds()/1e9;
	
	nav_msgs::msg::Odometry odometry_msg;
	odometry_msg.header.stamp = pose_msg.header.stamp;
	odometry_msg.header.frame_id = pose_msg.header.frame_id;
	odometry_msg.child_frame_id = "/body";

	odometry_msg.pose.pose.position.x = pose_msg.pose.position.x;
	odometry_msg.pose.pose.position.y = pose_msg.pose.position.y;
	odometry_msg.pose.pose.position.z = pose_msg.pose.position.z;

	tf2::Quaternion q(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch;
	m.getRPY(roll, pitch, yaw);
	odometry_msg.pose.pose.orientation = pose_msg.pose.orientation;

	geometry_msgs::msg::Twist t;
	t.linear.x = (odometry_msg.pose.pose.position.x - position(0))/dt;
	t.linear.y = (odometry_msg.pose.pose.position.y - position(1))/dt;
	t.linear.z = (odometry_msg.pose.pose.position.z - position(2))/dt;
	t.angular.x = (roll - orientation(0))/dt;
	t.angular.y = (pitch - orientation(1))/dt;
	t.angular.z = (yaw - orientation(2))/dt;
	odometry_msg.twist.twist = filter_mean_velocities(t);
	odometry_publisher->publish(odometry_msg);

	position << odometry_msg.pose.pose.position.x, odometry_msg.pose.pose.position.y, odometry_msg.pose.pose.position.z;
	orientation << roll, pitch, yaw;
	velocity << odometry_msg.twist.twist.linear.x, odometry_msg.twist.twist.linear.y, odometry_msg.twist.twist.linear.z;
	rates << odometry_msg.twist.twist.angular.x, odometry_msg.twist.twist.angular.y, odometry_msg.twist.twist.angular.z;

	time_old_pose = time;

	pose_available = 30;
}

void SafeAnafi::odometryCallback(const nav_msgs::msg::Odometry& odometry_msg){
	time = rclcpp::Time(odometry_msg.header.stamp.sec, odometry_msg.header.stamp.nanosec);
	dt = (time - time_old_odometry).nanoseconds()/1e9;

	position << odometry_msg.pose.pose.position.x, odometry_msg.pose.pose.position.y, odometry_msg.pose.pose.position.z;

	tf2::Quaternion q(odometry_msg.pose.pose.orientation.x, odometry_msg.pose.pose.orientation.y, odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch;
	m.getRPY(roll, pitch, yaw);

	if(!world_frame)
		yaw = yaw - initial_yaw;

	velocity << odometry_msg.twist.twist.linear.x, odometry_msg.twist.twist.linear.y, odometry_msg.twist.twist.linear.z;

	acceleration << (velocity - velocity_old)/dt;
	acceleration = filter_mean_acceleration(acceleration);

	velocity_old = velocity;
	time_old_odometry = time;

	odometry_available = 40;
}

void SafeAnafi::stateMachine(){
	// State-indipendent actions
	switch(action){
	case DISARM: // emergency
		emergency_client->async_send_request(trigger_request);
		return;
	case RESET_POSE:
		RCLCPP_INFO(this->get_logger(), "Resetting position and heading");
		position << 0, 0, 0;
		initial_yaw = yaw;
		action = NONE;
		return;
	case REMOTE_CONTROL:
		offboard_client->async_send_request(false_request);
		action = NONE;
		return;
	case OFFBOARD_CONTROL:
		offboard_client->async_send_request(true_request);
		action = NONE;
		return;
	default:
  		break;
	}

	switch(state){
	case LANDED:
	    controllers(); // ONLY FOR GROUND TEST DEBUG
		switch(action){
		case ARM:
		    if(hand_launch)
				arm_client->async_send_request(true_request);
			else{
				armed = !armed;
				if(armed)
					RCLCPP_WARN(this->get_logger(), "Armed");
				else
					RCLCPP_INFO(this->get_logger(), "Disarmed");
			}
			break;
		case TAKEOFF:
			if(armed)
				takeoff_client->async_send_request(trigger_request);
			else
				RCLCPP_WARN(this->get_logger(), "Arm the drone with 'Insert' before taking-off.");
			armed = false;
			break;
		case MISSION_START:
			if(mission_type == 0){ // flight plan
				if(armed){
					auto flightplan_request = std::make_shared<olympe_bridge_interfaces::srv::FlightPlan::Request>();
					flightplan_request->file = flightplan_file;
					flightplan_request->uid = "";
					flightplan_upload_client->async_send_request(flightplan_request);
					flightplan_start_client->async_send_request(flightplan_request);
				}
				else
					RCLCPP_WARN(this->get_logger(), "Arm the drone with 'Insert' before starting the mission.");
				armed = false;
			}
			else
				RCLCPP_WARN(this->get_logger(), "The drone has to be in the air.");
			break;
		case REBOOT:
			reboot_client->async_send_request(trigger_request);
			break;
		case CALIBRATE:
			calibrate_magnetometer_client->async_send_request(trigger_request);
			break;
		default:
			break;
		}
		break;
	case MOTOR_RAMPING:
		switch(action){
		case ARM: // disarm
		case LAND:
		case HALT:
			arm_client->async_send_request(false_request);
			break;
		default:
			break;
		}
		break;
	case USER_TAKEOFF:
		switch(action){
		case ARM: // disarm
		case LAND:
		case HALT:
			arm_client->async_send_request(false_request);
			break;
		case TAKEOFF:
			arm_client->async_send_request(false_request); // needs to stop the motors before taking off
			takeoff_client->async_send_request(trigger_request);
			break;
		default:
			break;
		}
		break;
	case TAKINGOFF:
		switch(action){
		case LAND:
		case HALT:
			land_client->async_send_request(trigger_request);
			break;
		default:
			if(takingoff_control)
				controllers();
		}
		break;
	case HOVERING:
	case FLYING:
		switch(action){
		case LAND:
			land_client->async_send_request(trigger_request);
			break;
		case RTH: // return-to-home
			rth_client->async_send_request(trigger_request);
			break;
		case MISSION_START:
			if(mission_type == 0){ // flight plan
				auto flightplan_request = std::make_shared<olympe_bridge_interfaces::srv::FlightPlan::Request>();
				flightplan_request->file = flightplan_file;
				flightplan_request->uid = "";
				flightplan_upload_client->async_send_request(flightplan_request);
				flightplan_start_client->async_send_request(flightplan_request);
			}
			else{ // follow me
				auto followme_request = std::make_shared<olympe_bridge_interfaces::srv::FollowMe::Request>();
				followme_request->mode = followme_mode;
				followme_request->horizontal = 2;
				followme_request->vertical = 10;
				followme_request->target_azimuth = 0;
				followme_request->target_elevation = 45;
				followme_request->change_of_scale = 0;
				followme_request->confidence_index = 255;
				followme_request->is_new_selection = true;
				followme_start_client->async_send_request(followme_request);
			}
			break;
		case MISSION_PAUSE:
			flightplan_pause_client->async_send_request(trigger_request);
			break;
		case MISSION_STOP:
			if(mission_type == 0) // flight plan
				flightplan_stop_client->async_send_request(trigger_request);
			else // follow me
				followme_stop_client->async_send_request(trigger_request);
			break;
		case HALT:
			halt_client->async_send_request(trigger_request);
			// TODO: set velocities to 0
			break;
		default:
			controllers();
		}
		break;
	case LANDING:
		switch(action){
		case TAKEOFF:
		case HALT:
			takeoff_client->async_send_request(trigger_request);
			break;
		default:
			if(landing_control)
				controllers();
		}
		break;
	case EMERGENCY:
		switch(action){
		case TAKEOFF:
		case HALT:
			takeoff_client->async_send_request(trigger_request);
			break;
		default:
			break;
		}
		break;
	case INVALID:
		switch(action){
		case HALT:
			takeoff_client->async_send_request(trigger_request);
			break;
		case REBOOT:
			reboot_client->async_send_request(trigger_request);
			break;
		default:
			break;
		}
	}
	
	action = NONE;
}

States SafeAnafi::resolveState(std::string input){
   if(input == "LANDED") return LANDED;
   if(input == "MOTOR_RAMPING") return MOTOR_RAMPING;
   if(input == "USER_TAKEOFF") return USER_TAKEOFF;
   if(input == "TAKINGOFF") return TAKINGOFF;
   if(input == "HOVERING") return HOVERING;
   if(input == "FLYING") return FLYING;
   if(input == "LANDING") return LANDING;
   if(input == "EMERGENCY") return EMERGENCY;
   return INVALID;
}

void SafeAnafi::controllers(){
	command_move << 	(mode_skycontroller(0) != COMMAND_NONE ? command_skycontroller(0) : (mode_keyboard(0) != COMMAND_NONE ? command_keyboard(0) : (mode_offboard(0) != COMMAND_NONE ? command_offboard(0) : 0))),
						(mode_skycontroller(0) != COMMAND_NONE ? command_skycontroller(1) : (mode_keyboard(0) != COMMAND_NONE ? command_keyboard(1) : (mode_offboard(0) != COMMAND_NONE ? command_offboard(1) : 0))),
						(mode_skycontroller(1) != COMMAND_NONE ? command_skycontroller(2) : (mode_keyboard(1) != COMMAND_NONE ? command_keyboard(2) : (mode_offboard(1) != COMMAND_NONE ? command_offboard(2) : 0))),
						(mode_skycontroller(2) != COMMAND_NONE ? command_skycontroller(3) : (mode_keyboard(2) != COMMAND_NONE ? command_keyboard(3) : (mode_offboard(2) != COMMAND_NONE ? command_offboard(3) : 0)));
	mode_move << 	(mode_skycontroller(0) != COMMAND_NONE ? mode_skycontroller(0) : (mode_keyboard(0) != COMMAND_NONE ? mode_keyboard(0) : (mode_offboard(0) != COMMAND_NONE ? mode_offboard(0) : ((velocity_available > 0 || odometry_available > 0) ? COMMAND_VELOCITY : COMMAND_ATTITUDE)))),
				 	(mode_skycontroller(1) != COMMAND_NONE ? mode_skycontroller(1) : (mode_keyboard(1) != COMMAND_NONE ? mode_keyboard(1) : (mode_offboard(1) != COMMAND_NONE ? mode_offboard(1) : COMMAND_VELOCITY))),
				 	(mode_skycontroller(2) != COMMAND_NONE ? mode_skycontroller(2) : (mode_keyboard(2) != COMMAND_NONE ? mode_keyboard(2) : (mode_offboard(2) != COMMAND_NONE ? mode_offboard(2) : COMMAND_RATE)));

    switch(mode_move(MODE_HORIZONTAL)){ // horizotal
	case COMMAND_NONE: // no command
		rpyg_msg.roll = 0;
		rpyg_msg.pitch = 0;
		break;
	case COMMAND_POSITION: // position
        if(pose_available > 0 || odometry_available > 0){
	        pose_available--;
	        odometry_available--;

			command_move(0) = BOUND(command_move(0), bounds(0,0), bounds(0,1));
			command_move(1) = BOUND(command_move(1), bounds(1,0), bounds(1,1));
			// TODO: IMPLEMENT POSITION CONTROLLER HERE
		}
		else{
			command_move(0) = 0;
			command_move(1) = 0;
			RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *get_clock(), 1000, "No localization feedback.");
		    break;
		}
		[[fallthrough]];
	case COMMAND_VELOCITY: // velocity
	    if(velocity_available > 0 || odometry_available > 0){
	        velocity_available--;
	        odometry_available--;

            command_move(0) = BOUND(command_move(0), max_horizontal_speed);
            command_move(1) = BOUND(command_move(1), max_horizontal_speed);

            if(fixed_frame)
                command_move << cos(yaw)*command_move(0) + sin(yaw)*command_move(1), -sin(yaw)*command_move(0) + cos(yaw)*command_move(1), command_move(2), command_move(3); // rotate command from world frame to body frame

            velocity_error << command_move(0) - velocity(0), command_move(1) - velocity(1), 0;
            velocity_error_d = -acceleration;

            command_move(0) = -(k_p_velocity*velocity_error(1) + k_d_velocity*velocity_error_d(1));
            command_move(1) =   k_p_velocity*velocity_error(0) + k_d_velocity*velocity_error_d(0);
		}else{
			command_move(0) = 0;
			command_move(1) = 0;
			RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *get_clock(), 1000, "No speed feedback.");
		    break;
		}
		[[fallthrough]];
	case COMMAND_ATTITUDE: // attitude
		command_move(0) = BOUND(command_move(0), max_tilt);
		command_move(1) = BOUND(command_move(1), max_tilt);
		rpyg_msg.roll = command_move(0);
		rpyg_msg.pitch = command_move(1);
		break;
	default:
		rpyg_msg.roll = 0;
		rpyg_msg.pitch = 0;
		RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *get_clock(), 1000, "Undefined horizontal mode (" << mode_move(MODE_HORIZONTAL) << ").");
		RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *get_clock(), 1000, "It should be '0' for no command, '1' for position command, '2' for velocity command, or '3' for attitude command.");
	}

	switch(mode_move(MODE_VERTICAL)){ // vertical
	case COMMAND_NONE: // no command
		rpyg_msg.gaz = 0;
		break;
	case COMMAND_POSITION: // position
	    if(altitude_available > 0 || pose_available > 0 || odometry_available > 0){
	        altitude_available--;
	        pose_available--;
	        odometry_available--;

            command_move(2) = BOUND(command_move(2), bounds(2,0), bounds(2,1));
            position_error(2) = command_move(2) - position(2);
            position_error_d(2) = derivative_command(2) - velocity(2);
            position_error_i(2) = BOUND(position_error_i(2) + position_error(2)*dt, max_i_position);
            command_move(2) = k_p_position*position_error(2) + k_i_position*position_error_i(2) + k_d_position*position_error_d(2);
		}else{
		    rpyg_msg.gaz = 0;
		    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *get_clock(), 1000, "No altitude feedback.");
		    break;
		}
		[[fallthrough]];
	case COMMAND_VELOCITY: // velocity
		command_move(2) = BOUND(command_move(2), max_vertical_speed);
		rpyg_msg.gaz = command_move(2);
		break;
	default:
		rpyg_msg.gaz = 0;
		RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *get_clock(), 1000, "Undefined vertical mode (" << mode_move(MODE_VERTICAL) << ").");
		RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *get_clock(), 1000, "It should be '0' for no command, '1' for position command or '2' for velocity command.");
	}

	switch(mode_move(MODE_HEADING)){ // heading
	case COMMAND_NONE: // no command
		rpyg_msg.yaw = 0;
		break;
	case COMMAND_ATTITUDE: // attitude
	    if(attitude_available > 0 || pose_available > 0 || odometry_available > 0){
	        attitude_available--;
	        pose_available--;
	        odometry_available--;

            yaw = denormalizeAngle(yaw, command_move(3));
            command_move(3) = k_p_yaw*(command_move(3) - yaw);
		}else{
		    rpyg_msg.yaw = 0;
		    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *get_clock(), 1000, "No attitude feedback.");
		    break;
		}
		[[fallthrough]];
	case COMMAND_RATE: // angular rate
		command_move(3) = BOUND(command_move(3), max_yaw_rate);
		rpyg_msg.yaw = command_move(3);
		break;
	default:
		rpyg_msg.yaw = 0;
		RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *get_clock(), 1000, "Undefined heading mode (" << mode_move(MODE_HEADING) << ").");
		RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *get_clock(), 1000, "It should be '0' for no command, '3' for yaw command or '4' for rate command.");
	}

	command_move << 0, 0, 0, 0;
	mode_move << COMMAND_NONE, COMMAND_NONE, COMMAND_NONE;

	rpyg_msg.header.stamp = this->get_clock()->now();
	rpyg_msg.header.frame_id = "body";
	rpyg_publisher->publish(rpyg_msg);
}

geometry_msgs::msg::Twist SafeAnafi::filter_mean_velocities(geometry_msgs::msg::Twist v){
	velocities.erase(velocities.begin());
	velocities.push_back(v);

	geometry_msgs::msg::Twist t;
	for(vector<geometry_msgs::msg::Twist>::iterator i = velocities.begin(); i != velocities.end(); ++i){
		t.linear.x += i->linear.x/FILTER_SIZE;
		t.linear.y += i->linear.y/FILTER_SIZE;
		t.linear.z += i->linear.z/FILTER_SIZE;
		t.angular.x += i->angular.x/FILTER_SIZE;
		t.angular.y += i->angular.y/FILTER_SIZE;
		t.angular.z += i->angular.z/FILTER_SIZE;
	}
	return t;
}

Vector3d SafeAnafi::filter_mean_acceleration(Eigen::Ref<Eigen::VectorXd> a){
	accelerations.topRows(FILTER_SIZE - 1) = accelerations.bottomRows(FILTER_SIZE - 1);
	accelerations.row(FILTER_SIZE - 1) = a;

	return accelerations.colwise().sum()/FILTER_SIZE;
}

Vector3d SafeAnafi::filter_polinomial_acceleration(Eigen::Ref<Eigen::VectorXd> a){
	accelerations.topRows(FILTER_SIZE - 1) = accelerations.bottomRows(FILTER_SIZE - 1);
	accelerations.row(FILTER_SIZE - 1) = a;

	return accelerations.bottomRows(1);
}

double SafeAnafi::denormalizeAngle(double a1, double a2){
	if(abs(a1 - a2) > M_PI)
		a1 += (a1 < a2) ? 2*M_PI : -2*M_PI;
	return a1;
}


int main(int argc, char** argv){	
	rclcpp::init(argc, argv);
	
	rclcpp::spin(std::make_shared<SafeAnafi>());
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SafeAnafi is stopping...");

	rclcpp::shutdown();
	return 0;
}
