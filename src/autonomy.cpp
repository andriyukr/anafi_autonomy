/** *************************** autonomy.cpp ***************************
 *
 * This node collects all the messages:
 *	  - Feedback from the UAV: state estimates for odometry
 *	  - Reference trajectory: desired position and velocity from the trajectory generator
 *	  - UAV commands: position, velocity and attitude commands from PID
 *	  - Keyboard input commands: commands from user by keyboard input
 *	  - Constraints and safety: contains constraints and safety bounds for commands
 *	  - Controller: lets the user choose between position/velocity/attitude commands
 *
 * TODO's:
 *	  - Tune velocity controller
 *
 * *********************************************************************/

#include "anafi_autonomy/autonomy.h"

// Constructor
Autonomy::Autonomy() : Node("autonomy"){
	RCLCPP_INFO(this->get_logger(), "Autonomy is running...");

	// Create controller node
	//thread_controller = std::thread(&Autonomy::spin_controller, this);
	//thread_controller.detach();

	// Subscribers
	action_subscriber = this->create_subscription<std_msgs::msg::UInt8>("drone/action", rclcpp::SystemDefaultsQoS(), std::bind(&Autonomy::actionCallback, this, _1));
	skycontroller_subscriber = this->create_subscription<anafi_ros_interfaces::msg::SkycontrollerCommand>("skycontroller/command", rclcpp::SystemDefaultsQoS(), std::bind(&Autonomy::skycontrollerCallback, this, _1));
	keyboard_subscriber = this->create_subscription<anafi_autonomy::msg::KeyboardCommand>("keyboard/command", rclcpp::SystemDefaultsQoS(), std::bind(&Autonomy::keyboardCallback, this, _1));
	reference_pose_subscriber = this->create_subscription<anafi_autonomy::msg::PoseCommand>("drone/reference_pose", rclcpp::SystemDefaultsQoS(), std::bind(&Autonomy::referencePoseCallback, this, _1));
	reference_velocity_subscriber = this->create_subscription<anafi_autonomy::msg::VelocityCommand>("drone/reference_velocity", rclcpp::SystemDefaultsQoS(), std::bind(&Autonomy::referenceVelocityCallback, this, _1));
	reference_attitude_subscriber = this->create_subscription<anafi_autonomy::msg::AttitudeCommand>("drone/reference_attitude", rclcpp::SystemDefaultsQoS(), std::bind(&Autonomy::referenceAttitudeCallback, this, _1));
	reference_command_subscriber = this->create_subscription<anafi_autonomy::msg::ReferenceCommand>("drone/reference_command", rclcpp::SystemDefaultsQoS(), std::bind(&Autonomy::referenceCommandCallback, this, _1));
	derivative_command_subscriber = this->create_subscription<anafi_autonomy::msg::VelocityCommand>("drone/derivative_command", rclcpp::SystemDefaultsQoS(), std::bind(&Autonomy::derivativeCommandCallback, this, _1));
	reference_gimbal_subscriber = this->create_subscription<geometry_msgs::msg::Vector3>("drone/reference_gimbal", rclcpp::SystemDefaultsQoS(), std::bind(&Autonomy::referenceGimbalCallback, this, _1));
	reference_zoom_subscriber = this->create_subscription<std_msgs::msg::Float32>("drone/reference_zoom", rclcpp::SystemDefaultsQoS(), std::bind(&Autonomy::referenceZoomCallback, this, _1));
	state_subscriber = this->create_subscription<std_msgs::msg::String>("drone/state", rclcpp::SystemDefaultsQoS(), std::bind(&Autonomy::stateCallback, this, _1));
	gps_subscriber = this->create_subscription<sensor_msgs::msg::NavSatFix>("drone/gps/location", rclcpp::SensorDataQoS(), std::bind(&Autonomy::gpsCallback, this, _1));
	altitude_subscriber = this->create_subscription<std_msgs::msg::Float32>("drone/altitude", rclcpp::SensorDataQoS(), std::bind(&Autonomy::altitudeCallback, this, _1));
	attitude_subscriber = this->create_subscription<geometry_msgs::msg::QuaternionStamped>("drone/attitude", rclcpp::SensorDataQoS(), std::bind(&Autonomy::attitudeCallback, this, _1));
	gimbal_subscriber = this->create_subscription<geometry_msgs::msg::QuaternionStamped>("gimbal/attitude", rclcpp::SensorDataQoS(), std::bind(&Autonomy::gimbalCallback, this, _1));
	speed_subscriber = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("drone/speed", rclcpp::SensorDataQoS(), std::bind(&Autonomy::speedCallback, this, _1));
	mocap_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("drone/pose", rclcpp::SensorDataQoS(), std::bind(&Autonomy::mocapCallback, this, _1));
	pose_camera_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("camera/pose", rclcpp::SensorDataQoS(), std::bind(&Autonomy::cameraPoseCallback, this, _1));

	// Publishers
	rpyg_publisher = this->create_publisher<anafi_ros_interfaces::msg::PilotingCommand>("drone/command", rclcpp::SystemDefaultsQoS());
	moveto_publisher = this->create_publisher<anafi_ros_interfaces::msg::MoveToCommand>("drone/moveto", rclcpp::SystemDefaultsQoS());
	moveby_publisher = this->create_publisher<anafi_ros_interfaces::msg::MoveByCommand>("drone/moveby", rclcpp::SystemDefaultsQoS());
	camera_publisher = this->create_publisher<anafi_ros_interfaces::msg::CameraCommand>("camera/command", rclcpp::SystemDefaultsQoS());
	gimbal_publisher = this->create_publisher<anafi_ros_interfaces::msg::GimbalCommand>("gimbal/command", rclcpp::SystemDefaultsQoS());
	odometry_publisher = this->create_publisher<nav_msgs::msg::Odometry>("drone/odometry", rclcpp::SensorDataQoS());
	acceleration_publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("drone/linear_acceleration", rclcpp::SensorDataQoS());
	rate_publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("drone/angular_velocity", rclcpp::SensorDataQoS());
	imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("drone/imu", rclcpp::SensorDataQoS());
	camera_imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("camera/imu", rclcpp::SensorDataQoS());
	camera_imu_fast_publisher = this->create_publisher<sensor_msgs::msg::Imu>("camera/imu/interpolated", rclcpp::SensorDataQoS());  // FOR ORB_SLAM
	position_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>("drone/position/vision", rclcpp::SensorDataQoS());
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
	flightplan_upload_client = this->create_client<anafi_ros_interfaces::srv::FlightPlan>("flightplan/upload");
	flightplan_start_client = this->create_client<anafi_ros_interfaces::srv::FlightPlan>("flightplan/start");
	flightplan_pause_client = this->create_client<std_srvs::srv::Trigger>("flightplan/pause");
	flightplan_stop_client = this->create_client<std_srvs::srv::Trigger>("flightplan/stop");
	followme_start_client = this->create_client<anafi_ros_interfaces::srv::FollowMe>("followme/start");
	followme_stop_client = this->create_client<std_srvs::srv::Trigger>("followme/stop");
	reset_gimbal_client = this->create_client<std_srvs::srv::Trigger>("gimbal/reset");
	calibrate_gimbal_client = this->create_client<std_srvs::srv::Trigger>("gimbal/calibrate");
	reset_zoom_client = this->create_client<std_srvs::srv::Trigger>("camera/reset");
	take_photo_client = this->create_client<anafi_ros_interfaces::srv::Photo>("camera/photo/take");
	start_recording_client = this->create_client<anafi_ros_interfaces::srv::Recording>("camera/recording/start");
	stop_recording_client = this->create_client<anafi_ros_interfaces::srv::Recording>("camera/recording/stop");
	download_media_client = this->create_client<std_srvs::srv::SetBool>("storage/download");

	// Static requests
	trigger_request = std::make_shared<std_srvs::srv::Trigger::Request>();
	false_request = std::make_shared<std_srvs::srv::SetBool::Request>();
	true_request = std::make_shared<std_srvs::srv::SetBool::Request>();
	photo_request = std::make_shared<anafi_ros_interfaces::srv::Photo::Request>();
	recording_request = std::make_shared<anafi_ros_interfaces::srv::Recording::Request>();
	false_request->data = false;
	true_request->data = true;

	// Parameters
	parameters_callback = this->add_on_set_parameters_callback(std::bind(&Autonomy::parameter_callback, this, std::placeholders::_1));

	rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
	rcl_interfaces::msg::IntegerRange integer_range;
	rcl_interfaces::msg::FloatingPointRange floating_point_range;
	
	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor();
	parameter_descriptor.description = "The drone is armed";
	this->declare_parameter("armed", false, parameter_descriptor);

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
	parameter_descriptor.description = "Yaw aligned with North";
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
	this->declare_parameter("flight_plan/file", package_path + "/missions/test.mavlink", parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "FollowMe mode: "
		"1 = look at the target without moving automatically, "
		"2 = follow the target keeping the same vector, "
		"3 = follow the target keeping the same orientation to its direction, "
		"4 = follow the target as it was held by a leash";
	integer_range = rcl_interfaces::msg::IntegerRange{};
	integer_range.from_value = 1;
	integer_range.to_value = 4;
	integer_range.step = 1;
	parameter_descriptor.integer_range.push_back(integer_range);
	this->declare_parameter("follow_me/mode", 2, parameter_descriptor);

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
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = -4000.0;
	floating_point_range.to_value = 4000.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("bounds/x/min", -10.0, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Max x bound";
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = -4000.0;
	floating_point_range.to_value = 4000.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("bounds/x/max", 10.0, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Min y bound";
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = -4000.0;
	floating_point_range.to_value = 4000.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("bounds/y/min", -10.0, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Max y bound";
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = -4000.0;
	floating_point_range.to_value = 4000.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("bounds/y/max", 10.0, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Min z bound";
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = 0.0;
	floating_point_range.to_value = 4000.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("bounds/z/min", 0.0, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Max z bound";
	floating_point_range = rcl_interfaces::msg::FloatingPointRange{};
	floating_point_range.from_value = 0.0;
	floating_point_range.to_value = 4000.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("bounds/z/max", 2.0, parameter_descriptor);

	// Timer
	timer = this->create_wall_timer(10ms, std::bind(&Autonomy::timer_callback, this));
	timer_camera_imu_fast = this->create_wall_timer(100ms, std::bind(&Autonomy::camera_imu_fast_callback, this));  // FOR ORB_SLAM

	// Parameters client
	auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "anafi");
	param_events_subscriber = parameters_client->on_parameter_event(std::bind(&Autonomy::parameter_events_callback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult Autonomy::parameter_callback(const std::vector<rclcpp::Parameter> &parameters){
	auto result = rcl_interfaces::msg::SetParametersResult();
	result.successful = true;

	for(auto &parameter:parameters){
		if(parameter.get_name() == "armed"){
			return result;
		}
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
				yaw_offset = yaw;
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'world_frame' set to %s", world_frame ? "true" : "false");
			return result;
		}
		if(parameter.get_name() == "mission_type"){
			mission_type = parameter.as_int();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'mission_type' set to %i", mission_type);
			return result;
		}
		if(parameter.get_name() == "flight_plan/file"){
			flightplan_file = parameter.as_string();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'flight_plan/file' set to '%s'", flightplan_file.c_str());
			return result;
		}
		if(parameter.get_name() == "follow_me/mode"){
			followme_mode = parameter.as_int();
			RCLCPP_DEBUG(this->get_logger(), "Parameter 'follow_me/mode' set to %i", followme_mode);
			return result;
		}
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

void Autonomy::parameter_events_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event){
	for(rcl_interfaces::msg::Parameter & changed_parameter : event->changed_parameters)
		parameter_assign(changed_parameter);
	for(rcl_interfaces::msg::Parameter & new_parameter : event->new_parameters)
		parameter_assign(new_parameter);
}

void Autonomy::parameter_assign(rcl_interfaces::msg::Parameter & parameter){
	if(parameter.name == "drone/max_vertical_speed"){
		max_vertical_speed = parameter.value.double_value;
		RCLCPP_DEBUG(this->get_logger(), "Parameter 'drone/max_vertical_speed' set to %.1f", max_vertical_speed);
	}
	if(parameter.name == "drone/max_yaw_rate"){
		max_yaw_rate = parameter.value.double_value;
		RCLCPP_DEBUG(this->get_logger(), "Parameter 'drone/max_yaw_rate' set to %.1f", max_yaw_rate);
	}
	if(parameter.name == "drone/max_horizontal_speed"){
		max_horizontal_speed = parameter.value.double_value;
		RCLCPP_DEBUG(this->get_logger(), "Parameter 'drone/max_horizontal_speed' set to %.1f", max_horizontal_speed);
	}
	if(parameter.name == "drone/max_pitch_roll"){
		max_tilt = parameter.value.double_value;
		RCLCPP_DEBUG(this->get_logger(), "Parameter 'drone/max_pitch_roll' set to %.1f", max_tilt);
	}
}

void Autonomy::timer_callback(){
	// Select position sourse
	position << 	(mocap_available > 0 ? position_mocap(0) : (vision_available > 0 ? position_vision(0) : NAN)), // priority: mocap > vision
					(mocap_available > 0 ? position_mocap(1) : (vision_available > 0 ? position_vision(1) : NAN)), // priority: mocap > vision
					(mocap_available > 0 ? position_mocap(2) : (barometer_available > 0 ? altitude : NAN)); // priority: mocap > barometer
	// Select velocity sourse
	velocity << 	(mocap_available > 0 ? velocity_mocap(0) : (optical_available > 0 ? velocity_optical(0) : NAN)), // priority: mocap > optical flow
					(mocap_available > 0 ? velocity_mocap(1) : (optical_available > 0 ? velocity_optical(1) : NAN)), // priority: mocap > optical flow
					(mocap_available > 0 ? velocity_mocap(2) : (optical_available > 0 ? velocity_optical(2) : NAN)); // priority: mocap > barometer
	// Select acceleration sourse
	acceleration << (mocap_available > 0 ? acceleration_mocap(0) : (optical_available > 0 ? acceleration_optical(0) : NAN)), // priority: mocap > optical flow
					(mocap_available > 0 ? acceleration_mocap(1) : (optical_available > 0 ? acceleration_optical(1) : NAN)), // priority: mocap > optical flow
					(mocap_available > 0 ? acceleration_mocap(2) : (optical_available > 0 ? acceleration_optical(2) : NAN)); // priority: mocap > barometer
	// Select yaw sourse
	yaw_old = yaw;
	yaw = (mocap_available > 0 ? yaw_mocap : (magnetometer_available > 0 ? yaw_magnetometer : (vision_available > 0 ? yaw_vision : NAN))); // priority: mocap > magnetometer > vision
	yaw = denormalizeAngle(yaw, yaw_old);
	// Select time difference sourse
	dt << 	(mocap_available > 0 ? dt_mocap : (vision_available > 0 ? dt_vision : NAN)), // priority: mocap > vision
			(mocap_available > 0 ? dt_mocap : (barometer_available > 0 ? dt_altitude : NAN)); // priority: mocap > onboard

	stateMachine();

	// Consume tokens
	mocap_available    		= mocap_available > 0    ? mocap_available - 1    : 0;
	vision_available   		= vision_available > 0   ? vision_available - 1   : 0;
	barometer_available 	= barometer_available > 0 ? barometer_available - 1 : 0;
	optical_available  		= optical_available > 0  ? optical_available - 1  : 0;
	magnetometer_available 	= magnetometer_available > 0 ? magnetometer_available - 1 : 0;

	controllerCamera();
}

void Autonomy::actionCallback(const std_msgs::msg::UInt8& action_msg){
	action_offboard = static_cast<Actions>(action_msg.data);
}

void Autonomy::skycontrollerCallback(const anafi_ros_interfaces::msg::SkycontrollerCommand& command_msg){
	// Drone commands
	command_skycontroller << max_horizontal_speed/100*command_msg.x, -max_horizontal_speed/100*command_msg.y, max_vertical_speed/100*command_msg.z, -max_yaw_rate/100*command_msg.yaw;
	mode_skycontroller << ((command_msg.x != 0 || command_msg.y != 0) ? COMMAND_VELOCITY : COMMAND_NONE), (command_msg.z != 0 ? COMMAND_VELOCITY : COMMAND_NONE), (command_msg.yaw != 0 ? COMMAND_RATE : COMMAND_NONE);
	
	// Gimbal commands
	controller_gimbal_command << 0, (float)command_msg.camera/100, 0;

	// Zoom command
	controller_zoom_command = -(float)command_msg.zoom/100;

	// Switch between manual and offboard
	if(command_msg.reset_camera)
		offboard_client->async_send_request(false_request);
	if(command_msg.reset_zoom)
		offboard_client->async_send_request(true_request);
}

void Autonomy::keyboardCallback(const anafi_autonomy::msg::KeyboardCommand& command_msg){
	// Drone actions
	action_keyboard = static_cast<Actions>(command_msg.drone_action);
	
	// Drone commands
	command_keyboard << max_horizontal_speed*command_msg.drone_x, max_horizontal_speed*command_msg.drone_y, max_vertical_speed*command_msg.drone_z, max_yaw_rate*command_msg.drone_yaw;
	mode_keyboard << ((command_msg.drone_x != 0 || command_msg.drone_y != 0) ? COMMAND_VELOCITY : COMMAND_NONE), (command_msg.drone_z != 0 ? COMMAND_VELOCITY : COMMAND_NONE), (command_msg.drone_yaw != 0 ? COMMAND_RATE : COMMAND_NONE);

	// Gimbal commands
	keyboard_gimbal_command << command_msg.gimbal_roll, command_msg.gimbal_pitch, command_msg.gimbal_yaw;
	
	// Zoom command
	keyboard_zoom_command = command_msg.zoom;

	// Camera actions
	switch(command_msg.camera_action){
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

void Autonomy::referencePoseCallback(const anafi_autonomy::msg::PoseCommand& command_msg){
	command_offboard << command_msg.x, command_msg.y, command_msg.z, command_msg.yaw*M_PI/180;
	mode_offboard << COMMAND_POSITION, COMMAND_POSITION, COMMAND_ATTITUDE;
	derivative_command << 0, 0, 0, 0;
}

void Autonomy::referenceVelocityCallback(const anafi_autonomy::msg::VelocityCommand& command_msg){
	command_offboard << command_msg.vx, command_msg.vy, command_msg.vz, command_msg.yaw_rate;
	mode_offboard << COMMAND_VELOCITY, COMMAND_VELOCITY, COMMAND_RATE;
}

void Autonomy::referenceAttitudeCallback(const anafi_autonomy::msg::AttitudeCommand& command_msg){
	command_offboard << command_msg.roll*M_PI/180, command_msg.pitch*M_PI/180, command_msg.trottle, command_msg.yaw*M_PI/180;
	mode_offboard << COMMAND_ATTITUDE, COMMAND_VELOCITY, COMMAND_ATTITUDE;
}

void Autonomy::referenceCommandCallback(const anafi_autonomy::msg::ReferenceCommand& command_msg){
	command_offboard <<	(command_msg.horizontal_mode == COMMAND_NONE ? command_offboard(0) : command_msg.x),
						(command_msg.horizontal_mode == COMMAND_NONE ? command_offboard(1) : command_msg.y),
						(command_msg.vertical_mode == COMMAND_NONE ? command_offboard(2) : command_msg.z),
						(command_msg.heading_mode == COMMAND_NONE ? command_offboard(3) : (command_msg.heading_mode == COMMAND_ATTITUDE ? command_msg.yaw*M_PI/180 : command_msg.yaw));
	mode_offboard << command_msg.horizontal_mode, command_msg.vertical_mode, command_msg.heading_mode;
}

void Autonomy::derivativeCommandCallback(const anafi_autonomy::msg::VelocityCommand& derivative_msg){
	derivative_command << derivative_msg.vx, derivative_msg.vy, derivative_msg.vz, derivative_msg.yaw_rate;
}

void Autonomy::referenceGimbalCallback(const geometry_msgs::msg::Vector3& command_msg){
	offboard_gimbal_command << command_msg.x, command_msg.y, command_msg.z;
}

void Autonomy::referenceZoomCallback(const std_msgs::msg::Float32& command_msg){
	offboard_zoom_command = command_msg.data; 
}

void Autonomy::stateCallback(const std_msgs::msg::String& state_msg){
	state = resolveState(state_msg.data); // 'LANDED', 'MOTOR_RAMPING', 'USER_TAKEOFF', 'TAKINGOFF', 'HOVERING', 'FLYING', 'LANDING', 'EMERGENCY'
}

void Autonomy::gpsCallback(__attribute__((unused)) const sensor_msgs::msg::NavSatFix& gps_msg){
	//TODO: Implement this function
}

void Autonomy::altitudeCallback(const std_msgs::msg::Float32& altitude_msg){
	double time = this->get_clock()->now().nanoseconds()/1e9;
	dt_altitude = time - time_old_altitude;
	time_old_altitude = time;

	altitude = altitude_msg.data;

	barometer_available = 7; // comes at 30Hz
}

void Autonomy::attitudeCallback(const geometry_msgs::msg::QuaternionStamped& quaternion_msg){
	double time = quaternion_msg.header.stamp.sec + quaternion_msg.header.stamp.nanosec/1e9;

	tf2::Quaternion q(quaternion_msg.quaternion.x, quaternion_msg.quaternion.y, quaternion_msg.quaternion.z, quaternion_msg.quaternion.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	Vector3d orientation(roll, pitch, yaw);
		
	nd_orientation.updateMeasurements(orientation, time);
	Vector3d rates = nd_orientation.getDerivative();

	geometry_msgs::msg::Vector3Stamped rate_msg;
	rate_msg.header = quaternion_msg.header;
	rate_msg.vector.x = rates(0);
	rate_msg.vector.y = rates(1);
	rate_msg.vector.z = rates(2);
	rate_publisher->publish(rate_msg);

	quaternion_drone = Quaterniond(quaternion_msg.quaternion.w, quaternion_msg.quaternion.x, quaternion_msg.quaternion.y, quaternion_msg.quaternion.z);
	nd_quaternion.updateQuaternion(quaternion_drone, time);
	Vector3d angular_velocity = nd_quaternion.getAngularVelocity();

	Vector3d linear_acceleration = nd_velocity.getUnfiltertedDerivative();

	sensor_msgs::msg::Imu imu_msg;
	imu_msg.header = quaternion_msg.header;
	imu_msg.orientation = quaternion_msg.quaternion;
	imu_msg.angular_velocity.x = angular_velocity(0);
	imu_msg.angular_velocity.y = angular_velocity(1);
	imu_msg.angular_velocity.z = angular_velocity(2);
	imu_msg.linear_acceleration.x = linear_acceleration(0);
	imu_msg.linear_acceleration.y = linear_acceleration(1);
	imu_msg.linear_acceleration.z = linear_acceleration(2);
	imu_publisher->publish(imu_msg);

	yaw_magnetometer = yaw;
	magnetometer_available = 7; // comes at 30Hz
}

void Autonomy::gimbalCallback(const geometry_msgs::msg::QuaternionStamped& quaternion_msg){
	double time = quaternion_msg.header.stamp.sec + quaternion_msg.header.stamp.nanosec/1e9;

	quaternion_gimbal = Quaterniond(quaternion_msg.quaternion.w, quaternion_msg.quaternion.x, quaternion_msg.quaternion.y, quaternion_msg.quaternion.z);
	quaternion_camera = quaternion_gimbal*quaternion_gimbal_camera;
	nd_quaternion_camera.updateQuaternion(quaternion_camera, time);
	Vector3d angular_velocity = nd_quaternion_camera.getAngularVelocity();

	Vector3d linear_acceleration = nd_velocity.getUnfiltertedDerivative(); // in drone frame
	linear_acceleration = quaternion_camera.inverse()*(quaternion_drone*linear_acceleration); // in camera frame
		
	sensor_msgs::msg::Imu imu_msg;
	imu_msg.header.stamp = quaternion_msg.header.stamp;
	imu_msg.header.frame_id = "/camera";
	imu_msg.orientation.x = quaternion_camera.x();
	imu_msg.orientation.y = quaternion_camera.y();
	imu_msg.orientation.z = quaternion_camera.z();
	imu_msg.orientation.w = quaternion_camera.w();
	imu_msg.angular_velocity.x = angular_velocity(0);
	imu_msg.angular_velocity.y = angular_velocity(1);
	imu_msg.angular_velocity.z = angular_velocity(2);
	imu_msg.linear_acceleration.x = linear_acceleration(0);
	imu_msg.linear_acceleration.y = linear_acceleration(1);
	imu_msg.linear_acceleration.z = linear_acceleration(2);
	camera_imu_publisher->publish(imu_msg);
}

void Autonomy::camera_imu_fast_callback(){ // FOR ORB_SLAM
	if(nd_quaternion_camera.getTime() == DBL_MAX) // don't publish before the first attitude arrives 
		return;

	double time = this->get_clock()->now().nanoseconds()/1e9;
	double dt = time - time_camera_old;
	time_camera_old = time;

	if(time_update != nd_quaternion_camera.getTime()){
		time_update = nd_quaternion_camera.getTime();
	
		angular_velocity_camera = nd_quaternion_camera.getAngularVelocity();
		angular_acceleration_camera = nd_quaternion_camera.getAngularAcceleration();

		linear_acceleration_camera = nd_velocity.getUnfiltertedDerivative(); // in drone frame
		linear_jerk_camera = nd_velocity.getUnfiltertedSecondDerivative(); // in drone frame
		linear_acceleration_camera = quaternion_camera.inverse()*(quaternion_drone*linear_acceleration_camera); // in camera frame
		linear_jerk_camera = quaternion_camera.inverse()*(quaternion_drone*linear_jerk_camera); // in camera frame
	}
	else{
		angular_velocity_camera += angular_acceleration_camera*dt;
		linear_acceleration_camera += linear_jerk_camera*dt;
	}

	if(time_camera < time_update)
		time_camera = time_update;
	else
		time_camera += dt;

	sensor_msgs::msg::Imu imu_msg;
	imu_msg.header.stamp.sec = floor(time_camera);
	imu_msg.header.stamp.nanosec = (time_camera - floor(time_camera))*1e9;
	imu_msg.header.frame_id = "/camera";
	imu_msg.orientation.x = quaternion_camera.x();
	imu_msg.orientation.y = quaternion_camera.y();
	imu_msg.orientation.z = quaternion_camera.z();
	imu_msg.orientation.w = quaternion_camera.w();
	imu_msg.angular_velocity.x = angular_velocity_camera(0);
	imu_msg.angular_velocity.y = angular_velocity_camera(1);
	imu_msg.angular_velocity.z = angular_velocity_camera(2);
	imu_msg.linear_acceleration.x = linear_acceleration_camera(0);
	imu_msg.linear_acceleration.y = linear_acceleration_camera(1);
	imu_msg.linear_acceleration.z = linear_acceleration_camera(2);
	camera_imu_fast_publisher->publish(imu_msg);
}

void Autonomy::speedCallback(const geometry_msgs::msg::Vector3Stamped& speed_msg){
	double time = speed_msg.header.stamp.sec + speed_msg.header.stamp.nanosec/1e9;

	velocity_optical << speed_msg.vector.x, speed_msg.vector.y, speed_msg.vector.z;

	nd_velocity.updateMeasurements(velocity_optical, time);
	acceleration_optical = nd_velocity.getDerivative();

	geometry_msgs::msg::Vector3Stamped acceleration_msg;
	acceleration_msg.header = speed_msg.header;
	acceleration_msg.vector.x = acceleration_optical(0);
	acceleration_msg.vector.y = acceleration_optical(1);
	acceleration_msg.vector.z = acceleration_optical(2);
	acceleration_publisher->publish(acceleration_msg);

	optical_available = 7; // comes at 30Hz
}

void Autonomy::mocapCallback(const geometry_msgs::msg::PoseStamped& pose_msg){
	double time = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec/1e9;
	dt_mocap = time - time_old_mocap;

	position_mocap << pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z;
	
	tf2::Quaternion q(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	Vector3d orientation(roll, pitch, yaw);
		
	nd_position.updateMeasurements(position_mocap, time);
	velocity_mocap = nd_position.getDerivative();

	velocity_mocap << rotateVector(velocity_mocap.head(2), yaw), velocity_mocap(2); // rotate velocity from mocap frame to body frame

	nd_orientation_mocap.updateMeasurements(orientation, time);
	Vector3d rates = nd_orientation_mocap.getDerivative();

	nav_msgs::msg::Odometry odometry_msg;
	odometry_msg.header = pose_msg.header;
	odometry_msg.pose.pose.position = pose_msg.pose.position;
	odometry_msg.pose.pose.orientation = pose_msg.pose.orientation;
	geometry_msgs::msg::Twist t;
	t.linear.x = velocity_mocap(0);
	t.linear.y = velocity_mocap(1);
	t.linear.z = velocity_mocap(2);
	t.angular.x = rates(0);
	t.angular.y = rates(1);
	t.angular.z = rates(2);
	odometry_publisher->publish(odometry_msg);

	acceleration_mocap = acceleration_optical;//nd_position.getSecondDerivative();

	time_old_mocap = time;

	yaw_mocap = yaw;
	mocap_available = 2; // comes at 100Hz
}

void Autonomy::cameraPoseCallback(const geometry_msgs::msg::PoseStamped& pose_msg){
	double time = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec/1e9;
	dt_vision = time - time_old_vision;
		
	position_vision << pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z; // in camera frame
	position_vision *= scale;
	if(pose_msg.header.frame_id == "orb_slam")
		position_vision = quaternion_gimbal_camera*position_vision; // in world frame

	geometry_msgs::msg::PointStamped position_msg;
	position_msg.header.stamp = pose_msg.header.stamp;
	position_msg.header.frame_id = "/world";
	position_msg.point.x = position_vision(0);
	position_msg.point.y = position_vision(1);
	position_msg.point.z = position_vision(2);
	position_publisher->publish(position_msg);

	time_old_vision = time;

	vision_available = 7; // comes at 30Hz
}

void Autonomy::initialize_visual_odometry(){
	RCLCPP_INFO(this->get_logger(), "Start Visual-Odometry initialization!");
	double altitude1 = altitude;
	double z1 = position_vision(2);
	command_offboard << 0.0, 0.0, 1.0, 0.0;
	mode_offboard << COMMAND_VELOCITY, COMMAND_VELOCITY, COMMAND_RATE;
	rclcpp::sleep_for(1s);
	command_offboard << 0.0, 0.0, 0.0, 0.0;
	mode_offboard << COMMAND_VELOCITY, COMMAND_VELOCITY, COMMAND_RATE;
	rclcpp::sleep_for(100ms);
	double altitude2 = altitude;
	double z2 = position_vision(2);
	command_offboard << 0.0, 0.0, -1.0, 0.0;
	mode_offboard << COMMAND_VELOCITY, COMMAND_VELOCITY, COMMAND_RATE;
	rclcpp::sleep_for(1s);
	command_offboard << 0.0, 0.0, 0.0, 0.0;
	mode_offboard << COMMAND_VELOCITY, COMMAND_VELOCITY, COMMAND_RATE;
	rclcpp::sleep_for(100ms);
	double altitude3 = altitude;
	double z3 = position_vision(2);
	if((z2 - z1 != 0) && (z3 - z2 != 0)){
		scale = ((altitude2 - altitude1)/(z2 - z1) + (altitude3 - altitude2)/(z3 - z2))/2;
		RCLCPP_INFO_STREAM(this->get_logger(), "Visual-Odometry initialized with scale = " << scale);
	}
	else
		RCLCPP_INFO(this->get_logger(), "Visual-Odometry not initialized!");
}

void Autonomy::stateMachine(){
	Actions action = (action_keyboard != NONE ? action_keyboard : action_offboard);
	action_keyboard = NONE;
	action_offboard = NONE;

	switch(action){ // State-indipendent actions
	case DISARM: // emergency
		emergency_client->async_send_request(trigger_request);
		this->set_parameter(rclcpp::Parameter("armed", false));
		return;
	case RESET_POSE:
		RCLCPP_INFO(this->get_logger(), "Resetting position and heading");
		position_offset = position;
		yaw_offset = yaw;
		action = NONE;
		return;
	case REMOTE_CONTROL:
		offboard_client->async_send_request(false_request);
		break;
	case OFFBOARD_CONTROL:
		offboard_client->async_send_request(true_request);
		break;
	case INITIALIZE_VIO:
		if(vision_available > 0){ 
			thread_initialize_vo = std::thread(&Autonomy::initialize_visual_odometry, this);
			thread_initialize_vo.detach();
		}else
			RCLCPP_WARN_STREAM(this->get_logger(), "No Visual-Odometry available.");
		break;
	default:
		break;
	}

	switch(state){
	case LANDED:
	    controllers(); // ONLY FOR GROUND TEST DEBUG
		switch(action){
		case ARM:
			if(hand_launch){
				arm_client->async_send_request(true_request);
				armed = true;
				this->set_parameter(rclcpp::Parameter("armed", true));
			}
			else{
				armed = !armed;
				if(armed)
					RCLCPP_WARN(this->get_logger(), "Armed");
				else
					RCLCPP_INFO(this->get_logger(), "Disarmed");
				this->set_parameter(rclcpp::Parameter("armed", armed));
			}			
			break;
		case TAKEOFF:
			if(armed)
				takeoff_client->async_send_request(trigger_request);
			else
				RCLCPP_WARN(this->get_logger(), "Arm the drone before taking-off.");
			break;
		case MISSION_START:
			if(mission_type == 0){ // flight plan
				if(armed){
					auto flightplan_request = std::make_shared<anafi_ros_interfaces::srv::FlightPlan::Request>();
					flightplan_request->file = flightplan_file;
					flightplan_request->uid = "";
					flightplan_upload_client->async_send_request(flightplan_request);
					flightplan_start_client->async_send_request(flightplan_request);
				}
				else
					RCLCPP_WARN(this->get_logger(), "Arm the drone before starting the mission.");
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
			armed = false;
			this->set_parameter(rclcpp::Parameter("armed", false));
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
			armed = false;
			this->set_parameter(rclcpp::Parameter("armed", false));
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
			armed = false;
			this->set_parameter(rclcpp::Parameter("armed", false));
			break;
		case RTH: // return-to-home
			rth_client->async_send_request(trigger_request);
			break;
		case MISSION_START:
			if(mission_type == 0){ // flight plan
				auto flightplan_request = std::make_shared<anafi_ros_interfaces::srv::FlightPlan::Request>();
				flightplan_request->file = flightplan_file;
				flightplan_request->uid = "";
				flightplan_upload_client->async_send_request(flightplan_request);
				flightplan_start_client->async_send_request(flightplan_request);
			}
			else{ // follow me
				auto followme_request = std::make_shared<anafi_ros_interfaces::srv::FollowMe::Request>();
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
			armed = true;
			this->set_parameter(rclcpp::Parameter("armed", true));
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
			armed = false;
			this->set_parameter(rclcpp::Parameter("armed", false));
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
}

States Autonomy::resolveState(std::string input){
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

void Autonomy::controllers(){
	Vector4d command_move;
	command_move.head(2) = (*controller).controlHorizontalVelocity(command_move.head(2), velocity.head(2), acceleration.head(2));
	// Select control input (priority: skycontroller > keyboard > offboard)
	command_move << (mode_skycontroller(0) != COMMAND_NONE ? command_skycontroller(0) : (mode_keyboard(0) != COMMAND_NONE ? command_keyboard(0) : (mode_offboard(0) != COMMAND_NONE ? command_offboard(0) : 0))),
					(mode_skycontroller(0) != COMMAND_NONE ? command_skycontroller(1) : (mode_keyboard(0) != COMMAND_NONE ? command_keyboard(1) : (mode_offboard(0) != COMMAND_NONE ? command_offboard(1) : 0))),
					(mode_skycontroller(1) != COMMAND_NONE ? command_skycontroller(2) : (mode_keyboard(1) != COMMAND_NONE ? command_keyboard(2) : (mode_offboard(1) != COMMAND_NONE ? command_offboard(2) : 0))),
					(mode_skycontroller(2) != COMMAND_NONE ? command_skycontroller(3) : (mode_keyboard(2) != COMMAND_NONE ? command_keyboard(3) : (mode_offboard(2) != COMMAND_NONE ? command_offboard(3) : 0)));	
	Vector3i mode_move;
	// Select control mode (priority: skycontroller > keyboard > offboard)
	mode_move << 	(mode_skycontroller(0) != COMMAND_NONE ? mode_skycontroller(0) : (mode_keyboard(0) != COMMAND_NONE ? mode_keyboard(0) : (mode_offboard(0) != COMMAND_NONE ? mode_offboard(0) : (!isnan(velocity(0)) ? COMMAND_VELOCITY : COMMAND_ATTITUDE)))),
				 	(mode_skycontroller(1) != COMMAND_NONE ? mode_skycontroller(1) : (mode_keyboard(1) != COMMAND_NONE ? mode_keyboard(1) : (mode_offboard(1) != COMMAND_NONE ? mode_offboard(1) : COMMAND_VELOCITY))),
				 	(mode_skycontroller(2) != COMMAND_NONE ? mode_skycontroller(2) : (mode_keyboard(2) != COMMAND_NONE ? mode_keyboard(2) : (mode_offboard(2) != COMMAND_NONE ? mode_offboard(2) : COMMAND_RATE)));
					
	geometry_msgs::msg::Vector3Stamped mode_msg; // FOR DEBUG
	mode_msg.header.stamp = this->get_clock()->now(); // FOR DEBUG
	mode_msg.vector.x = mode_move(0); // FOR DEBUG
	mode_msg.vector.y = mode_move(1); // FOR DEBUG
	mode_msg.vector.z = mode_move(2); // FOR DEBUG
	mode_publisher->publish(mode_msg); // FOR DEBUG
	
	if(!world_frame)
		yaw = yaw - yaw_offset;

	anafi_ros_interfaces::msg::PilotingCommand rpyg_msg;
	rpyg_msg.header.frame_id = "body";
	
	switch(mode_move(MODE_HORIZONTAL)){ // horizotal
	case COMMAND_NONE: // no command
		rpyg_msg.roll = 0;
		rpyg_msg.pitch = 0;
		break;
	case COMMAND_POSITION: // position
		if(!isnan(position(0)) && !isnan(position(1)) && !isnan(velocity(0)) && !isnan(velocity(1))){
			command_move(0) = BOUND(command_move(0), bounds(0,0), bounds(0,1));
			command_move(1) = BOUND(command_move(1), bounds(1,0), bounds(1,1));

			//command_move.head(2) = (*controller).controlHorizontalPosition(command_move.head(2), position.head(2), derivative_command.head(2), velocity.head(2), dt(0));
		
			Vector2d position_error;
			Vector2d position_error_d;
			position_error = command_move.head(2) - position.head(2);
			position_error_d = derivative_command.head(2) - rotateVector(velocity.head(2), -yaw); // rotate velocity from body frame to world frame
						
			position_error_i(0) = BOUND(position_error_i(0) + position_error(0)*dt(0), max_position_i);
			position_error_i(1) = BOUND(position_error_i(1) + position_error(1)*dt(0), max_position_i);

			command_move(0) = k_position_p*position_error(0) + k_position_i*position_error_i(0) + k_position_d*position_error_d(0);
			command_move(1) = k_position_p*position_error(1) + k_position_i*position_error_i(1) + k_position_d*position_error_d(1);
		}
		else{
			command_move(0) = 0;
			command_move(1) = 0;
			RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *get_clock(), 1000, "No position feedback.");
		}
		[[fallthrough]];
	case COMMAND_VELOCITY: // velocity
		if(!isnan(velocity(0)) && !isnan(velocity(1)) && !isnan(acceleration(0)) && !isnan(acceleration(1))){
			double v = sqrt(pow(command_move(0), 2) + pow(command_move(0), 2));
			if(v > max_horizontal_speed)
				command_move.head(2) = command_move.head(2)/v*max_horizontal_speed; // bound velocity

			if(fixed_frame || mode_move(MODE_HORIZONTAL) == COMMAND_POSITION)
				command_move.head(2) = rotateVector(command_move.head(2), yaw); // rotate velocity from world frame to body frame

			//command_move.head(2) = (*controller).controlHorizontalVelocity(command_move.head(2), velocity.head(2), acceleration.head(2));
			
			Vector2d velocity_error(command_move(0) - velocity(0), command_move(1) - velocity(1));
			Vector2d velocity_error_d(-acceleration(0), -acceleration(1));
	
			command_move(0) = -(k_velocity_p*velocity_error(1) + k_velocity_d*velocity_error_d(1));
			command_move(1) =   k_velocity_p*velocity_error(0) + k_velocity_d*velocity_error_d(0);
		}else{
			command_move(0) = 0;
			command_move(1) = 0;
			RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *get_clock(), 1000, "No speed feedback.");
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
		if(!isnan(position(2)) && !isnan(velocity(2))){
			command_move(2) = BOUND(command_move(2), bounds(2,0), bounds(2,1));

			//command_move(2) = (*controller).controlVerticalPosition(command_move(2), position(2), derivative_command(2), velocity(2), dt(1));

			double position_error = command_move(2) - position(2);
			double position_error_d = derivative_command(2) - velocity(2);
			position_error_i(2) = BOUND(position_error_i(2) + position_error*dt(1), max_position_i);
			command_move(2) = k_position_p*position_error + k_position_i*position_error_i(2) + k_position_d*position_error_d;
		}else{
			command_move(2) = 0;
			RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *get_clock(), 1000, "No altitude feedback.");
		}
		[[fallthrough]];
	case COMMAND_VELOCITY: // velocity
		command_move(2) = BOUND(command_move(2), max_vertical_speed);
		if(!isnan(position(2))){ // bound velocity to stay in the safe area
			if(position(2) <= bounds(2,0) && command_move(2) < 0){
				command_move(2) = 0;
				RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *get_clock(), 10000, "The drone is below the minimum altitude (" << bounds(2,0) << "m).");
			}
			if(position(2) >= bounds(2,1) && command_move(2) > 0){
				command_move(2) = 0;
				RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *get_clock(), 10000, "The drone is above the maximum altitude (" << bounds(2,1) << "m).");
			}
		}
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
		if(!isnan(yaw)){
			//command_move(3) = (*controller).controlYawOrientation(command_move(3), yaw, 0, 0);

			yaw = denormalizeAngle(yaw, command_move(3));
			command_move(3) = k_yaw_p*(command_move(3) - yaw);
		}else{
			command_move(3) = 0;
			RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *get_clock(), 1000, "No attitude feedback.");
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

	rpyg_msg.header.stamp = this->get_clock()->now();
	rpyg_publisher->publish(rpyg_msg);
}

void Autonomy::controllerCamera(){
	// Move gimbal
	Vector3d gimbal_command;
	gimbal_command <<	(controller_gimbal_command(0) != 0 ?  controller_gimbal_command(0) : (keyboard_gimbal_command(0) != 0 ? keyboard_gimbal_command(0) : offboard_gimbal_command(0))),
						(controller_gimbal_command(1) != 0 ? -controller_gimbal_command(1) : (keyboard_gimbal_command(1) != 0 ? keyboard_gimbal_command(1) : offboard_gimbal_command(1))),
						(controller_gimbal_command(2) != 0 ? -controller_gimbal_command(2) : (keyboard_gimbal_command(2) != 0 ? keyboard_gimbal_command(2) : offboard_gimbal_command(2)));

	anafi_ros_interfaces::msg::GimbalCommand gimbal_msg;
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
	double zoom_command = (controller_zoom_command != 0 ? controller_zoom_command : (keyboard_zoom_command != 0 ? keyboard_zoom_command : offboard_zoom_command));

	anafi_ros_interfaces::msg::CameraCommand camera_msg;
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
}

double Autonomy::denormalizeAngle(double a1, double a2){
	while(abs(a1 - a2) > M_PI)
		a1 += (a1 < a2) ? 2*M_PI : -2*M_PI;
	return a1;
}

Vector2d Autonomy::rotateVector(Vector2d v, double a){
	Vector2d v_rotated;
	v_rotated << cos(a)*v(0) + sin(a)*v(1), -sin(a)*v(0) + cos(a)*v(1);
	return v_rotated;
}

int main(int argc, char** argv){	
	rclcpp::init(argc, argv);
	
	rclcpp::spin(std::make_shared<Autonomy>());
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Autonomy is stopping...");

	rclcpp::shutdown();
	return 0;
}