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

// ********************** Callbacks **********************
/*void dynamicReconfigureCallback(anafi_autonomy::setSafeAnafiConfig &config, uint32_t level){
	if(level == -1 || level == 1){
		hand_launch = config.hand_launch;
		takingoff_control = config.takingoff_control;
		landing_control = config.landing_control;
		fixed_frame = config.fixed_frame;
		world_frame = config.world_frame;
		if(!world_frame)
			initial_yaw = yaw;
	}
	if(level == -1 || level == 2){ // mission
		mission_type = config.mission_type;
		flightplan_file = config.flightplan_file;
		followme_mode = config.followme_mode;
	}
	if(level == -1 || level == 3){ // gains
		k_p_position = config.k_p_position;
		k_i_position = config.k_i_position;
		k_d_position = config.k_d_position;
		max_i_position = config.max_i_position;
		k_p_velocity = config.k_p_velocity;
		k_d_velocity = config.k_d_velocity;
		k_p_yaw = config.k_p_yaw;
	}
}*/


// Constructor
SafeAnafi::SafeAnafi() : Node("safe_anafi"){
	RCLCPP_INFO(this->get_logger(), "SafeAnafi is running...");

	action_subscriber = this->create_subscription<std_msgs::msg::Int8>("keyboard/action", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::actionCallback, this, _1));
	command_skycontroller_subscriber = this->create_subscription<olympe_bridge_interfaces::msg::SkycontrollerCommand>("skycontroller/command", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::skycontrollerCallback, this, _1));
	command_keyboard_subscriber = this->create_subscription<anafi_autonomy::msg::KeyboardDroneCommand>("keyboard/drone_command", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::keyboardCallback, this, _1));
	command_camera_subscriber = this->create_subscription<anafi_autonomy::msg::KeyboardCameraCommand>("keyboard/camera_command", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::cameraCallback, this, _1));
	desired_pose_subscriber = this->create_subscription<anafi_autonomy::msg::PoseCommand>("drone/desired_pose", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::poseCallback, this, _1));
	desired_velocity_subscriber = this->create_subscription<anafi_autonomy::msg::VelocityCommand>("drone/desired_velocity", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::velocityCallback, this, _1));
	desired_attitude_subscriber = this->create_subscription<anafi_autonomy::msg::AttitudeCommand>("drone/desired_attitude", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::attitudeCallback, this, _1));
	desired_trajectory_subscriber = this->create_subscription<anafi_autonomy::msg::PoseCommand>("drone/desired_trajectory", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::trajectoryCallback, this, _1));
	desired_axis_subscriber = this->create_subscription<anafi_autonomy::msg::DesiredCommand>("drone/desired_command", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::commandCallback, this, _1));
	desired_gimbal_subscriber = this->create_subscription<geometry_msgs::msg::Vector3>("drone/desired_gimbal", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::gimbalCallback, this, _1));
	desired_zoom_subscriber = this->create_subscription<std_msgs::msg::Float32>("drone/desired_zoom", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::zoomCallback, this, _1));
	state_subscriber = this->create_subscription<std_msgs::msg::String>("drone/state", rclcpp::SystemDefaultsQoS(), std::bind(&SafeAnafi::stateCallback, this, _1));
	gps_subscriber = this->create_subscription<sensor_msgs::msg::NavSatFix>("drone/gps/location", rclcpp::SensorDataQoS(), std::bind(&SafeAnafi::gpsCallback, this, _1));
	odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("drone/odometry", rclcpp::SensorDataQoS(), std::bind(&SafeAnafi::odometryCallback, this, _1));
	optitrack_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/optitrack/odometry", rclcpp::SensorDataQoS(), std::bind(&SafeAnafi::optitrackCallback, this, _1));
	
	rpyg_publisher = this->create_publisher<olympe_bridge_interfaces::msg::PilotingCommand>("drone/rpyt", rclcpp::SystemDefaultsQoS());
	moveto_publisher = this->create_publisher<olympe_bridge_interfaces::msg::MoveToCommand>("drone/moveto", rclcpp::SystemDefaultsQoS());
	moveby_publisher = this->create_publisher<olympe_bridge_interfaces::msg::MoveByCommand>("drone/moveby", rclcpp::SystemDefaultsQoS());
	camera_publisher = this->create_publisher<olympe_bridge_interfaces::msg::CameraCommand>("camera/cmd", rclcpp::SystemDefaultsQoS());
	gimbal_publisher = this->create_publisher<olympe_bridge_interfaces::msg::GimbalCommand>("gimbal/cmd", rclcpp::SystemDefaultsQoS());
	odometry_publisher = this->create_publisher<nav_msgs::msg::Odometry>("drone/odometry", rclcpp::SensorDataQoS());
	desired_velocity_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("drone/debug/desired_velocity", rclcpp::SystemDefaultsQoS());
	acceleration_publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("drone/debug/acceleration", rclcpp::SystemDefaultsQoS());
	mode_publisher = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("drone/debug/mode", rclcpp::SystemDefaultsQoS());

	emergency_client = this->create_client<std_srvs::srv::Trigger>("drone/emergency");
	halt_client = this->create_client<std_srvs::srv::Trigger>("drone/halt");
	takeoff_client = this->create_client<std_srvs::srv::Trigger>("drone/takeoff");
	arm_client = this->create_client<std_srvs::srv::SetBool>("drone/arm");
	land_client = this->create_client<std_srvs::srv::Trigger>("drone/land");
	rth_client = this->create_client<std_srvs::srv::Trigger>("drone/rth");
	flightplan_upload_client = this->create_client<olympe_bridge_interfaces::srv::FlightPlan>("drone/flightplan_upload");
	flightplan_start_client = this->create_client<olympe_bridge_interfaces::srv::FlightPlan>("drone/flightplan_start");
	flightplan_pause_client = this->create_client<std_srvs::srv::Trigger>("drone/flightplan_pause");
	flightplan_stop_client = this->create_client<std_srvs::srv::Trigger>("drone/flightplan_stop");
	followme_start_client = this->create_client<olympe_bridge_interfaces::srv::FollowMe>("drone/followme_start");
	followme_stop_client = this->create_client<std_srvs::srv::Trigger>("drone/followme_stop");
	offboard_client = this->create_client<std_srvs::srv::SetBool>("skycontroller/offboard");
	calibrate_magnetometer_client = this->create_client<std_srvs::srv::Trigger>("drone/calibrate");
	calibrate_gimbal_client = this->create_client<std_srvs::srv::Trigger>("gimbal/calibrate");
	take_photo_client = this->create_client<std_srvs::srv::Trigger>("camera/take_photo");
	start_recording_client = this->create_client<std_srvs::srv::Trigger>("camera/start_recording");
	stop_recording_client = this->create_client<std_srvs::srv::Trigger>("camera/stop_recording");
	reset_zoom_client = this->create_client<std_srvs::srv::Trigger>("camera/reset");
	reset_gimbal_client = this->create_client<std_srvs::srv::Trigger>("gimbal/reset");
	download_media_client = this->create_client<std_srvs::srv::Trigger>("drone/download_media");
	reboot_client = this->create_client<std_srvs::srv::Trigger>("drone/reboot");
	
	trigger_request = std::make_shared<std_srvs::srv::Trigger::Request>();
	false_request = std::make_shared<std_srvs::srv::SetBool::Request>();
	true_request = std::make_shared<std_srvs::srv::SetBool::Request>();
	
	false_request->data = false;
	true_request->data = true;

	this->declare_parameter<double>("safe_anafi/bounds/min_x", 0);
	this->declare_parameter<double>("safe_anafi/bounds/min_y", 0);
	this->declare_parameter<double>("safe_anafi/bounds/min_z", 0);
	this->declare_parameter<double>("safe_anafi/bounds/max_x", 0);
	this->declare_parameter<double>("safe_anafi/bounds/max_y", 0);
	this->declare_parameter<double>("safe_anafi/bounds/max_z", 0);
	/*this->declare_parameter<double>("anafi/max_vertical_speed", 0);
	this->declare_parameter<double>("anafi/max_yaw_rotation_speed", 0);
	this->declare_parameter<double>("anafi/max_horizontal_speed", 0);
	this->declare_parameter<double>("anafi/max_vertical_speed", 0);
	this->declare_parameter<double>("anafi/max_yaw_rotation_speed", 0);
	this->declare_parameter<double>("anafi/max_horizontal_speed", 0);
	this->declare_parameter<double>("anafi/max_horizontal_speed", 0);
	this->declare_parameter<double>("anafi/max_tilt", 0);
	this->declare_parameter<double>("anafi/max_vertical_speed", 0);
	this->declare_parameter<double>("anafi/max_yaw_rotation_speed", 0);*/
	
	this->get_parameter("safe_anafi/bounds/min_x", bounds(0,0));
	this->get_parameter("safe_anafi/bounds/min_y", bounds(0,1));
	this->get_parameter("safe_anafi/bounds/min_z", bounds(0,2));
	this->get_parameter("safe_anafi/bounds/max_x", bounds(1,0));
	this->get_parameter("safe_anafi/bounds/max_y", bounds(1,1));
	this->get_parameter("safe_anafi/bounds/max_z", bounds(1,2));
	
	geometry_msgs::msg::Twist t;
	velocities.clear();
	for(int i = 0; i < FILTER_SIZE; ++i)
		velocities.push_back(t);
				
	timer = this->create_wall_timer(10ms, std::bind(&SafeAnafi::timer_callback, this));
}

void SafeAnafi::timer_callback(){
	stateMachine();

	// Move gimbal
	gimbal_command << (controller_gimbal_command(0) != 0 ?  controller_gimbal_command(0) : (keyboard_gimbal_command(0) != 0 ? keyboard_gimbal_command(0) : offboard_gimbal_command(0))),
                          (controller_gimbal_command(1) != 0 ? -controller_gimbal_command(1) : (keyboard_gimbal_command(1) != 0 ? keyboard_gimbal_command(1) : offboard_gimbal_command(1)));

	olympe_bridge_interfaces::msg::GimbalCommand gimbal_msg;
	gimbal_msg.header.stamp = this->get_clock()->now();
	gimbal_msg.header.frame_id = "body";
	gimbal_msg.mode = 1;
	if(!gimbal_command.isZero()){
		gimbal_msg.roll = gimbal_command(0);
		gimbal_msg.pitch = gimbal_command(1);
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
	this->get_parameter("anafi/max_vertical_speed", max_vertical_speed);
	this->get_parameter("anafi/max_yaw_rotation_speed", max_yaw_rotation_speed);
	this->get_parameter("anafi/max_horizontal_speed", max_horizontal_speed);
	command_skycontroller << max_horizontal_speed/100*command_msg.x, -max_horizontal_speed/100*command_msg.y, max_vertical_speed/100*command_msg.z, -max_yaw_rotation_speed/100*command_msg.yaw;
	mode_skycontroller << ((command_msg.x != 0 || command_msg.y != 0) ? COMMAND_VELOCITY : COMMAND_NONE), (command_msg.z != 0 ? COMMAND_VELOCITY : COMMAND_NONE),
			(command_msg.yaw != 0 ? COMMAND_RATE : COMMAND_NONE);
	
	// Move
	controller_gimbal_command << 0, (float)command_msg.camera/100;

	// Change zoom
	controller_zoom_command = -(float)command_msg.zoom/100;

	// Swithch between manual and offboard
	if(command_msg.reset_camera)
		offboard_client->async_send_request(false_request);
	if(command_msg.reset_zoom)
		offboard_client->async_send_request(true_request);
}

void SafeAnafi::keyboardCallback(const anafi_autonomy::msg::KeyboardDroneCommand& command_msg){
	this->get_parameter("anafi/max_vertical_speed", max_vertical_speed);
	this->get_parameter("anafi/max_yaw_rotation_speed", max_yaw_rotation_speed);
	this->get_parameter("anafi/max_horizontal_speed", max_horizontal_speed);
	command_keyboard << max_horizontal_speed*command_msg.x, max_horizontal_speed*command_msg.y, max_vertical_speed*command_msg.z, max_yaw_rotation_speed*command_msg.yaw;
	mode_keyboard << ((command_msg.x != 0 || command_msg.y != 0) ? COMMAND_VELOCITY : COMMAND_NONE), (command_msg.z != 0 ? COMMAND_VELOCITY : COMMAND_NONE),
			(command_msg.yaw != 0 ? COMMAND_RATE : COMMAND_NONE);
}

void SafeAnafi::cameraCallback(const anafi_autonomy::msg::KeyboardCameraCommand& command_msg){
	keyboard_gimbal_command << command_msg.roll, command_msg.pitch;
	keyboard_zoom_command = command_msg.zoom;

	switch(command_msg.action){
	case 1:
		take_photo_client->async_send_request(trigger_request);
		break;
	case 2:
		start_recording_client->async_send_request(trigger_request);
		break;
	case 3:
		stop_recording_client->async_send_request(trigger_request);
		break;
	case 4:
		download_media_client->async_send_request(trigger_request);
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

void SafeAnafi::poseCallback(const anafi_autonomy::msg::PoseCommand& command_msg){
	command_offboard << command_msg.x, command_msg.y, command_msg.z, command_msg.yaw*M_PI/180;
	mode_offboard << COMMAND_POSITION, COMMAND_POSITION, COMMAND_ANGLE;
}

void SafeAnafi::velocityCallback(const anafi_autonomy::msg::VelocityCommand& command_msg){
	command_offboard << command_msg.vx, command_msg.vy, command_msg.vz, command_msg.yaw_rate;
	mode_offboard << COMMAND_VELOCITY, COMMAND_VELOCITY, COMMAND_RATE;
}

void SafeAnafi::attitudeCallback(const anafi_autonomy::msg::AttitudeCommand& command_msg){
	command_offboard << command_msg.roll*M_PI/180, command_msg.pitch*M_PI/180, command_msg.trottle, command_msg.yaw*M_PI/180;
	mode_offboard << COMMAND_ANGLE, COMMAND_VELOCITY, COMMAND_ANGLE;
}

void SafeAnafi::trajectoryCallback(const anafi_autonomy::msg::PoseCommand& command_msg){ //TODO: Implement this function
	//desired_pose << command_msg.x, command_msg.y, command_msg.z, command_msg.yaw*M_PI/180;
	//mode_offboard << COMMAND_POSITION, COMMAND_POSITION, COMMAND_ANGLE;
}

void SafeAnafi::commandCallback(const anafi_autonomy::msg::DesiredCommand& command_msg){
	command_offboard << (command_msg.horizontal_mode != COMMAND_NONE ? command_msg.x : command_offboard(0)), (command_msg.horizontal_mode != COMMAND_NONE ? command_msg.y : command_offboard(1)),
			(command_msg.vertical_mode != COMMAND_NONE ? command_msg.z : command_offboard(2)), (command_msg.heading_mode != COMMAND_NONE ? command_msg.yaw*M_PI/180 : command_offboard(3));
	mode_offboard << command_msg.horizontal_mode, command_msg.vertical_mode, command_msg.heading_mode;
}

void SafeAnafi::gimbalCallback(const geometry_msgs::msg::Vector3& command_msg){
	offboard_gimbal_command << command_msg.x, command_msg.y;
}

void SafeAnafi::zoomCallback(const std_msgs::msg::Float32& command_msg){
	offboard_zoom_command = command_msg.data;
}

void SafeAnafi::stateCallback(const std_msgs::msg::String& state_msg){ // 'LANDED', 'MOTOR_RAMPING', 'USER_TAKEOFF', 'TAKINGOFF', 'HOVERING', 'FLYING', 'LANDING', 'EMERGENCY'
	state = resolveState(state_msg.data);
}

void SafeAnafi::gpsCallback(const sensor_msgs::msg::NavSatFix& gps_msg){ //TODO: Implement this function

}

void SafeAnafi::odometryCallback(const nav_msgs::msg::Odometry& odometry_msg){
	time = rclcpp::Time(odometry_msg.header.stamp.sec, odometry_msg.header.stamp.nanosec);
	dt = (time - time_old).nanoseconds()/1e9;
	
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
	time_old = time;
}

void SafeAnafi::optitrackCallback(const geometry_msgs::msg::PoseStamped& optitrack_msg){
	time = rclcpp::Time(optitrack_msg.header.stamp.sec, optitrack_msg.header.stamp.nanosec);
	dt = (time - time_old).nanoseconds()/1e9;
	
	nav_msgs::msg::Odometry odometry_msg;
	odometry_msg.header.stamp = optitrack_msg.header.stamp;
	odometry_msg.header.frame_id = optitrack_msg.header.frame_id;
	odometry_msg.child_frame_id = "body";

	odometry_msg.pose.pose.position.x = optitrack_msg.pose.position.x;
	odometry_msg.pose.pose.position.y = optitrack_msg.pose.position.y;
	odometry_msg.pose.pose.position.z = optitrack_msg.pose.position.z;

	tf2::Quaternion q(optitrack_msg.pose.orientation.x, optitrack_msg.pose.orientation.y, optitrack_msg.pose.orientation.z, optitrack_msg.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch;
	m.getRPY(roll, pitch, yaw);
	odometry_msg.pose.pose.orientation = optitrack_msg.pose.orientation;

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

	time_old = time;
}

void SafeAnafi::stateMachine(){
	// State-indipendent actions
	switch(action){
	case DISARM: // emergency
		emergency_client->async_send_request(trigger_request);
		return;
	case RESET_POSE:
		RCLCPP_INFO(this->get_logger(), "Reseting position and heading");
		position << 0, 0, 0;
		initial_yaw = yaw;
		return;
	case REMOTE_CONTROL:
		offboard_client->async_send_request(false_request);
		return;
	case OFFBOARD_CONTROL:
		offboard_client->async_send_request(true_request);
		return;
	}

	switch(state){
	case LANDED:
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
					flightplan_request->filepath = flightplan_file;
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
		}
		break;
	case MOTOR_RAMPING:
		switch(action){
		case ARM: // disarm
		case LAND:
		case HALT:
			arm_client->async_send_request(false_request);
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
				flightplan_request->filepath = flightplan_file;
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
		}
	case INVALID:
		switch(action){
		case HALT:
			takeoff_client->async_send_request(trigger_request);
			break;
		case REBOOT:
			reboot_client->async_send_request(trigger_request);
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
	mode_move << 	(mode_skycontroller(0) != COMMAND_NONE ? mode_skycontroller(0) : (mode_keyboard(0) != COMMAND_NONE ? mode_keyboard(0) : (mode_offboard(0) != COMMAND_NONE ? mode_offboard(0) : COMMAND_VELOCITY))),
	             	(mode_skycontroller(1) != COMMAND_NONE ? mode_skycontroller(1) : (mode_keyboard(1) != COMMAND_NONE ? mode_keyboard(1) : (mode_offboard(1) != COMMAND_NONE ? mode_offboard(1) : COMMAND_VELOCITY))),
	             	(mode_skycontroller(2) != COMMAND_NONE ? mode_skycontroller(2) : (mode_keyboard(2) != COMMAND_NONE ? mode_keyboard(2) : (mode_offboard(2) != COMMAND_NONE ? mode_offboard(2) : COMMAND_RATE)));

	switch(mode_move(MODE_HORIZONTAL)){ // horizotal
	case COMMAND_NONE: // no command
		rpyg_msg.roll = 0;
		rpyg_msg.pitch = 0;
		break;
	case COMMAND_POSITION: // position
		if(localization){
			command_move(0) = BOUND(command_move(0), bounds(0,0), bounds(1,0));
			command_move(1) = BOUND(command_move(1), bounds(0,1), bounds(1,1));
			// TODO: IMPLEMENT POSITION CONTROLLER HERE
		}
		else{
			command_move(0) = 0;
			command_move(1) = 0;
			// TODO: THROW A WARNING, IF NO LOCALISATION
		}
	case COMMAND_VELOCITY: // velocity
		this->get_parameter("anafi/max_horizontal_speed", max_horizontal_speed);
		command_move(0) = BOUND(command_move(0), max_horizontal_speed);
		command_move(1) = BOUND(command_move(1), max_horizontal_speed);

		if(fixed_frame)
			command_move << cos(yaw)*command_move(0) + sin(yaw)*command_move(1), -sin(yaw)*command_move(0) + cos(yaw)*command_move(1), command_move(2), command_move(3); // rotate command from world frame to body frame

		velocity_error << command_move(0) - velocity(0), command_move(1) - velocity(1), 0;
		velocity_error_d = -acceleration;

		command_move(0) = -(k_p_velocity*velocity_error(1) + k_d_velocity*velocity_error_d(1));
		command_move(1) =   k_p_velocity*velocity_error(0) + k_d_velocity*velocity_error_d(0);
	case COMMAND_ANGLE: // attitude
		this->get_parameter("anafi/max_tilt", max_tilt);
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
		command_move(2) = BOUND(command_move(2), bounds(0,2), bounds(1,2));
		position_error(2) = command_move(2) - position(2);
		position_error_d(2) = -velocity(2);
		position_error_i(2) = BOUND(position_error_i(2) + position_error(2)*dt, max_i_position);
		command_move(2) = k_p_position*position_error(2) + k_i_position*position_error_i(2) + k_d_position*position_error_d(2);
	case COMMAND_VELOCITY: // velocity
		this->get_parameter("anafi/max_vertical_speed", max_vertical_speed);
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
	case COMMAND_ANGLE: // attitude
		yaw = denormalizeAngle(yaw, command_move(3));
		command_move(3) = k_p_yaw*(command_move(3) - yaw);
	case COMMAND_RATE: // angular rate
		this->get_parameter("anafi/max_yaw_rotation_speed", max_yaw_rotation_speed);
		command_move(3) = BOUND(command_move(3), max_yaw_rotation_speed);
		rpyg_msg.yaw = command_move(3);
		break;
	default:
		rpyg_msg.yaw = 0;
		RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *get_clock(), 1000, "Undefined heading mode (" << mode_move(MODE_HEADING) << ").");
		RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *get_clock(), 1000, "It should be '0' for no command, '3' for yaw command or '4' for rate command.");
	}

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
	/*rpyg_msg.roll = 0;
	rpyg_msg.pitch = 0;
	rpyg_msg.yaw = 0;
	rpyg_msg.gaz = 0;
	rpyg_publisher->publish(rpyg_msg);
	rpyg_publisher->publish(rpyg_msg);
	land_client->async_send_request(trigger_request);
	land_client->async_send_request(trigger_request);*/
	rclcpp::shutdown();
	return 0;
}
