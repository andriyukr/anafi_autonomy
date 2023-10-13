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
	command_publisher = this->create_publisher<anafi_autonomy::msg::ReferenceCommand>("drone/reference_command", rclcpp::SystemDefaultsQoS());
	derivative_publisher = this->create_publisher<anafi_autonomy::msg::VelocityCommand>("drone/derivative_command", rclcpp::SystemDefaultsQoS());

	// Parameters
	callback = this->add_on_set_parameters_callback(std::bind(&Trajectory::parameterCallback, this, std::placeholders::_1));

	rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
	rcl_interfaces::msg::IntegerRange integer_range;
	rcl_interfaces::msg::FloatingPointRange floating_point_range;

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor();
	parameter_descriptor.description = 
		"Trajectory type: "
		"0 = no trajectory, "
		"1 = hover at (0, 0, 1, yaw_d), "
		"2 = defined by user with (x_d, y_d, z_d, yaw_d), "
		"3 = waipoints from file, "
		"4 = circle"
	;
	integer_range = rcl_interfaces::msg::IntegerRange();
	integer_range.from_value = 0;
	integer_range.to_value = 4;
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

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor();
	parameter_descriptor.description = "Desired speed over trajectory";
	floating_point_range = rcl_interfaces::msg::FloatingPointRange();
	floating_point_range.from_value = 0.0;
	floating_point_range.to_value = 10.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("desired/speed", 1.0, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor();
	parameter_descriptor.description = "Trajectory scale";
	floating_point_range = rcl_interfaces::msg::FloatingPointRange();
	floating_point_range.from_value = 0.0;
	floating_point_range.to_value = 10.0;
	floating_point_range.step = 0.0;
	parameter_descriptor.floating_point_range.push_back(floating_point_range);
	this->declare_parameter("scale", 1.0, parameter_descriptor);

	parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
	parameter_descriptor.description = "Absolute path to the waypoints file";
	parameter_descriptor.read_only = true;
	std::string package_path = ament_index_cpp::get_package_share_directory("anafi_autonomy");
	this->declare_parameter("waypoints_file", package_path + "/missions/waypoints.txt", parameter_descriptor); // TODO: get the name of the file as parameter from launch file

	if(file_waypoints != "")	
		readWaypoints(file_waypoints);

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
		if(parameter.get_name() == "desired/speed")
			speed = parameter.as_double();
		if(parameter.get_name() == "scale")
			scale = parameter.as_double();
		if(parameter.get_name() == "waypoints_file")
			file_waypoints = parameter.as_string();
	}

	changed = true;
	initial_t = this->get_clock()->now().nanoseconds()/1e9;

	return result;
}

void Trajectory::timer_callback(){
	anafi_autonomy::msg::ReferenceCommand command_msg;
	anafi_autonomy::msg::VelocityCommand derivative_msg;

	double t = this->get_clock()->now().nanoseconds()/1e9 - initial_t;

	switch(trajectory_type){
	case 0: // no command
		if(!changed) // publish no command only once
			return;
		command << 0, 0, 0, 0;
		derivative << 0, 0, 0, 0;
		mode << COMMAND_NONE, COMMAND_NONE, COMMAND_NONE;
		break;
	case 1: // hover
		if(!changed)
			return;
		command << 0, 0, 1, pose_d(3);
		derivative << 0, 0, 0, 0;
		mode << COMMAND_VELOCITY, COMMAND_POSITION, COMMAND_ATTITUDE;
		break;
	case 2: // user
		if(!changed)
			return;
		command = pose_d;
		derivative << 0, 0, 0, 0;
		mode << COMMAND_POSITION, COMMAND_POSITION, COMMAND_ATTITUDE;
		break;
	case 3: // waypoints
		if(t == 0)
			RCLCPP_INFO_STREAM(this->get_logger(), "waypoint #" << waypoint << ": " << ROUND(command));
		if(waypoint < waypoints.rows() - 1){
			w1 << waypoints.row(waypoint).transpose();
			w2 << waypoints.row(waypoint + 1).transpose();
			double d = distance(w1, w2);
			if(d == 0) // w1 == w2
				d = t*speed;
			command = (1 - t*speed/d)*w1 + t*speed/d*w2;
			derivative = (w2 - w1)*speed/d;
			if(t*speed >= d){
				initial_t = this->get_clock()->now().nanoseconds()/1e9;
				waypoint++;
				RCLCPP_INFO_STREAM(this->get_logger(), "waypoint #" << waypoint << ": " << w2.transpose());
			}
		}
		else{
			command << waypoints.bottomRows(1).transpose();
			derivative << 0, 0, 0, 0;
		}
		mode << COMMAND_POSITION, COMMAND_POSITION, COMMAND_ATTITUDE;
		break;
	case 4: // circle
		command << scale*cos(t*speed/sqrt(2)/scale) + pose_d(0), scale*sin(t*speed/sqrt(2)/scale) + pose_d(1), pose_d(2), t*speed/sqrt(2)/scale/M_PI*180 + pose_d(3);
		derivative << -speed/sqrt(2)*sin(speed*t/scale), speed/sqrt(2)*cos(speed*t/scale), 0, 0;
		mode << COMMAND_POSITION, COMMAND_POSITION, COMMAND_ATTITUDE;
		break;
	case 5: // controller tunning
		command = pose_d;
		mode << COMMAND_VELOCITY, COMMAND_VELOCITY, COMMAND_ATTITUDE;
		break;
	}

	changed = false;

	while(abs(command(3)) > 180)
		command(3) += (command(3) < 0) ? 360 : -360;

	// Publish the corresponding reference for each axis
	command_msg.header.stamp = this->get_clock()->now();
	command_msg.horizontal_mode = mode(0);
	command_msg.x = command(0);
	command_msg.y = command(1);
	command_msg.vertical_mode = mode(1);
	command_msg.z = command(2);
	command_msg.heading_mode = mode(2);
	command_msg.yaw = command(3);
	command_publisher->publish(command_msg);

	// Publish the corresponding reference trajectory velocity
	derivative_msg.header.stamp = this->get_clock()->now();
	derivative_msg.vx = derivative(0);
	derivative_msg.vy = derivative(1);
	derivative_msg.vz = derivative(2);
	derivative_msg.yaw_rate = derivative(3);
	derivative_publisher->publish(derivative_msg);
}

// Function to calculate distance between two waypoints
double Trajectory::distance(Vector4d v1, Vector4d v2){
	return sqrt(pow(v1(0) - v2(0), 2) + pow(v1(1) - v2(1), 2) + pow(v1(2) - v2(2), 2));
}

// Reads the file with waypoints
void Trajectory::readWaypoints(std::string fileName){
    string line;
    ifstream myfile(fileName);
    if(myfile.is_open()){       
        int points = 0;
        while(getline(myfile, line))
            ++points;
        waypoints = MatrixXd(points, 4);

        ifstream myfile(fileName);
        getline(myfile, line); // reads out the header
        waypoints.row(0) << 0, 0, 1, 0; // TODO: replace with the current pose
        for(int i = 1; getline(myfile, line); ++i){
            string delimiter = "\t";
            size_t pos = 0;
            for(int j = 0; (pos = line.find(delimiter)) != string::npos; ++j) {
                waypoints(i, j) = atof(line.substr(0, pos).c_str());
                line.erase(0, pos + delimiter.length());
            }
            waypoints(i, 3) = atof(line.c_str())/180*M_PI;
        }
        myfile.close();

        RCLCPP_INFO_STREAM(this->get_logger(), "File with waypoints loaded: " << fileName);
        RCLCPP_DEBUG_STREAM(this->get_logger(), "waypoints:\n" << waypoints);
    }
    else
        RCLCPP_WARN_STREAM(this->get_logger(), "Unable to open file with waypoints: " << fileName);
}

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	
	rclcpp::spin(std::make_shared<Trajectory>());
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trajectory is stopping...");
	rclcpp::shutdown();
	return 0;
}
