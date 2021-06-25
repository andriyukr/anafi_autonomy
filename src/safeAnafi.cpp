/** *************************** safeAnafi.cpp ***************************
 *
 * This node collects all the messages:
 *      - Feedback from the UAV: state estimates for odometry
 *      - Reference trajectory: desired position and velocity from the trajectory generator
 *      - UAV commands: position, velocity and attitude commands from PID or ANN/FNN-PD
 *      - Keyboard input commands: commands from user by keyboard input
 *      - Constraints and safety: contains constraints and safety bounds for commands
 *      - Controller: lets the user choose between position/velocity/attitude commands
 *
 * TODO's:
 *      - Check velocities frame
 *      - Tune velocity controller
 *      - Filter accelerations
 *
 * *********************************************************************/

#include "anafi_autonomy/safeAnafi.h"

Twist filter_mean_velocities(Twist v){
    velocities.erase(velocities.begin());
    velocities.push_back(v);

    Twist t;
    for(vector<Twist>::iterator i = velocities.begin(); i != velocities.end(); ++i){
        t.linear.x += i->linear.x/FILTER_SIZE;
        t.linear.y += i->linear.y/FILTER_SIZE;
        t.linear.z += i->linear.z/FILTER_SIZE;
        t.angular.x += i->angular.x/FILTER_SIZE;
        t.angular.y += i->angular.y/FILTER_SIZE;
        t.angular.z += i->angular.z/FILTER_SIZE;
    }
    return t;
}

Vector3d filter_mean_acceleration(Eigen::Ref<Eigen::VectorXd> a){
    accelerations.topRows(FILTER_SIZE - 1) = accelerations.bottomRows(FILTER_SIZE - 1);
    accelerations.row(FILTER_SIZE - 1) = a;

    return accelerations.colwise().sum()/FILTER_SIZE;
}

Vector3d filter_polinomial_acceleration(Eigen::Ref<Eigen::VectorXd> a){
    accelerations.topRows(FILTER_SIZE - 1) = accelerations.bottomRows(FILTER_SIZE - 1);
    accelerations.row(FILTER_SIZE - 1) = a;

    return accelerations.bottomRows(1);
}

// ********************** Callbacks **********************

/* Dynamic reconfigure callback function (from GUI)
 * Subscribes to the values set by user for controller type from the graphical interface
 */
void dynamicReconfigureCallback(anafi_autonomy::setSafeAnafiConfig &config, uint32_t level){
    if(level == -1 || level == 1){
    	fixed_frame = config.fixed_frame;
		world_frame = config.world_frame;
		if(!world_frame)
			initial_yaw = yaw;
    }
	if(level == -1 || level == 2){
		k_p_position = config.k_p_position;
		k_i_position = config.k_i_position;
		k_d_position = config.k_d_position;
		max_i_position = config.max_i_position;
		k_p_velocity = config.k_p_velocity;
		k_d_velocity = config.k_d_velocity;
		k_p_yaw = config.k_p_yaw;
	}
}

void odometryCallback(const nav_msgs::Odometry& odometry_msg){
    Duration dt = odometry_msg.header.stamp - time_old;
    
    position << odometry_msg.pose.pose.position.x, odometry_msg.pose.pose.position.y, odometry_msg.pose.pose.position.z;

    tf::Quaternion q(odometry_msg.pose.pose.orientation.x, odometry_msg.pose.pose.orientation.y, odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw);

	if(!world_frame)
    	yaw = yaw - initial_yaw;

    geometry_msgs::Vector3 euler_msg;
    euler_msg.x = roll/M_PI*180;
    euler_msg.y = pitch/M_PI*180;
    euler_msg.z = yaw/M_PI*180;
    euler_publisher.publish(euler_msg);

    velocity << odometry_msg.twist.twist.linear.x, odometry_msg.twist.twist.linear.y, odometry_msg.twist.twist.linear.z;

    acceleration << (velocity - velocity_old)/(dt.toNSec()/pow(10, 9));
    acceleration = filter_mean_acceleration(acceleration);

    velocity_old = velocity;
    time_old = odometry_msg.header.stamp;
}

void optitrackCallback(const geometry_msgs::PoseStamped& optitrack_msg){
    Duration dt = optitrack_msg.header.stamp - time_old;
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.seq = optitrack_msg.header.seq;
    odometry_msg.header.stamp = optitrack_msg.header.stamp;
    odometry_msg.header.frame_id = optitrack_msg.header.frame_id;
    odometry_msg.child_frame_id = "body";

    odometry_msg.pose.pose.position.x = optitrack_msg.pose.position.x;
    odometry_msg.pose.pose.position.y = optitrack_msg.pose.position.y;
    odometry_msg.pose.pose.position.z = optitrack_msg.pose.position.z;

    tf::Quaternion q(optitrack_msg.pose.orientation.x, optitrack_msg.pose.orientation.y, optitrack_msg.pose.orientation.z, optitrack_msg.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw);
    odometry_msg.pose.pose.orientation = optitrack_msg.pose.orientation;

    Twist t;
    t.linear.x = (odometry_msg.pose.pose.position.x - position(0)) / (dt.toNSec() / pow(10, 9));
    t.linear.y = (odometry_msg.pose.pose.position.y - position(1)) / (dt.toNSec() / pow(10, 9));
    t.linear.z = (odometry_msg.pose.pose.position.z - position(2)) / (dt.toNSec() / pow(10, 9));
    t.angular.x = (roll - orientation(0)) / (dt.toNSec() / pow(10, 9));
    t.angular.y = (pitch - orientation(1)) / (dt.toNSec() / pow(10, 9));
    t.angular.z = (yaw - orientation(2)) / (dt.toNSec() / pow(10, 9));
    odometry_msg.twist.twist = filter_mean_velocities(t);
    odometry_publisher.publish(odometry_msg);

    position << odometry_msg.pose.pose.position.x, odometry_msg.pose.pose.position.y, odometry_msg.pose.pose.position.z;
    orientation << roll, pitch, yaw;
    velocity << odometry_msg.twist.twist.linear.x, odometry_msg.twist.twist.linear.y, odometry_msg.twist.twist.linear.z;
    rates << odometry_msg.twist.twist.angular.x, odometry_msg.twist.twist.angular.y, odometry_msg.twist.twist.angular.z;

    time_old = odometry_msg.header.stamp;
}

void commandMetaCallback(const std_msgs::Int8& command_msg){
    std_msgs::Empty empty_msg;
    anafi_autonomy::PilotingCommand rpyg_msg;
    std_msgs::Bool offboard_msg;
    switch(command_msg.data){
    case 1: // arm
        arm = true;
        break;
    case 2: // take-off
        if(arm){
            offboard_msg.data = false;
            offboard_publisher.publish(offboard_msg);
            takeoff_publisher.publish(empty_msg);
            Duration(1).sleep();
            land = false;
        }else
            ROS_WARN("Arm the drone before taking-off.");
        arm = false;
        break;
    case 3: // hower
        rpyg_publisher.publish(rpyg_msg);
        rpyg_publisher.publish(rpyg_msg);
        ros::param::set("/trajectory/trajectory", 1);
        break;
    case 4: // land
        rpyg_publisher.publish(rpyg_msg);
        rpyg_publisher.publish(rpyg_msg);
        land_publisher.publish(empty_msg);
        land_publisher.publish(empty_msg);
        land = true;
        break;
    case 5: // disarm!
        emergency_publisher.publish(empty_msg);
        emergency_publisher.publish(empty_msg);
        land = true;
        break;
    case 6: // reset position and heading
        ROS_INFO("Reseting position and heading");
        position << 0, 0, 0;
        initial_yaw = yaw;
        break;
    case 101: // remote control!
        offboard_msg.data = false;
        offboard_publisher.publish(offboard_msg);
        break;
    case 102: // offboard control!
        offboard_msg.data = true;
        offboard_publisher.publish(offboard_msg);
        break;
    }
}

void commandKeyboardCallback(const anafi_autonomy::KeyboardMoveCommand& command_msg){
    ros::param::get("/anafi/max_vertical_speed", max_vertical_speed);
    ros::param::get("/anafi/max_yaw_rotation_speed", max_yaw_rotation_speed);
    ros::param::get("/anafi/max_horizontal_speed", max_horizontal_speed);
    command_keyboard << max_horizontal_speed*command_msg.x, max_horizontal_speed*command_msg.y, max_vertical_speed*command_msg.z, max_yaw_rotation_speed*command_msg.yaw;
    mode_keyboard << ((command_msg.x != 0 || command_msg.y != 0) ? COMMAND_VELOCITY : COMMAND_NONE), (command_msg.z != 0 ? COMMAND_VELOCITY : COMMAND_NONE),
    	(command_msg.yaw != 0 ? COMMAND_RATE : COMMAND_NONE);
}

void commandSkycontrollerCallback(const anafi_autonomy::SkyControllerCommand& command_msg){
	// Drone commands
    ros::param::get("/anafi/max_vertical_speed", max_vertical_speed);
    ros::param::get("/anafi/max_yaw_rotation_speed", max_yaw_rotation_speed);
    ros::param::get("/anafi/max_horizontal_speed", max_horizontal_speed);
    command_skycontroller << max_horizontal_speed/100*command_msg.x, -max_horizontal_speed/100*command_msg.y, max_vertical_speed/100*command_msg.z, -max_yaw_rotation_speed/100*command_msg.yaw;
    mode_skycontroller << ((command_msg.x != 0 || command_msg.y != 0) ? COMMAND_VELOCITY : COMMAND_NONE), (command_msg.z != 0 ? COMMAND_VELOCITY : COMMAND_NONE),
    	(command_msg.yaw != 0 ? COMMAND_RATE : COMMAND_NONE);
    
    // Move gimbal
    gimbal_command << 0, (float)command_msg.camera/100;

    // Change zoom
    zoom_command = -(float)command_msg.zoom/100;

    // Swithch between manual and offboard
    std_msgs::Bool offboard_msg;
    if(command_msg.reset_camera){
        offboard_msg.data = false;
        offboard_publisher.publish(offboard_msg);
    }
    if(command_msg.reset_zoom){
        offboard_msg.data = true;
        offboard_publisher.publish(offboard_msg);
    }
}

void desiredPoseCallback(const anafi_autonomy::PoseCommand& command_msg){
    desired_pose << command_msg.x, command_msg.y, command_msg.z, command_msg.yaw*M_PI/180;
    mode_offboard << COMMAND_POSITION, COMMAND_POSITION, COMMAND_ANGLE;
}

void desiredVelocityCallback(const anafi_autonomy::VelocityCommand& command_msg){
    desired_velocity << command_msg.vx, command_msg.vy, command_msg.vz, command_msg.yaw_rate;
    mode_offboard << COMMAND_VELOCITY, COMMAND_VELOCITY, COMMAND_RATE;
}

void desiredAttitudeCallback(const anafi_autonomy::AttitudeCommand& command_msg){
    desired_attitude << command_msg.roll*M_PI/180, command_msg.pitch*M_PI/180, command_msg.trottle, command_msg.yaw*M_PI/180;
    mode_offboard << COMMAND_ANGLE, COMMAND_VELOCITY, COMMAND_ANGLE;
}

void desiredTrajectoryCallback(const anafi_autonomy::PoseCommand& command_msg){ //TODO: Implement this function
    //desired_pose << command_msg.x, command_msg.y, command_msg.z, command_msg.yaw*M_PI/180;
    //mode_offboard << COMMAND_POSITION, COMMAND_POSITION, COMMAND_ANGLE;
}

void commandAxisCallback(const anafi_autonomy::AxesCommand& command_msg){
    command_offboard << (command_msg.horizontal_mode != COMMAND_NONE ? command_msg.x : command_offboard(0)), (command_msg.horizontal_mode != COMMAND_NONE ? command_msg.y : command_offboard(1)),
    	(command_msg.vertical_mode != COMMAND_NONE ? command_msg.z : command_offboard(2)), (command_msg.heading_mode != COMMAND_NONE ? command_msg.yaw*M_PI/180 : command_offboard(3));
    mode_offboard << command_msg.horizontal_mode, command_msg.vertical_mode, command_msg.heading_mode;
}

void commandCameraCallback(const anafi_autonomy::KeyboardCameraCommand& command_msg){
    gimbal_command << command_msg.roll, command_msg.pitch;
    zoom_command = command_msg.zoom;

    anafi_autonomy::TakePhoto take_photo_srv;
    std_srvs::Empty empty_srv;

	switch(command_msg.action){
	case 1:
        take_photo_client.call(take_photo_srv);
        break;
    case 2:
        start_recording_client.call(empty_srv);
        break;
    case 3:
        stop_recording_client.call(empty_srv);
        break;
    case 11:
        reset_zoom_client.call(empty_srv);
        reset_gimbal_client.call(empty_srv);
        break;
    }
}

// Constructor
SafeAnafi::SafeAnafi(int argc, char** argv){
    ros::init(argc, argv, "safeBebop");
    ros::NodeHandle node_handle;

    optitrack_subscriber = node_handle.subscribe("/optitrack/odometry", 1, optitrackCallback);
    odometry_subscriber = node_handle.subscribe("/anafi/odometry", 1, odometryCallback);
    command_subscriber = node_handle.subscribe("/keyboard/command_meta", 1, commandMetaCallback);
    command_keyboard_subscriber = node_handle.subscribe("/keyboard/command_move", 1, commandKeyboardCallback);
    command_camera_subscriber = node_handle.subscribe("/keyboard/command_camera", 1, commandCameraCallback);
    skycontroller_subscriber = node_handle.subscribe("/skycontroller/command", 1, commandSkycontrollerCallback);
    desired_pose_subscriber = node_handle.subscribe("/anafi/desired_pose", 1, desiredPoseCallback); // DEPRECATED
    desired_velocity_subscriber = node_handle.subscribe("/anafi/desired_velocity", 1, desiredVelocityCallback); // DEPRECATED
    desired_attitude_subscriber = node_handle.subscribe("/anafi/desired_attitude", 1, desiredAttitudeCallback); // DEPRECATED
    desired_trajectory_subscriber = node_handle.subscribe("/anafi/desired_trajectory", 1, desiredTrajectoryCallback);
    command_axis_subscriber = node_handle.subscribe("/anafi/command_offboard", 1, commandAxisCallback);

    emergency_publisher = node_handle.advertise<std_msgs::Empty>("/anafi/emergency", 1);
    takeoff_publisher = node_handle.advertise<std_msgs::Empty>("/anafi/takeoff", 1);
    land_publisher = node_handle.advertise<std_msgs::Empty>("/anafi/land", 1);
    offboard_publisher = node_handle.advertise<std_msgs::Bool>("/anafi/offboard", 1);
    moveto_publisher = node_handle.advertise<anafi_autonomy::MoveToCommand>("/anafi/cmd_moveto", 1);
    moveby_publisher = node_handle.advertise<anafi_autonomy::MoveByCommand>("/anafi/cmd_moveby", 1);
    rpyg_publisher = node_handle.advertise<anafi_autonomy::PilotingCommand>("/anafi/cmd_rpyt", 1);
    camera_publisher = node_handle.advertise<anafi_autonomy::CameraCommand>("/anafi/cmd_camera", 1);
    gimbal_publisher = node_handle.advertise<anafi_autonomy::GimbalCommand>("/anafi/cmd_gimbal", 1);
    euler_publisher = node_handle.advertise<geometry_msgs::Vector3>("/anafi/euler", 1);
    odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("/anafi/ground_truth/odometry", 1);
    desired_velocity_publisher = node_handle.advertise<geometry_msgs::TwistStamped>("/anafi/debug/desired_velocity", 1);
    acceleration_publisher = node_handle.advertise<geometry_msgs::Vector3Stamped>("/anafi/debug/acceleration", 1);
    mode_publisher = node_handle.advertise<geometry_msgs::Vector3Stamped>("/anafi/debug/mode", 1);

    take_photo_client = node_handle.serviceClient<anafi_autonomy::TakePhoto>("take_photo");
    start_recording_client = node_handle.serviceClient<std_srvs::Empty>("start_recording");
    stop_recording_client = node_handle.serviceClient<std_srvs::Empty>("stop_recording");
    reset_zoom_client = node_handle.serviceClient<std_srvs::Empty>("reset_zoom");
    reset_gimbal_client = node_handle.serviceClient<std_srvs::Empty>("reset_gimbal");

    ros::param::get("/safe_anafi/bounds/min_x", bounds(0,0));
    ros::param::get("/safe_anafi/bounds/min_y", bounds(0,1));
    ros::param::get("/safe_anafi/bounds/min_z", bounds(0,2));
    ros::param::get("/safe_anafi/bounds/max_x", bounds(1,0));
    ros::param::get("/safe_anafi/bounds/max_y", bounds(1,1));
    ros::param::get("/safe_anafi/bounds/max_z", bounds(1,2));
    
    Twist t;
    velocities.clear();
    for(int i = 0; i < FILTER_SIZE; ++i)
        velocities.push_back(t);
}

// Destructor
SafeAnafi::~SafeAnafi(){
    ROS_INFO("SafeAnafi is stopping...");
    std_msgs::Empty empty_msg;
    anafi_autonomy::PilotingCommand rpyg_msg;
    rpyg_publisher.publish(rpyg_msg);
    rpyg_publisher.publish(rpyg_msg);
    land_publisher.publish(empty_msg);
    land_publisher.publish(empty_msg);
    land = true;

    ros::shutdown();
    exit(0);
}

double denormalizeAngle(double a1, double a2){
    if(abs(a1 - a2) > M_PI)
        a1 += (a1 < a2) ? 2*M_PI : -2*M_PI;
    return a1;
}

void SafeAnafi::run(){
    ROS_INFO("SafeAnafi is running...");

    // Setup dynamic reconfigure (GUI) connections
    dynamic_reconfigure::Server<anafi_autonomy::setSafeAnafiConfig> server;
    dynamic_reconfigure::Server<anafi_autonomy::setSafeAnafiConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    double dt = (double)1/100;
    ros::Rate rate((double)1/dt);
    
    anafi_autonomy::PilotingCommand rpyg_msg;

    while(ros::ok()){
        ros::spinOnce();

        command_move << 
   (mode_skycontroller(0) != COMMAND_NONE ? command_skycontroller(0) : (mode_keyboard(0) != COMMAND_NONE ? command_keyboard(0) : (mode_offboard(0) != COMMAND_NONE ? command_offboard(0) : 0))), 
   (mode_skycontroller(0) != COMMAND_NONE ? command_skycontroller(1) : (mode_keyboard(0) != COMMAND_NONE ? command_keyboard(1) : (mode_offboard(0) != COMMAND_NONE ? command_offboard(1) : 0))), 
   (mode_skycontroller(1) != COMMAND_NONE ? command_skycontroller(2) : (mode_keyboard(1) != COMMAND_NONE ? command_keyboard(2) : (mode_offboard(1) != COMMAND_NONE ? command_offboard(2) : 0))), 
   (mode_skycontroller(2) != COMMAND_NONE ? command_skycontroller(3) : (mode_keyboard(2) != COMMAND_NONE ? command_keyboard(3) : (mode_offboard(2) != COMMAND_NONE ? command_offboard(3) : 0)));
        mode_move << 
   (mode_skycontroller(0) != COMMAND_NONE ? mode_skycontroller(0) : (mode_keyboard(0) != COMMAND_NONE ? mode_keyboard(0) : (mode_offboard(0) != COMMAND_NONE ? mode_offboard(0) : COMMAND_VELOCITY))), 
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
			ros::param::get("/anafi/max_horizontal_speed", max_horizontal_speed);
			command_move(0) = BOUND(command_move(0), max_horizontal_speed);
			command_move(1) = BOUND(command_move(1), max_horizontal_speed);
            
			if(fixed_frame)
				command_move << cos(yaw)*command_move(0) + sin(yaw)*command_move(1), -sin(yaw)*command_move(0) + cos(yaw)*command_move(1),
					command_move(2), command_move(3); // rotate command from world frame to body frame
                        
			velocity_error << command_move(0) - velocity(0), command_move(1) - velocity(1), 0;
			velocity_error_d = -acceleration;

			command_move(0) = -(k_p_velocity*velocity_error(1) + k_d_velocity*velocity_error_d(1));
			command_move(1) =   k_p_velocity*velocity_error(0) + k_d_velocity*velocity_error_d(0);
		case COMMAND_ANGLE: // attitude
			ros::param::get("/anafi/max_tilt", max_tilt);
			command_move(0) = BOUND(command_move(0), max_tilt);
			command_move(1) = BOUND(command_move(1), max_tilt);
			rpyg_msg.roll = command_move(0);
			rpyg_msg.pitch = command_move(1);
			break;
		default:
			rpyg_msg.roll = 0;
			rpyg_msg.pitch = 0;
			ROS_WARN_STREAM_THROTTLE(1, "Undefined horizontal mode (" << mode_move(MODE_HORIZONTAL) << ").");
			ROS_WARN_STREAM_THROTTLE(1, "It should be '0' for no command, '1' for position command, '2' for velocity command or '3' for attitude command.");
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
			ros::param::get("/anafi/max_vertical_speed", max_vertical_speed);
			command_move(2) = BOUND(command_move(2), max_vertical_speed);
			rpyg_msg.gaz = command_move(2);
			break;
		default:
			rpyg_msg.gaz = 0;
			ROS_WARN_STREAM_THROTTLE(1, "Undefined vertical mode (" << mode_move(MODE_VERTICAL) << ").");
			ROS_WARN_STREAM_THROTTLE(1, "It should be '0' for no command, '1' for position command or '2' for velocity command.");
		}
            
		switch(mode_move(MODE_HEADING)){ // heading
		case COMMAND_NONE: // no command
			rpyg_msg.yaw = 0;
			break;
		case COMMAND_ANGLE: // attitude
			yaw = denormalizeAngle(yaw, command_move(3));
			command_move(3) = k_p_yaw*(command_move(3) - yaw);
		case COMMAND_RATE: // angular rate
			ros::param::get("/anafi/max_yaw_rotation_speed", max_yaw_rotation_speed);
			command_move(3) = BOUND(command_move(3), max_yaw_rotation_speed);
			rpyg_msg.yaw = command_move(3);
			break;
		default:
			rpyg_msg.yaw = 0;
			ROS_WARN_STREAM_THROTTLE(1, "Undefined heading mode (" << mode_move(MODE_HEADING) << ").");
			ROS_WARN_STREAM_THROTTLE(1, "It should be '0' for no command, '3' for yaw command or '4' for rate command.");
		}

        //if(!land) // to prevent sending commands while landing
            rpyg_msg.header.stamp = ros::Time::now();
            rpyg_msg.header.frame_id = "/body";
            rpyg_publisher.publish(rpyg_msg);
            
        // Move gimbal
        anafi_autonomy::GimbalCommand gimbal_msg;
        gimbal_msg.mode = 1;
        if(!gimbal_command.isZero()){
            gimbal_msg.roll = gimbal_command(0);
            gimbal_msg.pitch = gimbal_command(1);
            gimbal_publisher.publish(gimbal_msg);
            stop_gimbal = false;
        }
        else
            if(!stop_gimbal){
                gimbal_publisher.publish(gimbal_msg);
                stop_gimbal = true;
            }

        // Zoom
        anafi_autonomy::CameraCommand camera_msg;
        camera_msg.mode = 1;
        if(zoom_command != 0){
            camera_msg.zoom = zoom_command;
            camera_publisher.publish(camera_msg);
            stop_zoom = false;
        }
        else
            if(!stop_zoom){
                camera_publisher.publish(camera_msg);
                stop_zoom = true;
            }
            
		geometry_msgs::Vector3Stamped acceleration_msg; // FOR DEBUG
        acceleration_msg.header.stamp = ros::Time::now(); // FOR DEBUG
        acceleration_msg.vector.x = acceleration(0); // FOR DEBUG
        acceleration_msg.vector.y = acceleration(1); // FOR DEBUG
        acceleration_msg.vector.z = acceleration(2); // FOR DEBUG
        acceleration_publisher.publish(acceleration_msg); // FOR DEBUG
        
		geometry_msgs::Vector3Stamped mode_msg; // FOR DEBUG
        mode_msg.header.stamp = ros::Time::now(); // FOR DEBUG
        mode_msg.vector.x = mode_move(0); // FOR DEBUG
        mode_msg.vector.y = mode_move(1); // FOR DEBUG
        mode_msg.vector.z = mode_move(2); // FOR DEBUG
        mode_publisher.publish(mode_msg); // FOR DEBUG

        rate.sleep();
    }
}

int main(int argc, char** argv){
    SafeAnafi* anafi = new SafeAnafi(argc, argv);
    anafi->run();
}
