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
void dynamicReconfigureCallback(anafi_autonomy::setSafeAnafiConfig &config, uint32_t level){
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

void actionCallback(const std_msgs::Int8& action_msg){
    action = static_cast<Actions>(action_msg.data);
}

void keyboardDroneCallback(const anafi_autonomy::KeyboardDroneCommand& command_msg){
    ros::param::get("anafi/max_vertical_speed", max_vertical_speed);
    ros::param::get("anafi/max_yaw_rotation_speed", max_yaw_rotation_speed);
    ros::param::get("anafi/max_horizontal_speed", max_horizontal_speed);
    command_keyboard << max_horizontal_speed*command_msg.x, max_horizontal_speed*command_msg.y, max_vertical_speed*command_msg.z, max_yaw_rotation_speed*command_msg.yaw;
    mode_keyboard << ((command_msg.x != 0 || command_msg.y != 0) ? COMMAND_VELOCITY : COMMAND_NONE), (command_msg.z != 0 ? COMMAND_VELOCITY : COMMAND_NONE),
            (command_msg.yaw != 0 ? COMMAND_RATE : COMMAND_NONE);
}

void commandSkycontrollerCallback(const olympe_bridge::SkycontrollerCommand& command_msg){
    // Drone commands
    ros::param::get("anafi/max_vertical_speed", max_vertical_speed);
    ros::param::get("anafi/max_yaw_rotation_speed", max_yaw_rotation_speed);
    ros::param::get("anafi/max_horizontal_speed", max_horizontal_speed);
    command_skycontroller << max_horizontal_speed/100*command_msg.x, -max_horizontal_speed/100*command_msg.y, max_vertical_speed/100*command_msg.z, -max_yaw_rotation_speed/100*command_msg.yaw;
    mode_skycontroller << ((command_msg.x != 0 || command_msg.y != 0) ? COMMAND_VELOCITY : COMMAND_NONE), (command_msg.z != 0 ? COMMAND_VELOCITY : COMMAND_NONE),
            (command_msg.yaw != 0 ? COMMAND_RATE : COMMAND_NONE);
    
    // Move
    controller_gimbal_command << 0, (float)command_msg.camera/100;

    // Change zoom
    controller_zoom_command = -(float)command_msg.zoom/100;

    // Swithch between manual and offboard
    if(command_msg.reset_camera)
        offboard_client.call(false_srv);
    if(command_msg.reset_zoom)
        offboard_client.call(true_srv);
}

void desiredPoseCallback(const anafi_autonomy::PoseCommand& command_msg){
    command_offboard << command_msg.x, command_msg.y, command_msg.z, command_msg.yaw*M_PI/180;
    mode_offboard << COMMAND_POSITION, COMMAND_POSITION, COMMAND_ANGLE;
}

void desiredVelocityCallback(const anafi_autonomy::VelocityCommand& command_msg){
    command_offboard << command_msg.vx, command_msg.vy, command_msg.vz, command_msg.yaw_rate;
    mode_offboard << COMMAND_VELOCITY, COMMAND_VELOCITY, COMMAND_RATE;
}

void desiredAttitudeCallback(const anafi_autonomy::AttitudeCommand& command_msg){
    command_offboard << command_msg.roll*M_PI/180, command_msg.pitch*M_PI/180, command_msg.trottle, command_msg.yaw*M_PI/180;
    mode_offboard << COMMAND_ANGLE, COMMAND_VELOCITY, COMMAND_ANGLE;
}

void desiredTrajectoryCallback(const anafi_autonomy::PoseCommand& command_msg){ //TODO: Implement this function
    //desired_pose << command_msg.x, command_msg.y, command_msg.z, command_msg.yaw*M_PI/180;
    //mode_offboard << COMMAND_POSITION, COMMAND_POSITION, COMMAND_ANGLE;
}

void desiredCommandCallback(const anafi_autonomy::DesiredCommand& command_msg){
    command_offboard << (command_msg.horizontal_mode != COMMAND_NONE ? command_msg.x : command_offboard(0)), (command_msg.horizontal_mode != COMMAND_NONE ? command_msg.y : command_offboard(1)),
            (command_msg.vertical_mode != COMMAND_NONE ? command_msg.z : command_offboard(2)), (command_msg.heading_mode != COMMAND_NONE ? command_msg.yaw*M_PI/180 : command_offboard(3));
    mode_offboard << command_msg.horizontal_mode, command_msg.vertical_mode, command_msg.heading_mode;
}

void keyboardCameraCallback(const anafi_autonomy::KeyboardCameraCommand& command_msg){
    keyboard_gimbal_command << command_msg.roll, command_msg.pitch;
    keyboard_zoom_command = command_msg.zoom;

    std_srvs::Empty empty_srv;

    switch(command_msg.action){
    case 1:
        take_photo_client.call(empty_srv);
        break;
    case 2:
        start_recording_client.call(empty_srv);
        break;
    case 3:
        stop_recording_client.call(empty_srv);
        break;
    case 4:
        download_media_client.call(empty_srv);
        break;
    case 11:
        reset_zoom_client.call(empty_srv);
        reset_gimbal_client.call(empty_srv);
        break;
    case 111:
        calibrate_gimbal_client.call(empty_srv);
        break;
    }
}

void offboardGimbalCallback(const geometry_msgs::Vector3& command_msg){
    offboard_gimbal_command << command_msg.x, command_msg.y;
}

void offboardZoomCallback(const std_msgs::Float32& command_msg){
    offboard_zoom_command = command_msg.data;
}

States resolveState(std::string input){
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

void stateCallback(const std_msgs::StringConstPtr& state_msg){ // 'LANDED', 'MOTOR_RAMPING', 'USER_TAKEOFF', 'TAKINGOFF', 'HOVERING', 'FLYING', 'LANDING', 'EMERGENCY'
    state = resolveState(state_msg->data);
}

// Constructor
SafeAnafi::SafeAnafi(int argc, char** argv){
    ros::init(argc, argv, "safeBebop");
    ros::NodeHandle node_handle;

    optitrack_subscriber = node_handle.subscribe("/optitrack/odometry", 1, optitrackCallback);
    odometry_subscriber = node_handle.subscribe("drone/odometry", 1, odometryCallback);
    action_subscriber = node_handle.subscribe("keyboard/action", 1, actionCallback);
    command_keyboard_subscriber = node_handle.subscribe("keyboard/drone_command", 1, keyboardDroneCallback);
    command_camera_subscriber = node_handle.subscribe("keyboard/camera_command", 1, keyboardCameraCallback);
    skycontroller_subscriber = node_handle.subscribe("skycontroller/command", 1, commandSkycontrollerCallback);
    desired_pose_subscriber = node_handle.subscribe("drone/desired_pose", 1, desiredPoseCallback); // DEPRECATED
    desired_velocity_subscriber = node_handle.subscribe("drone/desired_velocity", 1, desiredVelocityCallback); // DEPRECATED
    desired_attitude_subscriber = node_handle.subscribe("drone/desired_attitude", 1, desiredAttitudeCallback); // DEPRECATED
    desired_trajectory_subscriber = node_handle.subscribe("drone/desired_trajectory", 1, desiredTrajectoryCallback);
    command_axis_subscriber = node_handle.subscribe("drone/desired_command", 1, desiredCommandCallback);
    gimbal_command_subscriber = node_handle.subscribe("drone/desired_gimbal", 1, offboardGimbalCallback);
    zoom_command_subscriber = node_handle.subscribe("drone/desired_zoom", 1, offboardZoomCallback);
    state_subscriber = node_handle.subscribe("drone/state", 1, stateCallback);

    moveto_publisher = node_handle.advertise<olympe_bridge::MoveToCommand>("drone/moveto", 1);
    moveby_publisher = node_handle.advertise<olympe_bridge::MoveByCommand>("drone/moveby", 1);
    rpyg_publisher = node_handle.advertise<olympe_bridge::PilotingCommand>("drone/rpyt", 1);
    camera_publisher = node_handle.advertise<olympe_bridge::CameraCommand>("camera/cmd", 1);
    gimbal_publisher = node_handle.advertise<olympe_bridge::GimbalCommand>("gimbal/cmd", 1);
    odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("drone/ground_truth/odometry", 1);
    desired_velocity_publisher = node_handle.advertise<geometry_msgs::TwistStamped>("drone/debug/desired_velocity", 1);
    acceleration_publisher = node_handle.advertise<geometry_msgs::Vector3Stamped>("drone/debug/acceleration", 1);
    mode_publisher = node_handle.advertise<geometry_msgs::Vector3Stamped>("drone/debug/mode", 1);

    arm_client = node_handle.serviceClient<std_srvs::SetBool>("drone/arm");
    takeoff_client = node_handle.serviceClient<std_srvs::Empty>("drone/takeoff");
    land_client = node_handle.serviceClient<std_srvs::Empty>("drone/land");
    emergency_client = node_handle.serviceClient<std_srvs::Empty>("drone/emergency");
    halt_client = node_handle.serviceClient<std_srvs::Empty>("drone/halt");
    rth_client = node_handle.serviceClient<std_srvs::Empty>("drone/rth");
    reboot_client = node_handle.serviceClient<std_srvs::Empty>("drone/reboot");
    calibrate_magnetometer_client = node_handle.serviceClient<std_srvs::Empty>("drone/calibrate");
    offboard_client = node_handle.serviceClient<std_srvs::SetBool>("skycontroller/offboard");
    flightplan_upload_client = node_handle.serviceClient<olympe_bridge::FlightPlan>("flightplan/upload");
    flightplan_start_client = node_handle.serviceClient<olympe_bridge::FlightPlan>("flightplan/start");
    flightplan_pause_client = node_handle.serviceClient<std_srvs::Empty>("flightplan/pause");
    flightplan_stop_client = node_handle.serviceClient<std_srvs::Empty>("flightplan/stop");
    followme_start_client = node_handle.serviceClient<olympe_bridge::FollowMe>("followme/start");
    followme_stop_client = node_handle.serviceClient<std_srvs::Empty>("followme/stop");
    reset_gimbal_client = node_handle.serviceClient<std_srvs::Empty>("gimbal/reset");
    calibrate_gimbal_client = node_handle.serviceClient<std_srvs::Empty>("gimbal/calibrate");
    reset_zoom_client = node_handle.serviceClient<std_srvs::Empty>("camera/reset");
    take_photo_client = node_handle.serviceClient<std_srvs::Empty>("camera/photo/take");
    start_recording_client = node_handle.serviceClient<std_srvs::Empty>("camera/recording/start");
    stop_recording_client = node_handle.serviceClient<std_srvs::Empty>("camera/recording/stop");
    download_media_client = node_handle.serviceClient<std_srvs::Empty>("storage/download");

    ros::param::get("safe_anafi/bounds/min_x", bounds(0,0));
    ros::param::get("safe_anafi/bounds/min_y", bounds(0,1));
    ros::param::get("safe_anafi/bounds/min_z", bounds(0,2));
    ros::param::get("safe_anafi/bounds/max_x", bounds(1,0));
    ros::param::get("safe_anafi/bounds/max_y", bounds(1,1));
    ros::param::get("safe_anafi/bounds/max_z", bounds(1,2));

    false_srv.request.data = false;
    true_srv.request.data = true;
    
    Twist t;
    velocities.clear();
    for(int i = 0; i < FILTER_SIZE; ++i)
        velocities.push_back(t);
}

// Destructor
SafeAnafi::~SafeAnafi(){
    ROS_INFO("SafeAnafi is stopping...");
    olympe_bridge::PilotingCommand rpyg_msg;
    rpyg_publisher.publish(rpyg_msg);
    rpyg_publisher.publish(rpyg_msg);
    land_client.call(empty_srv);
    land_client.call(empty_srv);

    ros::shutdown();
    exit(0);
}

double denormalizeAngle(double a1, double a2){
    if(abs(a1 - a2) > M_PI)
        a1 += (a1 < a2) ? 2*M_PI : -2*M_PI;
    return a1;
}

void controllers(){
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
        ros::param::get("anafi/max_horizontal_speed", max_horizontal_speed);
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
        ros::param::get("anafi/max_tilt", max_tilt);
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
        ros::param::get("anafi/max_vertical_speed", max_vertical_speed);
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
        ros::param::get("anafi/max_yaw_rotation_speed", max_yaw_rotation_speed);
        command_move(3) = BOUND(command_move(3), max_yaw_rotation_speed);
        rpyg_msg.yaw = command_move(3);
        break;
    default:
        rpyg_msg.yaw = 0;
        ROS_WARN_STREAM_THROTTLE(1, "Undefined heading mode (" << mode_move(MODE_HEADING) << ").");
        ROS_WARN_STREAM_THROTTLE(1, "It should be '0' for no command, '3' for yaw command or '4' for rate command.");
    }

    rpyg_msg.header.stamp = ros::Time::now();
    rpyg_msg.header.frame_id = "body";
    rpyg_publisher.publish(rpyg_msg);
}

void stateMachine(){
    // State-indipendent actions
    switch(action){
    case DISARM: // emergency
        emergency_client.call(empty_srv);
        return;
    case RESET_POSE:
        ROS_INFO("Reseting position and heading");
        position << 0, 0, 0;
        initial_yaw = yaw;
        return;
    case REMOTE_CONTROL:
        offboard_client.call(false_srv);
        return;
    case OFFBOARD_CONTROL:
        offboard_client.call(true_srv);
        return;
    }

    switch(state){
    case LANDED:
        switch(action){
        case ARM:
            if(hand_launch)
                arm_client.call(true_srv);
            else{
                armed = !armed;
                if(armed)
                    ROS_WARN("Armed");
                else
                    ROS_INFO("Disarmed");
            }
            break;
        case TAKEOFF:
            if(armed)
                takeoff_client.call(empty_srv);
            else
                ROS_WARN("Arm the drone with 'Insert' before taking-off.");
            armed = false;
            break;
        case MISSION_START:
            if(mission_type == 0){ // flight plan
                if(armed){
                    olympe_bridge::FlightPlan flightplan_srv;
                    flightplan_srv.request.filepath = flightplan_file;
                    flightplan_srv.request.uid = "";
                    flightplan_upload_client.call(flightplan_srv);
                    flightplan_start_client.call(flightplan_srv);
                }
                else
                    ROS_WARN("Arm the drone with 'Insert' before starting the mission.");
                armed = false;
            }
            else
                ROS_WARN("The drone has to be in the air.");
            break;
        case REBOOT:
            reboot_client.call(empty_srv);
            break;
        case CALIBRATE:
            calibrate_magnetometer_client.call(empty_srv);
            break;
        }
        break;
    case MOTOR_RAMPING:
        switch(action){
        case ARM: // disarm
        case LAND:
        case HALT:
            arm_client.call(false_srv);
            break;
        }
        break;
    case USER_TAKEOFF:
        switch(action){
        case ARM: // disarm
        case LAND:
        case HALT:
            arm_client.call(false_srv);
            break;
        case TAKEOFF:
            arm_client.call(false_srv); // needs to stop the motors before taking off
            takeoff_client.call(empty_srv);
            break;
        }
        break;
    case TAKINGOFF:
        switch(action){
        case LAND:
        case HALT:
            land_client.call(empty_srv);
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
            land_client.call(empty_srv);
            break;
        case RTH: // return-to-home
            rth_client.call(empty_srv);
            break;
        case MISSION_START:
            if(mission_type == 0){ // flight plan
                olympe_bridge::FlightPlan flightplan_srv;
                flightplan_srv.request.filepath = flightplan_file;
                flightplan_srv.request.uid = "";
                flightplan_upload_client.call(flightplan_srv);
                flightplan_start_client.call(flightplan_srv);
            }
            else{ // follow me
                olympe_bridge::FollowMe followme_srv;
                followme_srv.request.mode = followme_mode;
                followme_srv.request.horizontal = 2;
                followme_srv.request.vertical = 10;
                followme_srv.request.target_azimuth = 0;
                followme_srv.request.target_elevation = 45;
                followme_srv.request.change_of_scale = 0;
                followme_srv.request.confidence_index = 255;
                followme_srv.request.is_new_selection = true;
                followme_start_client.call(followme_srv);
            }
            break;
        case MISSION_PAUSE:
            flightplan_pause_client.call(empty_srv);
            break;
        case MISSION_STOP:
            if(mission_type == 0) // flight plan
                flightplan_stop_client.call(empty_srv);
            else // follow me
                followme_stop_client.call(empty_srv);
            break;
        case HALT:
            halt_client.call(empty_srv);
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
            takeoff_client.call(empty_srv);
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
            takeoff_client.call(empty_srv);
            break;
        }
    case INVALID:
        switch(action){
        case HALT:
            takeoff_client.call(empty_srv);
            break;
        case REBOOT:
            reboot_client.call(empty_srv);
            break;
        }
    }

    action = NONE;
}

void SafeAnafi::run(){
    ROS_INFO("SafeAnafi is running...");

    // Setup dynamic reconfigure (GUI) connections
    dynamic_reconfigure::Server<anafi_autonomy::setSafeAnafiConfig> server;
    dynamic_reconfigure::Server<anafi_autonomy::setSafeAnafiConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    dt = (double)1/100;
    ros::Rate rate((double)1/dt);

    while(ros::ok()){
        ros::spinOnce();

        stateMachine();

        // Move gimbal
        gimbal_command <<   (controller_gimbal_command(0) != 0 ? controller_gimbal_command(0) : (keyboard_gimbal_command(0) != 0 ? keyboard_gimbal_command(0) : offboard_gimbal_command(0))),
                            (controller_gimbal_command(1) != 0 ? controller_gimbal_command(1) : (keyboard_gimbal_command(1) != 0 ? keyboard_gimbal_command(1) : offboard_gimbal_command(1)));

        olympe_bridge::GimbalCommand gimbal_msg;
        gimbal_msg.header.stamp = ros::Time::now();
        gimbal_msg.header.frame_id = "body";
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
        zoom_command = (controller_zoom_command != 0 ? controller_zoom_command : (keyboard_zoom_command != 0 ? keyboard_zoom_command : offboard_zoom_command));

        olympe_bridge::CameraCommand camera_msg;
        camera_msg.header.stamp = ros::Time::now();
        camera_msg.header.frame_id = "gimbal";
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
