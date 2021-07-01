#include <anafi_autonomy/erl_bridge.hpp>

DroneFollower::DroneFollower(ros::NodeHandle n)
{
    node_handle_ = n;
    std::vector<double> values;
    double value;
    std::string str;

    std::string n_namespace = n.getNamespace();
    std::cout << "****** n_namespace=" << n_namespace << std::endl;
    n.getParam(n_namespace+"/this_port",value);
    this_port_ = value;

    n.getParam(n_namespace+"/remote_id",remote_id_);
    n.getParam(n_namespace+"/remote_port",value);
    remote_port_ = value;

    n.getParam(n_namespace+"/input_topic", str);
    input_topic_ = str;
    n.getParam(n_namespace+"/camera_topic", str);
    camera_topic_ = str;
    n.getParam(n_namespace+"/feedback_topic", str);
    feedback_topic_ = n_namespace + str;

    input_pub_ = n.advertise<anafi_autonomy::KeyboardMoveCommand>(input_topic_,0);
    // input_pub_ = n.advertise<anafi_autonomy::VelocityCommand>(input_topic_,0);
    camera_pub_ = n.advertise<anafi_autonomy::KeyboardCameraCommand>(camera_topic_,0);

    // init our timeout clock
    n.getParam(n_namespace+"/input_timeout_ms",value);
    input_timeout_ms_ = std::chrono::milliseconds { int(value) };
    last_input_ = std::chrono::high_resolution_clock::now();

    camera_zero_msg_.roll = 0;
    camera_zero_msg_.pitch = 0;
    camera_zero_msg_.zoom = 0;
    // tf::vectorEigenToMsg(Eigen::Vector3d(0,0,0), camera_zero_msg_.linear);
    // tf::vectorEigenToMsg(Eigen::Vector3d(0,0,0), camera_zero_msg_.angular);

    // feedback_sub_ = n.subscribe(feedback_topic_, 1, &DroneFollower::WrenchCallback, this); // --> we don't have feedback yet
}

void DroneFollower::Receive()
{
    haptic_input_msg_ = haptic_udp_client_->LatestHapticInput();
    // std::cout << haptic_input_msg_.wrench.force.x() << std::endl;

    // lets check that we actually received a new message
    if (! ((haptic_input_msg_.time_sec == input_time_sec_) && (haptic_input_msg_.time_nsec == input_time_nsec_))) {
        // std::cout << "new message" << std::endl;
        timed_out_ = false;
        timeout_handled_ = false;
        // std::cout << "new message DroneFollower::Receive" << std::endl;
        last_input_ = std::chrono::high_resolution_clock::now();

        anafi_autonomy::KeyboardMoveCommand input_msg;
        // anafi_autonomy::VelocityCommand input_msg;
        anafi_autonomy::KeyboardCameraCommand camera_msg;

        input_time_sec_ = haptic_input_msg_.time_sec;
        input_time_nsec_ = haptic_input_msg_.time_nsec;

        input_msg.x = haptic_input_msg_.wrench.force.x();
        input_msg.y = haptic_input_msg_.wrench.force.y();
        input_msg.z = haptic_input_msg_.wrench.force.z();
        input_msg.yaw = haptic_input_msg_.wrench.torque.z();
        // input_msg.vx = haptic_input_msg_.wrench.force.x();
        // input_msg.vy = haptic_input_msg_.wrench.force.y();
        // input_msg.vz = haptic_input_msg_.wrench.force.z();
        // input_msg.yaw_rate = haptic_input_msg_.wrench.torque.z();
        input_msg.header.stamp.sec = input_time_sec_;
        input_msg.header.stamp.nsec = input_time_nsec_;

        if (haptic_input_msg_.wrench.torque.x() > 0)
            // camera_msg.roll = 0.3;
            camera_msg.zoom = 0.3;
            // camera_msg.angular.x = 0.3;
        else if (haptic_input_msg_.wrench.torque.x() < 0)
            // camera_msg.roll = -0.3;
            camera_msg.zoom = -0.3;
            // camera_msg.angular.x = -0.3;
        if (haptic_input_msg_.wrench.torque.y() > 0)
            camera_msg.pitch = 0.3;
            // camera_msg.angular.y = 0.3;
        else if (haptic_input_msg_.wrench.torque.y() < 0)
            camera_msg.pitch = -0.3;
            // camera_msg.angular.y = -0.3;

        // camera_msg.angular.x = haptic_input_msg_.wrench.torque.x();
        // camera_msg.angular.y = haptic_input_msg_.wrench.torque.y();
        // if (camera_msg.angular.x != 0 || camera_msg.angular.y != 0)
        // if (input_msg.twist != input_msg_.twist)

        // if ((input_msg.x != input_msg_.x) || (input_msg.y != input_msg_.y) || (input_msg.z != input_msg_.z) || (input_msg.yaw != input_msg_.yaw))
        //     input_pub_.publish(input_msg);
        // if ((input_msg.vx != input_msg_.vx) || (input_msg.vy != input_msg_.vy) || (input_msg.vz != input_msg_.vz) || (input_msg.yaw_rate != input_msg_.yaw_rate))
        input_pub_.publish(input_msg);

        // if (camera_msg.angular != camera_msg_.angular) {
        //     std::cout << "      publishing camera message" << std::endl;
        //     // camera_pub_.publish(camera_zero_msg_);
        //     camera_pub_.publish(camera_msg);
        // }
        camera_pub_.publish(camera_msg);

        input_msg_ = input_msg;
        camera_msg_ = camera_msg;
    } else {
        // std::cout << "old message" << std::endl;
        // we didn't receive a new message yet --> honour the timeout
        auto now = std::chrono::high_resolution_clock::now();
        if (now <= last_input_ + input_timeout_ms_) {
            input_pub_.publish(input_msg_);
            // if (camera_msg_.angular.x != 0 || camera_msg_.angular.y != 0)
            // camera_pub_.publish(camera_zero_msg_);
            camera_pub_.publish(camera_msg_);

        } else {
            // timed_out_ = true;
            // if (timed_out_ && (! timeout_handled_)) {
            //     std::cout << "timeout DroneFollower!" << std::endl;
            //     input_msg_.x = 0;
            //     input_msg_.y = 0;
            //     input_msg_.z = 0;
            //     input_msg_.yaw = 0;
            //     camera_msg_.roll = 0;
            //     camera_msg_.pitch = 0;
            //     camera_msg_.zoom = 0;
            //     // tf::vectorEigenToMsg(Eigen::Vector3d(0,0,0), input_msg_.twist.linear);
            //     // tf::vectorEigenToMsg(Eigen::Vector3d(0,0,0), input_msg_.twist.angular);
            //     // tf::vectorEigenToMsg(Eigen::Vector3d(0,0,0), camera_msg_.linear);
            //     // tf::vectorEigenToMsg(Eigen::Vector3d(0,0,0), camera_msg_.angular);
            //     // input_pub_.publish(input_msg_);
            //     // camera_pub_.publish(camera_zero_msg_);
            //     // camera_pub_.publish(camera_msg_);
            //     timeout_handled_ = true;
            // }
            // input_pub_.publish(input_msg_);
            // camera_pub_.publish(camera_msg_);

            input_msg_.x = 0;
            input_msg_.y = 0;
            input_msg_.z = 0;
            input_msg_.yaw = 0;
            // input_msg_.vx = 0;
            // input_msg_.vy = 0;
            // input_msg_.vz = 0;
            // input_msg_.yaw_rate = 0;
            camera_msg_.roll = 0;
            camera_msg_.pitch = 0;
            camera_msg_.zoom = 0;
                // tf::vectorEigenToMsg(Eigen::Vector3d(0,0,0), input_msg_.twist.linear);
                // tf::vectorEigenToMsg(Eigen::Vector3d(0,0,0), input_msg_.twist.angular);
                // tf::vectorEigenToMsg(Eigen::Vector3d(0,0,0), camera_msg_.linear);
                // tf::vectorEigenToMsg(Eigen::Vector3d(0,0,0), camera_msg_.angular);
                // input_pub_.publish(input_msg_);
                // camera_pub_.publish(camera_zero_msg_);
                // camera_pub_.publish(camera_msg_);
                // timeout_handled_ = true;
            // }
            input_pub_.publish(input_msg_);
            camera_pub_.publish(camera_msg_);
        }
    }
}

void DroneFollower::Send()
{
    haptic::HapticFeedback feedback;
    feedback.time_sec = input_time_sec_;
    feedback.time_nsec = input_time_nsec_;
    haptic_udp_client_->PublishFeedback(feedback);
}

void DroneFollower::start_recording()
{
    // geometry_msgs::Twist camera_msg;
    // camera_msg.linear.z = 2;
    anafi_autonomy::KeyboardCameraCommand camera_msg;
    camera_msg.action = 2;
    camera_pub_.publish(camera_msg);
}

void DroneFollower::stop_recording()
{
    // geometry_msgs::Twist camera_msg;
    // camera_msg.linear.z = 3;
    anafi_autonomy::KeyboardCameraCommand camera_msg;
    camera_msg.action = 3;
    camera_pub_.publish(camera_msg);
}

// void DroneFollower::publish_camera_msg(const geometry_msgs::Twist& camera_msg)
void DroneFollower::publish_camera_msg(const anafi_autonomy::KeyboardCameraCommand& camera_msg)
{
    camera_pub_.publish(camera_msg);
}

ErlBridge::ErlBridge(ros::NodeHandle n, std::shared_ptr<DroneFollower> follower)
{
    node_handle_ = n;
    follower_ = follower;
    double value;
    std::string str;

    std::string n_namespace = n.getNamespace();
    std::cout << "****** n_namespace=" << n_namespace << std::endl;
    n.getParam(n_namespace+"/messenger_name",str);
    messenger_name_ = str;
    n.getParam(n_namespace+"/messenger_port",value);
    messenger_port_ = value;
    n.getParam(n_namespace+"/command_topic", str);
    command_topic_ = str;

    messenger_ = std::make_shared<RobotP2P::Messenger>(messenger_name_, messenger_port_);
    messenger_->set_spdlog_level(0);
    messenger_->bind_method("arm", std::bind(&ErlBridge::arm, this, std::placeholders::_1));
    messenger_->bind_method("stop_motors", std::bind(&ErlBridge::stop_motors, this, std::placeholders::_1));
    messenger_->bind_method("take_off", std::bind(&ErlBridge::take_off, this, std::placeholders::_1));
    messenger_->bind_method("land", std::bind(&ErlBridge::land, this, std::placeholders::_1));
    messenger_->bind_method("hover_at_position", std::bind(&ErlBridge::hover_at_position, this, std::placeholders::_1));
    messenger_->bind_method("start_recording", std::bind(&ErlBridge::start_recording, this, std::placeholders::_1));
    messenger_->bind_method("stop_recording", std::bind(&ErlBridge::stop_recording, this, std::placeholders::_1));
    messenger_->start_server();
    // messenger_->set_spdlog_level(0);

    command_pub_ = n.advertise<std_msgs::Int8>(command_topic_,0);
    // state_sub_ = n.subscribe("/anafi/state", 1, &ErlBridge::state_cb, this);
}

void ErlBridge::state_cb(const std_msgs::StringConstPtr& str)
{
    json request = {{"state", str->data.c_str()}};
    messenger_->request("robot_web_tp", "report_state", request, "leader");
}

bool ErlBridge::start_bridge()
{

}

bool ErlBridge::stop_bridge()
{

}

json ErlBridge::arm(json args)
{
    command_.data = 1;
    command_pub_.publish(command_);
    json result = {{"success", true}};
    return result;
}

json ErlBridge::take_off(json args)
{
    command_.data = 2;
    command_pub_.publish(command_);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    command_.data = 102;
    command_pub_.publish(command_);
    json result = {{"success", true}};
    return result;
}

json ErlBridge::hover_at_position(json args)
{
    command_.data = 3;
    command_pub_.publish(command_);
    json result = {{"success", true}};
    return result;
}

json ErlBridge::land(json args)
{
    command_.data = 4;
    command_pub_.publish(command_);
    json result = {{"success", true}};
    return result;
}

json ErlBridge::stop_motors(json args)
{
    command_.data = 5;
    command_pub_.publish(command_);
    json result = {{"success", true}};
    return result;
}

json ErlBridge::start_recording(json args)
{
    follower_->start_recording();
    json result = {{"success", true}};
    return result;
}

json ErlBridge::stop_recording(json args)
{
    follower_->stop_recording();
    json result = {{"success", true}};
    return result;
}

