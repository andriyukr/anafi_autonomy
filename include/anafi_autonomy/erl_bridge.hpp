#pragma once
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <anafi_autonomy/KeyboardMoveCommand.h>
// #include <anafi_autonomy/VelocityCommand.h>
#include <anafi_autonomy/KeyboardCameraCommand.h>
#include <nlohmann/json.hpp>
#include <robotp2p/robotp2p.hpp>
#include <haptic.hpp>

using json = nlohmann::json;

class DroneFollower : public haptic::HapticUdpFollower {
public:
    DroneFollower(ros::NodeHandle n);
    ~DroneFollower() {
        Stop();
    }
    void Receive();
    void Send();
    // void publish_camera_msg(const geometry_msgs::Twist& camera_msg);
    void publish_camera_msg(const anafi_autonomy::KeyboardCameraCommand& camera_msg);
    void start_recording();
    void stop_recording();

private:
    ros::NodeHandle node_handle_;
    ros::Publisher input_pub_;
    ros::Publisher camera_pub_;
    std::string input_topic_;
    std::string camera_topic_;
    std::string feedback_topic_;
    anafi_autonomy::KeyboardMoveCommand input_msg_;
    // anafi_autonomy::VelocityCommand input_msg_;
    anafi_autonomy::KeyboardCameraCommand camera_msg_;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_input_;
    std::chrono::milliseconds input_timeout_ms_;
    std::atomic<bool> timed_out_{true}; // if true, sending zero input, if false, sending actual input
    std::atomic<bool> timeout_handled_{false};
    anafi_autonomy::KeyboardCameraCommand camera_zero_msg_;
};

class ErlBridge {
public:
    ErlBridge(ros::NodeHandle n, std::shared_ptr<DroneFollower> follower);

    bool start_bridge();
    bool stop_bridge();

    json arm(json args);
    json stop_motors(json args);
    json take_off(json args);
    json land(json args);
    json hover_at_position(json args);
    json start_recording(json args);
    json stop_recording(json args);
    void state_cb(const std_msgs::StringConstPtr& str);

private:
    std::string messenger_name_;
    unsigned messenger_port_;
    std::shared_ptr<RobotP2P::Messenger> messenger_;
    ros::NodeHandle node_handle_;
    std::shared_ptr<DroneFollower> follower_;

    ros::Publisher command_pub_;
    std::string command_topic_;
    std_msgs::Int8 command_;
    ros::Subscriber state_sub_;
};
