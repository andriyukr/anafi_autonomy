#include <iostream>
#include <thread>

#include "ros/ros.h"
#include <haptic.hpp>
#include <anafi_autonomy/erl_bridge.hpp>

static volatile bool interrupted = false;

void SignalHandler(int sig)
{
    std::cout << "Interrupted" << std::endl;
    interrupted = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "erl_bridge");
    ros::NodeHandle n(ros::this_node::getName());
    std::string n_namespace = n.getNamespace();

    double rate;
    n.getParam(n_namespace+"/rate", rate);
    ros::Rate loop_rate(rate);

    asio::io_context io_context;
    // std::shared_ptr<haptic::HapticUdpFollower> drone_follower;
    std::shared_ptr<DroneFollower> drone_follower;
    drone_follower = std::make_shared<DroneFollower>(n);
    if (!drone_follower->Connect(io_context)) {
        std::cerr << "Follower could not connect with remote" << std::endl;
    }
    // event_server = std::make_shared<MiosInterface::EventServer>(n);
    // panda_client = std::make_shared<MiosInterface::PandaClient>(n);
    // auto messenger = std::make_shared<MiosInterface::Messenger>(n, event_server, panda_client);
    std::shared_ptr<ErlBridge> erl_bridge = std::make_shared<ErlBridge>(n, drone_follower);
    // erl_bridge = std::make_shared<ErlBridge>(n);
    // erl_bridge->start_bridge();

    // Signal handlers
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    while(ros::ok()) {
        if (interrupted) {
            std::cerr << "Stopping Follower" << std::endl;
            drone_follower->Stop();
            // erl_bridge->stop_bridge();
            io_context.stop();
            break;
        }

        // action
        try {
            // drone_follower->Send(); // no feedback yet
            drone_follower->Receive();
            drone_follower->Send(); // for timestamp only, to measure latency
        } catch (std::exception& e) {
            std::cerr << "Exception: " << e.what() << "\n";
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}
