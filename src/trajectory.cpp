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

/* Dynamic reconfigure callback function (from GUI)
 * Subscribes to the values set by user for desired_yaw, trajectory_type, and speed from the graphical interface
 */
void dynamicReconfigureCallback(anafi_autonomy::setTrajectoryConfig &config, uint32_t level){
    trajectory_type = config.trajectory;
    pose_d << config.x_d, config.y_d, config.z_d, config.yaw_d;
}

// Constructor
Trajectory::Trajectory(int argc, char** argv){
    ros::init(argc, argv, "trajectory");
    ros::NodeHandle node_handle;
    
    // Publishers
    pose_publisher = node_handle.advertise<anafi_autonomy::PoseCommand>("/anafi/reference_pose", 1);
    velocity_publisher = node_handle.advertise<anafi_autonomy::VelocityCommand>("/anafi/reference_velocity", 1);
    axes_publisher = node_handle.advertise<anafi_autonomy::AxesCommand>("/anafi/command_offboard", 1);
}

// Destructor
Trajectory::~Trajectory(){
    ros::shutdown();
    exit(0);
}

void Trajectory::run(){
    ROS_INFO("Trajectory is running...");

    // Setup dynamic reconfigure (GUI) connections
    dynamic_reconfigure::Server<anafi_autonomy::setTrajectoryConfig> server;
    dynamic_reconfigure::Server<anafi_autonomy::setTrajectoryConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    Vector4d pose;
    Vector4d velocity;
    Vector3d mode;

    double dt = (double)1/100;
    ros::Rate rate((double)1/dt);
    // Main loop
    while(ros::ok()){
        rate.sleep();

        // Main switch case
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
        anafi_autonomy::PoseCommand pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.x = pose(0);
        pose_msg.y = pose(1);
        pose_msg.z = pose(2);
        pose_msg.yaw = pose(3);
        pose_publisher.publish(pose_msg);

        // Publish the corresponding reference trajectory velocity
        anafi_autonomy::VelocityCommand velocity_msg;
        velocity_msg.header.stamp = ros::Time::now();
        velocity_msg.vx = velocity(0);
        velocity_msg.vy = velocity(1);
        velocity_msg.vz = velocity(2);
        velocity_msg.yaw_rate = velocity(3);
        velocity_publisher.publish(velocity_msg);

        // Publish the corresponding reference for each axis
        anafi_autonomy::AxesCommand axes_msg;
        axes_msg.header.stamp = ros::Time::now();
        axes_msg.horizontal_mode = mode(0);
        axes_msg.x = pose(0);
        axes_msg.y = pose(1);
        axes_msg.vertical_mode = mode(1);
        axes_msg.z = pose(2);
        axes_msg.heading_mode = mode(2);
        axes_msg.yaw = pose(3);
        axes_publisher.publish(axes_msg);

        ros::spinOnce();
    }
}

int main(int argc, char** argv){
    Trajectory* controller = new Trajectory(argc, argv);
    controller->run();
}
