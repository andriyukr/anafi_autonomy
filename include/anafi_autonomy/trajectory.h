#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <anafi_autonomy/PoseCommand.h>
#include <anafi_autonomy/VelocityCommand.h>
#include <anafi_autonomy/AttitudeCommand.h>
#include <anafi_autonomy/AxesCommand.h>
#include <dynamic_reconfigure/server.h>
#include <anafi_autonomy/setTrajectoryConfig.h>
#include <Eigen/Dense>

using namespace std;
using namespace ros;
using namespace Eigen;

// Publishers
ros::Publisher pose_publisher;
ros::Publisher velocity_publisher;
ros::Publisher axes_publisher;

// Pose
Vector4d pose_d  = Vector4d::Zero();

// Trajectory type
int trajectory_type = 0;

class Trajectory{
        public:
          Trajectory(int, char**);
          ~Trajectory();
          void run();
};
