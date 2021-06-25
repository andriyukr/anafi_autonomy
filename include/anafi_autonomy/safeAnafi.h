#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <anafi_autonomy/setSafeAnafiConfig.h>
#include <anafi_autonomy/PoseCommand.h>
#include <anafi_autonomy/VelocityCommand.h>
#include <anafi_autonomy/AttitudeCommand.h>
#include <anafi_autonomy/AxesCommand.h>
#include <anafi_autonomy/PilotingCommand.h>
#include <anafi_autonomy/MoveToCommand.h>
#include <anafi_autonomy/MoveByCommand.h>
#include <anafi_autonomy/CameraCommand.h>
#include <anafi_autonomy/GimbalCommand.h>
#include <anafi_autonomy/SkyControllerCommand.h>
#include <anafi_autonomy/KeyboardMoveCommand.h>
#include <anafi_autonomy/KeyboardCameraCommand.h>
#include <anafi_autonomy/TakePhoto.h>

#define FILTER_SIZE 		3
#define COMMAND_NONE 		0
#define COMMAND_POSITION 	1
#define COMMAND_VELOCITY 	2
#define COMMAND_ANGLE 		3
#define COMMAND_RATE 		4
#define MODE_HORIZONTAL 	0
#define MODE_VERTICAL 		1
#define MODE_HEADING 		2

#define BOUND3(x, min_x, max_x) (x > max_x ? max_x : (x < min_x ? min_x : x))
#define BOUND2(x, min_max) BOUND3(x, -min_max, min_max)
#define GET_MACRO(_1, _2, _3, NAME, ...) NAME
#define BOUND(...) GET_MACRO(__VA_ARGS__, BOUND3, BOUND2)(__VA_ARGS__)

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;

// Subsribers
Subscriber command_subscriber;
Subscriber command_keyboard_subscriber;
Subscriber command_camera_subscriber;
Subscriber skycontroller_subscriber;
Subscriber desired_pose_subscriber;
Subscriber desired_velocity_subscriber;
Subscriber desired_attitude_subscriber;
Subscriber desired_trajectory_subscriber;
Subscriber command_axis_subscriber;
Subscriber optitrack_subscriber;
Subscriber aruco_subscriber;
Subscriber gps_subscriber;
Subscriber odometry_subscriber;

// Publishers
Publisher emergency_publisher;
Publisher takeoff_publisher;
Publisher land_publisher;
Publisher offboard_publisher;
Publisher moveto_publisher;
Publisher moveby_publisher;
Publisher rpyg_publisher;
Publisher camera_publisher;
Publisher gimbal_publisher;
Publisher euler_publisher;
Publisher odometry_publisher;
Publisher desired_velocity_publisher;
Publisher acceleration_publisher;
Publisher mode_publisher;

// Clients
ros::ServiceClient take_photo_client;
ros::ServiceClient start_recording_client;
ros::ServiceClient stop_recording_client;
ros::ServiceClient reset_zoom_client;
ros::ServiceClient reset_gimbal_client;

// Flags
bool arm = false;
bool land = true;
bool localization = false;
bool fixed_frame = false;
bool world_frame = false;
bool stop_gimbal = false;
bool stop_zoom = false;

// Position
Vector3d position = Vector3d::Zero();
Vector3d position_error = Vector3d::Zero();
Vector3d position_error_i = Vector3d::Zero();
Vector3d position_error_d = Vector3d::Zero();
MatrixXd bounds(2, 3); // min_x, min_y, min_z; max_x, max_y, max_z

// Attitude
Vector3d orientation = Vector3d::Zero();
double yaw = 0;
double initial_yaw = 0;

// Velocity
Vector3d velocity_error = Vector3d::Zero();
Vector3d velocity_error_i = Vector3d::Zero();
Vector3d velocity_error_d = Vector3d::Zero();
Vector3d velocity = Vector3d::Zero();
Vector3d velocity_old = Vector3d::Zero();
Vector3d velocity_d = Vector3d::Zero();
vector<Twist> velocities;
Vector3d rates = Vector3d::Zero();

// Acceleration
Vector3d acceleration = Vector3d::Zero();
MatrixXd accelerations = MatrixXd::Zero(FILTER_SIZE, 3);

// Commands
Vector4d move_command = Vector4d::Zero();
Vector4d command_keyboard = Vector4d::Zero();
Vector4d command_skycontroller = Vector4d::Zero();
Vector4d command_offboard = Vector4d::Zero();
Vector4d desired_pose = Vector4d::Zero();
Vector4d desired_velocity = Vector4d::Zero();
Vector4d desired_attitude = Vector4d::Zero();
Vector4d command_move = Vector4d::Zero();
Vector3i mode_keyboard = Vector3i::Zero();
Vector3i mode_skycontroller = Vector3i::Zero();
Vector3i mode_offboard = Vector3i::Zero();
Vector3i mode_move = Vector3i::Zero();
Vector2d gimbal_command = Vector2d::Zero();
double zoom_command = 0;

// Gains
double k_p_position = 0;
double k_i_position = 0;
double k_d_position = 0;
double max_i_position = 0;
double k_p_velocity = 0;
double k_d_velocity = 0;
double k_p_yaw = 0;

// Parameters
int controller = 2;
double max_tilt;
double max_vertical_speed;
double max_horizontal_speed;
double max_yaw_rotation_speed;

Time time_old;

class SafeAnafi{
    public:
    	SafeAnafi(int, char**);
        ~SafeAnafi();
        void run();
    private:

};