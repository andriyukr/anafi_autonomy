#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
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
#include <std_srvs/SetBool.h>
#include <dynamic_reconfigure/server.h>

#include <olympe_bridge/PilotingCommand.h>
#include <olympe_bridge/MoveToCommand.h>
#include <olympe_bridge/MoveByCommand.h>
#include <olympe_bridge/CameraCommand.h>
#include <olympe_bridge/GimbalCommand.h>
#include <olympe_bridge/SkycontrollerCommand.h>
#include <olympe_bridge/FlightPlan.h>
#include <olympe_bridge/FollowMe.h>

#include <anafi_autonomy/setSafeAnafiConfig.h>
#include <anafi_autonomy/PoseCommand.h>
#include <anafi_autonomy/VelocityCommand.h>
#include <anafi_autonomy/AttitudeCommand.h>
#include <anafi_autonomy/DesiredCommand.h>
#include <anafi_autonomy/KeyboardDroneCommand.h>
#include <anafi_autonomy/KeyboardCameraCommand.h>

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

enum States{
	LANDED,
	MOTOR_RAMPING,
	USER_TAKEOFF,
	TAKINGOFF,
	HOVERING,
	FLYING,
	LANDING,
	EMERGENCY,
	INVALID
};
States state = LANDED;

enum Actions{
	NONE = 0,
	ARM = 1,
	TAKEOFF = 2,
	LAND = 4,
	DISARM = 5,
	HALT = 3,
	RESET_POSE = 6,
	RTH = 7,
	MISSION_START = 11,
	MISSION_PAUSE = 12,
	MISSION_STOP = 13,
	REMOTE_CONTROL = 101,
	OFFBOARD_CONTROL = 102,
	REBOOT = 110,
	CALIBRATE = 111
};
Actions action;

// Subsribers
Subscriber action_subscriber;
Subscriber command_keyboard_subscriber;
Subscriber command_camera_subscriber;
Subscriber skycontroller_subscriber;
Subscriber desired_pose_subscriber;
Subscriber desired_velocity_subscriber;
Subscriber desired_attitude_subscriber;
Subscriber desired_trajectory_subscriber;
Subscriber command_axis_subscriber;
Subscriber gimbal_command_subscriber;
Subscriber zoom_command_subscriber;
Subscriber optitrack_subscriber;
Subscriber aruco_subscriber;
Subscriber gps_subscriber;
Subscriber odometry_subscriber;
Subscriber state_subscriber;

// Publishers
Publisher moveto_publisher;
Publisher moveby_publisher;
Publisher rpyg_publisher;
Publisher camera_publisher;
Publisher gimbal_publisher;
Publisher odometry_publisher;
Publisher desired_velocity_publisher;
Publisher acceleration_publisher;
Publisher mode_publisher;

// Clients
ServiceClient emergency_client;
ServiceClient halt_client;
ServiceClient arm_client;
ServiceClient takeoff_client;
ServiceClient land_client;
ServiceClient rth_client;
ServiceClient flightplan_upload_client;
ServiceClient flightplan_start_client;
ServiceClient flightplan_pause_client;
ServiceClient flightplan_stop_client;
ServiceClient followme_start_client;
ServiceClient followme_stop_client;
ServiceClient offboard_client;
ServiceClient take_photo_client;
ServiceClient start_recording_client;
ServiceClient stop_recording_client;
ServiceClient reset_zoom_client;
ServiceClient reset_gimbal_client;
ServiceClient download_media_client;
ServiceClient calibrate_magnetometer_client;
ServiceClient calibrate_gimbal_client;
ServiceClient reboot_client;

// Messages
olympe_bridge::PilotingCommand rpyg_msg;

// Services
std_srvs::Empty empty_srv;
std_srvs::SetBool true_srv;
std_srvs::SetBool false_srv;

// Flags
bool armed = false;
bool localization = false;
bool fixed_frame = false;
bool world_frame = false;
bool stop_gimbal = false;
bool stop_zoom = false;
bool hand_launch = false;
bool takingoff_control = false;
bool landing_control = false;

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
Vector2d controller_gimbal_command = Vector2d::Zero();
Vector2d keyboard_gimbal_command = Vector2d::Zero();
Vector2d offboard_gimbal_command = Vector2d::Zero();
double zoom_command = 0;
double controller_zoom_command = 0;
double keyboard_zoom_command = 0;
double offboard_zoom_command = 0;

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
short mission_type = 0;
string flightplan_file;
short followme_mode = 0;

// Time
Time time_old;
double dt;

class SafeAnafi{
    public:
    	SafeAnafi(int, char**);
        ~SafeAnafi();
        void run();
    private:

};
