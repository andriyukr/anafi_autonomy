#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <olympe_bridge_interfaces/msg/piloting_command.hpp>
#include <olympe_bridge_interfaces/msg/move_to_command.hpp>
#include <olympe_bridge_interfaces/msg/move_by_command.hpp>
#include <olympe_bridge_interfaces/msg/camera_command.hpp>
#include <olympe_bridge_interfaces/msg/gimbal_command.hpp>
#include <olympe_bridge_interfaces/msg/skycontroller_command.hpp>
#include <olympe_bridge_interfaces/srv/flight_plan.hpp>
#include <olympe_bridge_interfaces/srv/follow_me.hpp>
#include <olympe_bridge_interfaces/srv/photo.hpp>
#include <olympe_bridge_interfaces/srv/recording.hpp>

#include <anafi_autonomy/msg/reference_command.hpp>
#include <anafi_autonomy/msg/pose_command.hpp>
#include <anafi_autonomy/msg/velocity_command.hpp>
#include <anafi_autonomy/msg/attitude_command.hpp>
#include <anafi_autonomy/msg/keyboard_drone_command.hpp>
#include <anafi_autonomy/msg/keyboard_camera_command.hpp>

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

using namespace std;
using namespace std::chrono_literals;
using namespace Eigen;

using std::placeholders::_1;

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

class SafeAnafi : public rclcpp::Node{
	public:
		SafeAnafi();

	private:			
		// Subsribers	
		rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr action_subscriber;
		rclcpp::Subscription<olympe_bridge_interfaces::msg::SkycontrollerCommand>::SharedPtr command_skycontroller_subscriber;
		rclcpp::Subscription<anafi_autonomy::msg::KeyboardDroneCommand>::SharedPtr command_keyboard_subscriber;
		rclcpp::Subscription<anafi_autonomy::msg::KeyboardCameraCommand>::SharedPtr command_camera_subscriber;
		rclcpp::Subscription<anafi_autonomy::msg::PoseCommand>::SharedPtr reference_pose_subscriber;
		rclcpp::Subscription<anafi_autonomy::msg::VelocityCommand>::SharedPtr reference_velocity_subscriber;
		rclcpp::Subscription<anafi_autonomy::msg::AttitudeCommand>::SharedPtr reference_attitude_subscriber;
	    	rclcpp::Subscription<anafi_autonomy::msg::ReferenceCommand>::SharedPtr reference_command_subscriber;
		rclcpp::Subscription<anafi_autonomy::msg::VelocityCommand>::SharedPtr derivative_command_subscriber;
		rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr reference_gimbal_subscriber;
		rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr reference_zoom_subscriber;
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscriber;
		rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscriber;
		rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr altitude_subscriber;
		rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr attitude_subscriber;
		rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr speed_subscriber;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber;

		
		// Publishers
		rclcpp::Publisher<olympe_bridge_interfaces::msg::PilotingCommand>::SharedPtr rpyg_publisher;
		rclcpp::Publisher<olympe_bridge_interfaces::msg::MoveToCommand>::SharedPtr moveto_publisher;
		rclcpp::Publisher<olympe_bridge_interfaces::msg::MoveByCommand>::SharedPtr moveby_publisher;
		rclcpp::Publisher<olympe_bridge_interfaces::msg::CameraCommand>::SharedPtr camera_publisher;
		rclcpp::Publisher<olympe_bridge_interfaces::msg::GimbalCommand>::SharedPtr gimbal_publisher;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
		rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr desired_velocity_publisher; // FOR DEBUGGING
		rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr acceleration_publisher; // FOR DEBUGGING
		rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr mode_publisher; // FOR DEBUGGING
		
		// Clients
		rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr emergency_client;
		rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr halt_client;
		rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_client;
		rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr takeoff_client;
		rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr land_client;
		rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr rth_client;
		rclcpp::Client<olympe_bridge_interfaces::srv::FlightPlan>::SharedPtr flightplan_upload_client;
		rclcpp::Client<olympe_bridge_interfaces::srv::FlightPlan>::SharedPtr flightplan_start_client;
		rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr flightplan_pause_client;
		rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr flightplan_stop_client;
		rclcpp::Client<olympe_bridge_interfaces::srv::FollowMe>::SharedPtr followme_start_client;
		rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr followme_stop_client;
		rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr offboard_client;
		rclcpp::Client<olympe_bridge_interfaces::srv::Photo>::SharedPtr take_photo_client;
		rclcpp::Client<olympe_bridge_interfaces::srv::Recording>::SharedPtr start_recording_client;
		rclcpp::Client<olympe_bridge_interfaces::srv::Recording>::SharedPtr stop_recording_client;
		rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_zoom_client;
		rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_gimbal_client;
		rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr download_media_client;
		rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr calibrate_magnetometer_client;
		rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr calibrate_gimbal_client;
		rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reboot_client;

		// Messages
		olympe_bridge_interfaces::msg::PilotingCommand rpyg_msg;

		// Requests
		std::shared_ptr<std_srvs::srv::Trigger_Request_<std::allocator<void>>> trigger_request;
		std::shared_ptr<std_srvs::srv::SetBool_Request_<std::allocator<void>>> true_request;
		std::shared_ptr<std_srvs::srv::SetBool_Request_<std::allocator<void>>> false_request;
		std::shared_ptr<olympe_bridge_interfaces::srv::Photo_Request_<std::allocator<void>>> photo_request;
		std::shared_ptr<olympe_bridge_interfaces::srv::Recording_Request_<std::allocator<void>>> recording_request;
		
		// Parameter client
		rclcpp::AsyncParametersClient::SharedPtr parameters_client;
		rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_events_subscriber;
		
		// Timers
		rclcpp::TimerBase::SharedPtr timer;
		
		// Callback
		OnSetParametersCallbackHandle::SharedPtr callback;

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
	        bool attitude_available = false;
	        bool pose_available = false;
	       	bool odometry_available = false;
	       	bool altitude_available = false;
	       	bool velocity_available = false;
		
		// Variables
		States state = LANDED;
		Actions action;

		// Position
		Vector3d position = Vector3d::Zero();
		Vector3d position_error = Vector3d::Zero();
		Vector3d position_error_i = Vector3d::Zero();
		Vector3d position_error_d = Vector3d::Zero();
		Matrix<double, 3, 2> bounds {
			{0, 0}, 	// {x_min, x_max}
			{0, 0}, 	// {y_min, y_max}
			{0, 0} 		// {z_min, z_max}
		};

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
		vector<geometry_msgs::msg::Twist> velocities;
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
		Vector4d derivative_command = Vector4d::Zero();
		Vector4d command_move = Vector4d::Zero();
		Vector3i mode_keyboard = Vector3i::Zero();
		Vector3i mode_skycontroller = Vector3i::Zero();
		Vector3i mode_offboard = Vector3i::Zero();
		Vector3i mode_move = Vector3i::Zero();
		Vector3d gimbal_command = Vector3d::Zero();
		Vector3d controller_gimbal_command = Vector3d::Zero();
		Vector3d keyboard_gimbal_command = Vector3d::Zero();
		Vector3d offboard_gimbal_command = Vector3d::Zero();
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
		rclcpp::Time time;
		rclcpp::Time time_old;
		double dt = DBL_MAX;
		
		// Callbacks
		void timer_callback();
		void parameter_events_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
		rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters);	
		void actionCallback(const std_msgs::msg::Int8& action_msg);
		void skycontrollerCallback(const olympe_bridge_interfaces::msg::SkycontrollerCommand& command_msg);
		void keyboardCallback(const anafi_autonomy::msg::KeyboardDroneCommand& command_msg);
		void cameraCallback(const anafi_autonomy::msg::KeyboardCameraCommand& command_msg);
		void referencePoseCallback(const anafi_autonomy::msg::PoseCommand& command_msg);
		void referenceVelocityCallback(const anafi_autonomy::msg::VelocityCommand& command_msg);
		void referenceAttitudeCallback(const anafi_autonomy::msg::AttitudeCommand& command_msg);
		void referenceCommandCallback(const anafi_autonomy::msg::ReferenceCommand& command_msg);
		void derivativeCommandCallback(const anafi_autonomy::msg::VelocityCommand& derivative_msg);
		void referenceGimbalCallback(const geometry_msgs::msg::Vector3& command_msg);
		void referenceZoomCallback(const std_msgs::msg::Float32& command_msg);
		void stateCallback(const std_msgs::msg::String& state_msg);
		void gpsCallback(const sensor_msgs::msg::NavSatFix& gps_msg);
		void altitudeCallback(const std_msgs::msg::Float32& altitude_msg);
		void attitudeCallback(const geometry_msgs::msg::QuaternionStamped& quaternion_msg);
		void speedCallback(const geometry_msgs::msg::Vector3Stamped& speed_msg);
		void poseCallback(const geometry_msgs::msg::PoseStamped& pose_msg);
		void odometryCallback(const nav_msgs::msg::Odometry& odometry_msg);

		// Functions
		void stateMachine();
		void parameter_assign(rcl_interfaces::msg::Parameter &parameter);
		States resolveState(std::string input);
		void controllers();
		geometry_msgs::msg::Twist filter_mean_velocities(geometry_msgs::msg::Twist v);
		Eigen::Vector3d filter_mean_acceleration(Eigen::Ref<Eigen::VectorXd> a);
		Eigen::Vector3d filter_polinomial_acceleration(Eigen::Ref<Eigen::VectorXd> a);
		double denormalizeAngle(double a1, double a2);
};
