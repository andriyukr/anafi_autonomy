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

// Dynamic reconfigure callback function (from GUI)
/*void dynamicReconfigureCallback(anafi_autonomy::setTrajectoryConfig &config, uint32_t level){
	trajectory_type = config.trajectory;
	pose_d << config.x_d, config.y_d, config.z_d, config.yaw_d;
}*/

Trajectory::Trajectory() : Node("trajectory"){
	RCLCPP_INFO(this->get_logger(), "Trajectory is running...");

	pose_publisher = this->create_publisher<anafi_autonomy::msg::PoseCommand>("drone/reference_pose", rclcpp::SystemDefaultsQoS());
	velocity_publisher = this->create_publisher<anafi_autonomy::msg::VelocityCommand>("drone/reference_velocity", rclcpp::SystemDefaultsQoS());
	axes_publisher = this->create_publisher<anafi_autonomy::msg::AxesCommand>("drone/command_offboard", rclcpp::SystemDefaultsQoS());
	
	timer = this->create_wall_timer(10ms, std::bind(&Trajectory::timer_callback, this));
}

void Trajectory::timer_callback(){
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
	pose_msg.header.stamp = this->get_clock()->now();
	pose_msg.x = pose(0);
	pose_msg.y = pose(1);
	pose_msg.z = pose(2);
	pose_msg.yaw = pose(3);
	pose_publisher->publish(pose_msg);

	// Publish the corresponding reference trajectory velocity
	velocity_msg.header.stamp = this->get_clock()->now();
	velocity_msg.vx = velocity(0);
	velocity_msg.vy = velocity(1);
	velocity_msg.vz = velocity(2);
	velocity_msg.yaw_rate = velocity(3);
	velocity_publisher->publish(velocity_msg);

	// Publish the corresponding reference for each axis
	axes_msg.header.stamp = this->get_clock()->now();
	axes_msg.horizontal_mode = mode(0);
	axes_msg.x = pose(0);
	axes_msg.y = pose(1);
	axes_msg.vertical_mode = mode(1);
	axes_msg.z = pose(2);
	axes_msg.heading_mode = mode(2);
	axes_msg.yaw = pose(3);
	axes_publisher->publish(axes_msg);
}

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	
	rclcpp::spin(std::make_shared<Trajectory>());
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Trajectory is stopping...");
	rclcpp::shutdown();
	return 0;
}
