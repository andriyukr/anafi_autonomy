cmake_minimum_required(VERSION 3.5)

project(anafi_autonomy)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(anafi_ros_interfaces REQUIRED)

set(msg_files
	"msg/ReferenceCommand.msg"
	"msg/PoseCommand.msg"
	"msg/VelocityCommand.msg"
	"msg/AttitudeCommand.msg"
	"msg/KeyboardCommand.msg"
	"msg/Position.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
	${msg_files}
	DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
	include 
	${EIGEN3_INCLUDE_DIRS}
)

add_executable(safe_anafi src/safeAnafi.cpp)
ament_target_dependencies(safe_anafi 
	rclcpp 
	std_msgs
	geometry_msgs
	sensor_msgs
	nav_msgs
	std_srvs
	tf2
	anafi_ros_interfaces)
target_link_libraries(safe_anafi "${cpp_typesupport_target}")

add_executable(teleop_key src/teleopKey.cpp)
ament_target_dependencies(teleop_key 
	rclcpp 
	std_msgs
	geometry_msgs)
target_link_libraries(teleop_key "${cpp_typesupport_target}")

add_executable(trajectory src/trajectory.cpp)
ament_target_dependencies(trajectory 
	rclcpp)
target_link_libraries(trajectory "${cpp_typesupport_target}")

add_executable(example src/example.cpp)
ament_target_dependencies(example 
	rclcpp 
	std_msgs
	geometry_msgs
	sensor_msgs)
target_link_libraries(example "${cpp_typesupport_target}")

install(TARGETS
	safe_anafi
	teleop_key
	trajectory
	example
	DESTINATION lib/${PROJECT_NAME})
	
# Install launch files
install(DIRECTORY
	launch
	config
	missions
	DESTINATION share/${PROJECT_NAME}/
)
	
ament_package()
