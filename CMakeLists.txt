cmake_minimum_required(VERSION 3.0.2)
project(anafi_autonomy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  geometry_msgs
  roslib
  sensor_msgs
  tf
  message_generation
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_python_setup()

add_message_files(
  FILES
  DesiredCommand.msg
  PoseCommand.msg
  VelocityCommand.msg
  AttitudeCommand.msg
  PilotingCommand.msg 
  MoveByCommand.msg 
  MoveToCommand.msg 
  SkyControllerCommand.msg 
  CameraCommand.msg
  GimbalCommand.msg
  KeyboardDroneCommand.msg
  KeyboardCameraCommand.msg
  TargetTrajectory.msg
)

add_service_files(
  FILES
  PilotedPOI.srv
  FlightPlan.srv
  FollowMe.srv
  Location.srv
  String.srv
)

generate_messages(
  DEPENDENCIES 
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/setAnafi.cfg 
  cfg/setSafeAnafi.cfg
  cfg/setTrajectory.cfg
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime 
  roscpp 
  rospy 
  std_msgs
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(safe_anafi src/safeAnafi.cpp)
target_link_libraries(safe_anafi ${catkin_LIBRARIES})
add_dependencies(safe_anafi ${PROJECT_NAME}_gencfg)

add_executable(teleop_key src/teleopKey.cpp)
target_link_libraries(teleop_key ${catkin_LIBRARIES})
add_dependencies(teleop_key ${PROJECT_NAME}_gencfg)

add_executable(trajectory src/trajectory.cpp)
target_link_libraries(trajectory ${catkin_LIBRARIES})
add_dependencies(trajectory ${PROJECT_NAME}_gencfg)
