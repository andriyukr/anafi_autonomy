cmake_minimum_required(VERSION 3.0.2)
project(anafi_autonomy)

add_compile_options(-std=c++17)

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
get_filename_component(extern "extern"
    REALPATH BASE_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(FETCHCONTENT_BASE_DIR ${extern})

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    dynamic_reconfigure
    geometry_msgs
    roslib
    sensor_msgs
    tf
    tf_conversions
    eigen_conversions
    message_generation
    cv_bridge
    image_transport
)

find_package(OpenCV REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost 1.54 COMPONENTS system thread coroutine context REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(robotp2p 0.3.0 REQUIRED)
find_package(asio REQUIRED)
find_package(haptic_udp REQUIRED)

# Get json
find_package(json)
if(json_FOUND)
    messsage(STATUS "Found json")
else()
    message(STATUS "json not found: getting it now")
    FetchContent_Declare(
        json
        GIT_REPOSITORY https://github.com/ArthurSonzogni/nlohmann_json_cmake_fetchcontent.git
        GIT_TAG v3.7.3)
    # FetchContent_MakeAvailable(json)
    FetchContent_GetProperties(json)
    if(NOT json_POPULATED)
        message("not json populated")
        FetchContent_Populate(json)
        add_subdirectory(${json_SOURCE_DIR} ${json_BINARY_DIR} EXCLUDE_FROM_ALL)
        message("json_SOURCE_DIR: ${json_SOURCE_DIR}")
        message("json_BINARY_DIR: ${json_BINARY_DIR}")
    endif()
endif()
message("json include dirs: ${json_INCLUDE_DIRS}")

catkin_python_setup()

add_message_files(
    FILES
    AxesCommand.msg
    PoseCommand.msg
    VelocityCommand.msg
    AttitudeCommand.msg
    PilotingCommand.msg 
    MoveByCommand.msg 
    MoveToCommand.msg 
    SkyControllerCommand.msg 
    CameraCommand.msg
    GimbalCommand.msg
    KeyboardMoveCommand.msg
    KeyboardCameraCommand.msg
)

add_service_files(
    FILES
    TakePhoto.srv
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
    ${haptic_udp_INCLUDE_DIRS}
    ${asio_INCLUDE_DIRS}
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

add_executable(erl_bridge_main
    src/erl_bridge.cpp
    src/erl_bridge_main.cpp)

target_link_libraries(erl_bridge_main
    ${catkin_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    robotp2p::robotp2p
    ${haptic_udp_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OPENSSL_LIBRARIES}
    nlohmann_json::nlohmann_json
)

add_executable(rtp_stream
    src/rtp_stream.cpp)

target_link_libraries(rtp_stream
    ${catkin_LIBRARIES}
)
