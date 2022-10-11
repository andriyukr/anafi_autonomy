/** *************************** teleopKey.cpp ***************************
 *
 * This code converts the keyboard inputs from user into different functions
 *
 * *********************************************************************/

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <pthread.h>
#include <chrono>
#include <anafi_autonomy/KeyboardDroneCommand.h>
#include <anafi_autonomy/KeyboardCameraCommand.h>

#define KEYCODE_a       97
#define KEYCODE_d       100
#define KEYCODE_w       119
#define KEYCODE_s       115
#define KEYCODE_h       104
#define KEYCODE_r       114
#define KEYCODE_t       116
#define KEYCODE_l       108
#define KEYCODE_b       98
#define KEYCODE_c       99
#define KEYCODE_H       72
#define KEYCODE_R       82
#define KEYCODE_L       76

#define KEYCODE_LEFT    1792836
#define KEYCODE_RIGHT   1792835
#define KEYCODE_UP      1792833
#define KEYCODE_DOWN    1792834
#define KEYCODE_Esc     27
#define KEYCODE_Insert  458961534
#define KEYCODE_SPACE   32

#define KEYCODE_F1      1789776
#define KEYCODE_F2      1789777
#define KEYCODE_F4      1789779
#define KEYCODE_F5      117494068606
#define KEYCODE_F6      117494069118
#define KEYCODE_F7      117494069374
#define KEYCODE_F8      117494069630
#define KEYCODE_F9      117494132862
#define KEYCODE_F12     117494133886

#define KEYCODE_ACCENT  96

#define KEYCODE_8       56
#define KEYCODE_4       52
#define KEYCODE_5       53
#define KEYCODE_6       54
#define KEYCODE_2       50
#define KEYCODE_MINUS   45
#define KEYCODE_PLUS    43
#define KEYCODE_Enter   10
#define KEYCODE_0       48
#define KEYCODE_DOT     46
#define KEYCODE_SLASH   47

using namespace std;

int kfd = 0;
unsigned long int key = 0;
struct termios cooked, raw;

class Teleop{
private:
    ros::NodeHandle node_handle;
    ros::Publisher action_publisher;
    ros::Publisher drone_publisher;
    ros::Publisher camera_publisher;
    std_msgs::Int8 action;

public:
    Teleop(){
        // Publishers
        action_publisher = node_handle.advertise<std_msgs::Int8>("keyboard/action", 1);
        drone_publisher = node_handle.advertise<anafi_autonomy::KeyboardDroneCommand>("keyboard/drone_command", 1);
        camera_publisher = node_handle.advertise<anafi_autonomy::KeyboardCameraCommand>("keyboard/camera_command", 1);
    }

    ~Teleop(){
        tcsetattr(kfd, TCSANOW, &cooked);
        ros::shutdown();
        exit(0);
    }

    void keyLoop(){
        ros::Rate rate(30);

        while(ros::ok()){
            rate.sleep();

            action.data = 0;
            anafi_autonomy::KeyboardDroneCommand drone_msg;
            anafi_autonomy::KeyboardCameraCommand camera_msg;

            if(key != 0)
                ROS_DEBUG_STREAM("(loop) key = " << key);

            switch(key){
            case KEYCODE_H: // help
            case KEYCODE_h:
                cout << "Press: \n";
                cout << "Insert \t - arm \n";
                cout << "t \t - take-off \n";
                cout << "l \t - land \n";

                cout << "Esc \t - cut motors \n";
                cout << "UP \t - move forward \n";
                cout << "DOWN \t - move backward \n";
                cout << "LEFT \t - move left \n";
                cout << "RIGHT \t - move right \n";
                cout << "w \t - move up \n";
                cout << "s \t - move down \n";
                cout << "a \t - yaw clockwise \n";
                cout << "d \t - yaw counterclockwise \n";
                cout << "b \t - return to home \n";
                cout << "SPACE \t - halt \n";
                cout << "r \t - reset pose \n";
                break;

                /* UAV commands */
            case KEYCODE_Insert: // arm
                action.data = 1;               
                break;
            case KEYCODE_t: // take-off
                action.data = 2;
                break;
            case KEYCODE_SPACE: // halt
                action.data = 3;
                break;
            case KEYCODE_L: // land
            case KEYCODE_l:
                action.data = 4;
                break;
            case KEYCODE_Esc: // disarm!
                action.data = 5;
                break;
            case KEYCODE_R: // reset pose
            case KEYCODE_r:
                action.data = 6;
                break;
            case KEYCODE_b: // return-to-home
                action.data = 7;
                break;
            case KEYCODE_F1: // remote control!
                action.data = 101;
                break;
            case KEYCODE_F2: // offboard control!
                action.data = 102;
                break;
            case KEYCODE_F4: // reboot!
                action.data = 110;
                break;
            case KEYCODE_F5: // start mission
                action.data = 11;
                break;
            case KEYCODE_F6: // pause mission
                action.data = 12;
                break;
            case KEYCODE_F7: // stop mission
                action.data = 13;
                break;
            case KEYCODE_F12: // calibrate megnetometer!
                action.data = 111;
                break;

                /* UAV movements */
            case KEYCODE_a:
                drone_msg.yaw = 1;
                break;
            case KEYCODE_d:
                drone_msg.yaw = -1;
                break;
            case KEYCODE_w:
                drone_msg.z = 1;
                break;
            case KEYCODE_s:
                drone_msg.z = -1;
                break;
            case KEYCODE_RIGHT: // move right
                drone_msg.y = -1;
                break;
            case KEYCODE_LEFT: // move left
                drone_msg.y = 1;
                break;
            case KEYCODE_UP: // move forward
                drone_msg.x = 1;
                break;
            case KEYCODE_DOWN: // move backward
                drone_msg.x = -1;
                break;

                /* gimbal commadns */
            case KEYCODE_F9: // calibrate gimbal
                camera_msg.action = 111;
                break;
            case KEYCODE_5: // reset to (0, 0)
                camera_msg.action = 11;
                break;
            case KEYCODE_4: // pitch left
                camera_msg.roll = -1;
                break;
            case KEYCODE_6: // pitch right
                camera_msg.roll = 1;
                break;
            case KEYCODE_8: // roll up
                camera_msg.pitch = -1;
                break;
            case KEYCODE_2: // roll down
                camera_msg.pitch = 1;
                break;

                /* camera commands */
            case KEYCODE_PLUS: // zoom in
                camera_msg.zoom = 1;
                break;
            case KEYCODE_MINUS: // zoom out
                camera_msg.zoom = -1;
                break;
            case KEYCODE_Enter: // take picture
                camera_msg.action = 1;
                break;
            case KEYCODE_0: // start recording
                camera_msg.action = 2;
                break;
            case KEYCODE_DOT: // stop recording
                camera_msg.action = 3;
                break;
            case KEYCODE_SLASH: // download media
                camera_msg.action = 4;
                break;
            }

            action_publisher.publish(action);

            drone_msg.header.stamp = ros::Time::now();
            drone_msg.header.frame_id = "body";
            drone_publisher.publish(drone_msg);

            camera_msg.header.stamp = ros::Time::now();
            camera_msg.header.frame_id = "body";
            camera_publisher.publish(camera_msg);

            key = 0;
        }
    }
};

void quit(int sig){
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

// This function is called from a thread
void *readKey(void *) {
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    tcsetattr(kfd, TCSANOW, &raw);

    char c = 0;
    while(true){
        auto t_start = std::chrono::high_resolution_clock::now();
        read(kfd, &c, 1); // get the next event from the keyboard
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
        if(elapsed_time_ms < 1)
            key = (key<<8) + c;
        else
            key = c;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "keyboard_teleop");

    ROS_WARN("Press 'H' to show the keyboard mapping");

    pthread_t t;
    // Launch a thread
    pthread_create(&t, NULL, readKey, NULL);

    signal(SIGINT, quit);

    Teleop teleop;
    teleop.keyLoop();

    ros::spin();

    return 0;
}
