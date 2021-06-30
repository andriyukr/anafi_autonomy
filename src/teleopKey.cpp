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
#include <anafi_autonomy/KeyboardMoveCommand.h>
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
#define KEYCODE_H       72
#define KEYCODE_R       82
#define KEYCODE_L       76

#define KEYCODE_LEFT    68
#define KEYCODE_RIGHT   67
#define KEYCODE_UP      65
#define KEYCODE_DOWN    66
#define KEYCODE_Esc     27
#define KEYCODE_Insert  126
#define KEYCODE_SPACE   32

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

#define KEYCODE_F1      80
#define KEYCODE_F2      81

using namespace std;

int kfd = 0;
char c = 0;
int seq = 1;
struct termios cooked, raw;

class Teleop{
private:
    ros::NodeHandle node_handle;
    ros::Publisher command_publisher;
    ros::Publisher move_publisher;
    ros::Publisher camera_publisher;
    std_msgs::Int8 command;

public:
    Teleop(){
        command.data = 0;

        // Publishers
        command_publisher = node_handle.advertise<std_msgs::Int8>("/keyboard/command_meta", 1);
        move_publisher = node_handle.advertise<anafi_autonomy::KeyboardMoveCommand>("/keyboard/command_move", 1);
        camera_publisher = node_handle.advertise<anafi_autonomy::KeyboardCameraCommand>("/keyboard/command_camera", 1);
    }

    ~Teleop(){
        tcsetattr(kfd, TCSANOW, &cooked);
        ros::shutdown();
        exit(0);
    }

    void keyLoop(){
        ros::Rate rate(20);

        while(ros::ok()){
            rate.sleep();

            anafi_autonomy::KeyboardMoveCommand move_msg;
            anafi_autonomy::KeyboardCameraCommand camera_msg;

            if(c != 0){
                ROS_DEBUG_STREAM("key = " << (int)c << endl);
            }

            switch(c){
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
                cout << "SPACE \t - hover \n";
                cout << "r \t - reset pose \n";
                break;

                /* UAV commands */
            case KEYCODE_Insert: // arm
                command.data = 1;
                command_publisher.publish(command);
                break;
            case KEYCODE_t: // take-off
                command.data = 2;
                command_publisher.publish(command);
                break;
            case KEYCODE_SPACE: // hower
                command.data = 3;
                command_publisher.publish(command);
                command_publisher.publish(command);
                break;
            case KEYCODE_L: // land
            case KEYCODE_l:
                command.data = 4;
                command_publisher.publish(command);
                command_publisher.publish(command);
                break;
            case KEYCODE_Esc: // disarm!
                command.data = 5;
                command_publisher.publish(command);
                command_publisher.publish(command);
                break;
            case KEYCODE_R: // reset pose
            case KEYCODE_r:
                command.data = 6;
                command_publisher.publish(command);
                break;
            case KEYCODE_b: // return-to-home
                command.data = 7;
                command_publisher.publish(command);
                break;
            case KEYCODE_F1: // remote control!
                command.data = 101;
                command_publisher.publish(command);
                command_publisher.publish(command);
                break;
            case KEYCODE_F2: // offboard control!
                command.data = 102;
                command_publisher.publish(command);
                command_publisher.publish(command);
                break;

                /* UAV movements */
            case KEYCODE_a:
                move_msg.yaw = 1;
                break;
            case KEYCODE_d:
                move_msg.yaw = -1;
                break;
            case KEYCODE_w:
                move_msg.z = 1;
                break;
            case KEYCODE_s:
                move_msg.z = -1;
                break;
            case KEYCODE_RIGHT: // move right
                move_msg.y = -1;
                break;
            case KEYCODE_LEFT: // move left
                move_msg.y = 1;
                break;
            case KEYCODE_UP: // move forward
                move_msg.x = 1;
                break;
            case KEYCODE_DOWN: // move backward
                move_msg.x = -1;
                break;

                /* gimbal commadns */
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
            }

            move_msg.header.stamp = ros::Time::now();
            move_msg.header.frame_id = "/body";
            move_publisher.publish(move_msg);

            camera_msg.header.stamp = ros::Time::now();
            camera_msg.header.frame_id = "/body";
            camera_publisher.publish(camera_msg);

            c = 0;
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

    while(true)
        read(kfd, &c, 1); // get the next event from the keyboard
}

int main(int argc, char** argv){
    ros::init(argc, argv, "keyboard_teleop");

    ROS_WARN("Press H for help!!!");

    pthread_t t;
    // Launch a thread
    pthread_create(&t, NULL, readKey, NULL);

    signal(SIGINT, quit);

    Teleop teleop;
    teleop.keyLoop();

    ros::spin();

    return 0;
}
