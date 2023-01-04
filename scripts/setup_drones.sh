#!/bin/bash

if [[ $# -ne 1 || $1 == '-h' || $1 == '--help' ]]
then
	echo "
	Setups the connection with multiple drones.
	     
	USAGE:                                                                 
	  $0 ['-h' | '--help' | number_drones]
	  '-h' | '--help'   Prints this message.
	  number_drones     Setups the connection with 'number_drones' drones.
		                                                               
	E.g. $0 2 - will connect to two drones on usb0 and usb1 interfaces.
	"                                                                      
	exit 0                                                             
fi

if ! [[ $1 =~ ^[0-9]+$ && $1 -ge 0 ]]
then
	echo "The number of drones must be a positive numeric value."
	exit 1
fi

number_drones=$1

dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for ((i=1; i<=$number_drones; i++))
do
	$dir/config_skycontroller_ip.sh clean usb$((i - 1)) 192.168.6$i.1 20$i
done

for ((i=1; i<=$number_drones; i++))
do
	$dir/config_skycontroller_ip.sh setup usb$((i - 1)) 192.168.6$i.1 20$i
done

for ((i=1; i<=$number_drones; i++))
do
	ros2 run topic_tools relay /keyboard/action /anafi$i/keyboard/action --ros-args -r __ns:=/anafi$i -r __node:=relay_action &
	ros2 run topic_tools relay /keyboard/camera_command /anafi$i/keyboard/camera_command --ros-args -r __ns:=/anafi$i -r __node:=relay_camera_command &
	ros2 run topic_tools relay /keyboard/drone_command /anafi$i/keyboard/drone_command --ros-args -r __ns:=/anafi$i -r __node:=relay_drone_command &
	ros2 launch anafi_autonomy safe_anafi_launch.py namespace:=anafi$i ip:=192.168.6$i.1 &
done

ros2 run rqt_reconfigure rqt_reconfigure &
ros2 run anafi_autonomy teleop_key

kill -9 $(pgrep -f reconfigure)
kill -9 $(pgrep -f anafi) &

for ((i=1; i<=$number_drones; i++))
do
	$dir/config_skycontroller_ip.sh clean usb$((i - 1)) 192.168.6$i.1 20$i
done

ros2 daemon stop
#ros2 daemon start
