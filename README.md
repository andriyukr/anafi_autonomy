# ROS2 Package for Parrot Anafi Drones Navigation
This ROS2 package contains an interface to control drones from the Parrot Anafi family (4K, Thermal, USA, Ai, Sphinx).

## Overview

**Author:** Andriy Sarabakha<br />
**Affiliation:** [Nanyang Technological University (NTU)](https://www.ntu.edu.sg), Singapore<br />
**Maintainer:** Andriy Sarabakha, andriy.sarabakha@ntu.edu.sg

**Keywords:** Parrot, UAV, controller

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

This package has been tested with **python3** in **ROS2 Humble**/**Ubuntu 22.04**.

### Dependencies

- [anafi_ros](https://github.com/andriyukr/anafi_ros) - ROS bridge for Parrot Olympe SDK
      
- (optional) [xterm](https://invisible-island.net/xterm/xterm.html) - terminal emulator:

      sudo apt-get install xterm

### Clone

To build from source, clone the latest version from this repository into your ROS2 workspace and build the package using:

    cd ~/ros2_ws/src
    git clone -b ros2 https://github.com/andriyukr/anafi_autonomy.git
    sudo chmod -R 777 anafi_autonomy/
    cd ..
    colcon build

## Usage

### Method 1 (without xterm):

To control the drone,

1. in the terminal 1, run:

       ros2 launch anafi_autonomy anafi_autonomy_launch.py ip:='192.168.53.1' model:='ai'
         
   where
   * `ip` has to be
     * `'192.168.53.1'`, for connection through Skycontroller (recommended),
     * `'192.168.42.1'`, for direct connection to the drone through WiFi,
     * `'10.202.0.1'`, for connection to the simulated drone in Sphinx;
   * `model` can be `'4k'`, `'thermal'`, `'usa'` or `'ai'`, depending on the drone model you are connectiong to.

1. in terminal 2, run:

       ros2 run anafi_autonomy keyboard --ros-args -r __ns:=/anafi
    
The commands can be keyed in terminal 2.
    
### Method 2 (with xterm):

To control the drone, in the terminal, run:

    ros2 launch anafi_autonomy control_anafi_launch.py ip:='192.168.53.1' model:='ai'
         
where
* `ip` has to be
  * `'192.168.53.1'`, for connection through Skycontroller (recommended),
  * `'192.168.42.1'`, for direct connection to the drone through WiFi,
  * `'10.202.0.1'`, for connection to the simulated drone in Sphinx;
* `model` can be `'4k'`, `'thermal'`, `'usa'` or `'ai'`, depending on the drone model you are connectiong to.
    
A new window will pop up where commands can be keyed in.

### Demonstration Video

https://github.com/andriyukr/anafi_autonomy/assets/5682865/d29c227a-b389-4c62-a23b-3e2a12b1a5fe

> :warning: To pilot the drone using the keyboard, you must switch to offboard mode. To switch to offboard mode, <ins>press F2 in the action terminal</ins> or <ins>press the zoom reset button on the Skycontroller</ins>.
