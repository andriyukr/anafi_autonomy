# ROS Package for Parrot Anafi Drones Navigation
This ROS package contains interface to control drones from Parrot Anafi family (4K, Thermal, USA, AI, Sphinx).

## Overview

**Author:** Andriy Sarabakha<br />
**Affiliation:** [Nanyang Technological University (NTU)](https://www.ntu.edu.sg), Singapore<br />
**Maintainer:** Andriy Sarabakha, andriy.sarabakha@ntu.edu.sg

**Keywords:** Parrot, UAV, controller

This package has been tested in **ROS Melodic**/**Ubuntu 18.04** and **ROS Noetic**/**Ubuntu 20.04**.

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### Dependencies

- [olympe_bridge](https://github.com/andriyukr/olympe_bridge) - ROS bridge for Parrot Olympe SDK:

	cd ~/catkin_ws/src
	git clone https://github.com/andriyukr/olympe_bridge.git
	sudo chmod -R 777 olympe_bridge/
	catkin build
    
## Clone

Clone the latest version from this repository into your ROS workspace:

	cd ~/catkin_ws/src
	git clone https://github.com/andriyukr/anafi_autonomy.git
	sudo chmod -R 777 anafi_autonomy/
  
## Install

Build the package:

	source ~/.bashrc
	catkin build


## Run

Execute to connect to the drone:

	roslaunch anafi_autonomy control_anafi.launch

