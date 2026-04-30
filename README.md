# sentry_chassis_controller

## Overview

This is controller package for the DynamicX final assessment (see Chinese [requirement](doc/requirement.md)) 
in the Season 2026.

**Keywords:** RoboMaster, ROS, ros_control, PID, TF, Odometer

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: cqincat<br />
Affiliation: GDUT()<br />
Maintainer: cqincat, 2891343933@qq.com**

The sentry_chassis_controller package has been tested under [ROS] Noetic on respectively 18.04 and 20.04. This is
research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [rm_description](https://github.com/gdut-dynamic-x/rm_description)
- controller_interface
- forward_command_controller
- hardware_interface
- pluginlib

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone git@github.com:CQin-Cat/sentry_chassis_controller.git
    # git clone https://github.com/CQin-Cat/sentry_chassis_controller.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin build

## Usage

Run the simulation and controller with:

	roslaunch sentry_chassis_controller run_simulation_and_controller.launch

## Config files

### Config folder /config

- **controllers.yaml**   Controller params, including dual-loop pivot PID:
	- `left_*_pivot/pid_pos`: outer position loop
	- `left_*_pivot/pid_vel/pid`: inner velocity loop
	- `*_wheel/pid`: wheel velocity loop

## Launch files

* **run_simulation_and_controller.launch:** Sentry chassis only simulation and simple chassis controller

## Bugs & Feature Requests

Please report bugs and request features using
the [Issue Tracker](https://github.com/gdut-dynamic-x/simple_chassis_controller/issues)
.

[ROS]: http://www.ros.org
