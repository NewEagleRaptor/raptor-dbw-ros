# New Eagle DBW (drive-by-wire) ROS interface

This repository contains a collection of ROS packages, which allow DBW kit developers to quickly implement a generic ROS node for interacting with the New Eagle Raptor controller.

The current development branch is `master` and targets ROS `kinetic` and `melodic`.

ROS Kinetic: [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdoc__pacifica_dbw_ros__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdoc__pacifica_dbw_ros__ubuntu_xenial_amd64/)

ROS Melodic: [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdoc__pacifica_dbw_ros__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdoc__pacifica_dbw_ros__ubuntu_xenial_amd64/)

For more on New Eagle Raptor controllers see: https://store.neweagle.net/product/1793-196-1503-general-control-module-raptor/ 

# Installing and building

We are still working on releasing these packages into official ROS distros, so for now, you have to build them from a catkin workspace using these steps:

1. Create and initialize a new catkin workspace if you don't already have one. Instructions below assume the workspace was created in catkin_ws.
2. cd catkin_ws/src
3. git clone https://github.com/NewEagleRaptor/pacifica-dbw-ros.git
4. cd ..
5. catkin_make

# Usage

## Running the joystick demo

Prerequisites:
* a game controller compatible with the ROS [joy driver](http://wiki.ros.org/joy)
* a Kvaser CAN adapter

1. Make sure there is a ROS master running (roscore)
2. cd catkin_ws
3. source devel/setup.bash
4. roslaunch dbw_pacifica_joystick_demo joystick_demo.launch
