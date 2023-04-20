# Cartesian Position Control 
This repository contains a controller that moves the robot to the desired position along the desired axis. 

Author: Ayan Mazhitov

E-Mail: mazhitovayan@gmail.com

# Installation and launching

## Requirements
The machine has to have ROS and the following libraries have to be installed beforehand:
- libfranka (minimum version 0.7.0)
- franka_hw
- controller_interface
- pluginlib
- realtime_tools

## Installation

Change directory to your workspace and run `catkin_make` or `catkin_build`

## Launching

After installation, the controller can be launched by executing this command

`roslaunch franka_cartesian_control tactile_pose_controller.launch`

# How to use

The controller creates a ROS service under the name of `CartesianPoseTactile/delta_pose` that can be called with the `franka_cartesian_control/Delta` message that has following architecture: 

Float32 delta_x (distance to move along **x-axis** either negative or positive)

Float32 delta_y (distance to move along **y-axis** either negative or positive)

Float32 delta_z (distance to move along **z-axis** either negative or positive)

This service 
