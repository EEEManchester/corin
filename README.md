# Corin

## Overview

This is a python library with [ROS] interface to control the Corin hexapod. There are some C++ tools but these are not used.

Features:

* **ROS interface:** The robot and the core modules of the library interfaces to ROS. A number of library functions are able to run independently.
* **Visualizations:** The robot can be visualised in RViZ and Gazebo.
* **Physics Engine:** The gazebo simulator enables evaluation of motion control algorithms in real-world scenarios.

The Corin package has been tested with [ROS] Kinetic (under Ubuntu 16.04). This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

The source code is released under a proprietary software license and should not be released to any person without the author's permission.

**Author: Wei Cheah**
**Maintainer: Wei Cheah, wei.cheah@manchester.ac.uk**
**With contributions by: Hassan Hakim Khalili**
**Robotics for Extreme Environment Group, University of Manchester**

## Installation

### Library Dependencies

The following libraries are used (installed via pip):

    cvxopt
    cython
    qpsolvers

### Building from Source

#### Dependencies

The Corin package depends on the following packages:

    ROBOTIS-framework (https://github.com/ROBOTIS-GIT/ROBOTIS-Framework)
    DynamixelSDK (https://github.com/ROBOTIS-GIT/DynamixelSDK/releases)

To build these packages:

    cd catkin_ws/src
    git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework
    git clone https://github.com/ROBOTIS-GIT/DynamixelSDK/releases
    cd ../
    catkin_make

The other packages depend additionally on the [ROS] standard installation. 

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

    cd catkin_ws/src
    git clone https://github.com/EEEManchester/corin.git
    cd ../
    catkin_make

Don't forget to check out to the master branch using

    git checkout master

### Packages Overview

This repository consists of following packages:

* ***corin*** is the meta-package for the Corin library.
* ***corin_control*** implements the algorithms of the Corin library. 
* ***corin_description*** contains the urdf description of Corin and the RViZ visualizer launch file.
* ***corin_gazebo*** contains the launch file for the gazebo simulator and the different worlds.
* ***corin_joint_manager*** is the node to interface with the Dynamixel motors on Corin using ROBOTIS-Framework package.
* ***corin_rviz_plugin*** is an [RViz] plugin for user control interface.

## Usage

### Demonstrations
The Gazebo simulator for Corin is launched using:

        roslaunch corin_gazebo corin_world.launch

To control the robot's joints using position controller in Gazebo:

        roslaunch corin_control position_control.launch

To run a pre-scripted motion:

        rosrun corin_control main.py

The RViZ visualizer for Corin is launched using:

        roslaunch corin_description rviz.launch


## Packages

### corin_rviz_plugin

This [RViz] plugin introduces a user interface for controlling the simulation in RViZ.

### corin_manager

This node launches the controller manager for the Dynamixel motors used on the Corin hexapod.

### corin_gazebo

This node launches the Gazebo simulator with the Corin hexapod. The default settings launch the robot in an empty environment. Different worlds can be created and should be stored in 'worlds'. This will allow easy access during launch by passing the name of the world.

### corin_description

This node launches the RViZ visualizer. The visualizer allows visualisation of robot states, maps, markers, etc. The difference to Gazebo is that there is no physics engine running, and is commonly used to visualise the actual robot states. Currently, it has been set up to be used as a visualizer for evaluating motion planning algorithms rather than viewing the actual robot state. 

### corin_control

This package is the robot's core controller. The launch files in this package is used to control the robot's joints in Gazebo. 