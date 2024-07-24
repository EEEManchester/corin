# Corin
This is a copy of the old readme in case there is important information in it that should be preserved. Everything after this line is the original readme written by DR Wei Cheah and may no longer be accurate.

The source code is released under a proprietary software license and should not be released to any person without the author's permission.

**Author: Wei Cheah**
**Maintainer: Wei Cheah, wei.cheah@manchester.ac.uk**
**With contributions by: Hassan Hakim Khalili**
**Robotics for Extreme Environment Group, University of Manchester**

## Installation

### System Dependencies

Install the following programs in Ubuntu (or other Linux distro)[This is supposing the system is using Melodic as the ROS distribution, change the distribution on the packages names if that is not the case]:

    sudo apt-get install  ros-melodic-effort-controllers ros-melodic-gazebo-ros-control ros-melodic-ros-control  ros-melodic-ros-controllers python-numpy python-scipy

### Library Dependencies (may be needed)

The following libraries may be required (try running first and see if the following dependencies are required):

    cvxopt
    cython
    qpsolvers
    ROBOTIS-framework
    DynamixelSDK
    networkX

To build the last two:

    cd catkin_ws/src
    git clone https://weicwc@bitbucket.org/weicwc/robotis-framework.git
    git clone https://bitbucket.org/weicwc/dynamixelsdk/src/master/
    cd ../
    catkin build

The other packages depend additionally on the [ROS] standard installation.

#### Building

To build from source, place the folder in your catkin_ws/src folder and build using `catkin_make` or `catkin build`.
### Packages Overview

This repository consists of following packages:

* ***corin*** is the meta-package for the Corin library.
* ***corin_control*** implements the algorithms of the Corin library.
* ***corin_description*** contains the urdf description of Corin and the RViZ visualizer launch file.
* ***corin_gazebo*** contains the launch file for the gazebo simulator and the different worlds.
* ***corin_manager*** is the node to interface with the Dynamixel motors on Corin using ROBOTIS-Framework package.

## Usage

### Demonstrations
To run the full simulation requires four terminals.

In the first terminal, launch the Gazebo simulator for Corin using:

        roslaunch corin_gazebo corin_world.launch

Wait for a few seconds after launching, the robot should move to its stand up position from a flatten position.

In the second terminal, launch the robot's joints controller:

        roslaunch corin_control position_control.launch

In the third terminal, run Corin's controller:

        rosrun corin_control main.py

In the fourth terminal, enter one of either:

        rosparam set walkleft true
        rosparam set walkright true

And then hit `y` or `n` in the third terminal for the robot to start moving. This prompt can be removed by commenting out line 489-491 in `robot_controller.py`.

The distance that the robot moves can be set in line 124 or 129 of `control_interface.py`.

To move the robot back to its default position, terminate main.py and run the following:

        rosrun corin_description robot_default.py
