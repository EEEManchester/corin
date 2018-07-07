# corin_manager

## Overview
This node launches the controller manager for EITHER the Dynamixel motors or Gazebo joints used on the Corin hexapod. 
Gazebo: The joints are controlled using position controller [TODO - velocity and effort controllers].
Dynamixel: The communication employs the use of Sync Write/Read, allowing control bandwidth of 100 Hz. This package is dependant on the ROBOTIS-framework package (https://github.com/ROBOTIS-GIT/ROBOTIS-Framework).

## Demonstration
To control the robot's joints using position controller for:
### Gazebo

        roslaunch corin_manager corin_manager.launch

### DYNAMIXEL

        roslaunch corin_manager corin_manager.launch gazebo:=false

## Dynamixel
### Convenience Function
On starting, the joint torques are activated and will hold their default position. To enable or disable the joint torque,

		rosrun corin_manager torque_server.py

### Subscribed Topics

* **`/robotis/sync_write_multi_float`** ([robotis_controller_msgs/SyncWriteMultiFloat])

    The joint states to write onto motors.

### Published Topics

The topic of interest from this node is:

* **`/robotis/present_joint_states`** ([sensor_msgs/JointState])

    The robot's joint state.