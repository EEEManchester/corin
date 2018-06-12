# corin_manager

## Overview
This node launches the controller manager for the Dynamixel motors used on the Corin hexapod. The communication employs the use of Sync Write/Read, allowing control bandwidth of 100 Hz. This package is dependant on the ROBOTIS-framework package (https://github.com/ROBOTIS-GIT/ROBOTIS-Framework).

## Demonstration
To control the robot's joints using position controller:

        roslaunch corin_manager corin_manager.launch

## Convenience Function
On starting, the joint torques are activated and will hold their default position. To enable or disable the joint torque,

		rosrun corin_manager torque_server.py

## Subscribed Topics

* **`/robotis/sync_write_multi_float`** ([robotis_controller_msgs/SyncWriteMultiFloat])

    The joint states to write onto motors.

## Published Topics

The topic of interest from this node is:

* **`/robotis/present_joint_states`** ([sensor_msgs/JointState])

    The robot's joint state.