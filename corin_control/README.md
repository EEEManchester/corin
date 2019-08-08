# corin_control

## Dependencies
This package depends on:

	- [StabiliPy](https://github.com/haudren/stabilipy.git)

Run the following:

	sudo apt-get install liblapack-dev libatlas-dev libblas-dev libgmp-dev libppl-dev

Navigate to the cloned Stabilipy folder:

	pip install cython && pip install -r requirements.txt

### Additional Notes on Installation:
The newer version of Scipy required may not install with an error about it being a dist package. Install the latest version as follows:

	cd usr/lib/python2.7/dist-packages/
	sudo rm -r scipy
	sudo rm scipy-0.17.0.egg-info
	sudo pip install --upgrade scipy

## Overview

This package is the robot's core controller. The file 'main.py', creates an instance of the robot controller and all actions are invoked from here. 

## Demonstrations
To run a prescripted motion:

        rosrun corin_control main.py
        
## Subscribed Topics

The primary topic of interest is the robot joint states. The topic name and type differs depending on the controller it is interfaced to, either Gazebo or Dynamixel joints.

* **`/corin/joint_states`** ([sensor_msgs/JointState])

    The joint states of the robot in Gazebo.

* **`/robotis/present_joint_states`** ([sensor_msgs/JointState])

    The joint states of the robot from the Dynamixel motors.

Other useful topics are the IMU, robot state model (for Gazebo only), user interface:

* **`/corin/imu/base/data`** ([sensor_msgs/Imu])

* **`/gazebo/model_states`** ([gazebo_msgs/ModelStates])

* **`/corin/ui_execute`** ([std_msgs/String])

## Published Topics

The publisher to control the joints differs depending on the controller it is interfaced to (Gazebo, Dynamixel or RViZ). The RViZ interface enables fast simulation of robot motion without physics engine dependency. 

* **`/corin/joint_states`** ([sensor_msgs/JointState])

    This topic is used for RViZ interface.

* **`/corin/XX_qX_joint/command`** ([std_msgs/Float64])

    This topic is used for Gazebo, where each joint is addressed individually.

* **`/robotis/sync_write_multi_float`** ([robotis_controller_msgs/SyncWriteMultiFloat])

    This topic is used for the Dynamixel interface. A single message is used to address all the joints simultaneously.

* **`/corin/setpoint_states `** ([sensor_msgs/JointState])

    This topic is for logging the robot's pose and joint states setpoints.

