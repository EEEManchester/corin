# corin_control

## Overview

This package is the robot's core controller. The launch files in this package is used to control the robot's joints in Gazebo. The file 'main.py', creates an instance of the robot controller and all actions are invoked from here. 

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

