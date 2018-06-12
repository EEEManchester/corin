# corin_gazebo

## Overview
This node launches the Gazebo simulator with the Corin hexapod. The default settings launch the robot in an empty environment. Different worlds can be created and should be stored in 'worlds'. This will allow easy access during launch by passing the name of the world.

## Demonstration
The Gazebo simulator for Corin is launched using:

        roslaunch corin_gazebo corin_world.launch

The will run the Gazebo simulator and spawn the Corin hexapod in it. The script 'goto0' pauses the physics engine and sets the robot's pose to the stand-up position. 

Additional parameters that can be passed during the launch are 'fixed' and 'world'. The former will spawn Corin in a fixed position, elevated slightly off the ground while the latter will spawn world models. E.g.: 

        roslaunch corin_gazebo corin_world.launch fixed:=true
        roslaunch corin_gazebo corin_world.launch world:=wall

## Published Topics

The topic of interest from this node are:

* **`/corin/foot_wrench/XX`** ([gazebo_msgs/ContactsState])

    The foot wrench for leg XX (lf, lm, lr, rf, rm, rr).

* **`/corin/imu/base/data`** ([sensor_msgs/Imu])

    The IMU data mounted on the base of Corin.

* **`/gazebo/model_states`** ([gazebo_msgs/ModelStates])

    The 6D pose and twist of the models in the world.