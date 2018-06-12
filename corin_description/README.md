# corin_description

## Overview
This node launches the RViZ visualizer. The visualizer allows visualisation of robot states, maps, markers, etc. The difference to Gazebo is that there is no physics engine running, and is commonly used to visualise the actual robot states. Currently, it has been setup to be used as a visualizer for evaluating motion planning algorithms rather than viewing the actual robot state. A number of user interface buttons have been created with limited functionality. Of interest is the 'Run'/'Pause' button which starts the robot's motion.

The robot model is defined in the 'robots' folder. The model defined utilises xacros (commonly known as functions) defined in the 'urdfs' folders. This allows common features to be defined once and called with different parameters, e.g. the robot's leg is similar but placed at different locations around the robot's base. The 'common.xacro' is used to define fixed properties of the robot. Any changes here will be propogated through to the robot on running RViZ or Gazebo.


## Demonstration
The RViZ visualizer for Corin is launched using:

        roslaunch corin_description rviz.launch


## Convenience Functions
A few scripts are available which will move the robot's pose or joints to default positions:

        rosrun corin_description robot_default.py

or
		
		rosrun corin_description joint_default.py


## RViZ Subscribed Topics

* **`/corin/footholds`** ([visualization_msgs/MarkerArray])

    The footholds planned for the robot.

* **`/corin/joint_states`** ([sensor_msgs/JointState])

    The joint state of the robot.

* **`/corin/path`** ([nav_msgs/Path])

    The base trajectory for the robot.

* **`/corin/point_cloud`** ([sensor_msgs/PointCloud2])

    The point cloud data for visualising grid map.

## RViZ Published Topics

* **`/corin/ui_execute`** ([std_msgs/String])

    The topic for starting or stopping the robot motion.