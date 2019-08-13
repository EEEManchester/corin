#!/usr/bin/env python

## This file published the force measured by the force sensor
## in the robot body frame.
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'library'))
sys.dont_write_bytecode = True

import time
import datetime
import warnings
import numpy as np

## Personal libraries
from library import *			# library modules to include
from constant import *
import control_interface 								# action selection from ROS parameter server
import robot_class 										# class for robot states and function
import tf 												# SE(3) transformation library

## ROS messages
import rospy
from sensor_msgs.msg import Imu 						# sub msg for IMU
from sensor_msgs.msg import JointState 				#	# sub msg for joint states
from trajectory_msgs.msg import JointTrajectoryPoint	# pub msg for Robotis joints
from std_msgs.msg import Float64 						# pub msg for Gazebo joints
from std_msgs.msg import ByteMultiArray 				# foot contact state
from std_msgs.msg import Float32MultiArray				# foot contact force
from geometry_msgs.msg import PoseStamped				# IMU pose
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import matplotlib.pyplot as plt

## Robotis ROS msgs for joint control
from robotis_controller_msgs.msg import SyncWriteItem 	# pub msg for Robotis joints
from robotis_controller_msgs.msg import SyncWriteMulti 	# pub msg for Robotis joints

from State_Estimator import StateEstimator

import rosbag

np.set_printoptions(suppress=True) # suppress scientific notation
np.set_printoptions(precision=5) # number display precision
np.set_printoptions(formatter={'float': '{: 0.7f}'.format})
np.set_printoptions(linewidth=1000)

class CorinStateTester:
    def __init__(self):
        rospy.init_node('Corin_force_sensor')

        self.robot 	= robot_class.RobotState()
        self.joint_sub_  = rospy.Subscriber('robotis/present_joint_states', JointState, self.joint_state_callback, queue_size=5)
        self.force_sub_  = rospy.Subscriber('force_vector_0', Vector3Stamped, self.force_0_callback, queue_size=5)

        self.pub_f_body = rospy.Publisher('body_f', Vector3, queue_size=1)

    def force_0_callback(self, msg):
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[0:3] = f

        if self.robot.qc is not None:
            self.robot.update_state()
        f_sensor0 = self.robot.Leg[0].F6c.tibia_X_foot[:3].flatten()
        f_base0 = self.robot.Leg[0].F6c.base_X_foot[:3].flatten()

        self.pub_f_body.publish(Vector3(*f_base0.tolist()))


    def joint_state_callback(self, msg):
        """ robot joint state callback """
        self.robot.qc = msg


if __name__ == "__main__":

    state = CorinStateTester()
    rospy.spin()

    "COMPLETE"
