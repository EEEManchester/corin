#!/usr/bin/env python

## State estimation for the robot
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'library'))
sys.dont_write_bytecode = True

import time
import warnings
import numpy as np
import copy
import random

## Personal libraries
from library import *			# library modules to include
from constant import *
import control_interface 								# action selection from ROS parameter server
import robot_class 										# class for robot states and function
import tf 												# SE(3) transformation library

## ROS messages
import rospy
from sensor_msgs.msg import Imu 						# sub msg for IMU
from sensor_msgs.msg import JointState 					# sub msg for joint states
from trajectory_msgs.msg import JointTrajectoryPoint	# pub msg for Robotis joints
from std_msgs.msg import Header
from std_msgs.msg import Float64 						# pub msg for Gazebo joints
from std_msgs.msg import ByteMultiArray 				# foot contact state
from std_msgs.msg import Float32MultiArray				# foot contact force
from std_msgs.msg import Float64MultiArray				# foot contact force
from geometry_msgs.msg import PoseStamped					# IMU pose
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Quaternion
import matplotlib.pyplot as plt

## Robotis ROS msgs for joint control
from robotis_controller_msgs.msg import SyncWriteItem 	# pub msg for Robotis joints
from robotis_controller_msgs.msg import SyncWriteMulti 	# pub msg for Robotis joints

from State_Estimator import StateEstimator

np.set_printoptions(suppress=True) # suppress scientific notation
np.set_printoptions(precision=5) # number display precision
np.set_printoptions(formatter={'float': '{: 0.7f}'.format})
np.set_printoptions(linewidth=1000)

R1 = np.array( [[1, 0, 0],
				[0, -1, 0],
				[0, 0, -1]])
R2 = np.array( [[-1, 0, 0],
				[0, 1, 0],
				[0, 0, -1]])
print "R1(R2):", R1.dot(R2)
q1 = quaternion_from_matrix(R1)
q2 = quaternion_from_matrix(R2)
q1q2 = quaternion_multiply(q1, q2)
print "R[q1(q2)]:", quaternion_matrix(q1q2)[0:3,0:3]

q1 = quaternion_from_matrix_JPL(R1)
q2 = quaternion_from_matrix_JPL(R2)
q1q2 = quaternion_multiply_JPL(q1, q2)
print "R[q1(q2)]:", quaternion_matrix_JPL(q1q2)[0:3,0:3]
