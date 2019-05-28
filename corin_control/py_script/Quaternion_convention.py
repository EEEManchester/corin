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

		# print quaternion_from_matrix()
		# exit()

		# x = [1]*5 + [0]*5 + [-1]*5
		# y = [j/2.0 for j in x]
		# #z = [-9.81]*len(x)
		# roll_speed = [10]*len(x)
		#
		# #print "initial", self.estimator.get_fixed_angles()
		#
		# for i in range(0):
		# #for i in range(len(x)):
		#
		# 	self.robot.imu.angular_velocity.x = roll_speed[i]
		# 	self.robot.imu.angular_velocity.y = roll_speed[i]/2.0
		#
		# 	self.estimator.predict_state()
		#
		# 	#print self.estimator.r
		# 	print "phi:", self.estimator.get_fixed_angles()
		# 	ps = PoseStamped()
		# 	ps.header.stamp 	 = rospy.Time.now()
		# 	ps.header.frame_id = "trunk"
		# 	#pose.pose.position.x = path_arr.X.xp[i][0] + offset[0]
		# 	ps.pose.orientation.w = self.estimator.q[0]
		# 	ps.pose.orientation.x = self.estimator.q[1]
		# 	ps.pose.orientation.y = self.estimator.q[2]
		# 	ps.pose.orientation.z = self.estimator.q[3]
		# 	self.pub_pose.publish(ps)
		# 	#print ps
		# 	rospy.sleep(.1)
		# 	print "ph2:", self.estimator.get_fixed_angles2()
		# 	#self.robot.qc.position = [0]*18
		# 	#self.robot.qc.velocity = [0]*18
		# 	#print(self.robot.qc)
		# 	#self.robot.update_state()
		# 	#self.estimator.update_state()<origin xyz="0 0 0.0" rpy="0 0 0"/>
		#
		# 	else:
		# 		self.o[1:4] = -self.o[1:4] # inverse
		# 		dq = quaternion_multiply_JPL(o, self.o)
		# 		#dq = dq.round(6)	# ensure dq[0] !> 1
		# 		phi_norm = 2 * np.arccos(dq[0])
		# 		if phi_norm > 0:
		# 			phi = dq[1:4] * phi_norm / np.sin(phi_norm/2)
		# 		else:
		# 			phi = np.zeros(3)
		#
		# 		w = phi/self.T_imu
		# 		#w = self.estimator.IMU_R.T.dot(w)
		# 		msg.angular_velocity = Vector3(*w.tolist())
		#
		#
		# 	q_av = self.cal_sum / self.cal_N
		# 	q_av = q_av / np.sqrt(q_av.dot(q_av))
		# 	# self.estimator.q = q_av
		# 	print q_av
		# 	q0 = quaternion_from_matrix(self.estimator.IMU_R.T) # Rotates points from base to IMU
		# 	#self.estimator.q = quaternion_multiply_JPL(q_av, self.estimator.IMU_q)
		# 	self.estimator.q = quaternion_multiply(q_av, q0) # Rotate vector from base to IMU then from IMU to world
		#
		# 	q = self.estimator.q
		# 	#ps.pose.orientation = Quaternion(*(q[1:4].tolist()+ [q[0]]) )
		# 	ps.pose.orientation = Quaternion(q[1], q[2], q[3], q[0])
		#
		#
		#
		# q = np.array([o.w, o.x, o.y, o.z])
		# C = quaternion_matrix_JPL(q)[0:3, 0:3]
		#
		# C = quaternion_matrix_JPL(self.estimator.q)[0:3, 0:3]
		# C2 = quaternion_matrix_JPL(o)[0:3, 0:3]
		#
		# 	angles_tuple = euler_from_quaternion_JPL(o, 'rxyz') # relative: rxyz
		# 	angles = np.asarray(angles_tuple).tolist()
		#
