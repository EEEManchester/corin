#!/usr/bin/env python

## Main controller for the robot
import rospy
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'library'))
sys.dont_write_bytecode = True

import time
from fractions import Fraction
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import copy
import warnings

## personal libraries
from constant import *
import robot_class
import gait_class as Gaitgenerator
import param_gait
import path_generator as Pathgenerator
import plotgraph as Plot
import transformations as tf
import pspline_generator as Pspline
import bspline_generator as Bspline

from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from std_msgs.msg import ByteMultiArray 	# foot contact state
from std_msgs.msg import Float32MultiArray	# foot contact force

## arebgun ROS msgs
from dynamixel_msgs.msg import PositionVelocity

## Robotis ROS msgs for joint control
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from robotis_controller_msgs.msg import SyncWriteItem
from robotis_controller_msgs.msg import SyncWriteMulti

## User control files
from sensor_msgs.msg import Joy
import connex
import control_interface

def rad2raw(radian):

	value_of_0_radian_position_      = 2048
	value_of_min_radian_position_    = 0
	value_of_max_radian_position_    = 4095
	min_radian_                      = -3.14159265
	max_radian_                      =  3.14159265

	if (radian > 0):
		value = (radian * (value_of_max_radian_position_ - value_of_0_radian_position_) / max_radian_) + value_of_0_radian_position_;

	elif (radian < 0):
		value = (radian * (value_of_min_radian_position_ - value_of_0_radian_position_) / min_radian_) + value_of_0_radian_position_;
	else:
		value = 2048;

	return int(value)

def rads2raw(rads):
	velocity_to_value_ratio_ = 1.0/(0.229*2*3.14159265/60.0)

	value = int(round(abs(rads) * velocity_to_value_ratio_))
	if (value == 0):
		value = 1
	return value # int(math.ceil(abs(rads) * velocity_to_value_ratio_))

def raw2rads(raw):
	velocity_to_value_ratio_ = 1.0/(0.229*2*3.14159265/60.0)

	return (rads / velocity_to_value_ratio_)

class CorinManager:
	def __init__(self):
		rospy.init_node('read_joint_state') 		#Initialises node
		self.robot_ns = ROBOT_NS
		self.interval = TRAC_INTERVAL
		self.rate 	  = rospy.Rate(1.0/TRAC_INTERVAL)	# frequency

		self.cp = 0.0						# magnitude of linear translation in horizontal plane

		self.lin_v = np.zeros(3) 			# linear velocity vector
		self.ang_v = np.zeros(3) 			# angular velocity vector
		self.ang_p = np.zeros(3) 			# angular position vector
		# self.snorm = np.array([0.,0.,1.])	# surface normal

		self.h_v  = np.zeros(2)				# linear translation vector in horizontal plane
		self.zc_v = np.zeros(2) 			# zero crossing vector

		self.Robot 	  = robot_class.RobotState() 		# robot class
		self.b_spline = Bspline.SplineGenerator() 		# B-spline class
		self.p_spline = Pspline.SplineGenerator() 		# polynomial spline class
		self.connex   = connex.Connexion() 				# connex hardware controller
		self.planner  = Pathgenerator.PathGenerator() 	# trajectory following

		self.hardware = self.hardware_selection()		# returns interface to simulation or hardware

		self.point = JointTrajectoryPoint()
		self.point.time_from_start = rospy.Duration(TRAC_INTERVAL)

		self._start()
		rospy.sleep(0.5)

		self.com_tracking = np.zeros(3)

		self.resting = False 		# Flag indicating robot standing or resting

	def hardware_selection(self):
		if rospy.has_param('/gazebo/auto_disable_bodies'):
			return 'simulation'
		else:
			return 'robotis'

	def joint_state_callback(self, msg):
		self.Robot.qc = msg

		print msg.effort[13]

	def _start(self):
		#######################################
		## initialise publishers/subscribers ##
		#######################################

		##***************** PUBLISHERS ***************##
		self.sync_qpub_  	= rospy.Publisher('/robotis/sync_write_multi', SyncWriteMulti, queue_size=1)

		self.qpub = {}

		if (self.robot_ns == 'corin' and (self.hardware == 'simulation' or self.hardware == 'real' or self.hardware == 'robotis')):

			for i in range(0,18):
				if (self.hardware == 'simulation'):
					## Publish to Gazebo
					self.qpub[i] = rospy.Publisher(self.robot_ns + '/' + JOINT_TOPICS[i] + '/command', Float64, queue_size=1)

				elif (self.hardware == 'real'):
					## Publish to Dynamixel motors
					self.qpub[i] = rospy.Publisher(self.robot_ns + '/' + JOINT_TOPICS[i] + '/command_pv', PositionVelocity, queue_size=1)

		else:
			# shutdown if wrong hardware selected
			rospy.logerr("Invalid Hardware Interface Selected!")
			rospy.signal_shutdown("Invalid Hardware Interface Selected - shutting down")

		##***************** SUBSCRIBERS ***************##
		if (self.hardware == 'simulation'):
			self.joint_sub_  = rospy.Subscriber(self.robot_ns + '/joint_states', JointState, self.joint_state_callback, queue_size=5)
		elif (self.hardware == 'robotis'):
			self.joint_sub_  = rospy.Subscriber('robotis/present_joint_states', JointState, self.joint_state_callback, queue_size=5)

		self.on_start = False

	

if __name__ == "__main__":

	manager = CorinManager()
	manager.Robot.updateState()
	print 'Robot Initiated'

	while not rospy.is_shutdown():
		manager.rate.sleep()