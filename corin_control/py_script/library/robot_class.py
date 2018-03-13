#!/usr/bin/env python

## Class for robot and leg
## Indexing for leg starts with 0, 1 .... 5

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'class'))
sys.dont_write_bytecode = True

# import rospy 										# ROS dependent file
# from sensor_msgs.msg import Imu 					# sub msg for IMU
# from sensor_msgs.msg import JointState 				# sub msg for joint states
# import tf as RTF 									# ROS transform library

import math
import numpy as np

from constant import * 								# constants used
import TrajectoryPoints								# class for 3D time array of points
import Twist
import State
import declarations as cd 							# custom class for robot classes
import transformations as tf 						# SE(3) transformation library
import gait_class as Gaitgen						# class for gait coordinator
# import param_gait									# class for setting gait parameters in RT 
import kdl 											# kinematic & dynamics library
import pspline_generator as Pspline 				# spline generator for body
import bspline_generator as Bspline 				# spline generator for leg
import plotgraph as Plot 							# plot library
import transforms
import LegClass

class RobotState:
	def __init__(self):
		self.qc  = None;#JointState() 					# ROS JointState class for all joints
		self.imu = None;#Imu()							# ROS IMU class
		
		self.Leg = [] 								# leg class
		self.gaitgen = Gaitgen.GaitClass(GAIT_TYPE)	# gait class

		# self.x_spline = TrajectoryPoints() 			# CoB linear trajectory
		# self.w_spline = TrajectoryPoints() 			# CoB angular trajectory

		self.world_X_base = State.StateClass('Twist') 				# FrameClass for world to base transformation
		# self.fr_world_X_base = tf.rotation_zyx(np.zeros(3)) 	# SO(3) rotation using pose

		self.invalid = False 						# robot state: invalid - do not do anything
		self.suspend = False 						# robot state: suspend support (TODO)

		self.active_legs  = 6 						# robot state: number of legs selected to be active
		self.phase_change = False 					# robot state: phase changed, enables transfer trajectory to be generated
		self.reset_state  = True 					# robot state: reset whole of robot state

		self.support_mode = False 					# robot state: puts robot into support mode

		self.cstate = [None]*6 						# foot contact binary states
		self.cforce = [None]*18						# foot contact forces

		self.XHc = transforms.HomogeneousTransform() 	# position: current state
		self.XHd = transforms.HomogeneousTransform()	# position: desired state
		self.V6c = transforms.Vector6D() 				# velocity: current state
		self.V6d = transforms.Vector6D() 				# velocity: desired state
		self.A6c = transforms.ArrayVector6D() 			# acceleration: current
		self.A6d = transforms.ArrayVector6D() 			# acceleration: desired

		self.KDL = kdl.corin_kinematics()
		self._initialise()

	def _initialise(self):
		""" Initialises robot class for setting up number of legs """

		for i in range(6):
			self.Leg.append(LegClass.LegClass(i))
			self.Leg[i].XHc.update_base_X_NRP(self.KDL.Leg_IK(LEG_STANCE[i]))
		

		# set NRP for each leg 
		# self.XHc.update_fr_base_X_LF_NRP(self.KDL.Leg_IK(LEG_STANCE[0]))
		# self.XHc.update_fr_base_X_LM_NRP(self.KDL.Leg_IK(LEG_STANCE[1]))
		# self.XHc.update_fr_base_X_LR_NRP(self.KDL.Leg_IK(LEG_STANCE[2]))
		# self.XHc.update_fr_base_X_RF_NRP(self.KDL.Leg_IK(LEG_STANCE[3]))
		# self.XHc.update_fr_base_X_RM_NRP(self.KDL.Leg_IK(LEG_STANCE[4]))
		# self.XHc.update_fr_base_X_RR_NRP(self.KDL.Leg_IK(LEG_STANCE[5]))
		
		print ">> INITIALISED ROBOT CLASS"

	def UpdateState(self):
		""" update robot state using readings from topics """ 
		self.UpdateLegState()
		self.UpdateImuState()

	def UpdateLegState(self):
		""" update legs state """
		## TODO: INTEGRATE HW FORCE SENSOR READINGS

		self.XHc.update_legs(self.qc.position)

		# offset to deal with base joints - suspended control of base (for )
		if (len(self.qc.name)==18):
			offset = 0
		else:
			offset = 1

		# update leg states and check if boundary exceeded
		for i in range(0,self.active_legs):

			bstate = self.Leg[i].UpdateJointState(self.qc.position[offset+i*3:offset+(i*3)+3], self.reset_state)

			self.Leg[i].UpdateForceState(self.cstate[i], self.cforce[i*3:(i*3)+3])

			if (bstate==True and self.gaitgen.cs[i]==0 and self.support_mode==False):
				self.suspend = True
				print 'suspended'

		# clear reset state if set
		self.reset_state = False if self.reset_state == True else False

	def UpdateImuState(self):
		""" update imu state """
		## TODO: INCLUDE STATE ESTIMATION
		## quaternion to euler transformation
		try:
			# rpy = RTF.transformations.euler_from_quaternion([self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w])
			rpy = np.zeros((3,1))
		except:
			rpy = np.zeros((3,1))
		
		## state estimation
		xyz = np.zeros((3,1))

		self.XHc.update_base(np.concatenate([xyz,rpy]))

	def task_X_joint(self,legs_phase=None): # TODO - to be revisited
		""" Convert all leg task space to joint space 		"""								
		""" Output: joint space setpoint for all joints		"""

		## Define variables ##
		error = 0
		qt = []
		qp = []
		qv = []
		qa = []
		print legs_phase
		for j in range(0,6):
			error, qpd, qvd, qad = self.Leg[j].tf_task_X_joint()
			## append to list if valid, otherwise break and raise error
			try:
				if (error == 0):
					for z in range(0,3):
						qp.append(qpd.item(z))#[z])
						qv.append(qvd.item(z))#[z])
						qa.append(qad.item(z))#[z])
				else:
					if (error == 1):
						err_str = 'Error - joint limit exceeded in leg, '
					elif (error == 2):
						err_str = 'Error - singularity in leg, '
					else:
						err_str = 'Unknown error'
					raise ValueError, err_str
			except Exception, e:
				print e, j

		return TrajectoryPoints.JointTrajectoryPoints(18,(qt,qp,qv,qa))

# robot = RobotState()
## ====================================================================================================================================== ##
## ====================================================================================================================================== ##


