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
import TrajectoryPoints as TP						# class for 3D time array of points
import Twist
import State
import declarations as cd 							# custom class for robot classes
from matrix_transforms import *						# SE(3) transformation library
import gait_class as Gaitgen						# class for gait coordinator
# import param_gait									# class for setting gait parameters in RT 
import kdl 											# kinematic & dynamics library
import pspline_generator as Pspline 				# spline generator for body
import bspline_generator as Bspline 				# spline generator for leg
import plotgraph as Plot 							# plot library
import robot_transforms

class LegClass:
	#common base class for Leg
	def __init__(self, name):
		self.number = name
		self.Joint 	= cd.Joint_class(3)

		# library functions
		self.KDL 	= kdl.KDL()
		self.bspline = Bspline.SplineGenerator() 	# bSplineClass for leg transfer spline generation
		
		# transfer phase variables - created in controller class and stored here
		self.xspline = TP.TrajectoryPoints() 		# trajectory in cartesian position 
		self.qspline = TP.JointTrajectoryPoints()	# trajectory in joint space
		self.spline_counter = 1 		# counter to track execution of spline
		self.spline_length  = 0			# spline length
		self.feedback_state	= 0 		# 0 = idle, 1 = command received (executing), 2 = command completed

		self.cstate = 0 	# foot contact state: 1 or 0

		# transformation frames
		self.XHc = robot_transforms.ArrayHomogeneousTransform(self.number)
		self.XHd = robot_transforms.ArrayHomogeneousTransform(self.number)
		self.V6c = robot_transforms.ArrayVector6D() 							# velocity: current
		self.V6d = robot_transforms.ArrayVector6D() 							# velocity: desired
		self.A6c = robot_transforms.ArrayVector6D() 							# acceleration: current
		self.A6d = robot_transforms.ArrayVector6D() 							# acceleration: desired

		## AIM: GET RID OF FOLLOWING
		self.hip_X_ee 	= State.StateClass('Twist')
		self.base_X_ee 	= State.StateClass('Twist')
		self.foot_XF_ee = State.StateClass('Wrench')
		self.hip_XF_ee 	= State.StateClass('Wrench')
		self.base_XF_ee	= State.StateClass('Wrench')
		self.tf_base_X_hip 	= rotation_matrix(TF_BASE_X_HIP[self.number],Z_AXIS)[:3, :3]
		self.AEP = np.zeros((3,1)) 		# Anterior Extreme Position for leg wrt base frame
		self.PEP = np.zeros((3,1))		# Posterior Extreme Position for leg wrt base frame
		self.REP = np.zeros((3,1)) 		# nominal stance for leg wrt base frame
		self.hip_AEP = np.zeros((3,1)) 	# for storing transfer AEP
		self.SetLegREP() 				# set rest position of legs

		## TODO: CHANGE TO HOMOGENOUS AND VECTOR FORM
		self.qsurface = np.array([0.,0.,0.]) 		# surface orientation in leg frame, (roll, pitch, yaw)
		self.qsnorm   = np.array([[0.],[0.],[1.]]) 	# surface normal

		self.phase_change = False 		# Flag for enabling trajectory to be generated at transfer

	## Update functions
	def update_joint_state(self, mx_world_X_base, jointState, resetState):
		""" Update current joint state of legs and forward transformations 			"""
		""" Input: 	1) jointState -> joint angles 
					2) resetState -> flag to set desired state as current state 	""" 

		## Error Compensation
		q_compensated = (jointState[0], jointState[1]+QCOMPENSATION, jointState[2]) 		# offset q2 by 0.01 - gravity

		## current
		self.hip_X_ee.cs.xp  = self.KDL.leg_FK(q_compensated)
		self.base_X_ee.cs.xp = self.hip_X_base_ee(self.hip_X_ee.cs.xp)

		## error
		self.hip_X_ee.es.xp  = self.hip_X_ee.ds.xp  - self.hip_X_ee.cs.xp
		self.base_X_ee.es.xp = self.base_X_ee.ds.xp - self.base_X_ee.cs.xp

		## check work envelope
		bound_exceed = self.check_boundary_limit()

		self.XHc.update_coxa_X_foot(q_compensated)
		self.XHc.update_base_X_foot(q_compensated)
		self.XHc.update_world_base_X_foot(mx_world_X_base, q_compensated)
		self.XHc.update_world_base_X_NRP(mx_world_X_base)

		## state reset
		if (resetState):
			self.hip_X_ee.ds.xp  = self.hip_X_ee.cs.xp.copy()
			self.base_X_ee.ds.xp = self.base_X_ee.cs.xp.copy()

			self.XHd.coxa_X_foot = self.XHc.coxa_X_foot.copy()
			self.XHd.base_X_foot = self.XHc.base_X_foot.copy()
			print 'RESET TRUE'
		# if (self.number == 0):
		# 	print self.number, ' ', jointState
		# 	print 'bXe_ds: ', np.round(self.base_X_ee.ds.xp.flatten(),4)
		# 	print 'bXe_cs: ', np.round(self.base_X_ee.cs.xp.flatten(),4)
		# 	print 'hXe_ds: ', np.round(self.hip_X_ee.ds.xp.flatten(),4)
		# 	print 'hXe_cs: ', np.round(self.hip_X_ee.cs.xp.flatten(),4)

		return bound_exceed

	def update_force_state(self, cState, cForce):
		""" contact force of leg """
		## ********************************	Contact Force  ******************************** ##
		self.cstate = cState
		self.foot_XF_ee.cs = np.reshape(np.array(cForce),(3,1))

		## TODO: TRANSFORM FROM FOOT FRAME TO HIP, BASE, WORLD FRAME

	def generate_spline(self, start, end, snorm, phase=1, reflex=False, ctime=2.0, tn=0.1):
		""" generate leg trajectory using bspline and check for kinematic constraints 	"""
		""" Input:  a) nLeg   -> Leg number
			 		b) start  -> start position in 3D space wrt base frame
					c) end    -> end position in 3D space wrt base frame
					d) snorm  -> surface normal
					e) reflex -> boolen for reflex trajectory
					f) ctime  -> time for trajectory						
					g) tn 	  -> rate of trajectory 						"""

		self.xspline = TP.TrajectoryPoints(self.bspline.generate_leg_spline(start.flatten(), end.flatten(), snorm, phase, reflex, ctime, tn))
		self.spline_counter = 1
		self.spline_length  = len(self.xspline.t)
		# if (self.number==1):
		# 	Plot.plot_2d(self.xspline.t,self.xspline.xp)

		## checks spline for kinematic constraint
		qt = [];	qp = [];	qv = [];	qa = [];

		try:
			for i in range(0,self.spline_length):
				error, qpd, qvd, qad = self.tf_task_X_joint(self.xspline.xp[i],self.xspline.xv[i],self.xspline.xa[i])

				if (error == 0):
					qt.append(i*CTR_INTV)
					qp.append(qpd.tolist())
					qv.append(qvd.tolist())
					qa.append(qad.tolist())
				else:
					if (error == 1):
						err_str = 'Error - joint limit exceeded in Leg '
						raise ValueError
					elif (error == 2):
						err_str = 'Error - singularity in Leg '
						raise ValueError
					else:
						err_str = 'Unknown error in Leg '
						raise ValueError
			
			self.qspline = TP.JointTrajectoryPoints(18,(qt,qp,qv,qa))
			return True

		except ValueError:
			print err_str, self.number
			return False

	def tf_task_X_joint(self, xp=None, xv=None, xa=None):
		""" Converts task space position wrt hip frame to joint space 	  """
		""" Input: 	1) xp -> task space position 						
			Output: joint position, velocity and acceleration in 1D array """
		
		## Define variables ##
		error = 0 					# Error indicator

		if (xp is None):
			xp = self.XHd.coxa_X_foot[:3,3:4]
			# xp = self.hip_X_ee.ds.xp.reshape(3,1) # TEMP: remove later
			# print self.number, np.round( (xpn - xp).transpose(),4) 	# TODO: LEG 5 ERROR IS LARGE
		self.Joint.qpd = self.KDL.leg_IK(xp)
		
		# checks if joint limit exceeded and singularity occurs
		if (self.check_joint_limit(self.Joint.qpd) is True):
			error = 1
		else:
			if (self.KDL.check_singularity(self.Joint.qpd) is False):
				self.Joint.qvd, self.Joint.qad = self.KDL.joint_speed(self.Joint.qpd, self.V6d.coxa_X_foot, self.A6d.coxa_X_foot)
			else:
				error = 2
		
		return error, self.Joint.qpd, self.Joint.qvd, self.Joint.qad 	# TEMP: change to normal (huh?)

	
	def check_boundary_limit(self):
		""" leg boundary area projected to 2D space """

		bound_violate = False
		BOUND_FACTOR  = 1.8 	# relaxes the boundary constraint

		# Vector to aep and current position wrt nominal point
		v3_NRP_X_AEP  = self.XHd.world_base_X_AEP[:3,3:4]  - self.XHd.world_base_X_NRP[:3,3:4]
		v3_NRP_X_foot = self.XHc.world_base_X_foot[:3,3:4] - self.XHd.world_base_X_NRP[:3,3:4]

		# Magnitude of current point
		mag_nom_X_ee  = np.dot(v3_NRP_X_AEP.flatten(), v3_NRP_X_foot.flatten())

		# Angle between hip frame and AEP
		vec_ang = np.arctan2(v3_NRP_X_AEP.item(1), v3_NRP_X_AEP.item(0))

		## Ellipse boundary - for the front half: p_nom to AEP
		if (mag_nom_X_ee > 0.):
			# print self.number, ' ellipse boundary'
			try:
				# ellipse major, minor radius, rotation
				a  = np.sqrt(v3_NRP_X_AEP.item(0)**2 + v3_NRP_X_AEP.item(1)**2)
				b  = STEP_STROKE/2.
				qr = vec_ang
				x  = v3_NRP_X_foot.item(0)
				y  = v3_NRP_X_foot.item(1)

				r_state = ((x*np.cos(qr)+y*np.sin(qr))**2)/(a**2) + ((x*np.sin(qr)-y*np.cos(qr))**2)/(b**2)

				if (BOUND_FACTOR < r_state):
					bound_violate = True
					# print self.number, ' ellipse boundary violated ', r_state
			except:
				pass

		## Circle boundary - for the back half: p_nom to PEP
		else:
			r_state = (v3_NRP_X_foot.item(0)**2 + v3_NRP_X_foot.item(1)**2)/(STEP_STROKE/2.)**2

			if (BOUND_FACTOR < r_state):
				bound_violate = True
				# print self.number, ' circle boundary violated ', r_state

		return bound_violate

	## Kinematic functions
	def check_joint_limit(self, qp):
		""" checks if joint angle exceeds limit 			"""
		""" Input:  qp -> joint angles
			Output: boolean -> True: exceeded, False: OK	"""

		exceeded = False
		# take correct q1 limit
		if (self.number != 1 or self.number != 4):
			q1_lim = Q1_F_LIM
		else:
			q1_lim = Q1_M_LIM

		if (abs(self.Joint.qpd[0])>q1_lim):
			print 'Leg ', self.number, ' q1 limit exceeded'
			exceeded = True
		if (abs(self.Joint.qpd[1])>Q2_A_LIM):
			print 'Leg ', self.number, ' q2 limit exceeded'
			exceeded = True
		if (abs(self.Joint.qpd[2])>Q3_A_LIM):
			print 'Leg ', self.number, ' q3 limit exceeded'
			exceeded = True

		return False

	def update_from_spline(self):
		""" Update positions from generated leg spline & increment counter	"""
		""" Trajectory feedback_state set to 2 (finished execution) when 
			spline has been finished 										"""
		""" Output: boolean: True if update valid, False otherwise 			"""

		## Variable mapping ##
		i = self.spline_counter
		try:
			self.hip_X_ee.ds.xp = self.xspline.xp[i]
			self.hip_X_ee.ds.xv = self.xspline.xv[i]
			self.hip_X_ee.ds.xa = self.xspline.xa[i]

			self.XHd.update_coxa_X_foot(self.qspline.xp[i])
			self.V6d.coxa_X_foot[:3]  = self.xspline.xv[i].reshape(3,1)
			self.A6d.coxa_X_foot[:3]  = self.xspline.xa[i].reshape(3,1)

			self.spline_counter += 1

			return True
		except:
			self.feedback_state = 2
			return False

	def update_NRP(self, bodypose, base_X_surface):
		""" updates nominal stance of robot """

		# update only if surface orientation above deadzone
		if (abs(self.qsurface.item(0)) > QDEADZONE or abs(self.qsurface.item(1)) > QDEADZONE):
			# self.REP = FR_base_X_hip[self.number] + self.KDL.update_nominal_stance(bodypose, base_X_surface, self.qsurface)
			# TODO: this needs checking
			self.XHc.coxa_X_NRP[:3,3:4] = (self.KDL.update_nominal_stance(bodypose, base_X_surface, self.qsurface)).reshape(3,1)
			self.XHc.base_X_NRP[:3,3:4] = np.dot(self.base_X_coxa, self.XHc.coxa_X_NRP)
			# print 'updating REP for leg ', self.number
			# if (self.number == 0):
	
	## TODO: FOLLOWING SHOULD NOT BE REQUIRED ANYMORE
	def base_X_hip_ee(self, base_X_ee_xp):
		return np.dot( rotation_matrix(TF_BASE_X_HIP[self.number],Z_AXIS)[:3, :3], (-FR_base_X_hip[self.number] + base_X_ee_xp))

	def hip_X_base_ee(self, hip_X_ee_xp):
		return (FR_base_X_hip[self.number] + np.dot( rotation_matrix(TF_HIP_X_BASE[self.number],Z_AXIS)[:3, :3], hip_X_ee_xp))

	def SetLegREP(self):
		self.REP = FR_base_X_hip[self.number] + np.dot( rotation_matrix(TF_HIP_X_BASE[self.number],Z_AXIS)[:3, :3], np.reshape(LEG_STANCE[self.number],(3,1)) )

	