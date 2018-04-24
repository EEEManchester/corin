#!/usr/bin/env python

## Class for robot and leg
## Indexing for leg starts with 0, 1 .... 5

import sys; sys.dont_write_bytecode = True
from traits import *

import math
import numpy as np

from constant import * 								# constants used
import robot_transforms
from matrix_transforms import *						# SE(3) transformation library
import kdl 											# kinematic & dynamics library
import path_generator
import gait_class as Gaitgen						# class for gait coordinator

import plotgraph as Plot 							# plot library

class LegClass:
	#common base class for Leg
	def __init__(self, name):
		self.number = name
		self.Joint 	= Joint(3)

		# library functions
		self.KDL  = kdl.KDL()
		self.Path = path_generator.PathGenerator()

		# transfer phase variables - created in controller class and stored here
		self.xspline = TrajectoryPoints() 		# trajectory in cartesian position 
		self.qspline = JointTrajectoryPoints()	# trajectory in joint space
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
		self.F6c = robot_transforms.ArrayVector6D()
		self.F6d = robot_transforms.ArrayVector6D()

		self.transfer_phase_change = False 		# Flag for enabling trajectory to be generated at transfer
		self.support_phase_change  = False 		# Flag for enabling trajectory to be generated at transfer

		self.XH_world_X_base = np.eye(4)
		self.P6_world_X_base = np.zeros((6,1))

	## Update functions
	def update_joint_state(self, P6_world_X_base, jointState, resetState):
		""" Update current joint state of legs and forward transformations 			"""
		""" Input: 	1) jointState -> joint angles 
					2) resetState -> flag to set desired state as current state 	""" 

		## Error Compensation
		q_compensated = (jointState[0], jointState[1]-QCOMPENSATION, jointState[2]) 		# offset q2 by 0.01 - gravity

		## Updates current state
		self.XHc.update_coxa_X_foot(q_compensated)
		self.XHc.update_base_X_foot(q_compensated)

		self.XHc.update_world_base_X_foot(P6_world_X_base, q_compensated)
		# self.XHc.update_world_base_X_NRP(P6_world_X_base)
		# self.XHd.update_world_base_X_NRP(P6_world_X_base)
		
		## Check work envelope
		bound_exceed = self.check_boundary_limit(self.XHc.world_base_X_foot, self.XHc.world_base_X_NRP)

		## Resets state
		if (resetState):
			self.XHd.coxa_X_foot = self.XHc.coxa_X_foot.copy()
			self.XHd.base_X_foot = self.XHc.base_X_foot.copy()

		return bound_exceed

	def update_force_state(self, cState, cForce):
		""" contact force of leg """
		
		self.cstate = cState
		self.F6c.tibia_X_foot[:3] = np.reshape(np.array(cForce),(3,1))

		return None
		## TODO: TRANSFORM FROM FOOT FRAME TO HIP, BASE, WORLD FRAME

	def generate_spline(self, frame, snorm, phase=1, reflex=False, ctime=2.0, tn=0.1):
		""" generate leg trajectory using bspline and check for kinematic constraints 	"""
		""" Input:  a) nLeg   -> Leg number
			 		b) start  -> start position in 3D space wrt base frame
					c) end    -> end position in 3D space wrt base frame
					d) snorm  -> surface normal
					e) reflex -> boolen for reflex trajectory
					f) ctime  -> time for trajectory						
					g) tn 	  -> rate of trajectory 						"""
		
		## Interpolate via points in base frame
		# bpx, td = self.Path.interpolate_leg_path(self.XHc.base_X_foot[:3,3].flatten(), self.XHd.base_X_foot[:3,3].flatten(), snorm, phase, reflex, ctime, tn)

		if (frame is 'world'):
			## Interpolate via points in world frame from base to foot
			start = self.XHc.world_base_X_foot[:3,3].copy()
			end   = mX(self.XH_world_X_base[:3,:3], self.XHd.base_X_foot[:3,3])
			wpx, td = self.Path.interpolate_leg_path(start, end, snorm, phase, reflex, ctime)
			
			# Transform each via point from world to leg frame
			wcp = np.zeros((len(wpx),3))
			wcp[0] = self.XHc.coxa_X_foot[0:3,3].copy()

			for i in range (1, len(wpx)):
				basXft = mX(np.transpose(self.XH_world_X_base[:3,:3]), wpx[i]) 	# transfrom from world to base frame
				wcp[i] = mX(self.XHc.coxa_X_base, v3_X_m(basXft))[:3,3] 		# transform from base to leg frame
		elif (frame is 'leg'):
			wcp, td = self.Path.interpolate_leg_path(self.Robot.Leg[j].XHc.coxa_X_foot[0:3,3], 
														self.Robot.Leg[j].XHd.coxa_X_foot[0:3,3], 
														snorm, phase, reflex, ctime)
		self.xspline = TrajectoryPoints(self.Path.generate_leg_path(wcp, td, tn))
		self.spline_counter = 1
		self.spline_length  = len(self.xspline.t)
		# if (self.number==1):
			# print 'dwXb: ', np.round(XD_world_base_X_foot,4)#self.XH_world_X_base[:3,3],4)
			# print 'cp: ', np.round(wcp,4)
			
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
			
			self.qspline = JointTrajectoryPoints(18,(qt,qp,qv,qa))
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
			# Use previous known state
			xp = self.XHd.coxa_X_foot[:3,3]
		# if (self.number==2):
			# print 'bf: ', np.round(self.XHd.base_X_foot[:3,3],4)
			# print 'cf: ', np.round(xp,4)
			# print 'qp: ', np.round(self.Joint.qpd,4)
		self.Joint.qpd = self.KDL.leg_IK(xp)
		

		if (self.Joint.qpd is not None):
			# checks if joint limit exceeded and singularity occurs
			if (self.check_joint_limit(self.Joint.qpd) is True):
				error = 1
			else:
				if (self.KDL.check_singularity(self.Joint.qpd) is False):
					self.Joint.qvd, self.Joint.qad = self.KDL.joint_speed(self.Joint.qpd, 
																			self.V6d.coxa_X_foot, 
																			self.A6d.coxa_X_foot)
				else:
					error = 2
		else:
			error = 3
		
		return error, self.Joint.qpd, self.Joint.qvd, self.Joint.qad 	# TEMP: change to normal (huh?)

	
	def check_boundary_limit(self, world_base_X_foot, world_base_X_NRP):
		""" leg boundary area projected to 2D space """

		bound_violate = False
		BOUND_FACTOR  = 1.8 	# relaxes the boundary constraint

		# Vector to aep and current position wrt nominal point
		v3_NRP_X_AEP  = self.XHd.world_base_X_AEP[:3,3:4]  - self.XHd.world_base_X_NRP[:3,3:4]
		v3_NRP_X_foot = world_base_X_foot[:3,3:4] - world_base_X_NRP[:3,3:4]

		# if (self.number == 0):
		# 	print 'bXN : ', np.round(world_base_X_NRP[:3,3].flatten(),4)
		# 	print 'bXf : ', np.round(world_base_X_foot[:3,3].flatten(),4)
		# 	print 'NXf : ', np.round(v3_NRP_X_foot.flatten(),4)
		# 	print 'rst : ', np.round((v3_NRP_X_foot.item(0)**2 + v3_NRP_X_foot.item(1)**2)/(STEP_STROKE/2.)**2,4)

		# Magnitude of current point
		mag_nom_X_ee  = np.dot(v3_NRP_X_AEP.flatten(), v3_NRP_X_foot.flatten())

		# Angle between hip frame and AEP
		vec_ang = np.arctan2(v3_NRP_X_AEP.item(1), v3_NRP_X_AEP.item(0))

		## Ellipse boundary - for the front half: p_nom to AEP
		if (mag_nom_X_ee > 0.):
			# if (self.number == 0):
			# 	print self.number, ' ellipse boundary'
			# try:
			# 	# ellipse major, minor radius, rotation
			# 	a  = np.sqrt(v3_NRP_X_AEP.item(0)**2 + v3_NRP_X_AEP.item(1)**2)
			# 	b  = STEP_STROKE/2.
			# 	qr = vec_ang
			# 	x  = v3_NRP_X_foot.item(0)
			# 	y  = v3_NRP_X_foot.item(1)

			# 	r_state = ((x*np.cos(qr)+y*np.sin(qr))**2)/(a**2) + ((x*np.sin(qr)-y*np.cos(qr))**2)/(b**2)

			# 	# if (BOUND_FACTOR < r_state):
			# 	# 	bound_violate = True

			# except:
			pass

		## Circle boundary - for the back half: p_nom to PEP
		else:
			r_state = (v3_NRP_X_foot.item(0)**2 + v3_NRP_X_foot.item(1)**2)/(STEP_STROKE/2.)**2
			
			if (BOUND_FACTOR < r_state):
				bound_violate = True
		
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
			# print 'Leg ', self.number, ' q1 limit exceeded ', np.round(self.Joint.qpd[0],4)
			exceeded = True
		if (abs(self.Joint.qpd[1])>Q2_A_LIM):
			# print 'Leg ', self.number, ' q2 limit exceeded ', np.round(self.Joint.qpd[1],4)
			exceeded = True
		if (abs(self.Joint.qpd[2])>Q3_A_LIM):
			# print 'Leg ', self.number, ' q3 limit exceeded ', np.round(self.Joint.qpd[2],4)
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

			self.XHd.update_coxa_X_foot(self.qspline.xp[i])
			self.V6d.coxa_X_foot[:3]  = self.xspline.xv[i].reshape(3,1)
			self.A6d.coxa_X_foot[:3]  = self.xspline.xa[i].reshape(3,1)

			self.spline_counter += 1

			return True
		except:
			self.feedback_state = 2
			return False
