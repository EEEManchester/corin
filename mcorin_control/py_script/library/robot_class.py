#!/usr/bin/env python

## Class for robot and leg
## Indexing for leg starts with 0, 1 .... 5

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'class'))
sys.dont_write_bytecode = True

import rospy 										# ROS dependent file
from sensor_msgs.msg import Imu 					# sub msg for IMU
from sensor_msgs.msg import JointState 				# sub msg for joint states
import tf as RTF 									# transform library

import math
import numpy as np

from constant import * 								# constants used
from TrajectoryPoints import TrajectoryPoints		# class for 3D time array of points
import Twist
import State
import transformations as tf 						# SE(3) transformation library
import gait_class as Gaitgen						# class for gait coordinator
import param_gait									# class for setting gait parameters in RT 
import kdl 											# kinematic & dynamics library
import pspline_generator as Pspline 				# spline generator for body
import bspline_generator as Bspline 				# spline generator for leg
import plotgraph as Plot 							# plot library
import transforms

class RobotState:
	def __init__(self):
		self.qc  = JointState() 					# ROS JointState class for all joints
		self.imu = Imu()							# ROS IMU class
		
		self.Leg = [] 								# leg class
		self.gaitgen = Gaitgen.GaitClass(GAIT_TYPE)	# gait class
		self.bspline = Bspline.SplineGenerator() 	# bSplineClass for spline generation

		self.x_spline = TrajectoryPoints() 			# CoB linear trajectory
		self.w_spline = TrajectoryPoints() 			# CoB angular trajectory

		self.world_X_base = State.StateClass('Twist') 				# FrameClass for world to base transformation
		self.fr_world_X_base = tf.rotation_zyx(np.zeros(3)) 	# SO(3) rotation using pose

		self.invalid = False 						# robot state: invalid - do not do anything
		self.suspend = False 						# robot state: suspend support (TODO)

		self.active_legs  = 6 						# robot state: number of legs selected to be active
		self.phase_change = False 					# robot state: phase changed, enables transfer trajectory to be generated
		self.reset_state  = True 					# robot state: reset whole of robot state

		self.support_mode = False 					# robot state: puts robot into support mode

		self.cstate = [None]*6 						# foot contact binary states
		self.cforce = [None]*18						# foot contact forces

		self.XHc = transforms.HomogeneousTransform()
		self.XHd = transforms.HomogeneousTransform()

		self._initialise()

	def _initialise(self):
		""" Initialises robot class for setting up number of legs """

		for i in range(6):
			self.Leg.append(LegClass(i))
		print ">> INITIALISED ROBOT CLASS"

	def updateState(self):
		""" update robot state using readings from topics """ 
		self.update_LegState()
		# self.update_ImuState()

	def update_LegState(self):
		""" update legs state """

		self.XHc.update_legs(self.qc.position)

		# offset to deal with base joints - suspended control of base
		if (len(self.qc.name)==18):
			offset = 0
		else:
			offset = 1

		# update leg states and check if boundary exceeded
		for i in range(0,self.active_legs):

			bstate = self.Leg[i].updateJointState(self.qc.position[offset+i*3:offset+(i*3)+3], self.reset_state)

			self.Leg[i].updateForceState(self.cstate[i], self.cforce[i*3:(i*3)+3])

			if (bstate==True and self.gaitgen.cs[i]==0 and self.support_mode==False):
				self.suspend = True
				print 'suspended'

		# clear reset state
		self.reset_state = False if self.reset_state == True else False

	def update_ImuState(self):
		""" update imu state """

		## quaternion to euler transformation
		neuler = RTF.transformations.euler_from_quaternion([self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w])

		## update variables
		self.world_X_base.es.wp = self.world_X_base.ds.wp - self.world_X_base.cs.wp

	def generateSpline(self, nLeg, start, end, snorm, phase, reflex=False, ctime=2.0, tn=0.1):
		""" generate leg trajectory using bspline and save to leg class 	"""
		""" Input:  a) nLeg   -> Leg number
			 		b) start  -> start position in 3D space wrt base frame
					c) end    -> end position in 3D space wrt base frame
					d) snorm  -> surface normal
					e) phase  -> leg phase: transfer = 1, support = 0
					f) reflex -> boolen for reflex trajectory
					g) ctime  -> time for trajectory						
					h) tn 	  -> rate of trajectory 						"""

		self.Leg[nLeg].spline = TrajectoryPoints(self.bspline.generate_leg_spline(start.flatten(), end.flatten(), snorm, phase, reflex, ctime, tn))
		self.Leg[nLeg].spline_counter = 1
		self.Leg[nLeg].spline_length  = len(self.Leg[nLeg].spline.t)

class Joint_class:
	#common base class for Joint
	def __init__(self, num):
		self.name 	= np.array([ 'leg_' + str(num) + '_q1', 'leg_' + str(num) + '_q2', 'leg_' + str(num) + '_q3'])
		self.qpc 	= np.zeros(3) 		# joint current position
		self.qpd 	= np.zeros(3)		# joint desired position
		self.qpe 	= np.zeros(3)		# joint position error
		self.qvc	= np.zeros(3)		# joint current velocity
		self.qvd	= np.zeros(3)		# joint desired velocity
		self.qac 	= np.zeros(3) 		# joint current acceleration
		self.qad 	= np.zeros(3) 		# joint desired acceleration
		self.qtc 	= np.zeros(3)		# joint current effort
		self.qtd 	= np.zeros(3)		# joint desired effort

	def update_state(self, jointState):
		self.qpc = np.array(jointState.m_state.actual.positions)
		self.qpe = np.array(jointState.m_state.error.positions)

		self.qvc = np.array(jointState.m_state.actual.velocities)
		self.qtc = np.array(jointState.m_state.actual.effort)

	def state_display(self):
		print 'q position: ', self.qpc[0:3]
		print 'q p  error: ', self.qpe[0:3]
		print 'q velocity: ', self.qvc[0:3]
		print 'q  effort : ', self.qtc[0:3]

class LegClass:
	#common base class for Leg
	def __init__(self, name):
		self.number = name
		self.Joint 	= Joint_class(self.number)

		# kinematic library functions
		self.KL 	= kdl.corin_kinematics()
		# self.DL 	= kdl.corin_dynamics()

		# transfer phase variables
		self.spline = TrajectoryPoints()
		self.spline_counter = 1
		self.spline_length  = 0

		self.feedback_state	= 0 				# 0 = idle, 1 = command received (executing), 2 = command completed

		self.cstate = 0 	# foot contact state: 1 or 0

		# transformation frames
		self.hip_X_ee 	= State.StateClass('Twist')
		self.base_X_ee 	= State.StateClass('Twist')

		self.foot_XF_ee = State.StateClass('Wrench')
		self.hip_XF_ee 	= State.StateClass('Wrench')
		self.base_XF_ee	= State.StateClass('Wrench')

		self.tf_base_X_hip 	= tf.rotation_matrix(TF_BASE_X_HIP[self.number],Z_AXIS)[:3, :3]

		self.AEP = np.zeros((3,1)) 		# Anterior Extreme Position for leg wrt base frame
		self.PEP = np.zeros((3,1))		# Posterior Extreme Position for leg wrt base frame
		self.REP = np.zeros((3,1)) 		# nominal stance for leg wrt base frame

		self.hip_AEP = np.zeros((3,1)) 	# for storing transfer AEP

		self.leg_positions() 			# set rest position of legs

		self.qsurface = np.array([0.,0.,0.]) 		# surface orientation in leg frame, (roll, pitch, yaw)
		self.qsnorm   = np.array([[0.],[0.],[1.]]) 	# surface normal

		self.phase_change = False 		# Flag for enabling trajectory to be generated at transfer

	## Transformation functions
	def base_X_hip_ee(self, base_X_ee_xp):
		return np.dot( tf.rotation_matrix(TF_BASE_X_HIP[self.number],Z_AXIS)[:3, :3], (-FR_base_X_hip[self.number] + base_X_ee_xp))

	def hip_X_base_ee(self, hip_X_ee_xp):
		return (FR_base_X_hip[self.number] + np.dot( tf.rotation_matrix(TF_HIP_X_BASE[self.number],Z_AXIS)[:3, :3], hip_X_ee_xp))

	## Update functions
	def updateJointState(self, jointState, resetState):

		## ********************************	Joint Angle  ******************************** ##
		## Error Compensation
		q_compensated = (jointState[0], jointState[1]+QCOMPENSATION, jointState[2]) 		# offset q2 by 0.01 - gravity

		## current
		self.hip_X_ee.cs.xp  = self.KL.FK(q_compensated)
		self.base_X_ee.cs.xp = self.hip_X_base_ee(self.hip_X_ee.cs.xp)

		## error
		self.hip_X_ee.es.xp  = self.hip_X_ee.ds.xp  - self.hip_X_ee.cs.xp
		self.base_X_ee.es.xp = self.base_X_ee.ds.xp - self.base_X_ee.cs.xp

		## check work envelope
		bound_exceed = self.boundary_limit()

		## state reset
		if (resetState):
			self.hip_X_ee.ds.xp  = self.hip_X_ee.cs.xp
			self.base_X_ee.ds.xp = self.base_X_ee.cs.xp

		# if (self.number == 0):
		# 	print self.number, ' ', jointState
		# 	print 'bXe_ds: ', np.round(self.base_X_ee.ds.xp.flatten(),4)
		# 	print 'bXe_cs: ', np.round(self.base_X_ee.cs.xp.flatten(),4)
		# 	print 'hXe_ds: ', np.round(self.hip_X_ee.ds.xp.flatten(),4)
		# 	print 'hXe_cs: ', np.round(self.hip_X_ee.cs.xp.flatten(),4)

		return bound_exceed

	def updateForceState(self, cState, cForce):
		""" contact force of leg """
		## ********************************	Contact Force  ******************************** ##
		self.cstate = cState
		self.foot_XF_ee.cs = np.reshape(np.array(cForce),(3,1))

		## TODO: TRANSFORM FROM FOOT FRAME TO HIP, BASE, WORLD FRAME

	def boundary_limit(self):
		bound_violate = False
		BOUND_FACTOR  = 1.8 	# relaxes the boundary constraint

		# vector to aep and current position wrt nominal point
		vec_nom_X_aep = self.hip_AEP - np.reshape(LEG_STANCE[self.number],(3,1))
		vec_nom_X_ee  = self.hip_X_ee.cs.xp - np.reshape(LEG_STANCE[self.number],(3,1))

		# magnitude of current point
		mag_nom_X_ee  = np.dot(vec_nom_X_aep.flatten(), vec_nom_X_ee.flatten())

		# angle between hip frame and AEP
		vec_ang = np.arctan2(vec_nom_X_aep.item(1), vec_nom_X_aep.item(0))

		# ellipse boundary - for the front half: p_nom to AEP
		if (mag_nom_X_ee > 0.):
			# print self.number, ' ellipse boundary'
			try:
				# ellipse major, minor radius, rotation
				a  = np.sqrt(vec_nom_X_aep.item(0)**2 + vec_nom_X_aep.item(1)**2)
				b  = STEP_STROKE/2.
				qr = vec_ang
				x  = vec_nom_X_ee.item(0)
				y  = vec_nom_X_ee.item(1)

				r_state = ((x*np.cos(qr)+y*np.sin(qr))**2)/(a**2) + ((x*np.sin(qr)-y*np.cos(qr))**2)/(b**2)

				if (BOUND_FACTOR < r_state):
					bound_violate = True
					# print self.number, ' ellipse boundary violated ', r_state
			except:
				pass

		# circle boundary - for the back half: p_nom to PEP
		else:
			# print self.number, ' circle boundary'
			r_state = (vec_nom_X_ee.item(0)**2 + vec_nom_X_ee.item(1)**2)/(STEP_STROKE/2.)**2

			if (BOUND_FACTOR < r_state):
				bound_violate = True
				# print self.number, ' circle boundary violated ', r_state

		return bound_violate

	## Kinematic functions
	def getJointKinematic(self,velocity):
		self.Joint.qpd = self.KL.IK(self.hip_X_ee.ds.xp)

		# checks if joint limit exceeded
		q_exceeded = self.checkJointLimit()

		# checks if there's need to compute joint velocity
		if (not self.KL.singularity_check(self.Joint.qpd)):
			if (velocity==True):
				self.Joint.qvd, self.Joint.qad = self.KL.joint_speed(self.Joint.qpd, self.hip_X_ee.ds.xv, self.hip_X_ee.ds.xa)
			return True
		else:
			print 'Singularity detected'
			return False

	def checkJointLimit(self):
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

	## ============ Convenience functions ============ ## 
	## Unstack next point
	def pointToArray(self):
		# if (self.number==0): print 'spline count: ', self.spline_counter, '\t', self.spline_length
		i = self.spline_counter
		self.hip_X_ee.ds.xp = self.spline.xp[i]
		self.hip_X_ee.ds.xv = self.spline.xv[i]
		self.hip_X_ee.ds.xa = self.spline.xa[i]

	## Configure leg positions
	def leg_positions(self):
		self.REP = FR_base_X_hip[self.number] + np.dot( tf.rotation_matrix(TF_HIP_X_BASE[self.number],Z_AXIS)[:3, :3], np.reshape(LEG_STANCE[self.number],(3,1)) )

	def update_REP(self, bodypose, base_X_surface):
		""" updates nominal stance of robot """

		# update only if surface orientation above deadzone
		if (abs(self.qsurface.item(0)) > QDEADZONE or abs(self.qsurface.item(1)) > QDEADZONE):
			self.REP = FR_base_X_hip[self.number] + self.KL.nominal_stance(bodypose, base_X_surface, self.qsurface)
			# print 'updating REP for leg ', self.number
			# if (self.number == 0):
