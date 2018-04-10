#!/usr/bin/env python

""" Robot class for specifying robot parameters and robot updates 
 	Indexing for leg starts with 0, 1 .... 5
""" 

import sys; sys.dont_write_bytecode = True

import math
import numpy as np

from traits import *
from constant import * 					# constants used
import leg_class 						# class for robot's leg
import gait_class as Gaitgen			# class for gait coordinator
import pspline_generator as Pspline 	# spline generator for body
import bspline_generator as Bspline 	# spline generator for leg
import stability_margin as SMargin 		# stability margin evaluation
import kdl 								# kinematic & dynamics library
import robot_transforms					# transformation and vector class for robot
from matrix_transforms import *			# generic transformation library

import plotgraph as Plot 				# plot library

class RobotState:
	def __init__(self):
		self.qc  = None;	# ROS JointState class for all joints
		self.qd  = None; 	# Vector array (Re^18x1) for joint position
		self.imu = None;	# ROS IMU class
		
		self.Leg  = [] 								# leg class
		self.Gait = Gaitgen.GaitClass(GAIT_TYPE)	# gait class
		self.KDL  = kdl.KDL()
		self.SM   = SMargin.StabilityMargin() 		# stability margin class

		self.invalid = False 						# robot state: invalid - do not do anything
		self.suspend = False 						# robot state: suspend support (TODO)

		self.active_legs  = 6 						# robot state: number of legs selected to be active
		self.phase_change = False 					# robot state: phase changed, enables transfer trajectory to be generated

		self.support_mode = False 					# robot state: puts robot into support mode

		self.cstate = [None]*6 						# foot contact binary states
		self.cforce = [None]*18						# foot contact forces

		self.XHc = robot_transforms.HomogeneousTransform() 	# position: current state
		self.XHd = robot_transforms.HomogeneousTransform()	# position: desired state
		self.P6c = robot_transforms.Vector6D() 				# position: current state in Re6
		self.P6d = robot_transforms.Vector6D() 				# position: current state in Re6
		self.V6c = robot_transforms.Vector6D() 				# velocity: current state
		self.V6d = robot_transforms.Vector6D() 				# velocity: desired state
		self.A6c = robot_transforms.Vector6D() 				# acceleration: current
		self.A6d = robot_transforms.Vector6D() 				# acceleration: desired
		
		self._initialise()

	def _initialise(self):
		""" Initialises robot class for setting up number of legs """

		self.P6c.world_X_base[2] = BODY_HEIGHT

		for j in range(6):
			self.qd = self.KDL.leg_IK(LEG_STANCE[j])
			self.Leg.append(leg_class.LegClass(j))
			self.Leg[j].XHc.update_base_X_femur(self.KDL.leg_IK(LEG_STANCE[j]))
			self.Leg[j].XHd.update_base_X_femur(self.KDL.leg_IK(LEG_STANCE[j]))
			self.Leg[j].XHc.update_base_X_foot(self.KDL.leg_IK(LEG_STANCE[j]))
			self.Leg[j].XHd.update_base_X_foot(self.KDL.leg_IK(LEG_STANCE[j]))
			self.Leg[j].XHc.update_base_X_NRP(self.KDL.leg_IK(LEG_STANCE[j]))
			self.Leg[j].XHd.update_base_X_NRP(self.KDL.leg_IK(LEG_STANCE[j]))
			self.Leg[j].XHc.update_coxa_X_foot(self.KDL.leg_IK(LEG_STANCE[j]))
			self.Leg[j].XHd.update_coxa_X_foot(self.KDL.leg_IK(LEG_STANCE[j]))
			self.Leg[j].XHc.update_coxa_X_NRP(self.KDL.leg_IK(LEG_STANCE[j]))
			self.Leg[j].XHd.update_coxa_X_NRP(self.KDL.leg_IK(LEG_STANCE[j]))
			self.Leg[j].XHc.world_X_foot = mX(self.XHc.world_X_base, self.Leg[j].XHc.base_X_foot)
			self.Leg[j].XHd.world_X_foot = mX(self.XHd.world_X_base, self.Leg[j].XHd.base_X_foot)
			self.Leg[j].XHd.base_X_AEP = mX(v3_X_m(np.array([ STEP_STROKE/2,0,0])), self.Leg[j].XHd.base_X_NRP)
			self.Leg[j].XHd.base_X_PEP = mX(v3_X_m(np.array([-STEP_STROKE/2,0,0])), self.Leg[j].XHd.base_X_NRP)
			self.Leg[j].XHd.world_base_X_AEP = self.Leg[j].XHd.base_X_AEP.copy()
			self.Leg[j].XHd.world_base_X_PEP = self.Leg[j].XHd.base_X_PEP.copy()
			self.Leg[j].XHc.base_X_AEP = mX(v3_X_m(np.array([ STEP_STROKE/2,0,0])), self.Leg[j].XHc.base_X_NRP)
			self.Leg[j].XHc.base_X_PEP = mX(v3_X_m(np.array([-STEP_STROKE/2,0,0])), self.Leg[j].XHc.base_X_NRP)
			self.Leg[j].XHc.world_base_X_AEP = self.Leg[j].XHc.base_X_AEP.copy()
			self.Leg[j].XHc.world_base_X_PEP = self.Leg[j].XHc.base_X_PEP.copy()

		self.task_X_joint()
		print ">> INITIALISED ROBOT CLASS"

	def update_state(self,**options):
		""" update robot state using readings from topics """ 

		reset = True if options.get("reset") == True else False
		cmode = "fast" if options.get("control_mode") == "fast" else "normal"

		self.update_bodypose_state(cmode)
		self.update_leg_state(reset, cmode)
		# self.update_stability_margin()
		
	def update_leg_state(self, reset, cmode):
		""" update legs state """
		## TODO: INTEGRATE HW FORCE SENSOR READINGS

		# self.XHc.update_legs(self.qc.position) 	# NOT USED

		## Offset to deal with base joints - suspended control of base for
		## fixed_robot evaluation in Gazebo
		offset = 0 if (len(self.qc.name)==18) else 1

		if (cmode == "fast"):
			## Updates robot state using setpoints
			wXb = self.P6d.world_X_base
			qpc = self.qd
		else:
			## Updates robot state based on measured states
			wXb = self.P6c.world_X_base
			qpc = self.qc.position

		# update leg states and check if boundary exceeded
		for j in range(0,self.active_legs):
			
			bstate = self.Leg[j].update_joint_state(wXb, qpc[offset+j*3:offset+(j*3)+3], reset)
			cstate = self.Leg[j].update_force_state(self.cstate[j], self.cforce[j*3:(j*3)+3])

			if (bstate==True and self.Gait.cs[j]==0 and self.support_mode==False):
				self.suspend = True
				print 'Suspend ', j
				
		# print '-------------------------------------------------------'

	def update_bodypose_state(self, cmode):
		""" update imu state """
		## TODO: INCLUDE STATE ESTIMATION

		if (cmode == "fast"):
			## Updates robot state using setpoints
			self.XHc.world_X_base = self.XHd.world_X_base.copy()
			self.V6c.world_X_base = self.V6d.world_X_base.copy()
			self.A6c.world_X_base = self.A6d.world_X_base.copy()
		else:
			self.imu = None
			if (self.imu is not None):
				## quaternion to euler transformation
				rpy = np.array(euler_from_quaternion([self.imu.orientation.w, self.imu.orientation.x,
														 self.imu.orientation.y, self.imu.orientation.z], 'sxyz')).reshape(3,1)
				wv3 = np.array([ [self.imu.angular_velocity.x], 	[self.imu.angular_velocity.y], 	  [self.imu.angular_velocity.z] ])
				ca3 = np.array([ [self.imu.linear_acceleration.x], 	[self.imu.linear_acceleration.y], [self.imu.linear_acceleration.z] ])
			
				## state estimation
				xyz = np.zeros((3,1))
				## Updates robot state based on measured states
				self.XHc.update_base(p6c)
				self.V6c.world_X_base[3:6] = wv3
				self.A6c.world_X_base[0:3] = ca3

			else:
				## Updates robot state using setpoints
				self.XHc.update_base(self.P6c.world_X_base)
				self.V6c.world_X_base = self.V6d.world_X_base.copy()
				self.A6c.world_X_base = self.A6d.world_X_base.copy()

	def update_stability_margin(self):
		""" updates the current stability margin """

		## Define Variables ##
		stack_base_X_world = []

		for j in range(6):
			stack_base_X_world.append(self.Leg[j].XHc.world_base_X_foot[:3,3])
		
		# compute Longitudinal Stability Margin
		sm = self.SM.LSM(stack_base_X_world, self.Gait.cs)
		try:
			if (sm[0]<=SM_MIN or sm[1]<=SM_MIN):
				print 'Stability Violated!'
				self.invalid = True
			else:
				self.invalid = False
		except Exception, e:
			print 'Error: ', e
			print 'Robot Stopping'
			self.invalid = True

	def alternate_phase(self):
		""" change gait to next phase """
		
		self.Gait.change_phase()

		# update robot leg phase_change
		for j in range(0,6):
			if (self.Gait.cs[j] == 1):
				self.Leg[j].transfer_phase_change = False

			elif (self.Gait.cs[j] == 0 and self.Gait.cs[j] != self.Gait.ps[j]): 
				# Update world_X_foot when phase changes from transfer to support to avoid drifting
				self.Leg[j].XHc.update_world_X_foot(self.XHc.world_X_base) 			
				self.Leg[j].XHd.world_X_foot = self.Leg[j].XHc.world_X_foot.copy()

		self.suspend = False
		# raw_input('cont')

	def task_X_joint(self,legs_phase=None): # TODO - to be revisited
		""" Convert all leg task space to joint space 		"""								
		""" Output: joint space setpoint for all joints		"""

		## Define variables ##
		qt = []
		qp = []
		qv = []
		qa = []
		
		for j in range(0,6):
			error, qpd, qvd, qad = self.Leg[j].tf_task_X_joint()
			# if (j==5):
			# 	print np.round(qpd.flatten(),4)
			## append to list if valid, otherwise break and raise error
			try:
				err_str = 'Unknown error'
				if (error == 0):
					for z in range(0,3):
						qp.append(qpd.item(z))
						qv.append(qvd.item(z))
						qa.append(qad.item(z))
				else:
					if (error == 1):
						err_str = 'Error - joint limit exceeded in leg, '
					elif (error == 2):
						err_str = 'Error - singularity in leg, '
					elif (error == 3):
						err_str = 'Error - no kinematic solution in leg, '
					raise ValueError, err_str
			except Exception, e:
				print e, j

		## check to ensure size is correct
		# print 'length: ', len(qp)
		if (len(qp)==18):
			self.qd = qp 	# remap for "fast"
			return JointTrajectoryPoints(18,(qt,qp,qv,qa))
		else:
			self.invalid = True
			return None

# robot = RobotState()
## ====================================================================================================================================== ##
## ====================================================================================================================================== ##


