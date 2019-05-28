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
import rigid_body_inertia as Rbi
import plotgraph as Plot 				# plot library

class RobotState:
	def __init__(self):
		self.qc  = None;	# ROS JointState class for all joints
		self.qd  = None; 	# Vector array (Re^18x1) for joint position
		self.imu = None;	# ROS IMU class

		self.Leg  = [] 								# leg class
		self.KDL  = kdl.KDL()
		self.SM   = SMargin.StabilityMargin() 		# stability margin class
		self.Gait = Gaitgen.GaitClass(GAIT_TYPE)	# gait class
		self.Rbdl = Rbi.RigidBodyInertia()

		self.invalid = False 						# robot state: invalid - do not do anything
		self.suspend = False 						# robot state: suspend support (TODO)

		self.active_legs  = 6 						# robot state: number of legs selected to be active
		self.phase_change = False 					# robot state: phase changed, enables transfer trajectory to be generated

		self.support_mode = False 					# robot state: puts robot into support mode

		self.cstate = [None]*6 						# foot contact binary states
		self.cforce = [None]*18						# foot contact forces

		self.stance_offset = (TETA_F, TETA_R)

		self.XHc = robot_transforms.HomogeneousTransform() 	# position: current state
		self.XHd = robot_transforms.HomogeneousTransform()	# position: desired state
		self.P6c = robot_transforms.Vector6D() 				# position: current state in Re6
		self.P6d = robot_transforms.Vector6D() 				# position: current state in Re6
		self.V6c = robot_transforms.Vector6D() 				# velocity: current state
		self.V6d = robot_transforms.Vector6D() 				# velocity: desired state
		self.A6c = robot_transforms.Vector6D() 				# acceleration: current
		self.A6d = robot_transforms.Vector6D() 				# acceleration: desired

		self.P6c.world_X_base[2] = BODY_HEIGHT
		leg_stance = self.init_robot_stance()
		# self._initialise(leg_stance)

	def init_robot_stance(self, stance_type="flat"):

		self._initialise( self.set_leg_stance(STANCE_WIDTH, BODY_HEIGHT, self.stance_offset, stance_type) )

	def _initialise(self, leg_stance):
		""" Initialises robot class for setting up number of legs """

		for j in range(6):
			self.qd = self.KDL.leg_IK(leg_stance[j])
			self.Leg.append(leg_class.LegClass(j))
			self.Leg[j].XHc.update_base_X_femur(self.KDL.leg_IK(leg_stance[j]))
			self.Leg[j].XHd.update_base_X_femur(self.KDL.leg_IK(leg_stance[j]))
			self.Leg[j].XHc.update_base_X_foot(self.KDL.leg_IK(leg_stance[j]))
			self.Leg[j].XHd.update_base_X_foot(self.KDL.leg_IK(leg_stance[j]))
			self.Leg[j].XHc.update_base_X_NRP(self.KDL.leg_IK(leg_stance[j]))
			self.Leg[j].XHd.update_base_X_NRP(self.KDL.leg_IK(leg_stance[j]))
			self.Leg[j].XHc.update_coxa_X_foot(self.KDL.leg_IK(leg_stance[j]))
			self.Leg[j].XHd.update_coxa_X_foot(self.KDL.leg_IK(leg_stance[j]))
			self.Leg[j].XHc.update_coxa_X_NRP(self.KDL.leg_IK(leg_stance[j]))
			self.Leg[j].XHd.update_coxa_X_NRP(self.KDL.leg_IK(leg_stance[j]))
			self.Leg[j].XHc.world_X_foot = mX(self.XHc.world_X_base, self.Leg[j].XHc.base_X_foot)
			self.Leg[j].XHd.world_X_foot = self.Leg[j].XHc.world_X_foot.copy()
			self.Leg[j].XHd.base_X_AEP = mX(v3_X_m(np.array([ self.Gait.step_stroke/2,0,0])), self.Leg[j].XHd.base_X_NRP)
			self.Leg[j].XHd.base_X_PEP = mX(v3_X_m(np.array([-self.Gait.step_stroke/2,0,0])), self.Leg[j].XHd.base_X_NRP)
			self.Leg[j].XHd.world_base_X_AEP = self.Leg[j].XHd.base_X_AEP.copy()
			self.Leg[j].XHd.world_base_X_PEP = self.Leg[j].XHd.base_X_PEP.copy()
			self.Leg[j].XHc.base_X_AEP = mX(v3_X_m(np.array([ self.Gait.step_stroke/2,0,0])), self.Leg[j].XHc.base_X_NRP)
			self.Leg[j].XHc.base_X_PEP = mX(v3_X_m(np.array([-self.Gait.step_stroke/2,0,0])), self.Leg[j].XHc.base_X_NRP)
			self.Leg[j].XHc.world_base_X_AEP = self.Leg[j].XHc.base_X_AEP.copy()
			self.Leg[j].XHc.world_base_X_PEP = self.Leg[j].XHc.base_X_PEP.copy()
			self.Leg[j].XHc.update_world_base_X_NRP(self.P6c.world_X_base)
			self.Leg[j].XHd.update_world_base_X_NRP(self.P6c.world_X_base)

		self.task_X_joint()
		self.Gait.set_step_stroke(leg_stance, LEG_CLEAR, STEP_STROKE)
		print ">> INITIALISED ROBOT CLASS"

	def update_state(self,**options):
		""" update robot state using readings from topics """

		reset = True if options.get("reset") == True else False
		cmode = "fast" if options.get("control_mode") == "fast" else "normal"

		self.update_bodypose_state(cmode)
		self.update_leg_state(reset, cmode)
		self.update_stability_margin()

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
			qpc = self.qd#self.qc.position

		# update leg states and check if boundary exceeded
		for j in range(0,self.active_legs):

			bstate = self.Leg[j].update_joint_state(wXb, qpc[offset+j*3:offset+(j*3)+3], reset, self.Gait.step_stroke)
			cstate = self.Leg[j].update_force_state(self.cstate[j], self.cforce[j*3:(j*3)+3])

			if (bstate==True and self.Gait.cs[j]==0 and self.support_mode==False):
				self.suspend = True

	def update_bodypose_state(self, cmode):
		""" update imu state """
		## TODO: INCLUDE STATE ESTIMATION

		if (cmode == "fast"):
			## Updates robot state using setpoints
			self.XHc.world_X_base = self.XHd.world_X_base.copy()
			self.V6c.world_X_base = self.V6d.world_X_base.copy()
			self.A6c.world_X_base = self.A6d.world_X_base.copy()
		else:
			## Hassan: I'm presuming the contents in this function will be changed. The next two lines are the variables 
			## that will be used in the main code, they are a row vector of 6x1 (linear, angular) 
			# self.P6c.world_X_base = hassan_state_estimation_output()
			# self.V6c.world_X_base = hassan_state_estimation_output()
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
		stack_world_bXw = []
		stack_base_bXw = []

		for j in range(6):
			stack_world_bXw.append(self.Leg[j].XHc.world_base_X_foot[:3,3])
			stack_base_bXw.append(self.Leg[j].XHc.base_X_foot[:3,3])

		# self.SM.check_angle(stack_world_bXw, self.Gait.cs)
		# compute Longitudinal Stability Margin
		valid, sm = self.SM.point_in_convex(np.zeros(3), stack_world_bXw, self.Gait.cs)

		if not valid:
			print 'Convex hull: ', valid, sm
			self.invalid = True
			self.SM.point_in_convex(np.zeros(3), stack_world_bXw, self.Gait.cs, True)
		else:
			self.invalid = False

		if sm < 0.:
			self.invalid = True
		# sm = self.SM.LSM(stack_world_bXw, self.Gait.cs)
		# # print self.Gait.cs
		# # print stack_world_bXw
		# print 'SM: ', np.round(sm,3)
		# try:
		# 	if (sm[0]<=SM_MIN or sm[1]<=SM_MIN):
		# 		print 'Stability Violated!'
		# 		self.invalid = True
		# 	else:
		# 		self.invalid = False
		# except Exception, e:
		# 	print 'Error: ', e
		# 	print 'Robot Stopping'
		# 	self.invalid = True

	def update_rbdl(self, qb, qp):
		""" Updates robot CoM and CRBI """
		
		self.Rbdl.update_CRBI(qb)
		self.Rbdl.update_CoM(qp)
		""" remaps tuple to numpy array """
		# nc = 0
		# for i in range(0,3):
		# 	self.P6c.base_X_CoM[i] = i_com[i]
		# 	for j in range(0,3):
		# 		self.CRBI[i,j] = i_crbi[nc]
		# 		nc += 1

	def alternate_phase(self, new_phase=None):
		""" change gait to next phase """

		if (new_phase is None):
			self.Gait.change_phase()
		else:
			self.Gait.ps = self.Gait.cs
			self.Gait.cs = new_phase

		# update robot leg phase_change
		for j in range(0,6):
			if (self.Gait.cs[j] == 1):
				self.Leg[j].transfer_phase_change = False

			elif (self.Gait.cs[j] == 0 and self.Gait.cs[j] != self.Gait.ps[j]):
				# Update world_X_foot when phase changes from transfer to support to avoid drifting
				self.Leg[j].XHc.update_world_X_foot(self.XHc.world_X_base)
				self.Leg[j].XHd.world_X_foot = self.Leg[j].XHc.world_X_foot.copy()

		self.suspend = False

	def task_X_joint(self,legs_phase=None): # TODO - to be revisited
		""" Convert all leg task space to joint space 		"""
		""" Output: joint space setpoint for all joints		"""

		## Define variables ##
		qt = []
		qp = []
		qv = []
		qa = []
		err_list = [0]*6

		for j in range(0,6):
			err_list[j], qpd, qvd, qad = self.Leg[j].tf_task_X_joint()

			## append to list if valid, otherwise break and raise error
			err_str = ''
			if (err_list[j] == 0):
				for z in range(0,3):
					qp.append(qpd.item(z))
					qv.append(qvd.item(z))
					qa.append(qad.item(z))
			else:
				if (err_list[j] == 1):
					err_str = 'Error - joint limit exceeded in leg, '
				elif (err_list[j] == 2):
					err_str = 'Error - singularity in leg, '
				elif (err_list[j] == 3):
					err_str = 'Error - no kinematic solution in leg, '
				else:
					err_str = 'Unknown error'
				print err_str

		## check to ensure size is correct
		if (len(qp)==18):
			self.qd = qp 	# remap for "fast"
			return JointTrajectoryPoints(18,(qt,qp,qv,qa)), err_list
		else:
			self.invalid = True
			return None, err_list

	def force_to_torque(self, fforce):
		""" compute joint torque from foot force """

		tau = []

		for j in range(0,6):
			# Transform foot force from world to hip frame
			self.Leg[j].XHd.update_world_X_coxa(self.XHd.world_X_base)
			self.Leg[j].F6d.world_X_foot[:3] = fforce[j*3:j*3+3]
			self.Leg[j].F6d.coxa_X_foot[:3] = mX(self.Leg[j].XHd.coxa_X_world[:3,:3],self.Leg[j].F6d.world_X_foot[:3])

			# Determine joint torque using Jacobian from hip to foot, tau = J^T.f
			tau_leg = self.KDL.force_to_torque(self.qd[j*3:j*3+3], self.Leg[j].F6d.coxa_X_foot[:3])

			for z in range(0,3):
				tau.append(tau_leg.item(z))

			# if (j==1):
			# 	print j, ' Fwor ', np.round(self.Leg[j].F6d.world_X_foot[:3].flatten(),3)
			# 	print j, ' Fhip ', np.round(self.Leg[j].F6d.coxa_X_foot[:3].flatten(),3)
			# 	print j, ' tau  ', np.round(tau_leg.flatten(),3)
		if (len(tau)==18):
			return tau
		else:
			print 'Error in force to torque conversion!'
			return None

	def duplicate_self(self, robot):
		""" Duplicates robot state by creating local copy of input robot """

		self.Gait.cs = list(robot.Gait.cs) 	# use local gait current state
		self.Gait.np = robot.Gait.np 		# copy gait counter

		self.XHc.duplicate(robot.XHc)
		self.XHd.duplicate(robot.XHd)
		self.P6c.duplicate(robot.P6c)
		self.P6d.duplicate(robot.P6d)
		self.V6c.duplicate(robot.V6c)
		self.V6d.duplicate(robot.V6d)
		self.A6c.duplicate(robot.A6c)
		self.A6d.duplicate(robot.A6d)

		for j in range(0,6):
			self.Leg[j].duplicate_self(robot.Leg[j])

	def set_leg_stance(self, stance_width, base_height, stance_offset, stance_type="flat"):
		""" Sets leg stance according to front & rear leg offset """

		leg_stance = {}
		teta_f = stance_offset[0]
		teta_r = stance_offset[1]

		# Flat ground stance - original
		if (stance_type == "flat"):
			# print 'Flat selected'
			leg_stance[0] = np.array([ stance_width*np.cos(teta_f*np.pi/180), stance_width*np.sin(teta_f*np.pi/180), -base_height ])
			leg_stance[1] = np.array([ stance_width, 0, -base_height])
			leg_stance[2] = np.array([ stance_width*np.cos(teta_r*np.pi/180), stance_width*np.sin(teta_r*np.pi/180), -base_height ])
			leg_stance[3] = np.array([ stance_width*np.cos(teta_f*np.pi/180), stance_width*np.sin(-teta_f*np.pi/180), -base_height ])
			leg_stance[4] = np.array([ stance_width, 0, -base_height])
			leg_stance[5] = np.array([ stance_width*np.cos(teta_r*np.pi/180), stance_width*np.sin(-teta_r*np.pi/180), -base_height ])

		elif (stance_type == "chimney"):
			base_height = 0.
			stance_width = 0.27

			leg_stance[0] = np.array([ stance_width*np.cos(teta_f*np.pi/180), stance_width*np.sin(teta_f*np.pi/180), -base_height ])
			leg_stance[1] = np.array([ stance_width, 0, -base_height])
			leg_stance[2] = np.array([ stance_width*np.cos(teta_r*np.pi/180), stance_width*np.sin(teta_r*np.pi/180), -base_height ])
			leg_stance[3] = np.array([ stance_width*np.cos(teta_f*np.pi/180), stance_width*np.sin(-teta_f*np.pi/180), -base_height ])
			leg_stance[4] = np.array([ stance_width, 0, -base_height])
			leg_stance[5] = np.array([ stance_width*np.cos(teta_r*np.pi/180), stance_width*np.sin(-teta_r*np.pi/180), -base_height ])

		else:
			print 'Invalid stance selected!'
			leg_stance = None

		return leg_stance
## ====================================================================================================================================== ##
## ====================================================================================================================================== ##

# robot = RobotState()
