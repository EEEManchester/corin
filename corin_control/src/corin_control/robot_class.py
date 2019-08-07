#!/usr/bin/env python

""" Robot class for specifying robot parameters and robot updates
 	Indexing for leg starts with 0, 1 .... 5
"""

import sys; sys.dont_write_bytecode = True

import math
import numpy as np
import time
from termcolor import colored

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
from impedance_controller import ImpedanceController
from fault_controller import FaultController
from pid_controller import PIDController
from state_estimator import StateEstimator  # EKF-based state estimation class
import stabilipy

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
		self.state_estimator = StateEstimator(self, 1./IMU_RATE)   # declare during update_state(reset)
		
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
		
		self.impedance_controller_x = ImpedanceController(2., 2.2, 0.002)
		self.impedance_controller_y = ImpedanceController(2., 2.2, 0.002)
		self.impedance_controller_z = ImpedanceController(2., 2.2, 0.002)	# fn, D, G
		self.Fault = FaultController()
		self.Base_Ctrl = PIDController('PI', 6, KP_P_BASE, KI_P_BASE)

		self.init_robot_stance("flat")
		
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
			self.Leg[j].XHd.world_base_X_AEP[:3,3] = mX(self.XHc.world_X_base[:3,:3], self.Leg[j].XHc.base_X_AEP[:3,3]) #self.Leg[j].XHd.base_X_AEP.copy()
			self.Leg[j].XHd.world_base_X_PEP[:3,3] = mX(self.XHc.world_X_base[:3,:3], self.Leg[j].XHc.base_X_AEP[:3,3])	#self.Leg[j].XHd.base_X_PEP.copy()
			# self.Leg[j].XHd.world_base_X_AEP = self.Leg[j].XHd.base_X_AEP.copy()
			# self.Leg[j].XHd.world_base_X_PEP = self.Leg[j].XHd.base_X_PEP.copy()
			self.Leg[j].XHc.base_X_AEP = mX(v3_X_m(np.array([ self.Gait.step_stroke/2,0,0])), self.Leg[j].XHc.base_X_NRP)
			self.Leg[j].XHc.base_X_PEP = mX(v3_X_m(np.array([-self.Gait.step_stroke/2,0,0])), self.Leg[j].XHc.base_X_NRP)
			self.Leg[j].XHc.world_base_X_AEP = self.Leg[j].XHc.base_X_AEP.copy()
			self.Leg[j].XHc.world_base_X_PEP = self.Leg[j].XHc.base_X_PEP.copy()
			self.Leg[j].XHc.update_world_base_X_NRP(self.P6c.world_X_base)
			self.Leg[j].XHd.update_world_base_X_NRP(self.P6c.world_X_base)
		self.task_X_joint()
		self.update_stability_region()
		self.Gait.set_step_stroke(leg_stance, LEG_CLEAR, STEP_STROKE)
		
		# print ">> INITIALISED ROBOT CLASS"

	def update_state(self,**options):
		""" update robot state using readings from topics """

		reset = True if options.get("reset") == True else False
		cmode = "fast" if options.get("control_mode") == "fast" else "normal"

		if reset:
			i = 0
			self.state_estimator.reset_state()
			# Wait for esimtate_state to be called enough times for calibration
			while not self.state_estimator.calibrated:
				time.sleep(0.002)
				i += 1
				if i > 1000:
					# raise Exception("Could not calibrate.")
					print colored("ERROR: Failed calibrating state estimator",'red')
					break
			else:
				print colored("State Estimator calibrated", 'green') 

		self.update_bodypose_state(cmode)
		self.update_leg_state(reset, cmode)
		self.update_stability_region()
		self.update_stability_margin()

	def estimate_state(self):
		# Gets called at IMU rate
		self.state_estimator.estimate()

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

			bstate = self.Leg[j].update_joint_state(wXb, qpc[offset+j*3:offset+(j*3)+3], reset, self.Gait.step_stroke)
			self.cstate[j] = self.Leg[j].update_force_state(self.cforce[j*3:(j*3)+3])

			self.Leg[j].P6_world_X_base = self.P6c.world_X_base.copy()

			if (bstate==True and self.Gait.cs[j]==0 and self.support_mode==False):
				# print 'Bound exceed ',j
				# self.suspend = True
				pass
		# self.cstate = [1 if i==True else 0 for i in self.cstate ]
		self.XHc.base_X_COM[:3,3:4] = self.Rbdl.update_CoM(qpc)
		self.XHc.update_world_X_COM()

	def update_bodypose_state(self, cmode):
		""" update imu state """
		## TODO: INCLUDE STATE ESTIMATION

		if (cmode == "fast"):
			## Updates robot state using setpoints
			self.XHc.world_X_base = self.XHd.world_X_base.copy()
			self.V6c.world_X_base = self.V6d.world_X_base.copy()
			self.A6c.world_X_base = self.A6d.world_X_base.copy()
		else:
			# Pull data from state estimator
			pos = np.reshape(self.state_estimator.r, (3,1)) + \
					np.array([self.P6c.world_X_base_offset.item(0),
 								self.P6c.world_X_base_offset.item(1),
 								BODY_HEIGHT]).reshape((3,1))
			angles = np.reshape(self.state_estimator.get_fixed_angles(), (3,1))
			vel = np.reshape(self.state_estimator.v, (3,1))
			angular = np.reshape(self.state_estimator.w, (3,1))

			# self.P6c.world_X_base = np.vstack((pos, angles))
			# self.V6c.world_X_base = np.vstack((vel, angular))
			
			# TODO Sort the logic here
			if (False and self.imu is not None):
				## quaternion to euler transformation
				rpy = np.array(euler_from_quaternion([self.imu.orientation.w, self.imu.orientation.x,
														 self.imu.orientation.y, self.imu.orientation.z], 'sxyz')).reshape(3,1)
				wv3 = np.array([ [self.imu.angular_velocity.x], 	[self.imu.angular_velocity.y], 	  [self.imu.angular_velocity.z] ])
				ca3 = np.array([ [self.imu.linear_acceleration.x], 	[self.imu.linear_acceleration.y], [self.imu.linear_acceleration.z] ])

				## state estimation
				self.P6c.world_X_base = np.vstack((np.zeros((3,1)), rpy))
				## Updates robot state based on measured states
				self.V6c.world_X_base[3:6] = wv3
				self.A6c.world_X_base[0:3] = ca3

			self.XHc.update_base(self.P6c.world_X_base)

	def update_stability_margin(self):
		""" updates the current stability margin """

		valid, sm = self.SM.point_in_convex(self.XHc.world_X_COM[:3,3])
		# valid, sm = self.SM.point_in_polygon(self.XHc.world_X_COM[:3,3])

		if not valid:
			print colored("Convex hull violated %.3f %.3f " %(valid, sm), 'red')
			self.invalid = True
			self.SM.point_in_convex(self.XHc.world_X_COM[:3,3], True)
			# self.SM.point_in_polygon(self.XHc.world_X_COM[:3,3])
		else:
			self.invalid = False

		## Force/Moment balance
		# stack_leg_forces = [self.Leg[j].F6c.world_X_foot[:3] for j in range(6) if self.Gait.cs[j] == 0]
		# stack_leg_normal = [self.Leg[j].snorm for j in range(6) if self.Gait.cs[j] == 0]
		# self.SM.force_moment(self.XHc.world_X_COM[:3,3], stack_leg_forces, stack_world_X_foot, stack_leg_normal)
	
	def update_stability_region(self):
		""" Updates the stability region for a set of fixed contacts - Bretl2008 """

		## Computes region if previous topology is different
		if self.Gait.cs != self.SM.prev_topology:
			## Define Variables ##
			stack_world_X_foot = [np.round(self.Leg[j].XHc.world_X_foot[:3,3],4).tolist() for j in range(6) if self.Gait.cs[j] == 0]
			stack_normals = [[self.Leg[j].snorm.tolist()] for j in range(6) if self.Gait.cs[j] == 0]
			
			self.SM.compute_support_region(stack_world_X_foot, stack_normals)
			# self.SM.compute_support_polygon(stack_world_X_foot)
			self.SM.prev_topology = list(self.Gait.cs)
		else:
			pass

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

	def update_world_desired_frames(self):
		""" Update frames wrt to world """

		self.XHd.update_world_X_base(self.P6d.world_X_base)
		for j in range(0,6):
			self.Leg[j].XHd.update_world_X_coxa(self.XHd.world_X_base)

	def alternate_phase(self, new_phase=None):
		""" change gait to next phase """

		if (new_phase is None):
			self.Gait.change_phase()
		else:
			self.Gait.ps = self.Gait.cs
			self.Gait.cs = new_phase
		# print self.Gait.cs, self.Gait.ps
		# update robot leg phase_change
		for j in range(0,6):
			if (self.Gait.cs[j] == 1):
				self.Leg[j].transfer_phase_change = False

			elif (self.Gait.cs[j] == 0 and self.Gait.cs[j] != self.Gait.ps[j]):
				# Update world_X_foot when phase changes from transfer to support to avoid drifting
				self.Leg[j].XHc.update_world_X_foot(self.XHc.world_X_base)
				self.Leg[j].XHd.world_X_foot = self.Leg[j].XHc.world_X_foot.copy()
			
		self.suspend = False

	def task_X_joint(self, foot_pos=None): # TODO - to be revisited
		""" Convert all leg task space to joint space 		"""
		""" Output: joint space setpoint for all joints		"""

		## Define variables ##
		qt = []
		qp = []
		qv = []
		qa = []
		err_list = [0]*6

		for j in range(0,6):

			# FAULT INDUCED
			if self.Fault.fault_index[j] == True:
				self.Leg[j].XHd.coxa_X_foot = mX(self.Leg[j].XHd.coxa_X_base, v3_X_m(self.Fault.base_X_foot[j]))

			if foot_pos is None:
				err_list[j], qpd, qvd, qad = self.Leg[j].tf_task_X_joint()
			else:
				err_list[j], qpd, qvd, qad = self.Leg[j].tf_task_X_joint(foot_pos[j*3:j*3+3])
			# if j == 4:
			# 	qpd = np.array([0.,  6.12682006e-01,  -2.61484057e+00])

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
				print colored(err_str, 'red')
		
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
			# self.Leg[j].XHd.update_world_X_coxa(self.XHd.world_X_base)
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

	def apply_leg_admittance(self, force_dist):
		""" Applies impedance control to track force for each leg """
		
		feet_pos = []
		for j in range(0,6):
			if self.Gait.cs[j]==0:
				offset = self.Leg[j].apply_admittance_controller(force_dist[j*3:j*3+3], True)
				# fsetpoint = np.array([0.,0.,15.]).reshape((3,1))
				# offset = self.Leg[j].apply_admittance_controller(fsetpoint, True)
			else:
				offset = np.zeros(3)
			new_pos = self.Leg[j].XHd.coxa_X_foot[:3,3] + offset
			feet_pos.append(new_pos)
			# if j==4:
			# 	print 'r: ', np.round(self.Leg[4].XHd.coxa_X_foot[:3,3],4)
			# 	print 'r: ', np.round(offset,4)
			# 	print 'r: ', np.round(new_pos,4)
			# 	print '======================================='
		feet_pos = [item for sublist in feet_pos for item in sublist]
		return self.task_X_joint(feet_pos)
		# return self.task_X_joint()

	def apply_base_admittance(self, desired_force):
		""" Applies base impedance control to track leg force """

		offset = np.zeros(3)

		if self.Fault.status:
			for j in range(0,6):
				if self.Fault.fault_index[j] == True:
					world_df = (self.Leg[j].F6c.world_X_foot[0:3] - desired_force[j*3:j*3+3]).flatten()

			offset[0] = self.impedance_controller_x.evaluate(world_df[0])
			offset[1] = self.impedance_controller_y.evaluate(world_df[1])
			offset[2] = self.impedance_controller_z.evaluate(world_df[2])

		return offset

	def apply_base_pos_ctrl(self, P6e_world_X_base):
		u = []
		for j in range(0,6):
			if self.Gait.cs[j]==0:
				u.append(self.Leg[j].PosCont.update(P6e_world_X_base))
		return u

	def check_contact(self):
		""" Checks for leg contact """

		transfer_count = 0
		contact_index  = [0]*6					# index for legs not in contact
		transfer_total = self.Gait.cs.count(1) 	# no. legs in transfer phase
		for j in range(0,6):
			if self.Gait.cs[j] == 1:
				if self.Leg[j].cstate == True:
					# transfer_index.append(-1)
					contact_index[j] = 1
				else:
					# transfer_index.append(j)
					contact_index[j] = 0

		if transfer_total == transfer_index.count(-1):
			# print 'All swing in contact!'
			return True, transfer_index
		else:
			return False, transfer_index
		

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

			# for j in range(5):
			# 	leg_stance[j][2] = 0.0

		elif (stance_type == "chimney"):
			# base_height = 0.
			# stance_width = STANCE_WIDTH
			# teta_f = stance_offset[0]
			# teta_r = stance_offset[1]

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

	def init_fault_stance(self):

		leg_stance = {}
		for j in range(0,6):
			# Fault leg - use fault configuration
			if self.Fault.fault_index[j] == True:
				coxa_X_foot = mX(self.Leg[j].XHd.coxa_X_base, v3_X_m(self.Fault.base_X_foot[j]))

			# Working leg - update foot position
			else:
				# Propogate base offset to legs
				world_X_foot = self.Leg[j].XHd.world_X_foot.copy()
				world_X_foot[0,3] = self.Leg[j].XHd.world_X_foot[0,3] + self.P6c.world_X_base[0]
				world_X_foot[1,3] = self.Leg[j].XHd.world_X_foot[1,3] + self.P6c.world_X_base[1]
				
				coxa_X_foot = mX(self.Leg[j].XHd.coxa_X_base, self.XHc.base_X_world, world_X_foot)
			leg_stance[j] = coxa_X_foot[:3,3].flatten()
			

		self._initialise(leg_stance)
## ====================================================================================================================================== ##
## ====================================================================================================================================== ##

# robot = RobotState()
