#!/usr/bin/env python

""" Main file for Corin """
__version__ = '1.0'
__author__  = 'Wei Cheah'

import sys
import rospy
import numpy as np
from termcolor import colored, cprint

from corin_control import *

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

class LegController:
	def __init__(self):
		rospy.init_node('LegController') 		# Initialises node
		self.rate 	  = rospy.Rate(CTR_RATE)	# Controller rate

		self.Leg = leg_class.LegClass(0)
		self.interface = 'gazebo'
		self.qc = None
		self.cf = None
		self.XHd = robot_transforms.HomogeneousTransform()

		self.initialize_topics()
		self.initialise_controller_variables()
		self.controller_setup()

	def joint_state_callback(self, msg):
		""" robot joint state callback """
		self.qc = msg

	def contact_force_callback(self, msg):
		""" Leg contact forces """
		self.cf = np.array(msg.data) 

	def initialize_topics(self):

		##***************** PUBLISHERS ***************##
		self.joint_pub_ = {}
		self.joint_pub_[0] = rospy.Publisher(ROBOT_NS + '/leg_q1_joint/command', Float64, queue_size=1)
		self.joint_pub_[1] = rospy.Publisher(ROBOT_NS + '/leg_q2_joint/command', Float64, queue_size=1)
		self.joint_pub_[2] = rospy.Publisher(ROBOT_NS + '/leg_q3_joint/command', Float64, queue_size=1)

		##***************** SUBSCRIBERS ***************##
		self.joint_sub_  = rospy.Subscriber(ROBOT_NS + '/joint_states', JointState, self.joint_state_callback, queue_size=1)
		self.cforce_sub_ = rospy.Subscriber(ROBOT_NS + '/contact_force', Float32MultiArray, self.contact_force_callback, queue_size=1)

		rospy.sleep(0.5)
		print colored('Robot Ready!','green')

	def publish_topics(self, qd):
		
		if (self.interface == 'gazebo'):
			for n in range(0,3):
				self.joint_pub_[n].publish(data=qd[n])

	def update_states(self):
		""" Update robot states """
		wXb = v3_X_m(np.array([0.03, 0., 0.1+self.qc.position[0]]))
		self.bstate = self.Leg.update_joint_state(wXb, self.qc.position[1:4], False, STEP_STROKE)
		self.cstate = self.Leg.update_force_state(self.cf)

	def initialise_controller_variables(self):
		# State machine loading/unloading timing parameters
		self.state_machine = 'hold'
		self.tload = 1						# loading counter
		self.iload = int(LOAD_T/CTR_INTV) 	# no. of intervals for loading
		self.hload = int(1./CTR_INTV)		# no. of intervals for initial holding
		# Force Distribution variables
		self.fmax_lim  = F_MAX	# maximum force array
		self.init_flim = F_MAX	# current force for linear decrease in unloading
		self.cur_gphase = 0
		self.pre_gphase = 0
		# Support phase counter
		self.s_max = int(GAIT_TPHASE/CTR_INTV)
		self.s_cnt = 1
		# Counts per gait phase interval
		# iahead = int(GAIT_TPHASE/CTR_INTV)
		self.invalid = False
		self.next_gait = 1

		self.XHd.world_X_base[:3,3] = np.array([0.03, 0., 0.1])

	def controller_setup(self):
		self.control_loop = 'open'
		self.ctrl_base_tracking = False
		self.ctrl_leg_impedance = False

	def controller(self):

		self.update_states()

		if self.state_machine == 'unload':

			if self.tload == 1:
				print 'State Machine: Unloading'
				gphase = 0
				self.fmax_lim  = 0
				self.init_flim = self.Leg.get_normal_force('setpoint') if (self.pre_gphase == 1) else F_MAX
				# print self.init_flim
				
			# reduce max force for legs that will be in transfer phase
			fmax = self.init_flim+1. - self.tload*(self.init_flim+1.)/float(self.iload) + 0.001
			self.fmax_lim = fmax if (self.pre_gphase == 1) else F_MAX
			self.tload += 1
			
			# self.update_phase_support(P6e_world_X_base, V6e_world_X_base)
			
			if self.tload == self.iload+1 or self.iload == 0:
				self.tload = 1
				self.s_cnt = 1
				self.state_machine = 'motion'
				self.cur_gphase = 1
				
		elif self.state_machine == 'load':

			if self.tload == 1:
				print 'State Machine: Loading'
				gphase = 0 		# all legs in support phase at start of unloading
				self.fmax_lim = 0	# max. force array

			# reduce max force for legs that will be in transfer phase
			# fmax = F_THRES + self.tload*(F_MAX-F_THRES)/float(self.iload)
			fmax = self.tload*F_MAX/float(self.iload)
			
			# gait phase updated prior to this state, so use previous gait phase
			self.fmax_lim = fmax + self.Leg.Fmax if (self.pre_gphase == 1) else F_MAX	
			self.tload += 1
			# self.update_phase_support(P6e_world_X_base, V6e_world_X_base)

			if self.tload == self.iload+1:
				self.tload = 1
				self.state_machine = 'unload'
				# Checks if robot meant to be in support mode 
		
		elif self.state_machine == 'motion':

			if self.s_cnt == 1:
				if self.next_gait == 0:
					self.cur_gphase = 0
				elif self.next_gait == 1:
					self.cur_gphase = 1
				print 'State Machine: Motion'
				## Force and gait phase array for Force Distribution
				fmax_lim = F_MAX	# max. force array
				gphase = self.cur_gphase
				self.Leg.Fmax = F_MAX if gphase == 0 else FORCE_THRES
				print 'gphase: ', self.cur_gphase
			# elif (self.s_cnt > s_max/2 and self.ctrl_contact_detect):
			# 	# Check if transfer has made contact
			# 	cearly = map(lambda x,y: x and y, self.Robot.cstate, self.Robot.Gait.cs)
			# 	cindex = [z for z, y in enumerate(cearly) if y == 1]
			# 	# Change legs in transfer phase with contact to support
			# 	if cindex:
			# 		print i, ' cindex ', cindex, self.Robot.cstate
			# 		for j in cindex:
			# 			self.cur_gphase = 0
			# 			self.Leg.change_phase('support', self.Robot.XHc.world_X_base)
			# 			self.Leg.snorm = self.GridMap.get_cell('norm', self.Leg.XHc.world_X_foot[0:3,3])

			# 	# Interpolate Fmax for legs in contact phase
			# 	fearly = map(lambda x,y: xor(bool(x), bool(y)), gphase, self.Robot.Gait.cs)
			# 	findex = [z for z, y in enumerate(fearly) if y == True]
			# 	# print 'findex ', findex, self.Robot.Gait.cs, gphase
			# 	if findex:
			# 		for j in findex:
			# 			self.Leg.Fmax += F_INC
			# 			# print self.Leg.Fmax
			# 		fmax_lim = []
			# 		for j in range(6):
			# 			if self.cur_gphase == 0:
			# 				fmax_lim.append(self.Leg.Fmax)
			# 		gphase = list(self.Robot.Gait.cs)
			# 		print i, 'findex ', findex, gphase, fmax_lim

			## Generate Transfer Trajectory
			if self.cur_gphase == 1 and self.Leg.transfer_phase_change == False:
				print 'Generate transfer trajectory'
				sn1 = np.array([0., 0., 1.])
				sn2 = np.array([0., 0., 1.])

				self.Leg.XHd.coxa_X_foot[0:3,3] = np.array([STANCE_WIDTH, 0., -BODY_HEIGHT])

				## Generate transfer spline
				svalid = self.Leg.generate_spline('leg', sn1, sn2, 1, False, GAIT_TPHASE, CTR_INTV)

				if (svalid is False):
					# set invalid if trajectory unfeasible for leg's kinematic
					self.invalid = True
					print 'Leg Transfer Trajectory Invalid'
				else:
					# set flag that phase has changed
					self.Leg.feedback_state = 1
					self.Leg.transfer_phase_change = True

			## Compute task space foot position for all legs
			self.update_phase_transfer()
			self.update_phase_support()
			
			if ( self.s_cnt == self.s_max):
				self.state_machine = 'load'
				self.s_cnt = 1
				if self.next_gait == 0:
					self.next_gait = 1
					self.Leg.transfer_phase_change = False
				elif self.next_gait == 1:
					self.next_gait = 0
				# # Legs in contact, change phase
				# if all(self.Robot.cstate) or self.interface == "rviz" or self.control_loop=="open":
				# 	self.Robot.suspend = False
				# 	for j in range(0,6):
				# 		if (self.Robot.Gait.cs[j] == 1):
				# 			self.Robot.Leg[j].change_phase('support', self.Robot.XHc.world_X_base)
				# 			self.Robot.Leg[j].snorm = self.GridMap.get_cell('norm', self.Robot.Leg[j].XHc.world_X_foot[0:3,3])
				# 	state_machine = 'load'
				# 	print i, ' Phase Timeout, Loading ...'
				# 	# print fmax_lim, gphase
				# 	# raw_input('next')
				# 	self.s_cnt = 1
				# # Extend leg swing motion, suspend base
				# else:
				# 	# print 'Not all contacts made, suspend robot: ', self.Robot.cstate, self.Robot.Gait.cs
				# 	self.Robot.suspend = True
				# 	## Foot displacement in surface normal direction
				# 	for j in [z for z, y in enumerate(self.Robot.cstate) if y == False]:
				# 		sn1 = self.GridMap.get_cell('norm', self.Robot.Leg[j].XHc.world_X_foot[0:3,3])
				# 		# Incremental displacement in world frame - support legs 
				# 		self.Robot.Leg[j].XHc.world_X_foot[:3,3] -= sn1*np.array([D_MOVE,D_MOVE,D_MOVE])
				# 		# Transform displacement to leg frame - transfer legs
				# 		self.Robot.Leg[j].XHc.update_world_X_coxa(self.Robot.Leg[j].XH_world_X_base)
				# 		delta_d = mX(self.Robot.Leg[j].XHc.coxa_X_world[:3,:3], sn1*np.array([D_MOVE,D_MOVE,D_MOVE]))
				# 		# Update new foot position
				# 		self.update_phase_transfer(delta_d)
				# 		self.update_phase_support(P6e_world_X_base, V6e_world_X_base)
				# 		# print j, np.round(self.Robot.Leg[j].XHd.coxa_X_foot[:3,3], 4)
				# 		# print j, np.round(self.Robot.Leg[j].XHc.world_X_foot[:3,3],4)
			self.s_cnt += 1

			# print state_machine, fmax_lim, self.Robot.Gait.cs, self.Robot.Gait.ps
			## Task to joint space
			err_no, qpd, qvd, qad = self.Leg.tf_task_X_joint()
			
			if (err_no != 0):
				print 'Error Occured, robot invalid! ', err_no
				
			# force_dist = self.compute_foot_force_distribution(self.Robot.P6d.world_X_base, qd.xp, xa_d, wa_d, temp_gphase, fmax_lim)
			# joint_torq = self.Robot.force_to_torque(force_dist)
			# print i, np.round(force_dist[17],3), np.round(fmax_lim[-1],3)
			# print temp_gphase, fmax_lim
			if self.interface != 'rviz' and self.control_loop != "open":
				# Leg impedance
				if self.ctrl_leg_impedance:
					self.apply_impedance_control(force_dist)

				## Recompute task to joint space
				err_no, qpd, qvd, qad = self.Leg.tf_task_X_joint()

			if (err_no != 0):
				print 'Error Occured, robot invalid! ', err_no

			## Data logging & publishing
			# qlog = self.set_log_setpoint(v3cp, v3cv, xa_d, v3wp, v3wv, wa_d, qd, joint_torq, force_dist)
			self.publish_topics(qpd)

		elif self.state_machine == 'hold':
			""" Holds position - for controller to settle or pause motion """
			
			gphase = 0
			self.fmax_lim = F_MAX

			if self.tload == 1:
				print 'State Machine: Holding'
				
			self.tload += 1

			if self.tload == self.hload+1:
				self.state_machine = 'unload'
				self.tload = 1
				
			# self.update_phase_support(P6e_world_X_base, V6e_world_X_base)
	def update_phase_support(self):

		# Determine foot position wrt base & coxa. 
		if (self.cur_gphase == 0):
			## Position-control
			# Closed-loop control on bodypose. Move by sum of desired pose and error
			if self.ctrl_base_tracking:
				self.Leg.XHd.base_X_foot = mX(comp_base_X_world, self.Leg.XHc.world_X_foot)
			else:
				self.Leg.XHd.base_X_foot = mX(self.XHd.base_X_world, self.Leg.XHc.world_X_foot)

			## Velocity-control - Focchi2018 eq(3) TODO: check robustness
			# vel_base_X_foot = -V6e_world_X_base[0:3].flatten() - np.cross(V6e_world_X_base[3:6].flatten(), 
			# 					(self.Leg.XHd.base_X_foot[0:3,3:4] - self.Robot.P6d.world_X_base[0:3]).flatten())
			# base_X_foot = self.Leg.XHd.base_X_foot[0:3,3] + vel_base_X_foot

			self.Leg.XHd.coxa_X_foot = mX(self.Leg.XHd.coxa_X_base, self.Leg.XHd.base_X_foot)

	def update_phase_transfer(self,delta_d=None):

		## Transfer phase
		if (self.cur_gphase == 1 and self.Leg.feedback_state == 1):
			## Update leg position from generated leg spline
			if (self.Leg.update_from_spline() is False):
				## Stops body from moving until transfer phase completes
				print 'Suspending in update_phase_transfer()'
				self.Robot.suspend = True
			# else:
			# 	print np.round(self.Leg.XHd.coxa_X_foot[:3,3].flatten(),3)

		elif (self.cur_gphase == 1 and self.Leg.feedback_state == 2 and delta_d is not None):
			self.Leg.XHd.coxa_X_foot[:3,3] -= delta_d

if __name__ == "__main__":

	print colored('Initialising Robot ....','yellow')
	manager = LegController()
	
	## Run continuously
	while not rospy.is_shutdown():
		manager.controller()
		manager.rate.sleep()
	
