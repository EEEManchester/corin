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
		self.XHc = robot_transforms.HomogeneousTransform()

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
		self.log_sp_pub_   = rospy.Publisher(ROBOT_NS + '/log_sp', Float32MultiArray, queue_size=1)
		self.log_ac_pub_   = rospy.Publisher(ROBOT_NS + '/log_ac', Float32MultiArray, queue_size=1)

		##***************** SUBSCRIBERS ***************##
		self.joint_sub_  = rospy.Subscriber(ROBOT_NS + '/joint_states', JointState, self.joint_state_callback, queue_size=1)
		self.cforce_sub_ = rospy.Subscriber(ROBOT_NS + '/contact_force', Float32MultiArray, self.contact_force_callback, queue_size=1)

		rospy.sleep(0.5)
		print colored('Robot Ready!','green')

	def publish_topics(self, qd, log_sp, log_ac):
		
		if (self.interface == 'gazebo'):
			for n in range(0,3):
				self.joint_pub_[n].publish(data=qd[n])
		
		self.log_sp_pub_.publish(Float32MultiArray(data=log_sp))
		self.log_ac_pub_.publish(Float32MultiArray(data=log_ac))

	def update_states(self):
		""" Update robot states """
		if len(self.qc.position) == 4:
			qpos = self.qc.position[1:4]
			qbase = self.qc.position[0]
		else:
			qpos = self.qc.position
			qbase = 0.

		self.XHc.world_X_base = v3_X_m(np.array([0.03, 0., 0.1+qbase]))
		# self.XHc.world_X_base = self.XHd.world_X_base
		self.XHc.base_X_world = np.linalg.inv(self.XHc.world_X_base)
		self.bstate = self.Leg.update_joint_state(self.XHc.world_X_base, qpos, False, STEP_STROKE)
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
		self.XHd.base_X_world = np.linalg.inv(self.XHd.world_X_base)
		self.Leg.XHd.world_X_foot[:3,3] = np.array([STANCE_WIDTH+0.03, 0., 0.])
		self.Leg.XHc.world_X_foot = self.Leg.XHd.world_X_foot.copy()

		self.P3e_integral = np.zeros(3)
		self.P3e_prev_1   = np.zeros(3)
		self.PI_control = np.zeros(3)
		self.pos = self.Leg.XHd.coxa_X_foot[:3,3].copy()

	def controller_setup(self):
		self.control_loop = 'open'
		self.ctrl_base_tracking = False
		self.ctrl_leg_impedance = True
		self.ctrl_contact_detect = True

	def controller(self):

		self.update_states()

		if self.state_machine == 'unload':

			if self.tload == 1:
				print 'State Machine: Unloading'
				gphase = 0
				self.fmax_lim  = 0
				self.init_flim = self.Leg.get_normal_force('setpoint') if (self.pre_gphase == 1) else F_MAX
				
				
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
					self.PI_control = np.zeros(3)
					self.Leg.reset_impedance_controller()
				print 'State Machine: Motion'
				## Force and gait phase array for Force Distribution
				fmax_lim = F_MAX	# max. force array
				gphase = self.cur_gphase
				self.Leg.Fmax = F_MAX if gphase == 0 else FORCE_THRES
				# print 'gphase: ', self.cur_gphase

			elif (self.s_cnt > self.s_max/2 and self.ctrl_contact_detect):
				# Check if transfer has made contact
				if self.cur_gphase==1 and self.cstate:
					print self.s_cnt, ' cindex ', self.cstate
					self.cur_gphase = 0
					self.Leg.change_phase('support', self.XHc.world_X_base)
					self.cur_wXb = self.XHc.world_X_base.copy()
					# raw_input('cstate true')
			
			## Generate Transfer Trajectory
			if self.cur_gphase == 1 and self.Leg.transfer_phase_change == False:
				print 'Generate transfer trajectory'
				sn1 = np.array([0., 0., 1.])
				sn2 = np.array([0., 0., 1.])

				self.Leg.XH_world_X_base = self.XHc.world_X_base.copy() 
				self.Leg.XHd.base_X_foot[0:3,3] = np.array([STANCE_WIDTH, 0., -BODY_HEIGHT])

				# self.Leg.XHd.coxa_X_foot[0:3,3] = np.array([STANCE_WIDTH, 0., -BODY_HEIGHT])
				# Updates to actual foot position (without Q_COMPENSATION)
				qpos = self.qc.position[1:4] if len(self.qc.position[1:4])==4 else self.qc.position 
				self.Leg.XHc.coxa_X_foot[0:3,3:4] = self.Leg.KDL.leg_FK(qpos)
				
				## Generate transfer spline
				svalid = self.Leg.generate_spline('world', sn1, sn2, 1, False, GAIT_TPHASE, CTR_INTV)
				
				if (svalid is False):
					# set invalid if trajectory unfeasible for leg's kinematic
					self.invalid = True
					print 'Leg Transfer Trajectory Invalid'
				else:
					# set flag that phase has changed
					self.Leg.feedback_state = 1
					self.Leg.transfer_phase_change = True
				# raw_input('cont')

			## Compute task space foot position for all legs
			self.update_phase_transfer()
			self.update_phase_support()
			
			if ( self.s_cnt == self.s_max):
				# Legs in contact, change phase
				if self.cstate or self.control_loop=="open":
					
					# Change to transfer
					if self.next_gait == 0:
						self.next_gait = 1
						self.Leg.transfer_phase_change = False
						self.state_machine = 'motion'#'unload'
						self.s_cnt = 0
						# self.P3e_integral = np.zeros(3)
						# self.PI_control = np.zeros(3)
						# self.Leg.reset_impedance_controller()

					# Change to support
					elif self.next_gait == 1:
						self.next_gait = 0
						if self.cur_gphase != 0:
							self.Leg.change_phase('support', self.XHc.world_X_base)
							self.cur_wXb = self.XHc.world_X_base.copy()
						self.state_machine = 'motion'#'load'
						self.s_cnt = 0
						
				# raw_input('Changing phase')
			self.s_cnt += 1

			## Task to joint space
			err_no, qpd, qvd, qad = self.Leg.tf_task_X_joint()
			
			if (err_no != 0):
				print 'Error Occured, robot invalid! ', err_no
				
			# force_dist = self.compute_foot_force_distribution(self.Robot.P6d.world_X_base, qd.xp, xa_d, wa_d, temp_gphase, fmax_lim)
			# joint_torq = self.Robot.force_to_torque(force_dist)
			
			if self.interface != 'rviz':# and self.control_loop != "open":
				# Leg impedance
				if self.ctrl_leg_impedance and self.cur_gphase==0:
					f_des = np.array([0.,0.,15.]).reshape((3,1))
					
					f_refr = self.Leg.control.pi_control(f_des, self.Leg.F6c.world_X_foot[0:3])
					# offset = self.Leg.apply_impedance_controller(f_refr)
					offset = self.Leg.apply_impedance_controller(f_des)
					pos = self.Leg.XHd.coxa_X_foot[:3,3] + offset
					qpd = self.Leg.KDL.leg_IK(pos)
					self.pos = pos.copy()
				
			## Data logging & publishing
			log_sp = self.set_log([self.Leg.XHd.coxa_X_foot[:3,3].flatten(), qpd])
			log_ac = self.set_log([self.Leg.XHc.coxa_X_foot[:3,3].flatten(), 
									self.Leg.Joint.qpc, self.Leg.F6c.tibia_X_foot[:3]])
			self.publish_topics(qpd, log_sp, log_ac)
			
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
				
	def update_phase_support(self):

		# Determine foot position wrt base & coxa. 
		if (self.cur_gphase == 0):
			base_error = self.XHd.world_X_base[:3,3]-self.XHc.world_X_base[:3,3]
			# Integral controller
			# if np.linalg.norm(base_error) > 0.001:
			# 	# self.P3e_integral += base_error*CTR_INTV*KI_P_BASE 	# Backward-euler
			# 	self.P3e_integral += (CTR_INTV*KI_P_BASE/2)*(base_error+self.P3e_prev_1) 	# Trapezoidal
			# else:
			# 	self.P3e_integral = np.zeros(3)
			# comp_world_X_base = v3_X_m(self.XHc.world_X_base[:3,3] + self.P3e_integral)			

			# PI controller
			self.PI_control += KP_P_BASE*(base_error-self.P3e_prev_1) + (CTR_INTV*KI_P_BASE/2)*(base_error+self.P3e_prev_1)
			self.P3e_prev_1 = base_error.copy()

			comp_world_X_base = v3_X_m(self.XHc.world_X_base[:3,3] + self.PI_control)
			comp_base_X_world = np.linalg.inv(comp_world_X_base)

			# cout3p(self.XHd.world_X_base)
			# cout3p(self.XHc.world_X_base)
			# # cout3v(self.P3e_integral)
			# cout('==========================')
			# self.ctrl_base_tracking = True

			## Position-control
			# Closed-loop control on bodypose. Move by sum of desired pose and error
			if self.ctrl_base_tracking:
				self.Leg.XHd.base_X_foot = mX(comp_base_X_world, self.Leg.XHc.world_X_foot)
			else:
				# self.Leg.XHd.base_X_foot = mX(self.XHc.base_X_world, self.Leg.XHc.world_X_foot)
				self.Leg.XHd.base_X_foot = mX(np.linalg.inv(self.cur_wXb), self.Leg.XHc.world_X_foot)

			self.Leg.XHd.coxa_X_foot = mX(self.Leg.XHd.coxa_X_base, self.Leg.XHd.base_X_foot)
		# cout3p(self.Leg.XHd.coxa_X_foot)
		# cout3p(self.Leg.XHc.coxa_X_foot)
		# print '==================================='
	def update_phase_transfer(self,delta_d=None):

		## Transfer phase
		if (self.cur_gphase == 1 and self.Leg.feedback_state == 1):
			## Update leg position from generated leg spline
			if (self.Leg.update_from_spline() is False):
				## Stops body from moving until transfer phase completes
				print 'Suspending in update_phase_transfer()'
				self.Robot.suspend = True
			
		elif (self.cur_gphase == 1 and self.Leg.feedback_state == 2 and delta_d is not None):
			self.Leg.XHd.coxa_X_foot[:3,3] -= delta_d

	def set_log(self, log_list):
		log = []
		for i in log_list:
			log.append(i.flatten().tolist())
		return [val for sublist in log for val in sublist]

if __name__ == "__main__":

	print colored('Initialising Robot ....','yellow')
	manager = LegController()
	
	## Run continuously
	while not rospy.is_shutdown():
		manager.controller()
		manager.rate.sleep()
	
