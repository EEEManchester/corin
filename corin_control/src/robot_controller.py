#!/usr/bin/env python

""" Main controller for the robot """
__version__ = '1.0'
__author__  = 'Wei Cheah'

import sys; sys.dont_write_bytecode = True
from itertools import cycle

from robot_manager import CorinManager
from corin_control import *			# library modules to include
from operator import xor

import rospy
import time

class RobotController(CorinManager):
	def __init__(self, initialise=False):
		CorinManager.__init__(self, initialise)
		
		# Enable/disable controllers
		self.ctrl_base_tracking  = True 	# base tracking controller
		self.ctrl_base_admittance = False 	# base impedance controller - fault
		self.ctrl_leg_admittance = False 	# leg impedance controller
		self.ctrl_contact_detect = True 		# switch gait for early contact detection

	def main_controller(self, motion_plan=None):

		## Variables ##
		cob_X_desired = np.zeros((3,1)) 	# track cob linear location
		cob_W_desired = np.zeros((3,1)) 	# track cob angular location
		wXbase_offset = self.Robot.P6c.world_X_base.copy()

		## Variable mapping from motion plan
		if motion_plan is not None:
			state_machine = 'hold' 			# Motion state machine: unload, motion, load

			base_path = motion_plan.qb
			world_X_base = motion_plan.qbp
			gait_phase 	 = motion_plan.gait_phase
			w_base_X_NRP = motion_plan.f_world_base_X_NRP
			world_X_footholds = motion_plan.f_world_X_foot
			base_X_footholds  = motion_plan.f_base_X_foot
			
			## Publish motion plan
			self.Visualizer.show_motion_plan(self.Robot.P6c.world_X_base, motion_plan.qbp, motion_plan.f_world_X_foot)
			if (self.interface == 'rviz'):
				self.publish_topics(JointTrajectoryPoints(18,([],self.Robot.qc.position,[],[])))

			# Plot.plot_2d(base_path.X.t, base_path.X.xp)
			# Plot.plot_2d(base_path.W.t, base_path.W.xp)
			# print world_X_footholds[0].xp
			## Remove initial set of footholds (visualization purpose)
			for j in range(0,6):
				try:
					world_X_footholds[j].xp.pop(0)
					base_X_footholds[j].xp.pop(0)
					w_base_X_NRP[j].xp.pop(0)
				except:
					pass

			## Use gait phase from planner if exists
			try:
				gait_stack = cycle(gait_phase)
				self.Robot.Gait.cs = next(gait_stack)
			except:
				print 'No defined gait phases, using periodic'

			path_length = len(base_path.X.t)
		else:
			state_machine = 'hold' 			# Motion state machine: unload, motion, load
			path_length = 5000		
			world_X_footholds = []
			base_X_footholds = []
			w_base_X_NRP = []

		## User input
		print '==============================================='
		print 'Execute Path in', self.interface, '?',
		if (self.interface == 'gazebo' or self.interface == 'robotis'):
			raw_input('')
			self.ui_state = 'play'#'hold'
		else:
			raw_input('')
			self.ui_state = 'play'#'hold'

		# State machine loading/unloading timing parameters
		tload = 1						# loading counter
		iload = int(LOAD_T/CTR_INTV) 	# no. of intervals for loading
		hload = int(0.5/CTR_INTV)		# no. of intervals for initial holding
		# Force Distribution variables
		fmax_lim  = [F_MAX]*6			# maximum force array
		init_flim = [F_MAX]*6			# current force for linear decrease in unloading
		gphase = [0]*6 					# gait phase array
		# Base impedance
		offset_z = 0.0
		offset_base = np.zeros(3) 		# offset for base impedance
		# Support phase counter
		s_max = int(GAIT_TPHASE/CTR_INTV)
		s_cnt = 1
		# Counts per gait phase interval
		iahead = int(GAIT_TPHASE/CTR_INTV)

		## Cycle through trajectory points until complete
		i = 1 		# skip first point since spline has zero initial differential conditions
		while (i != path_length and not rospy.is_shutdown()):
			# print 'counting: ', i, len(base_path.X.t), s_cnt, self.Robot.suspend
			# print 'wXb: ', np.round(self.Robot.P6c.world_X_base.flatten(),3)
			# Check if controller should be suspended
			self.suspend_controller()
			
			# Suppress trajectory counter as body support suspended
			# self.Robot.suspend = False # manual overwrite for selected motions
			if (self.Robot.suspend == True or state_machine == 'hold'):
				i -= 1 		# base trajectory counter
				s_cnt -= 1	# gait phase trajectory counter

			# Update state for RViZ and open loop control prior to setpoint update
			if (self.interface == 'rviz'):
				self.Robot.P6c.world_X_base = self.Robot.P6d.world_X_base.copy()

			## ====================================================================== ##
			## 		Setpoint update 	##
			## ======================== ##
			# Variable mapping to R^(3x1) for base linear and angular position, velocity, acceleration

			if motion_plan is not None:
				# GRID PLAN: base trajectory in world frame
				# v3cp = wXbase_offset[0:3] + base_path.X.xp[i].reshape(3,1) 		\
				# 				+ offset_base.reshape((3,1))
								# + np.array([0., 0., offset_z]).reshape((3,1))
				## TEMP: HOLD IN PLACE
				v3cp = wXbase_offset[0:3] + offset_base.reshape((3,1))
				# v3cp = base_path.X.xp[i].reshape(3,1); 	# CORNERING: base trajectory relative to world frame
				v3cv = base_path.X.xv[i].reshape(3,1);
				v3ca = base_path.X.xa[i].reshape(3,1);

				v3wp = wXbase_offset[3:6] + base_path.W.xp[i].reshape(3,1)
				# v3wp = base_path.W.xp[i].reshape(3,1);	# CORNERING: base trajectory relative to world frame
				v3wv = base_path.W.xv[i].reshape(3,1);
				v3wa = base_path.W.xa[i].reshape(3,1);
			else:
				v3cp = wXbase_offset[0:3] + offset_base.reshape((3,1))#np.array([0., 0., offset_z]).reshape((3,1))
				# v3cp = wXbase_offset[0:3]
				v3cv = np.zeros((3,1))
				v3ca = np.zeros((3,1))

				v3wp = wXbase_offset[3:6]
				v3wv = np.zeros((3,1))
				v3wa = np.zeros((3,1))
			
			# Update robot's desired position, velocity & acceleration to next point on spline
			self.Robot.P6d.world_X_base = np.vstack((v3cp,v3wp))
			self.Robot.V6d.world_X_base = np.vstack((v3cv,v3wv)) 	# not used atm
			self.Robot.A6d.world_X_base = np.vstack((v3ca,v3wa)) 	# not used atm
			self.Robot.update_world_desired_frames()

			# Tracking desired translations
			if (self.Robot.suspend != True):
				cob_X_desired += v3cv*CTR_INTV
				cob_W_desired += v3wv*CTR_INTV

			## ====================================================================== ##
			## 		Robot state update 	##
			## ======================== ##
			
			self.Robot.update_state(control_mode=self.control_rate)
			
			# Error 
			P6e_world_X_base = self.Robot.P6d.world_X_base - self.Robot.P6c.world_X_base
			V6e_world_X_base = self.Robot.V6d.world_X_base - self.Robot.V6c.world_X_base
			
			## ====================================================================== ##
			## State Machine: Motion Execution ##
			## =============================== ##

			# print state_machine, tload, iload+1, self.Robot.Gait.cs
			if state_machine == 'unload' and not self.Robot.support_mode:

				if tload == 1:
					state_machine == 'motion'
					print 'State Machine: Unloading'
					self.Robot.Gait.support_mode()
					gphase = [0]*6 		# all legs in support phase at start of unloading
					fmax_lim = [0]*6	# max. force array
					for j in range(0,6):
						# init_flim[j] = self.Robot.Leg[j].F6c.world_X_foot[2] if (self.Robot.Gait.ps[j] == 1) else F_MAX
						init_flim[j] = self.Robot.Leg[j].get_normal_force('setpoint') if (self.Robot.Gait.ps[j] == 1) else F_MAX
					# print init_flim
					
				# reduce max force for legs that will be in transfer phase
				for j in range(0,6):
					fmax = init_flim[j]+1. - tload*(init_flim[j]+1.)/float(iload) + 0.001
					fmax_lim[j] = fmax if (self.Robot.Gait.ps[j] == 1) else F_MAX
				tload += 1
				
				self.update_phase_support(P6e_world_X_base, V6e_world_X_base)
				
				if tload == iload+1 or iload == 0:
					tload = 1
					s_cnt = 1
					state_machine = 'motion'
					self.Robot.Gait.walk_mode()
					
			elif state_machine == 'load' and not self.Robot.support_mode:

				if tload == 1:
					state_machine == 'motion'
					print 'State Machine: Loading'
					prev_phase = list(self.Robot.Gait.ps)
					self.Robot.Gait.support_mode()
					gphase = [0]*6 		# all legs in support phase at start of unloading
					fmax_lim = [0]*6	# max. force array

				# reduce max force for legs that will be in transfer phase
				# fmax = F_THRES + tload*(F_MAX-F_THRES)/float(iload)
				fmax = tload*F_MAX/float(iload)
				for j in range(0,6):
					# gait phase updated prior to this state, so use previous gait phase
					fmax_lim[j] = fmax + self.Robot.Leg[j].Fmax if (prev_phase[j] == 1) else F_MAX	
				tload += 1
				self.update_phase_support(P6e_world_X_base, V6e_world_X_base)

				if tload == iload+1:
					tload = 1
					state_machine = 'unload'
					# Checks if robot meant to be in support mode 
					if prev_phase == [0]*6:
						self.Robot.Gait.support_mode()
					else:
						# alternate gait phase as robot meant to walk
						try:
							self.Robot.alternate_phase(next(gait_stack))
						except:
							self.Robot.alternate_phase()
					self.Robot.Gait.walk_mode()
				
			elif state_machine ==  'motion':
				
				## Halfway through TIMEOUT - check swing leg contact state 
				if s_cnt == 1:
					print 'State Machine: Motion'
					## Force and gait phase array for Force Distribution
					fmax_lim = [F_MAX]*gphase.count(0)	# max. force array
					gphase = list(self.Robot.Gait.cs)
					for j in range(6):
						self.Robot.Leg[j].Fmax = F_MAX if gphase[j] == 0 else FORCE_THRES
						if self.Robot.Gait.cs[j] == 1:
							self.Robot.Leg[j].reset_impedance_controller()

				elif (s_cnt > s_max/2 and self.ctrl_contact_detect):
					# Check if transfer has made contact
					cearly = map(lambda x,y: x and y, self.Robot.cstate, self.Robot.Gait.cs)
					cindex = [z for z, y in enumerate(cearly) if y == 1]
					# Change legs in transfer phase with contact to support
					if cindex:
						print i, ' cindex ', cindex, self.Robot.cstate
						for j in cindex:
							self.Robot.Gait.cs[j] = 0
							self.Robot.Leg[j].change_phase('support', self.Robot.XHc.world_X_base)
							self.Robot.Leg[j].snorm = self.GridMap.get_cell('norm', self.Robot.Leg[j].XHc.world_X_foot[0:3,3])
							# if j==4:
							# self.Robot.Base_Ctrl.reset()
							self.Robot.Leg[j].PosCont.reset()
							raw_input('cont')

					# Interpolate Fmax for legs in contact phase
					fearly = map(lambda x,y: xor(bool(x), bool(y)), gphase, self.Robot.Gait.cs)
					findex = [z for z, y in enumerate(fearly) if y == True]
					# print 'findex ', findex, self.Robot.Gait.cs, gphase
					if findex:
						for j in findex:
							self.Robot.Leg[j].Fmax += F_INC
							# print self.Robot.Leg[j].Fmax
						fmax_lim = []
						for j in range(6):
							if self.Robot.Gait.cs[j] == 0:
								fmax_lim.append(self.Robot.Leg[j].Fmax)
						gphase = list(self.Robot.Gait.cs)
						print i, 'findex ', findex, gphase, fmax_lim
				
				## Generate trajectory for legs in transfer phase
				for j in range (0, self.Robot.active_legs):
					
					# Transfer phase - generate trajectory
					if (self.Robot.Gait.cs[j] == 1 and 
						self.Robot.Leg[j].transfer_phase_change == False and
						not self.Robot.Fault.fault_index[self.Robot.Gait.cs.index(1)]):
						# Set leg to execution mode
						self.Robot.Leg[j].feedback_state = 1 	
						# Use planned foothold if motion plan exists
						if motion_plan is not None:
							try:
								self.Robot.Leg[j].XHd.world_X_foot = v3_X_m(world_X_footholds[j].xp.pop(0))
							except:
								self.Robot.Leg[j].XHd.base_X_foot = self.Robot.Leg[j].XHd.base_X_AEP
								# self.Robot.Leg[j].XHd.base_X_foot = self.Robot.Leg[j].XHd.base_X_NRP

							try:
								self.Robot.Leg[j].XHd.base_X_foot  = v3_X_m(base_X_footholds[j].xp.pop(0))
								self.Robot.Leg[j].XHd.world_base_X_NRP = v3_X_m(w_base_X_NRP[j].xp.pop(0))
								self.Robot.Leg[j].XHc.world_base_X_NRP = self.Robot.Leg[j].XHd.world_base_X_NRP.copy()
								## Set bodypose in leg class
								self.Robot.Leg[j].XH_world_X_base = self.Robot.XHd.world_X_base.copy()

							except IndexError:
								# Get bodypose at end of gait phase, or end of trajectory
								try:
									x_ahead = base_path.X.xp[i+iahead].reshape(3,1) + wXbase_offset[0:3]
									w_ahead = base_path.W.xp[i+iahead].reshape(3,1) + wXbase_offset[3:6]
								except IndexError:
									x_ahead = base_path.X.xp[-1].reshape(3,1) + wXbase_offset[0:3]
									w_ahead = base_path.W.xp[-1].reshape(3,1) + wXbase_offset[3:6]
								
								self.Robot.Leg[j].XH_world_X_base = vec6d_to_se3(np.array([x_ahead, w_ahead]).reshape((6,1)))
								self.Robot.Leg[j].XHd.base_X_foot = mX(np.linalg.inv(self.Robot.Leg[j].XH_world_X_base),
																		self.Robot.Leg[j].XHd.world_X_foot)
								## TEMP!
								self.Robot.Leg[j].XHd.base_X_foot = self.Robot.Leg[j].XHd.base_X_AEP
						## TEMP: stamping
						self.Robot.Leg[j].XHd.base_X_foot = self.Robot.Leg[j].XHd.base_X_NRP

						## Get surface normal at current and desired footholds
						try:
							sn1 = self.GridMap.get_cell('norm', self.Robot.Leg[j].XHc.world_X_foot[0:3,3])
							sn2 = self.GridMap.get_cell('norm', self.Robot.Leg[j].XHd.world_X_foot[0:3,3])
						except:
							sn1 = np.array([0., 0., 1.])
							sn2 = np.array([0., 0., 1.])
							print 'norm error'
						# if j < 3:
						# 	print j, '\t', np.round(self.Robot.Leg[j].XHc.world_X_foot[0:3,3],3)
						# 	print j, '\t', np.round(self.Robot.Leg[j].XHd.world_X_foot[0:3,3],3)
						# 	print j, '\t', sn1, sn2
						## Updates to actual foot position (without Q_COMPENSATION)
						self.Robot.Leg[j].XHc.coxa_X_foot[0:3,3:4] = self.Robot.Leg[j].KDL.leg_FK(self.Robot.Leg[j].Joint.qpc)
						## Generate transfer spline
						svalid = self.Robot.Leg[j].generate_spline('world', sn1, sn2, 1, False, GAIT_TPHASE, CTR_INTV)

						## Update NRP
						self.Robot.Leg[j].XHd.base_X_NRP[:3,3] = mX(self.Robot.XHd.base_X_world[:3,:3],
																	self.Robot.Leg[j].XHc.world_base_X_NRP[:3,3])
						self.Robot.Leg[j].XHd.world_base_X_AEP[:3,3] = mX(self.Robot.XHd.world_X_base[:3,:3],
																			self.Robot.Leg[j].XHd.base_X_foot[:3,3])

						## CORNERING hack: insert here <<

						if (svalid is False):
							# set invalid if trajectory unfeasible for leg's kinematic
							self.Robot.invalid = True
							print 'Leg Transfer Trajectory Invalid'
						else:
							# set flag that phase has changed
							self.Robot.Leg[j].transfer_phase_change = True

				## Compute task space foot position for all legs
				self.update_phase_transfer()
				self.update_phase_support(P6e_world_X_base, V6e_world_X_base)
				
				# print i, s_cnt, s_max
				## Gait phase TIMEOUT
				if ( s_cnt == s_max  and not self.Robot.support_mode):
					# Legs in contact, change phase
					if all(self.Robot.cstate) or self.interface == "rviz" or self.control_loop=="open":
						self.Robot.suspend = False
						for j in range(0,6):
							if (self.Robot.Gait.cs[j] == 1):
								self.Robot.Leg[j].change_phase('support', self.Robot.XHc.world_X_base)
								self.Robot.Leg[j].snorm = self.GridMap.get_cell('norm', self.Robot.Leg[j].XHc.world_X_foot[0:3,3])
								self.Robot.Leg[j].PosCont.reset()
						state_machine = 'load'
						print i, ' Phase Timeout, Loading ...'
						# self.Robot.Base_Ctrl.reset()
						# print fmax_lim, gphase
						# raw_input('next')
						s_cnt = 1
					# Extend leg swing motion, suspend base
					else:
						# print 'Not all contacts made, suspend robot: ', self.Robot.cstate, self.Robot.Gait.cs
						self.Robot.suspend = True
						## Foot displacement in surface normal direction
						for j in [z for z, y in enumerate(self.Robot.cstate) if y == False]:
							sn1 = self.GridMap.get_cell('norm', self.Robot.Leg[j].XHc.world_X_foot[0:3,3])
							# Incremental displacement in world frame - support legs 
							self.Robot.Leg[j].XHc.world_X_foot[:3,3] -= sn1*np.array([D_MOVE,D_MOVE,D_MOVE])
							# Transform displacement to leg frame - transfer legs
							self.Robot.Leg[j].XHc.update_world_X_coxa(self.Robot.Leg[j].XH_world_X_base)
							delta_d = mX(self.Robot.Leg[j].XHc.coxa_X_world[:3,:3], sn1*np.array([D_MOVE,D_MOVE,D_MOVE]))
							# Update new foot position
							self.update_phase_transfer(delta_d)
							self.update_phase_support(P6e_world_X_base, V6e_world_X_base)
							# print j, np.round(self.Robot.Leg[j].XHd.coxa_X_foot[:3,3], 4)
							# print j, np.round(self.Robot.Leg[j].XHc.world_X_foot[:3,3],4)
				i += 1
				s_cnt += 1

			elif state_machine == 'hold':
				""" Holds position - for controller to settle or pause motion """
				# print 'Holding State'
				gphase = [0]*6
				fmax_lim = [F_MAX]*gphase.count(0)	# max. force array

				if tload == 1:
					print 'State Machine: Holding'
					self.Robot.Gait.support_mode()
				
				tload += 1

				if tload == hload+1:# and motion_plan:
					if self.Robot.support_mode:
						state_machine = 'motion'
						print 'Support mode: Motion'
					else:
						state_machine = 'unload'
						tload = 1
						self.Robot.Gait.walk_mode()
				self.update_phase_support(P6e_world_X_base, V6e_world_X_base)
				i += 1

			# print state_machine, fmax_lim, self.Robot.Gait.cs, self.Robot.Gait.ps
			## Task to joint space
			qd, tXj_error = self.Robot.task_X_joint()

			if (self.Robot.invalid == True):
				print 'Error Occured, robot invalid! ', tXj_error
				break

			## Force Distribution for all legs
			# if self.control_loop == 'close':
				# Virtual force-controlled closed-loop control - Focchi2017 eq(3,4)
			# xa_d = mX(np.diag(KPcom), P6e_world_X_base[0:3]) + mX(np.diag(KDcom), V6e_world_X_base[0:3])
			# wa_d = mX(np.diag(KPang), P6e_world_X_base[3:6]) + mX(np.diag(KDang), V6e_world_X_base[3:6])
			# else:
			xa_d = np.array([0.,0.,.0])
			wa_d = np.array([0.,0.,.0])

			temp_gphase = gphase
			# Fault tolerance: discontinuous phase
			if (all( map(lambda x: x == self.Robot.Gait.cs[0], self.Robot.Gait.cs ) ) and 
				state_machine == 'motion' and
				self.Robot.Fault.status):
				temp_gphase = map(lambda x: 0 if x is False else 1, self.Robot.Fault.fault_index )
			
			force_dist = self.compute_foot_force_distribution(self.Robot.P6d.world_X_base, qd.xp, xa_d, wa_d, temp_gphase, fmax_lim)
			joint_torq = self.Robot.force_to_torque(force_dist)
			# print i, np.round(force_dist[17],3), np.round(fmax_lim[-1],3)
			# print temp_gphase, fmax_lim
			if self.interface != 'rviz':# and self.control_loop != "open":
				
				## Leg admittance - force tracking
				if self.ctrl_leg_admittance:
					## Base admittance - fault force tracking (requires leg force tracking) 
					if self.ctrl_base_admittance:
						offset_base = self.Robot.apply_base_admittance(force_dist)

					qd, tXj_error = self.Robot.apply_leg_admittance(force_dist)

					if (self.Robot.invalid == True):
						print 'Error Occured, robot invalid! ', tXj_error
						break
			# 	## Recompute task to joint space
			# 	qd, tXj_error = self.Robot.task_X_joint()

			# if (self.Robot.invalid == True):
			# 	print 'Error Occured, robot invalid! ', tXj_error
			# 	break
			
			print 'c cXf: ', np.round(self.Robot.Leg[4].XHc.coxa_X_foot[:3,3],4)
			# print 'qd: ', np.round(qd.xp[12:15],3)
			# print 'qc: ', np.round(self.Robot.Leg[4].Joint.qpc,3)
			# cout3(self.Robot.Leg[4].Joint.qpc)
			print '======================================='
			## Data logging & publishing
			qlog = self.set_log_setpoint(v3cp, v3cv, xa_d, v3wp, v3wv, wa_d, qd, joint_torq, force_dist)
			self.publish_topics(qd, qlog)

			## ============================================================================================================================== ##
		else:
			# Finish off transfer legs trajectory onto ground
			# self.complete_transfer_trajectory()
			self.Robot.update_state(control_mode=self.control_rate)

			if motion_plan is not None:
				print 'Motion Plan Execution Success!'
				print 'Desired Goal: ', np.round(base_path.X.xp[-1],4), np.round(base_path.W.xp[-1],4)
				print 'Tracked Goal: ', np.round(cob_X_desired.flatten(),4), np.round(cob_W_desired.flatten(),4)

			return True

		if self.Robot.invalid:
			print 'Motion invalid, exiting!'
			return False

	def update_phase_support(self, P6e_world_X_base, V6e_world_X_base):

		## I controller
		# comp_world_X_base = vec6d_to_se3(self.Robot.P6d.world_X_base + KI_P_BASE*self.P6e_integral)
		# 					# mX(np.diag([KI_P_BASE,KI_P_BASE,KI_P_BASE,KI_W_BASE,KI_W_BASE,KI_W_BASE]),self.P6e_integral))
		# comp_base_X_world = np.linalg.inv(comp_world_X_base)
		
		## PI controller
		# u = self.Robot.Base_Ctrl.update(P6e_world_X_base)
		# cout3v(u)
		# comp_world_X_base = vec6d_to_se3(self.Robot.P6c.world_X_base + u)
		# comp_base_X_world = np.linalg.inv(comp_world_X_base)

		# for j in range (0, 6):
		# 	# Determine foot position wrt base & coxa. 
		# 	if (self.Robot.Gait.cs[j] == 0):
		# 		## Position-control
		# 		# Closed-loop control on bodypose. Move by sum of desired pose and error
		# 		if self.ctrl_base_tracking:
		# 			self.Robot.Leg[j].XHd.base_X_foot = mX(comp_base_X_world, self.Robot.Leg[j].XHc.world_X_foot)
		# 		elif self.ctrl_contact_detect:
		# 			self.Robot.Leg[j].XHd.base_X_foot = mX(self.Robot.XHc.base_X_world, self.Robot.Leg[j].XHc.world_X_foot)
		# 		else:
		# 			self.Robot.Leg[j].XHd.base_X_foot = mX(self.Robot.XHd.base_X_world, self.Robot.Leg[j].XHc.world_X_foot)

		# 		self.Robot.Leg[j].XHd.coxa_X_foot = mX(self.Robot.Leg[j].XHd.coxa_X_base, self.Robot.Leg[j].XHd.base_X_foot)


		u_arr = self.Robot.apply_base_pos_ctrl(P6e_world_X_base)

		for j in range (0, 6):
			# Determine foot position wrt base & coxa. 
			if (self.Robot.Gait.cs[j] == 0):
				## Position-control
				# Closed-loop control on bodypose. Move by sum of desired pose and error
				u = u_arr.pop()
				comp_world_X_base = vec6d_to_se3(self.Robot.P6c.world_X_base + u)
				comp_base_X_world = np.linalg.inv(comp_world_X_base)
				if self.ctrl_base_tracking:
					self.Robot.Leg[j].XHd.base_X_foot = mX(comp_base_X_world, self.Robot.Leg[j].XHc.world_X_foot)
				elif self.ctrl_contact_detect:
					self.Robot.Leg[j].XHd.base_X_foot = mX(self.Robot.XHc.base_X_world, self.Robot.Leg[j].XHc.world_X_foot)
				else:
					self.Robot.Leg[j].XHd.base_X_foot = mX(self.Robot.XHd.base_X_world, self.Robot.Leg[j].XHc.world_X_foot)

				self.Robot.Leg[j].XHd.coxa_X_foot = mX(self.Robot.Leg[j].XHd.coxa_X_base, self.Robot.Leg[j].XHd.base_X_foot)
				if j==4:
					print self.Robot.Leg[j].PosCont.error
					print 'u: ', np.round(u.flatten(),4)
					print 'S c wXb: ', np.round(self.Robot.XHc.world_X_base[:3,3],4)
					print 'S c wXb: ', np.round(comp_world_X_base[:3,3],4)
					temp1 = mX(self.Robot.XHc.base_X_world, self.Robot.Leg[j].XHc.world_X_foot)
					temp2 = mX(self.Robot.Leg[j].XHd.coxa_X_base, temp1)
					# print 'S c wXf: ', np.round(self.Robot.Leg[j].XHc.world_X_foot[:3,3],4)
					print 'S d cXf: ', np.round(self.Robot.Leg[4].XHd.coxa_X_foot[:3,3],4)
					# print 'temp1: ', np.round(temp2,3)
					print 'temp2: ', np.round(temp2[:3,3],3)

	def update_phase_transfer(self,delta_d=None):

		for j in range (0, self.Robot.active_legs):
			## Transfer phase
			if (self.Robot.Gait.cs[j] == 1 and self.Robot.Leg[j].feedback_state == 1):
				## Update leg position from generated leg spline
				if (self.Robot.Leg[j].update_from_spline() is False):
					## Stops body from moving until transfer phase completes
					print 'Suspending in update_phase_transfer()'
					self.Robot.suspend = True
				if j==4:
					print 'T d cXf: ', np.round(self.Robot.Leg[4].XHd.coxa_X_foot[:3,3],4)
			elif (self.Robot.Gait.cs[j] == 1 and self.Robot.Leg[j].feedback_state == 2 and delta_d is not None):
				self.Robot.Leg[j].XHd.coxa_X_foot[:3,3] -= delta_d
				# print delta_d

	def compute_foot_force_distribution(self, qb, qp, v3ca, v3wa, gphase, fmax=None):
		""" Computes foot force distribution. First gets the Composite Rigid Body Inertia Matrix
		 	and CoM which are both function of leg joint angles. Next, compute foot force in
		 	world frame using Quadratic Programming (QP)
		 	Input: 	qb: base pose, Re^6
		 			qp: joint angles, Re^18
					v3ca: base linear acceleration, Re^3
					v3wa: base angular acceleration, Re^3
		 	Output: foot force for legs in stance phase """

		# now = rospy.get_time()

		## Compute CRBI and CoM, then compute foot force distribution and joint torque
		self.Robot.update_rbdl(qb, qp)
		# if (self.rbim_serv_.available):
		# 	try:
		# 		resp1 = self.rbim_serv_.call(qp)
		# 		self.Robot.update_rbdl(resp1.CoM, resp1.CRBI)
		# 	except:
		# 		rbim_valid = False
		
		## Append legs and joint angles in stance phase
		s_norm = []
		p_foot = [] 		
		q_contact = []
		# print np.round(self.Robot.P6c.base_X_CoM[:3].flatten(),3)
		# print np.round(self.Robot.Rbdl.com.flatten(),3)
		for j in range(0,6):
			if (gphase[j] == 0):
				# world_CoM_X_foot = mX( self.Robot.XHd.world_X_base[:3,:3],
				# 	  					(-self.Robot.P6c.base_X_CoM[:3].flatten()+self.Robot.Leg[j].XHd.base_X_foot[:3,3]) )
				
				world_CoM_X_foot = mX( self.Robot.XHd.world_X_base[:3,:3],
					  					(-self.Robot.Rbdl.com.flatten() + self.Robot.Leg[j].XHd.base_X_foot[:3,3]) )
				p_foot.append(world_CoM_X_foot.copy())
				q_contact.append(qp[(j*3):(j*3)+3])
				s_norm.append(self.Robot.Leg[j].snorm)
				# s_norm.append(np.array([0., 0., 1.]))
				# snorm_left  = np.array([0., 0., 1.]) # np.array([0., -1., 0.])
				# snorm_right = np.array([0., 0., 1.]) # np.array([0.,  1., 0.])
				# if j <3:
				# 	s_norm.append(snorm_left)
				# 	self.Robot.Leg[j].snorm = snorm_left
				# else:
				# 	s_norm.append(snorm_right)
				# 	self.Robot.Leg[j].snorm = snorm_right
		# print gphase, fmax
		# print p_foot
		# print s_norm
		# print '======================'
		# print np.round(v3ca.flatten(),3) 
		# print np.round(v3wa.flatten(),3) 
		## Compute force distribution using QP
		foot_force = self.ForceDist.resolve_force(v3ca, v3wa, p_foot,
													self.Robot.Rbdl.com,
													self.Robot.Rbdl.crbi, gphase, fmax, #)
													s_norm, qb[3:6], qp)
		# print np.round(foot_force[17],3), np.round(fmax[-1],3)
		# end = rospy.get_time()
		# print 'Tdiff: ', end - now
		if abs(foot_force[16]) > 38:
			raw_input('force high')
		return foot_force

	def suspend_controller(self):
		""" User Interface for commanding robot motion state """

		while (self.ui_state == 'hold' or self.ui_state == 'pause'):
			# loop until instructed to start or cancel
			if (rospy.is_shutdown()):
				break
			rospy.sleep(0.15)
		else:
			if (self.ui_state == 'play'):
				pass
			else:
				print 'Motion Cancelled!'
				return False