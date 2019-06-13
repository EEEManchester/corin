#!/usr/bin/env python

""" Main controller for the robot """
__version__ = '1.0'
__author__  = 'Wei Cheah'

import sys; sys.dont_write_bytecode = True
from itertools import cycle

from robot_manager import CorinManager
from corin_control import *			# library modules to include

import rospy
import time

class RobotController(CorinManager):
	def __init__(self, initialise=False):
		CorinManager.__init__(self, initialise)

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
		print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
		print 'Execute Path? '
		print 'con in: ', self.interface, self.control_loop
		if (self.interface == 'gazebo' or self.interface == 'robotis'):
			raw_input('cont')
			self.ui_state = 'play'#'hold'
		else:
			self.ui_state = 'play'#'hold'

		# State machine loading/unloading timing parameters
		tload = 1						# timing for loading
		iload = int(LOAD_T/CTR_INTV) 	# no. of intervals for loading
		hload = int(0.5/CTR_INTV)		# no. of intervals for initial holding
		# Force Distribution parameters
		fmax_lim  = [F_MAX]*6			# maximum force array
		init_flim = [F_MAX]*6			# current force for linear decrease in unloading
		gphase = [0]*6 					# gait phase array
		# Base impedance
		offset_z = 0.0
		offset_base = np.zeros(3)
		# Support phase counter
		s_max = int(GAIT_TPHASE/CTR_INTV)
		s_cnt = 1

		self.P6e_prev_1 = np.zeros((6,1))
		self.P6e_prev_2 = np.zeros((6,1))

		## Cycle through trajectory points until complete
		i = 1 		# skip first point since spline has zero initial differential conditions
		while (i != path_length and not rospy.is_shutdown()):

			# start = time.time()
			self.suspend_controller()
			print 'counting: ', i, len(base_path.X.t)

			## Update robot state
			self.Robot.update_state(control_mode=self.control_rate)
			#########################################################################
			## overwrite - TC: use IMU data
			self.Robot.XHc.world_X_base = self.Robot.XHd.world_X_base.copy()
			self.Robot.XHc.base_X_world = self.Robot.XHd.base_X_world.copy()

			if (self.interface == 'rviz' or self.control_loop == 'open'):
				self.Robot.P6c.world_X_base = self.Robot.P6d.world_X_base.copy()

			P6e_world_X_base = self.Robot.P6d.world_X_base - self.Robot.P6c.world_X_base
			V6e_world_X_base = self.Robot.V6d.world_X_base - self.Robot.V6c.world_X_base
			
			for j in range(0, self.Robot.active_legs):
				self.Robot.Leg[j].P6_world_X_base = self.Robot.P6c.world_X_base.copy()

			#########################################################################
			self.Robot.suspend = False # overwrite for selected motions
			## suppress trajectory counter as body support suspended
			if (self.Robot.suspend == True or state_machine == 'hold'):
				i -= 1
				s_cnt -= 1

			#########################################################################

			## Variable mapping to R^(3x1) for base linear and angular time, position, velocity, acceleration
			## Next Point along base trajectory
			if motion_plan is not None:
				# GRID PLAN: base trajectory in world frame
				v3cp = wXbase_offset[0:3] + base_path.X.xp[i].reshape(3,1) 		\
								+ offset_base.reshape((3,1))
								# + np.array([0., 0., offset_z]).reshape((3,1))
				# v3cp = base_path.X.xp[i].reshape(3,1); 	# CORNERING: base trajectory relative to world frame
				v3cv = base_path.X.xv[i].reshape(3,1);
				v3ca = base_path.X.xa[i].reshape(3,1);

				v3wp = wXbase_offset[3:6] + base_path.W.xp[i].reshape(3,1)
				# v3wp = base_path.W.xp[i].reshape(3,1);	# CORNERING: base trajectory relative to world frame
				v3wv = base_path.W.xv[i].reshape(3,1);
				v3wa = base_path.W.xa[i].reshape(3,1);
			else:
				v3cp = wXbase_offset[0:3] + + offset_base.reshape((3,1))#np.array([0., 0., offset_z]).reshape((3,1))
				# v3cp = wXbase_offset[0:3]
				v3cv = np.zeros((3,1))
				v3ca = np.zeros((3,1))

				v3wp = wXbase_offset[3:6]
				v3wv = np.zeros((3,1))
				v3wa = np.zeros((3,1))
			
			
			## Tracking desired translations
			if (self.Robot.suspend != True):
				cob_X_desired += v3cv*CTR_INTV
				cob_W_desired += v3wv*CTR_INTV

			## ============================================================================================================================== ##
			## Execution of command ##
			## ==================== ##

			## Update robot's desired position, velocity & acceleration to next point on spline
			self.Robot.P6d.world_X_base = np.vstack((v3cp,v3wp))
			self.Robot.V6d.world_X_base = np.vstack((v3cv,v3wv)) 			# not used atm
			self.Robot.A6d.world_X_base = np.vstack((v3ca,v3wa)) 			# not used atm
			self.Robot.update_world_desired_frames()

			# print state_machine, tload, iload+1, self.Robot.Gait.cs
			if state_machine == 'unload':

				if tload == 1:
					self.Robot.Gait.support_mode()
					gphase = [0]*6 		# all legs in support phase at start of unloading
					fmax_lim = [0]*6	# max. force array
					for j in range(0,6):
						# init_flim[j] = self.Robot.Leg[j].F6c.world_X_foot[2] if (self.Robot.Gait.ps[j] == 1) else F_MAX
						init_flim[j] = self.Robot.Leg[j].get_normal_force() if (self.Robot.Gait.ps[j] == 1) else F_MAX
					print init_flim
					
				# reduce max force for legs that will be in transfer phase
				for j in range(0,6):
					# fmax_lim[j] = fmax if (self.Robot.Gait.cs[j] == 1) else F_MAX
					fmax = init_flim[j]+1. - tload*(init_flim[j]+1.)/float(iload) + 0.001
					fmax_lim[j] = fmax if (self.Robot.Gait.ps[j] == 1) else F_MAX
				tload += 1

				self.support_phase_update(P6e_world_X_base, V6e_world_X_base)

				if tload == iload+1:
					tload = 1
					state_machine = 'motion'
					self.Robot.Gait.walk_mode()
					print 'Motion ...'

			elif state_machine == 'load':

				if tload == 1:
					self.Robot.Gait.support_mode()
					gphase = [0]*6 		# all legs in support phase at start of unloading
					fmax_lim = [0]*6	# max. force array

				# reduce max force for legs that will be in transfer phase
				# fmax = F_THRES + tload*(F_MAX-F_THRES)/float(iload)
				fmax = tload*F_MAX/float(iload)
				for j in range(0,6):
					# gait phase updated prior to this state, so use previous gait phase
					fmax_lim[j] = fmax if (self.Robot.Gait.ps[j] == 1) else F_MAX	
				tload += 1

				self.support_phase_update(P6e_world_X_base, V6e_world_X_base)

				if tload == iload+1:
					tload = 1
					state_machine = 'unload'
					print 'Unloading ...'
					try:
						self.Robot.alternate_phase(next(gait_stack))
					except:
						self.Robot.alternate_phase()
					# raw_input('cont')
					self.Robot.Gait.walk_mode()
				
			elif state_machine ==  'motion':
				
				fmax_lim = [F_MAX]*gphase.count(0)	# max. force array
				if state_machine ==  'motion':
					gphase = self.Robot.Gait.cs

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
								self.Robot.Leg[j].XHd.base_X_foot  = v3_X_m(base_X_footholds[j].xp.pop(0))
								self.Robot.Leg[j].XHd.world_base_X_NRP = v3_X_m(w_base_X_NRP[j].xp.pop(0))
								self.Robot.Leg[j].XHc.world_base_X_NRP = self.Robot.Leg[j].XHd.world_base_X_NRP.copy()

								## Set bodypose in leg class
								self.Robot.Leg[j].XH_world_X_base = self.Robot.XHd.world_X_base.copy()

							except IndexError:
								## TODO: plan on the fly. Currently set to default position
								print 'Leg: ', j, ' No further foothold planned!', i
								iahead = int(GAIT_TPHASE/CTR_INTV)
								# Get bodypose at end of gait phase, or end of trajectory
								try:
									x_ahead = base_path.X.xp[i+iahead]
									w_ahead = base_path.W.xp[i+iahead]
								except IndexError:
									x_ahead = base_path.X.xp[-1]
									w_ahead = base_path.W.xp[-1]

								self.Robot.Leg[j].XH_world_X_base = vec6d_to_se3(np.array([x_ahead, w_ahead]).reshape((6,1)))
								self.Robot.Leg[j].XHd.base_X_foot = mX(np.linalg.inv(self.Robot.Leg[j].XH_world_X_base),
																		self.Robot.Leg[j].XHd.world_X_foot)
						
						## Compute average surface normal from cell surface normal at both footholds
						try:
							sn1 = self.GridMap.get_cell('norm', self.Robot.Leg[j].XHc.world_X_foot[0:3,3], j)
							sn2 = self.GridMap.get_cell('norm', self.Robot.Leg[j].XHd.world_X_foot[0:3,3], j)
						except:
							sn1 = np.array([0., 0., 1.])
							sn2 = np.array([0., 0., 1.])
						
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
				self.transfer_phase_update()
				self.support_phase_update(P6e_world_X_base, V6e_world_X_base)
				
				# ends gait phase early if transfer phase completes - TODO: ANOTHER WAY?
				transfer_total = 0;
				leg_complete   = 0;

				for j in range(0,6):
					if (self.Robot.Gait.cs[j]==1):
						transfer_total += 1
						if (self.Robot.Leg[j].feedback_state == 2):
							leg_complete += 1

					
				# triggers only when all transfer phase legs complete and greater than zero
				# > 0: prevents trigger during all leg support
				if ( (transfer_total == leg_complete and transfer_total > 0 and leg_complete > 0) or
					# For discontinuous phase gait
					(all( map(lambda x: x == self.Robot.Gait.cs[0], self.Robot.Gait.cs ) ) and s_cnt == s_max+1)):# or
					# (s_cnt > s_max/2. and self.Robot.Gait.cs[5]==1 and self.Robot.Leg[5].cstate == True) ):
					# print s_cnt, s_max/2., transfer_total, leg_complete
					# if (self.Robot.Gait.cs[5]==1 and self.Robot.Leg[5].cstate == True):
					# 	print 'HERE HERE HERE cstate early, switch phase'
					# Update new foothold setpoint for leg finishing transfer phase
					for j in range(0,6):
						if (self.Robot.Gait.cs[j] == 1):
							self.Robot.Leg[j].transfer_phase_change = False
							self.Robot.Leg[j].XHc.update_world_X_foot(self.Robot.XHc.world_X_base)
							self.Robot.Leg[j].XHd.world_X_foot = self.Robot.Leg[j].XHc.world_X_foot.copy()
					state_machine = 'load'
					print 'Loading ...'
					s_cnt = 1

				i += 1
				s_cnt += 1

			elif state_machine == 'hold':
				""" Holds position - for controller to settle or pause motion """
				# print 'Holding State'
				gphase = [0]*6
				fmax_lim = [F_MAX]*gphase.count(0)	# max. force array

				if tload == 1:
					print 'Holding ...'
					self.Robot.Gait.support_mode()
				tload += 1
				if tload == hload+1:# and motion_plan:
					state_machine = 'unload'
					tload = 1
					self.Robot.Gait.walk_mode()
					print 'Unloading ...'
				self.support_phase_update(P6e_world_X_base, V6e_world_X_base)
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
			xa_d = mX(np.diag(KPcom), P6e_world_X_base[0:3]) + mX(np.diag(KDcom), V6e_world_X_base[0:3])
			wa_d = mX(np.diag(KPang), P6e_world_X_base[3:6]) + mX(np.diag(KDang), V6e_world_X_base[3:6])
			
			# else:
			xa_d = np.array([0.,0.,.0])
			wa_d = np.array([0.,0.,.0])

			temp_gphase = gphase
			if i > 281 and i < 840:
				# if all( map(lambda x: x == self.Robot.Gait.cs[0], self.Robot.Gait.cs ) ) and state_machine=='motion':
				temp_gphase = [0,0,0,0,1,0]

			force_dist = self.compute_foot_force_distribution(self.Robot.P6d.world_X_base, qd.xp, xa_d, wa_d, temp_gphase, fmax_lim)
			joint_torq = self.Robot.force_to_torque(force_dist)
			# print np.round(force_dist.flatten(),3)

			if self.interface != 'rviz' and self.control_loop != "open":
				## Base impedance
				# offset_base = self.Robot.apply_base_impedance(force_dist[12:15])

				## Impedance controller for each legs
				self.Robot.apply_impedance_control(force_dist)

				## Recompute task to joint space
				qd, tXj_error = self.Robot.task_X_joint()

			if (self.Robot.invalid == True):
				print 'Error Occured, robot invalid! ', tXj_error
				break

			## Data logging & publishing
			qlog = self.set_log_setpoint(v3cp, v3cv, xa_d, v3wp, v3wv, wa_d, qd, joint_torq, force_dist)
			
			self.publish_topics(qd, qlog)

			self.P6e_prev_2 = self.P6e_prev_1.copy()
			self.P6e_prev_1 = P6e_world_X_base.copy()

			## ============================================================================================================================== ##
		else:
			# Finish off transfer legs trajectory onto ground
			# self.complete_transfer_trajectory()
			self.Robot.update_state(control_mode=self.control_rate)

			if motion_plan is not None:
				print 'Trajectory executed'
				print 'Desired Goal: ', np.round(base_path.X.xp[-1],4), np.round(base_path.W.xp[-1],4)
				print 'Tracked Goal: ', np.round(cob_X_desired.flatten(),4), np.round(cob_W_desired.flatten(),4)

			return True

		if self.Robot.invalid:
			print 'Motion invalid, exiting!'
			return False

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
		# TEMP: base_X_foot
		# base_X_foot = [np.array([ 0.25,  0.251, -BODY_HEIGHT]),
		# 				np.array([ 0.,    0.300, -BODY_HEIGHT]),
		# 				np.array([-0.25,  0.251, -BODY_HEIGHT]),
		# 				np.array([ 0.25, -0.251, -BODY_HEIGHT]),
		# 				np.array([ 0.,   -0.300, -BODY_HEIGHT]),
		# 				np.array([-0.25, -0.251, -BODY_HEIGHT])]

		## Append legs and joint angles in stance phase
		s_norm = []
		p_foot = [] 		
		q_contact = []
		for j in range(0,6):
			if (gphase[j] == 0):
				world_CoM_X_foot = mX( self.Robot.XHd.world_X_base[:3,:3],
					  					(-self.Robot.P6c.base_X_CoM[:3].flatten()+self.Robot.Leg[j].XHd.base_X_foot[:3,3]) )
				# world_CoM_X_foot = mX( self.Robot.XHd.world_X_base[:3,:3],
				# 	  					(-self.Robot.Rbdl.com.flatten() + base_X_foot[j]) )
				p_foot.append(world_CoM_X_foot.copy())
				q_contact.append(qp[(j*3):(j*3)+3])
		
				# s_norm.append(np.array([0., 0., 1.]))
				snorm_left  = np.array([0., 0., 1.]) # np.array([0., -1., 0.])
				snorm_right = np.array([0., 0., 1.]) # np.array([0.,  1., 0.])
				if j <3:
					s_norm.append(snorm_left)
					self.Robot.Leg[j].snorm = snorm_left
				else:
					s_norm.append(snorm_right)
					self.Robot.Leg[j].snorm = snorm_right

		# print np.round(v3ca.flatten(),3) 
		# print np.round(v3wa.flatten(),3) 
		## Compute force distribution using QP
		foot_force = self.ForceDist.resolve_force(v3ca, v3wa, p_foot,
													self.Robot.Rbdl.com,
													self.Robot.Rbdl.crbi, gphase, fmax, #)
													s_norm, qb[3:6], qp)

		# end = rospy.get_time()
		# print 'Tdiff: ', end - now

		return foot_force

	def support_phase_update(self, P6e_world_X_base, V6e_world_X_base):

		kcorrect = np.zeros((6,1))

		# PID controller
		Ts = CTR_INTV
		Kp = 0.#0.2
		Ki = 0.#0.
		Kd = 0.#0.0005

		a = Kp + Ki*Ts/2 + Kd/Ts
		b = -Kp + Ki*Ts/2 - 2*Kd/Ts
		c = Kd/Ts
		# linear correction only
		kcorrect[0:3] = a*P6e_world_X_base[0:3] + b*self.P6e_prev_1[0:3] + c*self.P6e_prev_2[0:3]
		# full body correction
		kcorrect[3:6] = a*P6e_world_X_base[3:6] + b*self.P6e_prev_1[3:6] + c*self.P6e_prev_2[3:6]

		# Virtual Base controller
		# xa_d = mX(np.diag(KPcom), P6e_world_X_base[0:3]) + mX(np.diag(KDcom), V6e_world_X_base[0:3])
		# wa_d = mX(np.diag(KPang), P6e_world_X_base[3:6]) + mX(np.diag(KDang), V6e_world_X_base[3:6])
		# kcorrect = np.array([xa_d,wa_d]).reshape((6,1))
		
		comp_world_X_base = vec6d_to_se3(self.Robot.P6d.world_X_base + kcorrect)
		comp_base_X_world = np.linalg.inv(comp_world_X_base)

		# print 'error: ', np.round(P6e_world_X_base[0:3].flatten(),3)
		# print np.round(self.P6e_prev_1[0:3].flatten(),3)
		# print 'corr: ', np.round(kcorrect[0:3].flatten(),4)

		for j in range (0, 6):
			if (self.Robot.Gait.cs[j] == 0 and self.Robot.suspend == False):
				## Determine foot position wrt base & coxa - REQ: world_X_foot position
				if (self.control_loop == "open"):
					self.Robot.Leg[j].XHd.base_X_foot = mX(self.Robot.XHd.base_X_world, self.Robot.Leg[j].XHc.world_X_foot)

				elif (self.control_loop == "close"):
					## Closed-loop control on bodypose. Move by sum of desired pose and error
					
					## Foot position: either position or velocity
					## Position-control
					# comp_world_X_base = vec6d_to_se3(self.Robot.P6d.world_X_base + K_BP*P6e_world_X_base)
					# if j >2:
					self.Robot.Leg[j].XHd.base_X_foot = mX(comp_base_X_world, self.Robot.Leg[j].XHc.world_X_foot)
					# else:
					# 	self.Robot.Leg[j].XHd.base_X_foot = mX(self.Robot.XHd.base_X_world, self.Robot.Leg[j].XHc.world_X_foot)
				# 	## Velocity-control - Focchi2018 eq(3) TODO: check robustness
				# 	vel_base_X_foot = -V6e_world_X_base[0:3].flatten() - np.cross(V6e_world_X_base[3:6].flatten(), 
				# 						(self.Robot.Leg[j].XHd.base_X_foot[0:3,3:4] - self.Robot.P6d.world_X_base[0:3]).flatten())
				# 	base_X_foot = self.Robot.Leg[j].XHd.base_X_foot[0:3,3] + vel_base_X_foot
				# 	# print j, '  ', base_X_foot - self.Robot.Leg[j].XHd.base_X_foot[0:3,3]

				# self.Robot.Leg[j].XHd.base_X_foot = mX(self.Robot.XHd.base_X_world, self.Robot.Leg[j].XHc.world_X_foot)
				self.Robot.Leg[j].XHd.coxa_X_foot = mX(self.Robot.Leg[j].XHd.coxa_X_base, self.Robot.Leg[j].XHd.base_X_foot)

				# if j==4:
					# print j, ' bXw: \n', np.round(self.Robot.XHd.base_X_world,3)
					# print j, ' wXf: ', np.round(self.Robot.Leg[j].XHc.world_X_foot[0:3,3],3)
					# print j, ' bXf: ', np.round(self.Robot.Leg[j].XHd.base_X_foot[0:3,3],3)
					# print j, ' cXf: ', np.round(self.Robot.Leg[j].XHd.coxa_X_foot[0:3,3],3)
					# print '========================================================='

	def transfer_phase_update(self):

		for j in range (0, self.Robot.active_legs):
			## Transfer phase
			if (self.Robot.Gait.cs[j] == 1 and self.Robot.Leg[j].feedback_state == 1):
				## Update leg position from generated leg spline
				if (self.Robot.Leg[j].update_from_spline() is False):
					## Stops body from moving until transfer phase completes
					self.Robot.suspend = True

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