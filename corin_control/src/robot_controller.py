#!/usr/bin/env python

""" Main controller for the robot """
__version__ = '1.0'
__author__  = 'Wei Cheah'

import sys; sys.dont_write_bytecode = True
from itertools import cycle

from robot_manager import CorinManager
from corin_control import *			# library modules to include

import rospy

class RobotController(CorinManager):
	def __init__(self, initialise=False):
		CorinManager.__init__(self, initialise)

	def main_controller(self, motion_plan):

		## Variables ##
		cob_X_desired = np.zeros((3,1)) 	# cob linear location
		cob_W_desired = np.zeros((3,1)) 	# cob angular location
		wXbase_offset = self.Robot.P6c.world_X_base.copy()
		
		## Variable mapping from motion plan
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
			world_X_footholds[j].xp.pop(0)
			base_X_footholds[j].xp.pop(0)
			w_base_X_NRP[j].xp.pop(0)

		## User input
		print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'
		print 'Execute Path? '
		if (self.interface == 'gazebo'):
			raw_input('cont')
			self.ui_state = 'play'#'hold'
		else:
			self.ui_state = 'hold'

		## Use gait phase from planner if exists
		try:
			gait_stack = cycle(gait_phase)
			self.Robot.Gait.cs = next(gait_stack)
		except:
			print 'No defined gait phases, using periodic'
			
		## Cycle through trajectory points until complete
		i = 1 	# skip first point since spline has zero initial differential conditions
		while (i != len(base_path.X.t) and not rospy.is_shutdown()):
			
			# User Interface for commanding robot motion state
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

			# print 'counting: ', i, len(base_path.X.t)

			## Update robot state
			self.Robot.update_state(control_mode=self.control_rate)
			#########################################################################
			## overwrite - TC: use IMU data
			self.Robot.XHc.world_X_base = self.Robot.XHd.world_X_base.copy()
			self.Robot.XHc.base_X_world = self.Robot.XHd.base_X_world.copy()
			
			if (self.interface == 'rviz'):
				self.Robot.P6c.world_X_base = self.Robot.P6d.world_X_base.copy()
			elif (self.interface != 'rviz' and self.control_loop == 'close'):
				P6e_world_X_base = self.Robot.P6d.world_X_base - self.Robot.P6c.world_X_base
					
			for j in range(0, self.Robot.active_legs):
				self.Robot.Leg[j].P6_world_X_base = self.Robot.P6c.world_X_base.copy()
			
			#########################################################################
			self.Robot.suspend = False # overwrite for selected motions
			## suppress trajectory counter as body support suspended
			if (self.Robot.suspend == True):
				i -= 1

			########################################################

			## Variable mapping to R^(3x1) for base linear and angular time, position, velocity, acceleration
			## Next Point along base trajectory
			v3cp = wXbase_offset[0:3] + base_path.X.xp[i].reshape(3,1); 	# GRID PLAN: base trajectory in world frame
			# v3cp = base_path.X.xp[i].reshape(3,1); 	# CORNERING: base trajectory relative to world frame
			v3cv = base_path.X.xv[i].reshape(3,1);
			v3ca = base_path.X.xa[i].reshape(3,1);

			v3wp = wXbase_offset[3:6] + base_path.W.xp[i].reshape(3,1);
			# v3wp = base_path.W.xp[i].reshape(3,1);
			v3wv = base_path.W.xv[i].reshape(3,1);
			v3wa = base_path.W.xa[i].reshape(3,1);
			
			## Next CoB location - NOT USED ATM
			# try:
			# 	v3cp_n = wXbase_offset[0:3] + base_path.X.xp[i+int(CTR_RATE*GAIT_TPHASE)].reshape(3,1);
			# 	v3wp_n = wXbase_offset[3:6] + base_path.W.xp[i+int(CTR_RATE*GAIT_TPHASE)].reshape(3,1);
			# except:
			# 	v3cp_n = wXbase_offset[0:3] + base_path.X.xp[-1].reshape(3,1);
			# 	v3wp_n = wXbase_offset[3:6] + base_path.W.xp[-1].reshape(3,1);

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
			self.Robot.XHd.update_world_X_base(self.Robot.P6d.world_X_base)

			## Generate trajectory for legs in transfer phase
			for j in range (0, self.Robot.active_legs):
				# Transfer phase
				if (self.Robot.Gait.cs[j] == 1 and self.Robot.Leg[j].transfer_phase_change==False):
					self.Robot.Leg[j].feedback_state = 1 	# set leg to execution mode

					## Using planned foothold arrays
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
						
						self.Robot.Leg[j].XH_world_X_base = transform_world_X_base(np.array([x_ahead,
																							 w_ahead]).reshape((6,1)))
						self.Robot.Leg[j].XHd.base_X_foot = mX(np.linalg.inv(self.Robot.Leg[j].XH_world_X_base), 
																self.Robot.Leg[j].XHd.world_X_foot)
						# self.Robot.Leg[j].XHd.base_X_foot = self.Robot.Leg[j].XHd.base_X_NRP.copy()
						
					## Compute average surface normal from cell surface normal at both footholds
					sn1 = self.GridMap.get_cell('norm', self.Robot.Leg[j].XHc.world_X_foot[0:3,3], j)
					sn2 = self.GridMap.get_cell('norm', self.Robot.Leg[j].XHd.world_X_foot[0:3,3], j)
					
					# Following two lines used for cornering
					# sn1 = self.get_snorm(self.Robot.Leg[j].XHc.world_X_foot[0:3,3], j) 	
					# sn2 = self.get_snorm(self.Robot.Leg[j].XHd.world_X_foot[0:3,3], j)
					if (j<3):
						print j, np.round(self.Robot.Leg[j].XHc.world_X_foot[0:3,3],3)
						print j, np.round(self.Robot.Leg[j].XHd.world_X_foot[0:3,3],3)
						print j, sn1, sn2

					
					## Generate transfer spline
					svalid = self.Robot.Leg[j].generate_spline('world', sn1, sn2, 1, False, GAIT_TPHASE, CTR_INTV)
					
					## Update NRP
					self.Robot.Leg[j].XHd.base_X_NRP[:3,3] = mX(self.Robot.XHd.base_X_world[:3,:3], 
																self.Robot.Leg[j].XHc.world_base_X_NRP[:3,3])
					self.Robot.Leg[j].XHd.world_base_X_AEP[:3,3] = mX(self.Robot.XHd.world_X_base[:3,:3], 
																		self.Robot.Leg[j].XHd.base_X_foot[:3,3])

					## Identify knee up/down - first, checks only if body roll above threshold
					if ((self.Robot.P6d.world_X_base[3] > ROLL_TH and j >= 3) or (self.Robot.P6d.world_X_base[3] < -ROLL_TH and j < 3)):
						x_offset = np.array([COXA_X,0,0]).reshape((3,1)) 	# offset so that search at front of robot
						wclear = self.GridMap.get_median_width(self.Robot.P6d.world_X_base[:3] + x_offset)
						# change to knee down if width clearance below threshold
						if (wclear < WALL_WIDTH_NARROW):
							print j, ' Changing to knee down ', wclear
							if (self.Robot.Leg[j].KDL.knee_up == True):
								print 'Initiate change of knee config. routine'
							self.Robot.Leg[j].KDL.knee_up = False
						else:
							self.Robot.Leg[j].KDL.knee_up = True
					
					if (svalid is False):
						# set invalid if trajectory unfeasible for leg's kinematic
						self.Robot.invalid = True
						print 'Leg Transfer Trajectory Invalid'
					else:	
						# set flag that phase has changed
						self.Robot.Leg[j].transfer_phase_change = True

			## Compute task space foot position for all legs
			for j in range (0, self.Robot.active_legs):
				
				## Transfer phase
				if (self.Robot.Gait.cs[j] == 1 and self.Robot.Leg[j].feedback_state==1):
					## Update leg position from generated leg spline
					if (self.Robot.Leg[j].update_from_spline() is False):
						## Stops body from moving until transfer phase completes
						self.Robot.suspend = True

				## Support phase
				elif (self.Robot.Gait.cs[j] == 0 and self.Robot.suspend == False):
					## Determine foot position wrt base & coxa - REQ: world_X_foot position
					if (self.control_loop == "open"):
						self.Robot.Leg[j].XHd.base_X_foot = mX(self.Robot.XHd.base_X_world, self.Robot.Leg[j].XHc.world_X_foot)
					
					elif (self.control_loop == "close"):
						## Closed-loop control on bodypose. Move by sum of desired pose and error 
						comp_world_X_base = transform_world_X_base(self.Robot.P6d.world_X_base + K_BP*P6e_world_X_base)
						comp_base_X_world = np.linalg.inv(comp_world_X_base)
						self.Robot.Leg[j].XHd.base_X_foot = mX(comp_base_X_world, self.Robot.Leg[j].XHc.world_X_foot)
							
					self.Robot.Leg[j].XHd.coxa_X_foot = mX(self.Robot.Leg[j].XHd.coxa_X_base, self.Robot.Leg[j].XHd.base_X_foot)
					# print j, ' bXw: \n', np.round(self.Robot.XHd.base_X_world,3)
					# print j, ' wXf: ', np.round(self.Robot.Leg[j].XHc.world_X_foot[0:3,3],3)
					# print j, ' bXf: ', np.round(self.Robot.Leg[j].XHd.base_X_foot[0:3,3],3)
					# print j, ' cXf: ', np.round(self.Robot.Leg[j].XHd.coxa_X_foot[0:3,3],3)
					# print '========================================================='
			
			## Task to joint space
			qd, tXj_error = self.Robot.task_X_joint()	

			if (self.Robot.invalid == True):
				print 'Error Occured, robot invalid! ', tXj_error
				break
			else:
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
				if (transfer_total == leg_complete and transfer_total > 0 and leg_complete > 0):
					try:
						self.Robot.alternate_phase(next(gait_stack))
						print 'unstacked'
					except:
						self.Robot.alternate_phase()
					print i, self.Robot.Gait.cs, np.round(np.rad2deg(v3wp[2]).flatten(),4)
					print '======================================================='

				## Data logging
				qlog = self.set_log_values(v3cp, v3cv, v3ca, v3wp, v3wv, v3wa, qd)

				## Compute foot force distribution
				# qlog = self.compute_foot_force_distribution(qd.xp, v3ca, v3wa, qlog)

				## View stability polygon
				self.visualize_support_polygon()

				## Publish joint angles if motion valid
				self.publish_topics(qd, qlog)
				qd_prev = JointTrajectoryPoints(18,(qd.t, qd.xp, qd.xv, qd.xa))

				i += 1
			## ============================================================================================================================== ##
		else:	
			# Finish off transfer legs trajectory onto ground
			# self.complete_transfer_trajectory()
			self.Robot.update_state(control_mode=self.control_rate)
			
			print 'Trajectory executed'
			print 'Desired Goal: ', np.round(base_path.X.xp[-1],4), np.round(base_path.W.xp[-1],4)
			print 'Tracked Goal: ', np.round(cob_X_desired.flatten(),4), np.round(cob_W_desired.flatten(),4)

			return True
		
		if self.Robot.invalid:
			print 'Motion invalid, exiting!'
			return False

	def compute_foot_force_distribution(self, qp, v3ca, v3wa, qlog):
		""" Computes foot force distribution. First gets the Composite Rigid Body Inertia Matrix
		 	and CoM which are both function of leg joint angles. Next, compute foot force in
		 	world frame using Quadratic Programming (QP)
		 	Input: 	qp: joint angles, Re^18
					v3ca: base linear acceleration, Re^3
					v3wa: base angular acceleration, Re^3
					qlog: pre-filled LoggingState()
		 	Output: qlog: LoggingState() msg with updated variables on force and torque """

		## Compute CRBI and CoM, then compute foot force distribution and joint torque
		if (self.rbim_serv_.available):
			try:
				resp1 = self.rbim_serv_.call(qp)
				self.Robot.update_com_crbi(resp1.CoM, resp1.CRBI)
			except:
				rbim_valid = False
		
		p_foot = []
		pfoot_log = []
		for j in range(0,6):
			if (self.Robot.Gait.cs[j] == 0):
				world_CoM_X_foot = mX( self.Robot.XHd.world_X_base[:3,:3], 
					  					(-self.Robot.P6c.base_X_CoM[:3].flatten()+self.Robot.Leg[j].XHd.base_X_foot[:3,3]) )
				p_foot.append(world_CoM_X_foot.copy())
				pfoot_log += world_CoM_X_foot.flatten().tolist()
			else:
				pfoot_log += np.zeros(3).flatten().tolist()
		# Gravity vector wrt base orientation
		gv = mX(rotation_zyx(self.Robot.P6d.world_X_base[3:6]),np.array([0,0,G]))

		foot_force = self.ForceDist.resolve_force(gv, v3ca, v3wa, p_foot, 
													self.Robot.P6c.base_X_CoM[:3], 
													self.Robot.CRBI, self.Robot.Gait.cs )
		joint_torque = self.Robot.force_to_torque(foot_force)

		## update logging parameters
		qlog.effort = np.zeros(6).flatten().tolist() + joint_torque#.flatten().tolist()
		qlog.forces = np.zeros(6).flatten().tolist() + foot_force.flatten().tolist()
		qlog.qp_sum_forces  = self.ForceDist.sum_forces.flatten().tolist()
		qlog.qp_sum_moments = self.ForceDist.sum_moment.flatten().tolist()
		qlog.qp_desired_forces  = self.ForceDist.d_forces.flatten().tolist()
		qlog.qp_desired_moments = self.ForceDist.d_moment.flatten().tolist()

		return qlog
