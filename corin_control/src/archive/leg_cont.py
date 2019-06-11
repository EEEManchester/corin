def leg_controller(self):

		# State machine loading/unloading timing parameters
		tload = 1						# timing for loading
		iload = int(LOAD_T/CTR_INTV) 	# no. of intervals for loading
		# Force Distribution parameters
		fmax_lim  = [F_MAX]*6			# maximum force array
		init_flim = [F_MAX]*6			# current force for linear decrease in unloading
		gphase = [0]*6 					# gait phase array
		# Base impedance
		offset_z = 0.0
		# Support phase counter
		s_max = int(GAIT_TPHASE/CTR_INTV)
		s_cnt = 1
		print 'gphase: ', self.Robot.Gait.cs

		while (not rospy.is_shutdown()):
			## Update robot state
			self.Robot.update_state(control_mode=self.control_rate)

			## Generate trajectory for legs in transfer phase
			for j in range (0, self.Robot.active_legs):
				
				# Transfer phase - generate trajectory
				if (self.Robot.Gait.cs[j] == 1 and 
					self.Robot.Leg[j].transfer_phase_change == False and
					not self.Robot.Fault.fault_index[self.Robot.Gait.cs.index(1)]):
					self.Robot.Leg[j].feedback_state = 1 	# set leg to execution mode

					## Compute average surface normal from cell surface normal at both footholds
					sn1 = np.array([0., 0., 1.])
					sn2 = np.array([0., 0., 1.])
					
					## Generate transfer spline
					svalid = self.Robot.Leg[j].generate_spline('world', sn1, sn2, 1, False, GAIT_TPHASE, CTR_INTV)

					## Update NRP
					self.Robot.Leg[j].XHd.base_X_NRP[:3,3] = mX(self.Robot.XHd.base_X_world[:3,:3],
																self.Robot.Leg[j].XHc.world_base_X_NRP[:3,3])
					self.Robot.Leg[j].XHd.world_base_X_AEP[:3,3] = mX(self.Robot.XHd.world_X_base[:3,:3],
																		self.Robot.Leg[j].XHd.base_X_foot[:3,3])

					if (svalid is False):
						# set invalid if trajectory unfeasible for leg's kinematic
						self.Robot.invalid = True
						print 'Leg Transfer Trajectory Invalid'
					else:
						# set flag that phase has changed
						self.Robot.Leg[j].transfer_phase_change = True

			## Compute task space foot position for all legs
			self.transfer_phase_update()
			self.support_phase_update(np.zeros(6), np.zeros(6))
			
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
			if ( (transfer_total == leg_complete and transfer_total > 0 and leg_complete > 0)):# or
				# For discontinuous phase gait
				# (all( map(lambda x: x == self.Robot.Gait.cs[0], self.Robot.Gait.cs ) ) and s_cnt == s_max+1) ):
				try:
					self.Robot.alternate_phase(next(gait_stack))
					print 'unstacked'
				except:
					self.Robot.alternate_phase()
				print 'gphase: ', self.Robot.Gait.cs

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
			# xa_d = v3ca #np.array([0.,0.,5.0])
			# wa_d = v3wa
			# force_dist = self.compute_foot_force_distribution(self.Robot.P6d.world_X_base, qd.xp, xa_d, wa_d, gphase, fmax_lim)
			# joint_torq = self.Robot.force_to_torque(force_dist)
			# # print np.round(force_dist.flatten(),3)

			# if self.interface != 'rviz':
			# 	## Base impedance
			# 	# offset_z = self.Robot.apply_base_impedance(force_dist[12:15])

			# 	## Impedance controller for each legs
			# 	# self.Robot.apply_impedance_control(force_dist)

			# 	## Recompute task to joint space
			# 	qd, tXj_error = self.Robot.task_X_joint()

			if (self.Robot.invalid == True):
				print 'Error Occured, robot invalid! ', tXj_error
				break

			## Data logging & publishing
			qlog = self.set_log_setpoint(np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), qd, [], [])
			
			self.publish_topics(qd, qlog)
