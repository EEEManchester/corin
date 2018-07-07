def foothold_planner(self, base_path, Robot):
		""" Computes foothold based on generated path 	"""
		""" Input: 1) path -> linear and angular path 
			Output: Foothold array 						"""

		def set_motion_plan():
			## Set MotionPlan class
			motion_plan = MotionPlan()
			motion_plan.set_base_path(Robot.P6c.world_X_base.copy(), base_path, world_X_base)
			motion_plan.set_footholds(world_X_footholds, base_X_footholds, world_base_X_NRP)
			motion_plan.set_gait(Robot.Gait.np, Gait.np)

			return motion_plan

		def compute_ground_footholds():
			""" Computes NRP position for ground footholds """
			## TODO: this should also hold for normal ground walking
			# world_ground_X_base = self.Robot.XHd.world_X_base.copy()
			# world_ground_X_base[:2,3:4] = np.zeros((2,1))
			# world_ground_X_femur = mX(world_ground_X_base, self.Robot.Leg[j].XHd.base_X_femur)

			world_ground_X_base = XHd.world_X_base.copy()
			world_ground_X_base[:2,3:4] = np.zeros((2,1))
			world_ground_X_femur = mX(world_ground_X_base, Leg[j].base_X_femur)
			
			hy = world_ground_X_femur[2,3] - L3 - 0. 		# h_femur_X_tibia
			yy = np.sqrt(L2**2 - hy**2) 					# world horizontal distance from femur to foot
			by = np.cos(v3wp[0])*(COXA_Y + L1) 				# world horizontal distance from base to femur 
			sy = by + yy									# y_base_X_foot - leg frame
			py = sy*np.sin(np.deg2rad(ROT_BASE_X_LEG[j]+LEG_OFFSET[j])) 	# y_base_X_foot - world frame
			
			# Create temp array, which is the new base to foot position, wrt world frame
			# The x-component uses the base frame NRP so that it remains the same
			# temp = np.array([[self.Robot.Leg[j].XHd.base_X_NRP[0,3]],
			# 					[py],
			# 					[self.Robot.Leg[j].XHd.world_base_X_NRP[2,3]]])
			# self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4] = mX(rot_Z(v3wp[2]), temp)

			temp = np.array([[Leg[j].base_X_NRP[0,3]],[py],[Leg[j].world_base_X_NRP[2,3]]])
			Leg[j].world_base_X_NRP[:3,3:4] = mX(rot_Z(v3wp[2]), temp)
			
		def compute_wall_footholds(d_wall):
			""" Compute footholds for legs in wall contact """

			qj = np.pi/2 if (j < 3) else -np.pi/2 	# generalisation rotation from base to leg frame
			world_base_X_femur_z = abs((COXA_Y + L1)*np.sin(v3wp[0]))
			world_base_X_femur_y = (COXA_Y + L1)*np.cos(v3wp[0])*np.sin(qj)

			
			bXfy = d_wall[1]*np.sin(qj) 			# world horizontal distance base to foot
			dy = bXfy - world_base_X_femur_y 		# horizontal distance from femur joint to foot
			dh = np.sqrt(abs(L3**2 - dy**2)) 		# vertical distance from femur joint to foot
			bXfz = world_base_X_femur_z + L2/2 + dh # vertical distance from base to foot
			
			temp = np.array([[Leg[j].base_X_NRP[0,3]],[bXfy],[bXfz]])
			Leg[j].world_base_X_NRP[:3,3:4] = mX(rot_Z(v3wp[2]), temp)
			Leg[j].world_X_NRP[:3,3] = np.round( (v3cp + Leg[j].world_base_X_NRP[:3,3:4]).flatten(),4)

			# temp = np.array([[self.Robot.Leg[j].XHd.base_X_NRP[0,3]],[bXfy],[bXfz]])
			# self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4] = mX(rot_Z(v3wp[2]), temp)
			# self.Robot.Leg[j].XHd.world_X_NRP[:3,3] = np.round( (v3cp + self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4]).flatten(),4)
			# print '>>>> ', world_base_X_femur_y, world_base_X_femur_z, dy, dh

		## Define Variables ##
		i = 0
		XHd  = robot_transforms.HomogeneousTransform()
		Leg  = [None]*6
		Gait = gait_class.GaitClass(GAIT_TYPE)	# gait class
		do_wall = np.array([0., 0.31, 0.]) 		# Initial distance from robot's base to wall (base frame)

		gphase_intv  = [] 						# intervals in which gait phase changes
		world_X_base = []
		world_X_footholds = [None]*6
		base_X_footholds  = [None]*6
		world_base_X_NRP  = [None]*6

		Gait.cs = list(Robot.Gait.cs) 	# use local gait current state 
		Gait.np = Robot.Gait.np 		# copy gait counter
		v3cp_prev = Robot.P6c.world_X_base[0:3].copy()	# previous CoB position
		v3wp_prev = Robot.P6c.world_X_base[3:6].copy()	# previous CoB orientation
		world_X_base.append(Robot.P6c.world_X_base.flatten())

		## Instantiate leg transformation class & append initial footholds
		for j in range (0, 6):
			Leg[j] = robot_transforms.ArrayHomogeneousTransform(j)
			Leg[j].duplicate(Robot.Leg[j].XHd)

			world_X_footholds[j] = MarkerList()
			world_X_footholds[j].t.append(0.)
			world_X_footholds[j].xp.append(Robot.Leg[j].XHc.world_X_foot[:3,3:4])

			base_X_footholds[j] = MarkerList()
			base_X_footholds[j].t.append(0.)
			base_X_footholds[j].xp.append(Robot.Leg[j].XHc.base_X_foot[:3,3:4])

			world_base_X_NRP[j] = MarkerList()
			world_base_X_NRP[j].t.append(0.)
			world_base_X_NRP[j].xp.append(Robot.Leg[j].XHc.world_base_X_NRP[:3,3:4])

		## Returns as footholds remain fixed for full support mode
		if (Robot.support_mode is True):
			return set_motion_plan()

		## Cycle through trajectory
		while (i != len(base_path.X.t)):
			print i, ' Gait phase ', Gait.cs 
			bound_exceed = False

			# Cycles through one gait phase for support legs
			for m in range(0,int(GAIT_TPHASE*CTR_RATE)+1):
				## Variable mapping to R^(3x1) for base linear and angular time, position, velocity, acceleration
				try:
					v3cp = base_path.X.xp[i].reshape(3,1) #+ Robot.P6c.world_X_base[0:3]
					v3wp = base_path.W.xp[i].reshape(3,1) + Robot.P6c.world_X_base[3:6]
					
					# print np.round(v3cp.flatten(),4)
					P6d_world_X_base = np.vstack((v3cp,v3wp))

					## update robot's pose
					XHd.update_world_X_base(P6d_world_X_base)
					world_X_base_rot = XHd.world_X_base.copy()
					world_X_base_rot[:3,3:4] = np.zeros((3,1))
					
					for j in range (0, Robot.active_legs):
						## Legs in support phase:
						if (Gait.cs[j] == 0):
							Leg[j].base_X_foot = mX(XHd.base_X_world, Leg[j].world_X_foot)

							Leg[j].world_base_X_foot = mX(world_X_base_rot, Leg[j].base_X_foot)
							bound_exceed = Robot.Leg[j].check_boundary_limit(Leg[j].world_base_X_foot,
																					Leg[j].world_base_X_NRP)
							# if (j==0):
							# 	print 'wbXf: ', np.round(Leg[j].world_base_X_foot[:3,3],4)
							# 	print 'wbXn: ', np.round(Leg[j].world_base_X_NRP[:3,3],4)
							if (bound_exceed == True):
								print 'bound exceed on ', j, ' at n=', i#, i*CTR_INTV
								break

					if (bound_exceed is True):
						## To break through two layers of loops
						break
					else:	
						i += 1

				except IndexError:
					## Trajectory finished, skip support
					print 'Finishing foothold planning at ', i
					break

			## Stack to list next CoB location
			world_X_base.append(P6d_world_X_base.reshape(1,6))
			gphase_intv.append(i)
			print 'qbp: ', np.round(P6d_world_X_base.reshape(1,6),3)
			## Set foothold for legs in transfer phase
			for j in range (0, 6):
				if (Gait.cs[j] == 1 and i <= len(base_path.X.t)):
					
					## 1) Update NRP
					if (self.T_GND_X_WALL is True):
						
						# Distance robot's base travelled from origin
						dt_base = (mX(rot_Z(-v3wp[2]), (v3cp.reshape(3) - base_path.X.xp[0]).reshape(3,1) )).reshape(3)

						## Identify sides for ground or wall contact based on body roll
						delta_w = base_path.W.xp[-1]# - base_path.W.xp[0]
						
						if (delta_w[0] > 0.):
							## Right side ground contact, Left side wall contact
							if (j >= 3):
								compute_ground_footholds()
							else:
								di_wall = do_wall - np.round(dt_base ,3)
								compute_wall_footholds(di_wall)
								# print 'do wall ', np.round(dt_base,3), np.round(di_wall,3)
						else:
							## Left side in ground contact, Right side wall contact
							if (j >= 3):
								di_wall = do_wall + np.round(dt_base ,3)
								compute_wall_footholds(di_wall)
							else:
								compute_ground_footholds()
					
					elif (self.T_WALL_X_GND is True):
						if (j >= 3):
							compute_ground_footholds()
						else:
							compute_wall_footholds(di_wall)

					elif (self.W_WALL is True):
						# SE(3) with linear components and only yaw rotation
						XHy_world_X_base = mX(v3_X_m(P6d_world_X_base[:3]), r3_X_m(np.array([P6d_world_X_base[3],
																					0.,P6d_world_X_base[5]])))

						Leg[j].world_X_NRP = np.dot(XHy_world_X_base, Leg[j].base_X_NRP)
						Leg[j].world_base_X_NRP[:3,3:4] = mX((XHy_world_X_base[:3,:3]), Leg[j].base_X_NRP[:3,3:4])
						
					else:
						Leg[j].update_world_base_X_NRP(P6d_world_X_base)
					
					## 2) Compute magnitude & vector direction
					# if (j==1):
					# 	print j, ' vcp3: ', np.round(v3cp.flatten(),3)
					# 	print j, ' dwa : ', di_wall, np.round(dt_base ,3)
					# 	print j, ' wXn : ', np.round(Leg[j].world_X_NRP[:3,3],3)
					# 	print j, ' grp : ', ( (int(np.floor(Leg[j].world_X_NRP[0,3]/self.resolution))), 
					# 							(int(np.ceil(Leg[j].world_X_NRP[1,3]/self.resolution))) )
					# 	print j, ' wXbn: ', np.round(Leg[j].world_base_X_NRP[:3,3],3)

					# Get surface normal
					snorm = self.get_cell('norm', Leg[j].world_X_NRP[:3,3], j)

					v3_dv = (v3cp - v3cp_prev).flatten() 			# direction vector from previous to current CoB
					v3_pv = v3_dv - (np.dot(v3_dv,snorm))*snorm 	# project direction vector onto plane
					m1_dv = np.linalg.norm(v3_pv) 					# magnitude of direction vector
					v3_uv = np.nan_to_num(v3_pv/m1_dv) 				# unit vector direction

					## TEMP: overwrite last transfer phase on base spline
					if (i == len(base_path.X.t)):
						v3_uv = np.zeros(3)
					
					## 3) Compute AEP wrt base and world frame					
					Leg[j].world_base_X_AEP[:3,3] = Leg[j].world_base_X_NRP[:3,3] + (v3_uv*STEP_STROKE/2.)
					Leg[j].base_X_AEP[:3,3:4] = mX(XHd.base_X_world[:3,:3], Leg[j].world_base_X_AEP[:3,3:4])
					Leg[j].base_X_NRP[:3,3:4] = mX(XHd.base_X_world[:3,:3], Leg[j].world_base_X_NRP[:3,3:4])
					Leg[j].world_X_foot = mX(XHd.world_X_base, Leg[j].base_X_AEP)
					
					## Get cell height in (x,y) location of world_X_foot
					cell_h = np.array([0., 0., self.get_cell('cost', Leg[j].world_X_foot[:3,3], j)]) 	# cost=height
					
					# if (j==1):
					# 	print j, ' Transfer phase'
					# 	print j, ' grp : ', (Leg[j].world_X_NRP[0,3]/self.resolution), (Leg[j].world_X_NRP[1,3]/self.resolution)
					# 	print j, ' wXn : ', np.round(Leg[j].world_X_NRP[:3,3],3)
					# 	print j, ' wXbn: ', np.round(Leg[j].world_base_X_NRP[:3,3],3)
					# 	print j, ' wXba: ', np.round(Leg[j].world_base_X_AEP[:3,3],3)
					# 	print j, ' snorm ', snorm, cell_h, np.round(v3cp.flatten(),3)
					# 	print j, ' wXf:  ', np.round(Leg[j].world_X_foot[:3,3], 4)

					## Cell height above threshold gets ignored as this requires advanced motions
					if (cell_h.item(2) < 0.1):# and chim_trans is False and self.W_CHIM is False):
						Leg[j].world_X_foot[2,3] = cell_h.item(2) 	# set z-component to cell height

					## Check if foothold valid for chimney transition
					elif (self.T_GND_X_CHIM is True):

						## SIM DATA
						# if (Leg[j].world_X_foot[0,3]>0.3):
						cell_h[2] = -0.1

						dh = Leg[j].world_X_foot[2,3] - cell_h.item(2)
						# if (j==0):
						# 	print 'before: ', j, np.round(Leg[j].world_X_foot[:3,3],4)
						# 	print np.round(Leg[j].world_base_X_NRP[:3,3],4)
						if (dh > 0.001):
							compute_wall_footholds()
							## Recompute AEP wrt base and world frame					
							# Leg[j].world_base_X_AEP[:3,3] = Leg[j].world_base_X_NRP[:3,3].copy()
							Leg[j].world_base_X_AEP[:3,3] = Leg[j].world_base_X_NRP[:3,3] + (v3_uv*STEP_STROKE/2.)

							Leg[j].base_X_AEP[:3,3:4] = mX(XHd.base_X_world[:3,:3], Leg[j].world_base_X_AEP[:3,3:4])
							Leg[j].base_X_NRP[:3,3:4] = mX(XHd.base_X_world[:3,:3], Leg[j].world_base_X_NRP[:3,3:4])
							
							Leg[j].world_X_foot = mX(XHd.world_X_base, Leg[j].base_X_AEP)

						# if (j==0):
						# 	print 'after: ', j, np.round(Leg[j].world_X_foot[:3,3],4)
						# 	print np.round(Leg[j].world_base_X_NRP[:3,3],4)
							# print np.round(Leg[j].world_base_X_AEP[:3,3],4)

					elif (self.T_CHIM_X_GND is True):
						
						## SIM DATA
						if (Leg[j].world_X_foot[0,3]>0.86):
							cell_h[2] = 0.0
						else:
							cell_h[2] = -0.1

						if (cell_h[2] > -0.01):
							# set to default ground NRP
							KDL = kdl.KDL()
							Leg[j].update_base_X_NRP(KDL.leg_IK(LEG_STANCE[j])) 	
							Leg[j].world_base_X_NRP[:3,3:4] = mX(XHd.world_X_base[:3,:3], Leg[j].base_X_NRP[:3,3:4])

							# recompute AEP
							Leg[j].world_base_X_AEP[:3,3] = Leg[j].world_base_X_NRP[:3,3] + (v3_uv*STEP_STROKE/2.)
							Leg[j].base_X_AEP[:3,3:4] = mX(XHd.base_X_world[:3,:3], Leg[j].world_base_X_AEP[:3,3:4])
						
							Leg[j].world_X_foot = mX(XHd.world_X_base, Leg[j].base_X_AEP)

						# if (j==0):
						# 	print j, Leg[j].world_X_foot[0,3]
						# 	print np.round(Leg[j].world_base_X_AEP[:3,3],4)
					else:
						pass

					## Recompute base_X_AEP based on cell height
					Leg[j].base_X_AEP = mX(XHd.base_X_world, v3_X_m(Leg[j].world_X_foot[:3,3]))

					## Stack to array
					world_X_footholds[j].t.append(i*CTR_INTV)
					world_X_footholds[j].xp.append(Leg[j].world_X_foot[:3,3:4].copy())
					
					world_base_X_NRP[j].t.append(i*CTR_INTV)
					world_base_X_NRP[j].xp.append(Leg[j].world_base_X_NRP[:3,3:4].copy())

					base_X_footholds[j].t.append(i*CTR_INTV)
					base_X_footholds[j].xp.append(Leg[j].base_X_AEP[:3,3:4].copy())
					
					# if (j==1):
					# 	print j, ' snorm ', snorm, cell_h, self.W_WALL
					# 	# print 'v3pv: ', v3_pv.flatten()
					# 	# print 'v3uv: ', v3_uv.flatten()
						# print j, ' bXN:  ', np.round(Leg[j].base_X_NRP[:3,3],4)
						# print j, ' wXn : ', np.round(Leg[j].world_X_NRP[:3,3],3)
						# print j, ' wXf:  ', np.round(Leg[j].world_X_foot[:3,3], 4)
						# print j, ' wbXN: ', np.round(Leg[j].world_base_X_NRP[:3,3],4)
						# print j, ' wbXA: ', np.round(Leg[j].world_base_X_AEP[:3,3],4)
						# print 'wXN:  ', XHd.world_X_base[:3,3] + Leg[j].world_base_X_NRP[:3,3]
						# print 'wXA:  ', XHd.world_X_base[:3,3] + Leg[j].world_base_X_AEP[:3,3]
						# print 'wXb:  ', np.round(XHd.base_X_world,4)
						# 	print '--------------------------------------------'

			## Alternate gait phase
			Gait.change_phase()
			v3cp_prev = v3cp.copy()
			v3wp_prev = v3wp.copy()
		
		# Replace last footholds with default ground NRP
		if (self.T_WALL_X_GND is True or self.T_CHIM_X_GND is True):
			KDL = kdl.KDL()
			for j in range(0,6):
				Leg[j].update_base_X_NRP(KDL.leg_IK(LEG_STANCE[j]))
				base_X_footholds[j].xp[-1] = Leg[j].base_X_NRP[:3,3:4].copy()
				world_base_X_NRP[j].xp[-1] = Leg[j].base_X_NRP[:3,3:4].copy()
				world_X_footholds[j].xp[-1] = mX(XHd.world_X_base, Leg[j].base_X_NRP)[:3,3:4]
		print '============================================='
		## Smoothen base path
		# print gphase_intv
		glength = int(GAIT_TPHASE*CTR_RATE)+1 
		PathGenerator = Pathgenerator.PathGenerator()

		# for i in range(1,len(gphase_intv)-1):
		# 	# Identify phases to extend
		# 	if (gphase_intv[i] - gphase_intv[i-1] - glength < 0):
		# 		# print 'Extend', gphase_intv[i]*CTR_INTV, gphase_intv[i-1]*CTR_INTV
		# 		## Set initial and final position/orientation
		# 		## TODO: INCLUDE VELOCITY N ACCELERATION IN INTERPOLATION
		# 		xn_cob = base_path.X.xp[gphase_intv[i-1]]
		# 		xn_cob = np.vstack((xn_cob, base_path.X.xp[gphase_intv[i]]))

		# 		wn_cob = base_path.W.xp[gphase_intv[i-1]]
		# 		wn_cob = np.vstack((wn_cob, base_path.W.xp[gphase_intv[i]]))
				
		# 		## Generate new segment
		# 		new_segment = PathGenerator.generate_base_path(xn_cob, wn_cob, CTR_INTV, np.array([0.,GAIT_TPHASE]))
				
				## Append into existing trajectory
				# base_path.insert(gphase_intv[i-1],gphase_intv[i],new_segment)

		# Plot.plot_2d(base_path.X.t, base_path.X.xp)
		# print np.round(world_X_footholds[0].xp,3)
		# print 'in planner: ', np.round(world_X_footholds[0].xp,4)
		return set_motion_plan()