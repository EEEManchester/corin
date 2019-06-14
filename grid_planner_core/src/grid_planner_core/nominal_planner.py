""" Grid based motion planning using NetworkX
""" 

import sys; sys.dont_write_bytecode = True
sys.path.insert(0, '/home/wilson/catkin_ws/src/corin/corin_control/src')
from corin_control import *			# library modules to include 
from grid_map import *

import networkx as nx
import matplotlib.pyplot as plt

import numpy as np
import math
from fractions import Fraction
from collections import OrderedDict 	# remove duplicates
# import plotgraph as Plot

import csv

class PathPlanner:
	def __init__(self, Map):
		self.GridMap = Map
		

	def motion_planning(self, start, end, Robot=None):
		""" Plans path by interpolating using nominal foothold approach """

		def set_motion_plan():
			## Set MotionPlan class
			motion_plan = MotionPlan()
			motion_plan.set_base_path(start, path, world_X_base, gait_phase)
			motion_plan.set_footholds(world_X_footholds, base_X_footholds, world_base_X_NRP)
			# print 'planner: ', (motion_plan.f_world_X_foot[0].xp)
			return motion_plan

		# Variables
		d_nom = 0.05 	# distance for legs to move into nominal position
		ps = start.flatten()	# world frame
		pf = end.flatten()	# world frame
		print 'start: ',ps.flatten()
		print ' end: ', pf.flatten()

		Gait = gait_class.GaitClass(GAIT_TYPE)	# gait class

		world_X_base = [start.flatten()]
		world_X_footholds = [None]*6
		base_X_footholds  = [None]*6
		world_base_X_NRP  = [None]*6
		gphase_intv = []
		gait_phase = []

		## Instantiate leg transformation class & append initial footholds
		for j in range (0, 6):
			world_X_footholds[j] = MarkerList()
			base_X_footholds[j] = MarkerList()
			world_base_X_NRP[j] = MarkerList()
		
		w_base_X_foot = self.max_stride_foothold(Robot)

		# Compute magnitude
		d_dif = (pf - ps)
		d_mag = np.linalg.norm(d_dif[0:2])
		
		if GAIT_TYPE == 1:
			lamb = STEP_STROKE/(5./6.)
		elif GAIT_TYPE == 2:
			lamb = STEP_STROKE/4.
		elif GAIT_TYPE == 4:
			lamb = STEP_STROKE/(1./2.)

		d_nom = lamb/2.
		# Intermediate point for moving legs to nominal
		ntemp = d_mag/d_nom
		ptemp = ps #+ d_dif/ntemp
		
		# Max. stride foothold - fixed wrt base
		# world_X_base.append(ptemp.flatten())
		# gphase_intv.append(1)
		gait_phase.append(Gait.phases)

		for j in range(0,6):
			world_X_foot = ptemp[0:3] + w_base_X_foot[j]
			world_X_footholds[j].t.append(1)
			world_X_footholds[j].xp.append(world_X_foot.copy())
			
			world_base_X_NRP[j].t.append(1)
			world_base_X_NRP[j].xp.append(Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4].copy())

			base_X_footholds[j].t.append(1)
			base_X_footholds[j].xp.append(w_base_X_foot[j].copy())
		
		# Flattens list
		gait_phase = [val for sublist in gait_phase for val in sublist]

		# Interpolate following leg in max. stride position
		# nintv = math.ceil((d_mag-d_nom)/lamb)
		nintv = math.ceil((d_mag)/lamb)

		if not Robot.Fault.status:

			for i in range(1,int(nintv)+1):
				p_intv = ps + d_dif/nintv*i

				world_X_base.append(p_intv.flatten())
				gait_phase.append(Gait.phases)
				print p_intv

			# Spline base path
			path = self.spline_path(world_X_base)
			self.generate_linear_path(world_X_base)
			print 'this len', len(path.X.t), path.X.t
			# for j in range(0,6):
			# 	# world_X_foot = p_intv[0:3] + Robot.Leg[j].XHd.world_base_X_AEP[:3,3]
			# 	world_X_foot = p_intv[0:3] + w_base_X_foot[j]
			# 	world_X_footholds[j].t.append(1+i)
			# 	world_X_footholds[j].xp.append(world_X_foot.copy())
				
			# 	world_base_X_NRP[j].t.append(i+1)
			# 	world_base_X_NRP[j].xp.append(Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4].copy())

			# 	base_X_footholds[j].t.append(i+1)
			# 	base_X_footholds[j].xp.append(w_base_X_foot[j].copy())
		
		## CoB at every gait cycle interval
		elif Robot.Fault.status:
			
			# Set CoB and gait phases for moving legs into max. stride position
			t_cob = [0]*(len(Gait.phases)+1)
			
			for j in range(1,len(Gait.phases)+1):
				t_cob[j] = GAIT_TPHASE*j
				world_X_base.append(ps.copy())

				if Robot.Fault.fault_index[j-1]:
					gait_phase[5] = [0]*6
			
			# Base motion during discontinuous phase
			fault_offset_motion = np.array([0.,0.,0.05,0.,0.,0.]) # Ground walking
			# fault_offset_motion = np.array([0.,0.04,0.,0.,0.,0.]) # Chimney walking
			
			# no. of intervals increase as base only translate for one phase per gait cycle
			# nintv = math.ceil((d_mag)/(lamb/len(Gait.phases)))
			nintv = math.ceil((d_mag)/lamb)
			pintv = ps
			# Cycle through each gait phase
			for i in range(1, int(nintv)*len(Gait.phases)+1):
				# Fault leg in transfer phase: moves base
				if Robot.Fault.fault_index[Gait.cs.index(1)]:
					# Intermediate position for clearing contact surface 
					pintv += d_dif/nintv
					p_fault = pintv + fault_offset_motion - d_dif/(2*nintv)
					
					world_X_base.append(p_fault.flatten().copy())
					t_cob.append(GAIT_TPHASE*i + t_cob[len(Gait.phases)] - GAIT_TPHASE/2.)
					gait_phase.append([0]*6)

				# Working legs in transfer phase
				else:
					gait_phase.append(Gait.cs)
				
				world_X_base.append(pintv.flatten().copy())
				t_cob.append(GAIT_TPHASE*i + t_cob[len(Gait.phases)])
				Gait.change_phase()

			# Spline base path
			# path_old = self.spline_path(world_X_base, t_cob)
			path = self.generate_linear_path(world_X_base, t_cob)

		# Plot.plot_2d(path.X.t, path.X.xp)
		# Plot.plot_2d(path_old.X.t, path_old.X.xp)
		# Plot.plot_2d(path.W.t,path.W.xp)

		# Include initial foothold position
		for j in range(0,6):
			world_X_footholds[j].xp.insert(0, Robot.Leg[j].XHc.world_X_foot[:3,3].flatten().copy())
			base_X_footholds[j].xp.insert(0, Robot.Leg[j].XHc.base_X_foot[:3,3].flatten().copy())
			world_base_X_NRP[j].xp.insert(0, Robot.Leg[j].XHc.world_base_X_NRP[:3,3].flatten().copy())
			
		# Flattens list
		# gait_phase = [val for sublist in gait_phase for val in sublist]
		# print gait_phase
		
		return set_motion_plan()

	def max_stride_foothold(self, Robot):
		""" Compute footholds wrt base position for maximum stride """

		world_base_X_foot = []
		if GAIT_TYPE == 1:
			sint = STEP_STROKE/5
			for j in range(0,6):
				foothold = Robot.Leg[j].XHc.world_base_X_AEP[:3,3] - np.array([j*sint, 0., 0.])
				world_base_X_foot.append(foothold)
				
		# print world_base_X_foot
		# print self.Robot.Leg[5].XHc.world_base_X_AEP[:3,3]
		# print self.Robot.Leg[5].XHc.world_base_X_PEP[:3,3]
		return world_base_X_foot

	def spline_path(self, world_X_base, t_cob=None):

		x_cob = np.zeros((len(world_X_base),3))
		w_cob = np.zeros((len(world_X_base),3))
		
		for i in range(0,len(world_X_base)):
			x_cob[i,:] = world_X_base[i][0:3] - world_X_base[0][0:3]
			w_cob[i,:] = world_X_base[i][3:6] - world_X_base[0][3:6]

		if t_cob is None:
			t_cob = np.zeros(len(world_X_base))
			for i in range(0,len(world_X_base)):
				t_cob[i] = i*Robot.Gait.tcycle
		
		path_generator = Pathgenerator.PathGenerator()
		path_generator.V_MAX = path_generator.W_MAX = 10
		
		return path_generator.generate_base_path(x_cob, w_cob, CTR_INTV, t_cob)

	def generate_linear_path(self, world_X_base, t_cob=None):
	
		x_cob = np.zeros((len(world_X_base),3))
		w_cob = np.zeros((len(world_X_base),3))
		
		for i in range(0,len(world_X_base)):
			x_cob[i,:] = world_X_base[i][0:3] - world_X_base[0][0:3]
			w_cob[i,:] = world_X_base[i][3:6] - world_X_base[0][3:6]

		if t_cob is None:
			t_cob = np.zeros(len(world_X_base))
			for i in range(0,len(world_X_base)):
				t_cob[i] = i*Robot.Gait.tcycle
		# print t_cob
		tintv = int(math.ceil(t_cob[-1]/CTR_INTV))+1
		xp = np.zeros((tintv,3));	xv = np.zeros((tintv,3));	xa = np.zeros((tintv,3))
		wp = np.zeros((tintv,3));	wv = np.zeros((tintv,3));	wa = np.zeros((tintv,3))
		t_inp = np.zeros(tintv)
		
		for i in range(tintv):
			t_inp[i] = i*CTR_INTV

		count = 0
		for i in range(1,len(world_X_base)):
			dp = world_X_base[i] - world_X_base[i-1]
			dt = t_cob[i] - t_cob[i-1]

			if abs(math.ceil(dt/CTR_INTV) - (dt/CTR_INTV)) > 0.6:
				itv = np.round(dt/CTR_INTV)
			else:
				itv = math.ceil(dt/CTR_INTV)
			
			if i == len(world_X_base)-1:
				itv = int(itv)+1
			for j in range(0, int(itv)):
				xp[count] = world_X_base[i-1][0:3] + j*dp[0:3]/(dt/CTR_INTV) - world_X_base[0][0:3]
				wp[count] = world_X_base[i-1][3:6] + j*dp[3:6]/(dt/CTR_INTV) - world_X_base[0][3:6]
				xv[count] = (xp[count] - xp[count-1])/CTR_INTV
				wv[count] = (wp[count] - wp[count-1])/CTR_INTV
				xa[count] = (xv[count] - xv[count-1])/CTR_INTV
				wa[count] = (wv[count] - wv[count-1])/CTR_INTV
				count += 1
		print 'Check spline count match: ', count, len(t_inp)
		
		# xp[-1] = world_X_base[-1][0:3]
		# wp[-1] = world_X_base[-1][3:6]
		# xp[-1] = world_X_base[-1][0:3]
		# wp[-1] = world_X_base[-1][3:6]
		# print count, len(t_inp), len(p_inp)
		# print p_inp
		# Plot.plot_2d(t_inp, xp)
		
		return Trajectory6D(((t_inp,xp,xv,xa),(t_inp,wp,wv,wa)))



## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##

## Create map and plan path
grid_map = GridMap('flat')
planner = PathPlanner(grid_map)
Robot = robot_class.RobotState()

# ps = np.array([10,13]) 
# pf = np.array([16,13])
ps = np.array([0.30, 0.39, 0.1, 0., 0., 0.]) 
pf = np.array([0.35, 0.39, 0.1, 0., 0., 0.]) 

Robot.P6c.world_X_base = ps
Robot.P6d.world_X_base = Robot.P6c.world_X_base.copy()
Robot.XHc.update_world_X_base(Robot.P6c.world_X_base)
Robot.init_robot_stance()

motion_plan = planner.motion_planning(ps, pf, Robot)


a = np.array([0.,0.,0.])
b = np.array([1.,0.,0.])
d = np.array([2.,0.,0.])
c = [a,b,d]
# t = [0., 1.0, 1.4]

# tintv = math.ceil(t[-1]/CTR_INTV)
# t_intv = []
# for i in range(int(tintv)+1):
# 	t_intv.append(i*CTR_INTV)

# p_inp = []
# for i in range(1,len(c)):
# 	dp = c[i] - c[i-1]
# 	dt = t[i] - t[i-1]
	
# 	# print 'p ', c[i-1], c[i]
# 	# print 'dp ', dp, dt, CTR_INTV, dt/CTR_INTV, math.ceil(dt/CTR_INTV)
# 	# print 't ', int(math.ceil(dt/CTR_INTV))+1
# 	# print 'a ', dp/float((dt/CTR_INTV))

# 	for j in range(0, int(math.ceil(dt/CTR_INTV))):
# 		temp = c[i-1] + j*dp/(dt/CTR_INTV)
# 		p_inp.append(temp)
		
# p_inp.append(c[-1])
# print p_inp
# print len(p_inp), len(t_intv)