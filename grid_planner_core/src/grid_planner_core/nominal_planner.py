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
		self.Robot 	= robot_class.RobotState()

		self.max_stride_foothold()

	def motion_planning(self, start, end):
		""" Plans path by interpolating using nominal foothold approach """

		def set_motion_plan():
			## Set MotionPlan class
			motion_plan = MotionPlan()
			motion_plan.set_base_path(start, None, world_X_base, None)
			motion_plan.set_footholds(world_X_footholds, base_X_footholds, world_base_X_NRP)

			return motion_plan

		# Variables
		d_nom = 0.05 	# distance for legs to move into nominal position
		# ps = start*self.GridMap.resolution
		# pf = end*self.GridMap.resolution
		print 'start: ', ps
		print ' end: ', pf
		world_X_base = [start]
		world_X_footholds = [None]*6
		base_X_footholds  = [None]*6
		world_base_X_NRP  = [None]*6
		
		## Instantiate leg transformation class & append initial footholds
		for j in range (0, 6):
			world_X_footholds[j] = MarkerList()
			base_X_footholds[j] = MarkerList()
			world_base_X_NRP[j] = MarkerList()
		
		w_base_X_foot = self.max_stride_foothold()

		# Compute magnitude
		d_dif = (pf - ps)
		d_mag = np.linalg.norm(d_dif[0:2])
		
		if GAIT_TYPE == 1:
			lamb = STEP_STROKE/6
		elif GAIT_TYPE == 2:
			lamb = STEP_STROKE/5

		# Intermediate point for moving legs to nominal
		ntemp = d_mag/d_nom
		ptemp = ps + d_dif/ntemp
		
		# Max. stride foothold - fixed wrt base
		world_X_base.append(ptemp.flatten())

		for j in range(0,6):
			world_X_foot = ptemp[0:3] + w_base_X_foot[j]
			world_X_footholds[j].t.append(1)
			world_X_footholds[j].xp.append(world_X_foot.copy())
		
		# world_base_X_NRP[j].t.append(1)
		# world_base_X_NRP[j].xp.append(self.Robot.Leg[j].XHd.world_base_X_NRP[:3,3:4].copy())

		# base_X_footholds[j].t.append(1)
		# base_X_footholds[j].xp.append(self.Robot.Leg[j].XHd.base_X_AEP[:3,3:4].copy())

		# Interpolate following leg in max. stride position
		nintv = math.ceil((d_mag-d_nom)/lamb)

		for i in range(1,int(nintv)+1):
			p_intv = ps + d_dif/nintv*i

			for j in range(0,6):
				world_X_foot = p_intv[0:3] + w_base_X_foot[j]
				world_X_footholds[j].t.append(1+i)
				world_X_footholds[j].xp.append(world_X_foot.copy())

			world_X_base.append(p_intv.flatten())
		
		return set_motion_plan()

	def max_stride_foothold(self):
		""" Compute footholds wrt base position for maximum stride """

		world_base_X_foot = []
		if GAIT_TYPE == 1:
			sint = STEP_STROKE/5
			for j in range(0,6):
				foothold = self.Robot.Leg[j].XHc.world_base_X_AEP[:3,3] - np.array([j*sint, 0., 0.])
				world_base_X_foot.append(foothold)
				# print np.round(foothold,3)
		
		# print world_base_X_foot
		# print self.Robot.Leg[5].XHc.world_base_X_AEP[:3,3]
		# print self.Robot.Leg[5].XHc.world_base_X_PEP[:3,3]
		return world_base_X_foot



## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##

## Create map and plan path
grid_map = GridMap('flat')
planner = PathPlanner(grid_map)

# ps = np.array([10,13]) 
# pf = np.array([16,13])
ps = np.array([0.30, 0.39, 0.1, 0., 0., 0.]) 
pf = np.array([0.48, 0.39, 0.1, 0., 0., 0.]) 
planner.motion_planning(ps, pf)