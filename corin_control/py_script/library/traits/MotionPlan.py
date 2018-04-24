#!/usr/bin/env python

## Class which holds the entire motion plan

import numpy as np
from TrajectoryPoints import *
from MarkerList import *
import copy

__all__ = ['MotionPlan']

class MotionPlan():
	def __init__(self):
		self.qb_bias = np.zeros((6,1)) 		# offset for robot's base
		self.qb  = Trajectory6D() 			# base path trajectory
		self.qbp = [] 						# base location where gait phase changes
		self.f_world_X_foot = [None]*6 		# foothold wrt world frame
		self.f_base_X_foot  = [None]*6 		# foothold wrt base frame
		self.f_world_base_X_NRP = [None]*6 	# NRP from base frame wrt world frame

		self.gait_in = 0
		self.gait_fn = 0

		self.__initialise__()

	def __initialise__(self):
		
		for j in range(0,6):
			self.f_world_X_foot[j] = MarkerList()
			self.f_base_X_foot[j]  = MarkerList()
			self.f_world_base_X_NRP[j] = MarkerList()

	def set_base_path(self, qbias, qb, qbp):
		""" Set base path parameters """

		self.qb_bias = qbias.copy()
		self.qb  = qb 		## TODO: duplicate this
		self.qbp = list(qbp)

	def set_footholds(self, wXf, bXf, bXN):
		""" Sets the foothold for each """
		
		for j in range(0,6):
			self.f_world_X_foot[j].t  = list(wXf[j].t)
			self.f_world_X_foot[j].xp = list(wXf[j].xp)

			self.f_base_X_foot[j].t  = list(bXf[j].t)
			self.f_base_X_foot[j].xp = list(bXf[j].xp)

			self.f_world_base_X_NRP[j].t  = list(bXN[j].t)
			self.f_world_base_X_NRP[j].xp = list(bXN[j].xp)
			
	def set_gait(self, gait_initial, gait_final):
		self.gait_in = gait_initial
		self.gait_fn = gait_final

	def append(self, plan):
		""" Appends similar object to existing plan """
		
		self.qb_bias = plan.qb_bias.copy()
		self.qb.append(plan.qb)
		self.qbp += plan.qbp

		for j in range(0,6):
			self.f_world_X_foot.append(plan.f_world_X_foot)
			self.f_base_X_foot.append(plan.f_base_X_foot)
			self.f_world_base_X_NRP.append(plan.f_world_base_X_NRP)