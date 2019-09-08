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
		self.gait_phase = []				# gait phase
		self.f_world_X_foot = [None]*6 		# foothold wrt world frame
		self.f_base_X_foot  = [None]*6 		# foothold wrt base frame
		self.f_world_base_X_NRP = [None]*6 	# NRP from base frame wrt world frame
		self.surface_normals = [None]*6

		self.gait_in = 0
		self.gait_fn = 0

		self.__initialise__()

	def __initialise__(self):
		
		for j in range(0,6):
			self.f_world_X_foot[j] = MarkerList()
			self.f_base_X_foot[j]  = MarkerList()
			self.f_world_base_X_NRP[j] = MarkerList()
			self.surface_normals[j] = []

	def set_base_path(self, qbias, qb, qbp, gphase):
		""" Set base path parameters """

		self.qb_bias = qbias.copy()
		self.qb  = qb 		## TODO: duplicate this
		self.qbp = list(qbp)
		if gphase is not None:
			self.gait_phase = list(gphase)
		else:
			self.gait_phase = None

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

	def set_surface_normals(self, snorms):
		self.surface_normals = snorms
		
	def append(self, plan):
		""" Appends similar object to existing plan """
		
		# self.qb_bias = plan.qb_bias.copy()
		self.qb.append(plan.qb)
		self.qbp += plan.qbp
		self.gait_phase += plan.gait_phase

		for j in range(0,6):
			self.f_world_X_foot[j].append(plan.f_world_X_foot[j])
			self.f_base_X_foot[j].append(plan.f_base_X_foot[j])
			self.f_world_base_X_NRP[j].append(plan.f_world_base_X_NRP[j])

	def reverse_motion(self):
		""" Reverse the motion plan generated """

		self.qb.reverse()
		self.qbp = list(reversed(self.qbp))
		self.gait_phase = list(reversed(self.gait_phase))
		
		for j in range(0,6):
			self.f_world_X_foot[j].reverse()
			self.f_base_X_foot[j].reverse()
			self.f_world_base_X_NRP[j].reverse()