#!/usr/bin/env python

## Class for 2D arrays for trajectory points

import numpy as np

__all__ = ['JointTrajectoryPoints','BaseTrajectory','TrajectoryPoints']

class JointTrajectoryPoints():
	def __init__(self,size=1,data=None):
		if data is None:
			self.t  = []
			self.xp = np.zeros((size,1))
			self.xv = np.zeros((size,1))
			self.xa = np.zeros((size,1))
		else:
			self.t  = np.array(data[0])
			self.xp = np.array(data[1])
			self.xv = np.array(data[2])
			self.xa = np.array(data[3])

	def append(self, x):
		pass

## output for base trajectory
class BaseTrajectory():
	def __init__(self,data=None):
		if data is None:
			self.X = TrajectoryPoints()	# linear
			self.W = TrajectoryPoints()	# angular
		else:
			self.X = TrajectoryPoints(data[0])
			self.W = TrajectoryPoints(data[1])
			
class TrajectoryPoints():
	def __init__(self,data=None):
		if data is None:
			self.t  = []
			self.xp = np.zeros((0,3))
			self.xv = np.zeros((0,3))
			self.xa = np.zeros((0,3))
		else:
			self.t  = np.array(data[0])
			self.xp = np.array(data[1])
			self.xv = np.array(data[2])
			self.xa = np.array(data[3])

	def append(self, x):
		pass



