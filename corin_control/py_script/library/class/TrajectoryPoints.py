#!/usr/bin/env python

## Class for 2D arrays for trajectory points

import numpy as np

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

class BaseTrajectory():
	def __init__(self,data=None):
		if data is None:
			self.X = TrajectoryPoints()	# linear
			self.W = TrajectoryPoints()	# angular
		else:
			self.X = TrajectoryPoints(data[0])
			self.W = TrajectoryPoints(data[1])