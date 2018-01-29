#!/usr/bin/env python

## Class for 2D arrays for trajectory points

import numpy as np

class TrajectoryPoints():
	def __init__(self):
		self.xp = np.zeros((0,3))
		self.xv = np.zeros((0,3))
		self.xa = np.zeros((0,3))
		self.t  = []