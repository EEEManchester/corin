#!/usr/bin/env python

## Class for 2D arrays for trajectory points

import numpy as np
import copy

__all__ = ['JointTrajectoryPoints','Trajectory6D','TrajectoryPoints']

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

	def get_index(self, n):
		""" returns instant n of trajectory point """

		return self.xp[n], self.xv[n], self.xa[n]

	
## output for base trajectory
class Trajectory6D():
	def __init__(self,data=None):
		if data is None:
			self.X = TrajectoryPoints()	# linear
			self.W = TrajectoryPoints()	# angular
		else:
			self.X = TrajectoryPoints(data[0])
			self.W = TrajectoryPoints(data[1])

	def append(self,data):
		""" Appends to respective items assuming 
			input is of the same type 			"""
		
		self.X.t = self.X.t + data.X.t
		self.X.xp = np.vstack((self.X.xp, data.X.xp))
		self.X.xv = np.vstack((self.X.xv, data.X.xv))
		self.X.xa = np.vstack((self.X.xa, data.X.xa))
			
		self.W.t = self.W.t + data.W.t
		self.W.xp = np.vstack((self.W.xp, data.W.xp))
		self.W.xv = np.vstack((self.W.xv, data.W.xv))
		self.W.xa = np.vstack((self.W.xa, data.W.xa))

	def reverse(self):
		self.X.reverse()
		self.W.reverse()

	def insert(self,start,end,data):
		""" Removes items between start to end,
			replacing the items with data 		"""

		self.X.insert(start,end,data.X)
		self.W.insert(start,end,data.W)

class TrajectoryPoints():
	def __init__(self,data=None):
		if data is None:
			self.t  = []
			self.xp = np.zeros((0,3))
			self.xv = np.zeros((0,3))
			self.xa = np.zeros((0,3))
		else:
			self.t  = data[0]
			self.xp = np.array(data[1])
			self.xv = np.array(data[2])
			self.xa = np.array(data[3])

	def append(self, data):
		""" Stack data onto array assuming
			it is of the same data type 	"""
		
		self.t = self.t + data.t
		self.xp = np.vstack((self.xp, data.xp))
		self.xv = np.vstack((self.xv, data.xv))
		self.xa = np.vstack((self.xa, data.xa))

	def reverse(self):
		""" Reverses the sequence of the array """

		self.xp = self.xp[::-1]
		self.xv = self.xv[::-1]
		self.xa = self.xa[::-1]

	def insert(self,start,end,data):
		""" Removes items between start to end,
			replacing the items with data 		"""

		# Store back end of data
		temp = TrajectoryPoints()
		temp.t  = list(self.t[end:])
		temp.xp = self.xp[end:].copy()
		temp.xv = self.xv[end:].copy()
		temp.xa = self.xa[end:].copy()

		self.t  = self.t[0:start]
		self.xp = self.xp[0:start]
		self.xv = self.xv[0:start]
		self.xa = self.xa[0:start]
		
		self.append(data)
		self.append(temp)

		self.timing_correction()

	def timing_correction(self):
		""" Corrects the timing based on the intervals
			of the first and second items  				"""

		intv = self.t[1] - self.t[0]
		lent = len(self.t)

		self.t = np.arange(0., lent*intv, intv).tolist()