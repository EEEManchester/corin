#!/usr/bin/env python

## Class for implementing impedance control

import sys; sys.dont_write_bytecode = True

import math
import numpy as np

from constant import * 								# constants used
from scipy import signal

class FaultController:

	def __init__(self):
		self.status = False
		self.xb = None
		self.xb_offset = None
		self.fault_index = None
		self.base_X_foot = None

		self.__default_parameters__()
		# self.fault_set_single()
		# self.fault_set_double()

	def __default_parameters__(self):

		self.xb = np.array([0.,0.,BODY_HEIGHT, 0.,0.,0.]).reshape((6,1))
		self.xb_offset = np.zeros((6,1))
		self.fault_index = [False, False, False, False, False, False]
		self.base_X_foot = [np.array([ 0.25,  0.251, -BODY_HEIGHT]),
							np.array([ 0.,    0.300, -BODY_HEIGHT]),
							np.array([-0.25,  0.251, -BODY_HEIGHT]),
							np.array([ 0.25, -0.251, -BODY_HEIGHT]),
							np.array([ 0.,   -0.300, -BODY_HEIGHT]),
							np.array([-0.25, -0.251, -BODY_HEIGHT])]

	def fault_set_single(self):

		self.status = True
		self.fault_index = [False, False, False, False, True, False]	
		self.base_X_foot = [np.array([ 0.25,  0.251, -BODY_HEIGHT]),
							np.array([ 0.,    0.300, -BODY_HEIGHT]),
							np.array([-0.25,  0.251, -BODY_HEIGHT]),
							np.array([ 0.25, -0.251, -BODY_HEIGHT]),
							np.array([ 0.,   -0.210, -0.05]),
							np.array([-0.25, -0.251, -BODY_HEIGHT])]
							
	def fault_set_double(self):

		self.status = True
		self.fault_index = [True, False, False, False, True, False]
		self.base_X_foot = [np.array([ 0.25,  0.29, -BODY_HEIGHT-0.05]),
							np.array([ 0.,    0.300, -BODY_HEIGHT]),
							np.array([-0.25,  0.251, -BODY_HEIGHT]),
							np.array([ 0.25, -0.251, -BODY_HEIGHT]),
							np.array([ 0.,   -0.21, -0.05]),
							np.array([-0.25, -0.251, -BODY_HEIGHT])]		

	def set_fault_parameters(self, fault_index, base_X_foot):

		self.fault_index = fault_index
		self.base_X_foot = base_X_foot

		self.get_fault_pose()

	def check_feasible_fault(self):
		""" Rejects fault legs that are in sequence """
		
		if self.fault_index.count(True) <= 3:
			if (self.fault_index[0] and self.fault_index[1]) or \
				(self.fault_index[1] and self.fault_index[2]):
				return False
			if (self.fault_index[3] and self.fault_index[4]) or \
				(self.fault_index[4] and self.fault_index[5]):
				return False
		else:
			return False

		return True # Fault feasible

	def get_fault_pose(self):

		if self.check_feasible_fault():
			print 'Fault feasible, calculating new pose...'
			fidx = [i for i, j in enumerate(self.fault_index) if j == True]

			# TODO: Fault leg selection
			if self.fault_index.count(True) == 3:
				# LHS two fault legs
				if self.fault_index[0:3].count(True) > self.fault_index[3:6].count(True):
					s1 = self.base_X_foot[fidx[0]]
					s2 = self.base_X_foot[fidx[1]]
					anchor_leg = self.base_X_foot[fidx[2]]
				# RHS two fault legs
				else:
					s1 = self.base_X_foot[fidx[1]]
					s2 = self.base_X_foot[fidx[2]]
					anchor_leg = self.base_X_foot[fidx[0]]

			elif self.fault_index.count(True) == 2:
				# If all legs on same side, use opposite middle leg as anchor
				# All legs on LHS
				if max(fidx) < 3:
					s1 = self.base_X_foot[fidx[0]]
					s2 = self.base_X_foot[fidx[1]]
					anchor_leg = self.base_X_foot[4]
				# All legs on RHS
				elif min(fidx) > 3:
					s1 = self.base_X_foot[fidx[0]]
					s2 = self.base_X_foot[fidx[1]]
					anchor_leg = self.base_X_foot[1]
				# Legs on opposing side
				else:
					s1 = self.base_X_foot[fidx[0]]
					s2 = self.base_X_foot[fidx[1]]
					if fidx[0] != 2:
						anchor_leg = self.base_X_foot[2]
					elif fidx[0] != 5:
						anchor_leg = self.base_X_foot[5]
					else:
						s1 = self.base_X_foot[3]
						anchor_leg = self.base_X_foot[fidx[1]]

			elif self.fault_index.count(True) == 1:
				anchor_leg = self.base_X_foot[self.fault_index.index(True)]
				if self.fault_index[0:3].count(True):
					s2 = self.base_X_foot[3]
					s1 = self.base_X_foot[5]
				elif self.fault_index[3:6].count(True):
					s1 = self.base_X_foot[0]
					s2 = self.base_X_foot[2]

			# print anchor_leg
			e1 = s1 - anchor_leg
			e2 = s2 - anchor_leg
			R1 = gram_schmidt(e1,e2)

			rpy = np.array(euler_from_matrix(R1.T,'sxyz'))
			xyz = np.dot(R1.T, anchor_leg)

			if rpy[0] > np.pi/2:
				rpy[0] = rpy[0] - np.pi
				rpy[1] *= -1.
			elif rpy[0] < -np.pi/2:
				rpy[0] = rpy[0] + np.pi
				rpy[1] *= -1.

			# Overwrite and correction
			xyz[2] = abs(xyz[2])
			rpy[2] = 0.
			self.xb = np.hstack((xyz,rpy)).reshape((6,1))

		else:
			print 'ERROR: No feasible statically stable fault configuration'

		return self.xb
## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##
Fault = FaultController()

# Single leg
Fault.status = True
Fault.fault_index[3] = True
Fault.fault_index[5] = True

Fault.base_X_foot[0] = np.array([ 0.25,  0.251, -BODY_HEIGHT])
Fault.base_X_foot[1] = np.array([ 0.,    0.300, -BODY_HEIGHT])
Fault.base_X_foot[2] = np.array([-0.25,  0.251, -BODY_HEIGHT])
Fault.base_X_foot[3] = np.array([ 0.25, -0.251, -0.15])
Fault.base_X_foot[4] = np.array([ 0.,   -0.300, -BODY_HEIGHT])
Fault.base_X_foot[5] = np.array([-0.25, -0.251, -BODY_HEIGHT])

# Fault.set_fault_parameters(fault_index, base_X_foot)
# Fault.fault_index = [True, False, True, True, False, False]

# xb = Fault.get_fault_pose().flatten()
# print xb
# print 'roll: \t', np.round(np.rad2deg(xb[3]),3)
# print 'pitch: \t', np.round(np.rad2deg(xb[4]),3)
