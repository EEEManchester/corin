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
		self.fault_index = [False, False, False, True, False, False]	
		self.base_X_foot = [np.array([ 0.25,  0.251, -BODY_HEIGHT]),
							np.array([ 0.,    0.300, -BODY_HEIGHT]),
							np.array([-0.25,  0.251, -BODY_HEIGHT]),
							np.array([ 0.25, -0.251, -BODY_HEIGHT]),
							np.array([ 0.,   -0.300, -BODY_HEIGHT]),
							np.array([-0.25, -0.251, -BODY_HEIGHT])]

	def fault_set_double(self):

		self.status = True
		self.fault_index = [False, True, False, False, False, True]
		self.base_X_foot = [np.array([ 0.25,  0.251, -BODY_HEIGHT]),
							np.array([ 0.,    0.300, -BODY_HEIGHT]),
							np.array([-0.25,  0.251, -BODY_HEIGHT]),
							np.array([ 0.25, -0.251, -BODY_HEIGHT]),
							np.array([ 0.,   -0.300, -BODY_HEIGHT]),
							np.array([-0.25, -0.251, -BODY_HEIGHT])]		

	def set_fault_parameters(self, fault_index, base_X_foot):

		self.fault_index = fault_index
		self.base_X_foot = base_X_foot

		self.get_fault_pose()

	def get_fault_pose(self):

		# TODO: Fault leg selection
		if self.fault_index[0:3].count(True) > 2:
			if self.fault_index[0:3].count(True) > self.fault_index[3:6].count(True):
				anchor_leg = self.base_X_foot[self.fault_index[3:6].index(True)]
			else:
				anchor_leg = self.base_X_foot[self.fault_index[0:3].index(True)]
		else:
			anchor_leg = self.base_X_foot[self.fault_index.index(True)]
		# print anchor_leg
		e1 = self.base_X_foot[0] - anchor_leg
		e2 = self.base_X_foot[2] - anchor_leg
		R1 = gram_schmidt(e1,e2)

		rpy = np.array(euler_from_matrix(R1.T,'sxyz'))
		xyz = np.dot(R1.T, anchor_leg)

		self.xb = np.hstack((xyz,rpy)).reshape((6,1))

		return self.xb
## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##
Fault = FaultController()

# Single leg
# Fault.status = True
# Fault.fault_index[4] = True
# Fault.base_X_foot[4] = np.array([ 0.,   -0.300, -0.15])

# Double leg
Fault.status = True
Fault.fault_index[0] = True
Fault.fault_index[4] = True
Fault.base_X_foot[4] = np.array([ 0.25,  0.251, -0.07])
Fault.base_X_foot[4] = np.array([ 0.,   -0.300, -0.15])

# Fault.set_fault_parameters(fault_index, base_X_foot)

print Fault.get_fault_pose().flatten()

gphase = [1,0,0,0,0,0]
# print Fault.fault_index[gphase.index(1)]
# np.array([0., 0., -BODY_HEIGHT])
# print xb_off


