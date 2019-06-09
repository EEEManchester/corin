#!/usr/bin/env python

## Class for implementing impedance control

import sys; sys.dont_write_bytecode = True

import math
import numpy as np

from constant import * 								# constants used
from scipy import signal

class FaultController:

	def __init__(self):
		self.status = True
		self.xb = None
		self.xb_offset = None
		self.fault_index = None
		self.base_X_foot = None

		self.__default_parameters__()

	def __default_parameters__(self):

		self.xb = np.array([0.,0.,BODY_HEIGHT, 0.,0.,0.]).reshape((6,1))
		self.xb_offset = np.zeros((6,1))
		self.fault_index = [False, False, False, True, False, False]
		self.base_X_foot = [ np.array([0., 0., -BODY_HEIGHT]),
							 np.array([0., 0., -BODY_HEIGHT]),
							 np.array([0., 0., -BODY_HEIGHT]),
							 np.array([COXA_X, -(COXA_Y+STANCE_WIDTH), -BODY_HEIGHT]),
							 np.array([0., 0., -BODY_HEIGHT]),
							 np.array([0., 0., -BODY_HEIGHT])]

	def set_fault_parameters(self, fault_index, base_X_foot):

		self.fault_index = fault_index
		self.base_X_foot = base_X_foot

		self.get_fault_pose()

	def get_fault_pose(self):
		
		index_f = [i for i, j in enumerate(self.fault_index) if j == True]
		
		if index_f:
			havg = 0.
			for i in range(len(index_f)):
				pf = self.base_X_foot[index_f[i]]
				havg += pf[2]
			havg = -havg/len(index_f)
		else:
			havg = BODY_HEIGHT

		self.xb_offset = -self.xb + np.array([0.,0.,havg, 0.,0.,0.]).reshape((6,1))
		self.xb = np.array([0.,0.,havg, 0.,0.,0.]).reshape((6,1))

		return self.xb


## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##
Fault = FaultController()

fault_index = [False,False,False,False,False,True]
base_X_foot = [ np.array([0., 0., -BODY_HEIGHT]),
				np.array([0., 0., -0.05]),
				np.array([0., 0., -BODY_HEIGHT]),
				np.array([0., 0., -BODY_HEIGHT]),
				np.array([0., 0., -0.12]),
				np.array([0., 0., -BODY_HEIGHT])]

Fault.set_fault_parameters(fault_index, base_X_foot)

xb = Fault.get_fault_pose()
xb_off = Fault.xb_offset
# np.array([0., 0., -BODY_HEIGHT])
# print xb_off