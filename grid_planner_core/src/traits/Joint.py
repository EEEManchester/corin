#!/usr/bin/env python

## Joint class for robot leg

import numpy as np

__all__ = ['Joint']

class Joint:
	def __init__(self, size):
		self.qpc 	= np.zeros(3) 		# joint position current
		self.qpd 	= np.zeros(3)		# joint position desired
		self.qpe 	= np.zeros(3)		# joint position error
		self.qvc	= np.zeros(3)		# joint velocity current 
		self.qvd	= np.zeros(3)		# joint velocity desired 
		self.qac 	= np.zeros(3) 		# joint acceleration current 
		self.qad 	= np.zeros(3) 		# joint acceleration desired 
		self.qtc 	= np.zeros(3)		# joint effort current 
		self.qtd 	= np.zeros(3)		# joint effort desired