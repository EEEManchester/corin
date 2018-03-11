#!/usr/bin/env python

## Class for common robotic terminologies

import numpy as np

class Wrench:
	def __init__(self):
		self.force  = np.zeros((3,1))
		self.torque = np.zeros((3,1))

class Twist:
	def __init__(self):
		self.linear  = np.zeros((3,1))
		self.angular = np.zeros((3,1))

class Pose:
	def __init__(self):
		self.position    = np.zeros((3,1))
		self.orientation = np.zeros((3,1))

class Joint_class:
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

a = [1,2,3]
b = np.array([1,2,3])

# try:
# 	for i in range(0,6):
# 		print i
# 		if (i==4):
# 			raise ValueError
# 			break
# except ValueError:
# 	print 'value error'