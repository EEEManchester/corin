#!/usr/bin/env python
""" Implements a sliding gain """

from termcolor import colored
from scipy import signal

class SlidingGain:
	def __init__(self):
		self.step_num  = 40
		self.step_size = 0.1

		self.reset()

	def reset(self):

		self.step_size = 1./self.step_num
		self.buffer = [i*self.step_size for i in range(1, self.step_num)]

	def get_gain(self):
		""" Returns the gain """

		try:
			return self.buffer.pop(0)
		except:
			return 1.

# class SlidingGain3D:
# 	def __init__(self):
# 		self.gx = SlidingGain()
# 		self.gy = SlidingGain()
# 		self.gz = SlidingGain()

# 	def get_gains(self):
# 		""" Input	- p: data, Re^3 
# 			Output 	- filtered output, Re^3 """

# 		return np.array([ self.fx.filter(p[0]),
# 							self.fy.filter(p[1]),
# 							self.fz.filter(p[2]) ])

## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##
# gain = SlidingGain()
# gain.reset()
# print gain.buffer
# for i in range(0,15):
# 	print gain.get_gain()