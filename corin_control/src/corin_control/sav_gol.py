#!/usr/bin/env python
""" Implements the savitzy-golay filter for 1D and 3D """

from termcolor import colored
from scipy import signal

class SGfilter:
	def __init__(self):
		self.window = 5 		# filter window length
		self.degree = 2 		# polynomial degree
		self.mode = 'interp' 	# filter mode - 'interp', 'nearest', 'mirror'
		self.ori_history = [0]*10 	# history of original data
		self.fil_history = [0]*10 	# history of filtered data

	def filter(self, p):
		""" Input	- p: data, Re^1 
			Output 	- filtered output, Re^1 """

		# FILO buffer - data appended to the right
		self.ori_history.pop(0)
		self.ori_history.append(p)
		
		self.fil_history = signal.savgol_filter(self.ori_history, 
												self.window, 
												self.degree, 
												mode=self.mode)
		return self.fil_history[-1]

class SGfilter_3D:
	def __init__(self):
		self.fx = SGfilter()
		self.fy = SGfilter()
		self.fz = SGfilter()

	def filter(self, p):
		""" Input	- p: data, Re^3 
			Output 	- filtered output, Re^3 """

		return np.array([ self.fx.filter(p[0]),
							self.fy.filter(p[1]),
							self.fz.filter(p[2]) ])

## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##
filt = SGfilter()
filt.ori_history = [2, 2, 5, 2, 1, 0, 1, 4, 9, 9]
filt.ori_history = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
filt.window = 9
print filt.filter(0.044)
print filt.fil_history
# print signal.savgol_filter(filt.history, 5, 2)