#!/usr/bin/env python

## Class for implementing impedance control

import sys; sys.dont_write_bytecode = True

import math
import numpy as np

from constant import * 								# constants used
from scipy import signal

class ImpedanceController:

	def __init__(self, fn=None, D=None, G=None):
		fs = CTR_RATE
		if fn == None:
			fn = IMPEDANCE_FN 		# Natural frequency (Hz) 0.5 3
		wn = 2*3.142*fn 		# in rad
		if D == None:
			D = IMPEDANCE_DAMPING 	# Damping ratio 0.7 1.5
		if G == None:
			G = IMPEDANCE_GAIN 				# Gain

		# Continuous transfer function
		H = ([G*wn*wn], [1, 2*D*wn, wn*wn])

		# Discretisation
		num, den, dt = signal.cont2discrete(H, 1.0/fs, method='bilinear') #zoh, euler
		self.num = num[0]
		self.den = den[1:]
		
		# Filter state initialisation
		self.forward_filter = [0] * len(self.num)
		self.reverse_filter = [0] * len(self.den)

		self.fn = fn
		self.D = D
		self.G = G

	def evaluate(self, df):
		# delta_force is additional force applied to virtual mass-spring-damper
		# = measured force - desired force

		# force = self.Robot.Leg[j].F6c.tibia_X_foot[2] - force_desired# z-axis

		self.forward_filter.pop()
		self.forward_filter.insert(0,df)

		output = 0
		for k in range(len(self.forward_filter)):
			output += self.num[k]*self.forward_filter[k]

		for k in range(len(self.reverse_filter)):
			output -= self.den[k]*self.reverse_filter[k]

		self.reverse_filter.pop()
		self.reverse_filter.insert(0,output)

		# offset = np.array([0, 0, output])

		return output

	def fly_evaluate(self, df):

		wn = 2*3.142*self.fn 
		# Continuous transfer function
		H = ([self.G*wn*wn], [1, 2*self.D*wn, wn*wn])

		# Discretisation
		num, den, dt = signal.cont2discrete(H, 1.0/CTR_RATE, method='bilinear') #zoh, euler
		self.num = num[0]
		self.den = den[1:]

		# Filter state initialisation
		self.forward_filter = [0] * len(self.num)
		self.reverse_filter = [0] * len(self.den)

		return self.evaluate(df)


## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##

# ic = ImpedanceController()
H = ([2], [1,2,5])
# print signal.cont2discrete(H, 0.1, method='bilinear')
# print signal.cont2discrete(H, 0.1)