#!/usr/bin/env python

## Class for implementing impedance control

import sys; sys.dont_write_bytecode = True

import math
import numpy as np

from constant import * 								# constants used
from scipy import signal
from termcolor import colored

class PIDController:

	def __init__(self, ctrl_type='P', vec=6, kp=0, ki=0, kd=0):
		self.error  = [np.zeros((vec,1))]*3
		self.output = np.zeros((vec,1))
		self.size = vec
		self.type = ctrl_type
		self.Kp = kp
		self.Ki = ki
		self.Kd = kd

	def reset(self):
		# Reset controller
		self.error  = [np.zeros((self.size,1))]*3
		self.output = np.zeros((self.size,1))

	def update(self, e):

		self.error.pop()
		self.error.insert(0,e)
		
		if self.type == 'P':
			self.output = self.Kp*self.error[0]
			
		elif self.type == 'PI':
			self.output += self.Kp*(self.error[0]-self.error[1]) + \
								(CTR_INTV*self.Ki/2)*(self.error[0]+self.error[1])
			pout = self.Kp*(self.error[0]-self.error[1])
			iout = (CTR_INTV*self.Ki/2)*(self.error[0]+self.error[1])
			# print 'e1: ', np.round(self.error[0].flatten(),4)
			# print 'e2: ', np.round(self.error[1].flatten(),4)
			# print np.round(pout.flatten(),4)
			# print np.round(iout.flatten(),4)
			# print np.round(self.output.flatten(),4)
		else:
			print colored('Controller type %s not implemented'%self.type, 'red')

		return self.output 
		
## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##

# cont = PIDController('PI')
# p = np.array([0.,0.,15.,0.,0.,0.]).reshape((6,1))
# for i in range(10):
# 	print cont.update(p).flatten()
