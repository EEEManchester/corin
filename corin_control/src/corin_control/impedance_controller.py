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

		self.prev_fe = np.zeros((1,3))
		self.prev_dfe = np.zeros((1,3))
		self.prev_qt = np.zeros((1,3))
		self.iqt = np.zeros((1,3))
		self.iqe = np.zeros((1,3))
		self.iqde = np.zeros((1,3))
		self.ie = np.zeros((1,3))
		self.x0 = np.zeros((1,3))
		self.g0 = np.zeros((1,3))
		self.kp0 = np.zeros((1,3))
		self.kd0 = np.zeros((1,3))

		self.prev_x = np.zeros((1,3))
		self.prev_dx = np.zeros((1,3))
		self.gt = np.zeros((1,3))
		self.kpt = np.zeros((1,3))
		self.kdt = np.zeros((1,3))
		self.qdx = np.zeros((1,3))
		self.kv0 = np.zeros((1,3))
		self.kvt = np.zeros((1,3))

	def reset(self):
		# Filter state initialisation
		self.forward_filter = [0] * len(self.num)
		self.reverse_filter = [0] * len(self.den)		

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

	def adaptive_evaluate(self, fd, f):

		WP = 0.001
		WD = 0.001
		A1 = 0.001
		A2 = 0.001
		B1 = 0.001
		B2 = 0.001
		G1 = 0.001
		G2 = 0.001

		fe = fd - f
		dfe = (fe-self.prev_fe)/CTR_INTV
		
		qt = WP*fe + WD*dfe
		self.iqt += CTR_INTV/2*(qt+self.prev_qt)
		gt = self.g0 + A2*qt + A1*self.iqt

		self.iqe += (CTR_INTV/2*(qt+self.prev_qt))*(CTR_INTV/2*(fe+self.prev_fe))
		kpt = self.kp0 + B1*self.iqe + B2*qt*fe

		self.iqde += (CTR_INTV/2*(qt+self.prev_qt))*(CTR_INTV/2*(dfe+self.prev_dfe))
		kdt = self.kd0 + G1*self.iqde + G2*qt*dfe

		kps = A1*WD + A2*WP + kpt
		kds = A2*WD + kdt
		kis = A1*WP

		self.ie += (CTR_INTV/2)*(fe+self.prev_fe)
		xf = self.x0 + kps*fe + kds*dfe + kis*self.ie

		self.prev_fe = fe.copy()
		self.prev_dfe = dfe.copy()
		self.prev_qt = qt.copy()

		return xf

	def adaptive_vel_evaluate(self, fd, f, x):

		WP = 0.001
		WD = 0.001
		A1 = 0.001
		A2 = 0.001
		B1 = 0.001
		B2 = 0.001
		S1 = 0.001
		S2 = 0.001
		S3 = 0.001
		Q1 = 0.001
		Q2 = 0.001

		fe = fd - f
		dx = (x - self.prev_x)/CTR_INTV

		qt = WP*fe - WD*dx
		self.iqt += CTR_INTV/2*(qt+self.prev_qt)
		gt = self.g0 + A2*qt + A1*self.iqt - S1*self.gt
		self.gt += CTR_INTV/2*gt

		self.iqe += (CTR_INTV/2*(qt+self.prev_qt))*(CTR_INTV/2*(fe+self.prev_fe))
		kpt = self.kp0 + B1*self.iqe + B2*qt*fe - S2*self.kpt
		self.kpt += CTR_INTV/2*kpt

		self.qdx += (CTR_INTV/2*(qt+self.prev_qt))*(CTR_INTV/2*(dx+self.prev_dx))
		kvt = self.kv0 - Q1*self.qdx - Q2*qt*dx - S3*self.kvt
		self.kvt += CTR_INTV/2*kvt
		
		xf = gt + kpt*fe - kvt*dx

		self.prev_fe = fe.copy()
		self.prev_qt = qt.copy()

		self.prev_x = x.copy()
		self.prev_dx = dx.copy()

		return xf

## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##

ic = ImpedanceController()
fd = np.array([0.,0.,10.])
f  = np.array([0.,0.,9.])
x  =  np.array([0.,0.,0.1])
xf = np.zeros((1,3))
# print ic.adaptive_evaluate(fd, f)
# print ic.adaptive_vel_evaluate(fd, f, x)
for i in range(0,100):
	# print xf
	xf = ic.adaptive_vel_evaluate(fd, f, x+xf)
print x+xf
# print signal.cont2discrete(H, 0.1, method='bilinear')
# print signal.cont2discrete(H, 0.1)