#!/usr/bin/env python

""" Evaluates the stability margin of the robot """
import sys; sys.dont_write_bytecode = True
import numpy as np
from scipy import linalg

from constant import *

class StabilityMargin():
	def __init__(self):
		self.sm = np.zeros(2) 	# current stability margin

	def support_leg_identification(self, leg_phase):
		""" Identifies front and rear legs in support phase """ 
		p_state = []

		## Front SM
		if (leg_phase[0]==0):
			p_state.append(0)
		elif(leg_phase[1]==0):
			p_state.append(1)
		else:
			print "error"

		if (leg_phase[3]==0):
			p_state.append(3)
		elif(leg_phase[4]==0):
			p_state.append(4)
		else:
			print "error"

		## Rear SM
		if (leg_phase[2]==0):
			p_state.append(2)
		elif(leg_phase[1]==0):
			p_state.append(1)
		else:
			print "error"

		if (leg_phase[5]==0):
			p_state.append(5)
		elif(leg_phase[4]==0):
			p_state.append(4)
		else:
			print "error"
		return p_state

	def LSM(self, p, leg_phase):
		""" computes longitudinal stability margin 						"""
		""" Input  : 1) p -> array of foot positions: world_base_X_foot
				 	 2) leg_phase -> a list on the phase of legs
			Output : array of the front and rear magnitude of vector 
						perpendicular to support edge					"""

		## Define Variables ##
		sm = np.zeros(2)
		pf = self.support_leg_identification(leg_phase)
		# print 'support ', pf, leg_phase
		for i in range(0,2):
			p1 = p[pf[0+i*2]]
			p2 = p[pf[1+i*2]]
			
			v_lr  = p2 - p1 												# support edge vector
			p_p   = ( np.dot(-p1,v_lr)/np.linalg.norm(v_lr)**2 )*v_lr 		# foot vector projected onto support edge
			p_f   = p1 + p_p  												# rejection vector from base
			sm[i] = np.sqrt(p_f[0]**2 + p_f[1]**2) 							# magnitude of rejection vector
			# if (i==1):
			# 	print np.round(p1,4)
			# 	print np.round(p2,4)
		self.sm = sm
		return sm

	def force_moment(self):
		pass

## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##

BASE_X_LF_FOOT = np.array([1., 2., 0.1])
BASE_X_LM_FOOT = np.array([0., 2., 0.1])
BASE_X_LR_FOOT = np.array([-1., 2., 0.1])
BASE_X_RF_FOOT = np.array([1.,-2., 0.1])
BASE_X_RM_FOOT = np.array([0.,-2., 0.1])
BASE_X_RR_FOOT = np.array([-0.5,-2., 0.1])

Legs = [BASE_X_LF_FOOT, BASE_X_LM_FOOT, BASE_X_LR_FOOT,
		BASE_X_RF_FOOT, BASE_X_RM_FOOT, BASE_X_RR_FOOT]

leg_phase = [0,0,0,0,0,0]

SM = StabilityMargin()