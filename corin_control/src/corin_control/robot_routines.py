#!/usr/bin/env python

""" Defined routines for robot  """

__version__ = '1.0'
__author__ = 'Wei Cheah'

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'library'))
sys.dont_write_bytecode = True

import numpy as np
from constant import *

def air_suspend_legs():
	""" routine to move legs from current position to in the air """

	# Variables
	fix_stance = 0.18;
	fix_height = 0.01
	leg_stance = [None]*6
	leg_sequen = range(0,6)
	leg_phase  = [1,1,1,1,1,1]

	# stance for air suspension
	leg_stance[0] = np.array([ fix_stance*np.cos(TETA_F*np.pi/180), fix_stance*np.sin(TETA_F*np.pi/180), fix_height ])
	leg_stance[1] = np.array([ fix_stance, 0, fix_height])
	leg_stance[2] = np.array([ fix_stance*np.cos(TETA_R*np.pi/180), fix_stance*np.sin(TETA_R*np.pi/180), fix_height ])
	leg_stance[3] = np.array([ fix_stance*np.cos(TETA_F*np.pi/180), fix_stance*np.sin(-TETA_F*np.pi/180), fix_height ])
	leg_stance[4] = np.array([ fix_stance, 0, fix_height])
	leg_stance[5] = np.array([ fix_stance*np.cos(TETA_R*np.pi/180), fix_stance*np.sin(-TETA_R*np.pi/180), fix_height ])

	return (leg_sequen, leg_stance, leg_phase, 2.0)

def shuffle_legs(leg_stance):
	""" routine to shuffle legs back to NRP """
	print 'Shuffle legs'
	## Define Variables ##
	# leg_stance = LEG_STANCE
	leg_sequen = range(0,6)
	leg_phase  = [1,1,1,1,1,1]
	leg_period = [None]*6

	for j in range(0,6):
		if (j%2 == 0): 	# even number legs
			leg_period[j] = [0, GAIT_TPHASE]
		elif (j%2 == 1):
			leg_period[j] = [GAIT_TPHASE, GAIT_TPHASE*2]

	return (leg_sequen, leg_stance, leg_phase, leg_period)


## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##

# data = routine_shuffle()

# nleg, leg_stance, leg_phase, period = data

# te = np.amax(data[3])

# np = int(te/CTR_INTV+1)

# for p in range(0, np):
# 	# print p, np
# 	ti = p*CTR_INTV 	# current time in seconds
# 	for i in range(len(nleg)):
# 		j = nleg[i]
# 		# identify start and end of transfer trajectory
# 		if (ti >= period[j][0] and ti <= period[j][1]):
# 			pass
			# if (j==0 or j==1):
			# 	print j, period[j]