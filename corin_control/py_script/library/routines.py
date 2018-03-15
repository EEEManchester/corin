#!/usr/bin/env python

""" Main controller for the robot """
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
	## TODO: move elsewhere

	# Variables
	fix_stance = 0.18;
	fix_height = 0.01
	leg_stance = {}
	leg_seq = range(0,6)
	leg_phase = [1,1,1,1,1,1]

	# stance for air suspension
	leg_stance[0] = np.array([ fix_stance*np.cos(TETA_F*np.pi/180), fix_stance*np.sin(TETA_F*np.pi/180), fix_height ])
	leg_stance[1] = np.array([ fix_stance, 0, fix_height])
	leg_stance[2] = np.array([ fix_stance*np.cos(TETA_R*np.pi/180), fix_stance*np.sin(TETA_R*np.pi/180), fix_height ])
	leg_stance[3] = np.array([ fix_stance*np.cos(TETA_F*np.pi/180), fix_stance*np.sin(-TETA_F*np.pi/180), fix_height ])
	leg_stance[4] = np.array([ fix_stance, 0, fix_height])
	leg_stance[5] = np.array([ fix_stance*np.cos(TETA_R*np.pi/180), fix_stance*np.sin(-TETA_R*np.pi/180), fix_height ])

	return (leg_seq, leg_stance, leg_phase, 3)

def routine_shuffle(self):
	""" routine to shuffle legs back to NRP """

	## Define Variables ##
	leg_stance = [None]*6

	for j in range(0,self.Robot.active_legs):
		if (j%2 == 0): 	# even number legs
			self.Robot.Leg[j].update_from_spline();
			self.Robot.Leg[j].spline_counter += 1
		else: 			# odd number legs
			self.Robot.Leg[j].hip_X_ee.ds.xp = self.Robot.Leg[j].hip_X_ee.cs.xp

	# already in stand up position, shuffle in tripod fashion
	if (stand_state):
		for npoint in range(0, int(TRAC_PERIOD/CTR_INTV)):
			for j in range(0,self.Robot.active_legs):
				# set cartesian position for joint kinematics
				if (j%2 == 0): 	# even number legs
					self.Robot.Leg[j].update_from_spline();
					self.Robot.Leg[j].spline_counter += 1
				else: 			# odd number legs
					self.Robot.Leg[j].hip_X_ee.ds.xp = self.Robot.Leg[j].hip_X_ee.cs.xp
			qd = self.Robot.task_X_joint()
			self.publish_topics(qd)

		self.Robot.update_state()
		for npoint in range(0, int(TRAC_PERIOD/CTR_INTV)):
			for j in range(0,self.Robot.active_legs):
				# set cartesian position for joint kinematics
				if (j%2 == 1): 	# even number legs
					self.Robot.Leg[j].update_from_spline();
					self.Robot.Leg[j].spline_counter += 1
				else: 			# odd number legs
					self.Robot.Leg[j].hip_X_ee.ds.xp = self.Robot.Leg[j].hip_X_ee.cs.xp

			qd = self.Robot.task_X_joint()
			self.publish_topics(qd)

	return (leg_seq, leg_stance, leg_phase, 3)