#!/usr/bin/env python

## Gait class for bio-inspired gaits
## Class with four gaits are available
## When called 'change phase', the phase of legs are returned
from fractions import Fraction
import numpy as np
import copy
from constant import *

class GaitClass:
	def __init__(self, gselect):
		self.type  = gselect 	# gait type: 1=wave, 2=ripple, 3=tetrapod, 4=wave
		self.phases = [] 		# list of gait phases
		self.np = 0 			# counter for gait phase
		self.tphase = GAIT_TPHASE	# timing per gait phase
		self.gd = 1 			# gait direction

		self.step_stroke = STEP_STROKE#self.set_step_stroke(LEG_STANCE, LEG_CLEAR, STEP_STROKE)
		self.step_height = STEP_HEIGHT

		# States
		self.cs = [0]*6	# current 
		self.ds = [0]*6	# desired 
		self.ps = [0]*6	# previous

		self.__initialise__()

	def __initialise__(self):
		""" Set gait phases """

		# Reset list
		self.phases = []

		# Wave Gait
		if self.type == 1:
			self.phases.append([0,0,0,0,0,1])
			self.phases.append([0,0,0,0,1,0])
			self.phases.append([0,0,0,1,0,0])
			self.phases.append([0,0,1,0,0,0])
			self.phases.append([0,1,0,0,0,0])
			self.phases.append([1,0,0,0,0,0])
		# Ripple Gait
		elif (self.type == 2):
			self.phases.append([1,0,0,0,0,1])
			self.phases.append([0,0,0,0,1,0])
			self.phases.append([0,0,1,1,0,0])
			self.phases.append([0,1,0,0,0,0])
		# Tetrapod Gait
		elif (self.type == 3):
			self.phases.append([1,0,0,0,0,1])
			self.phases.append([0,0,1,0,1,0])
			self.phases.append([0,1,0,1,0,0])
		# Tripod Gait
		elif (self.type == 4):
			self.phases.append([0,1,0,1,0,1])
			self.phases.append([1,0,1,0,1,0])

		self.cs = self.phases[self.np]

	def update_phase(self):
		self.ps = copy.deepcopy(self.cs)

	def change_phase(self):
		""" Change to next gait phase """

		self.update_phase()

		try:
			self.np = self.np + self.gd
			self.cs = self.phases[self.np]
		except IndexError:
			# print 'index error'
			self.np = 0 if (self.gd == 1) else 5
			self.cs = self.phases[self.np]

	def change_exceeded_phase(self, leg=None):
		""" Change the input leg to transfer """

		self.update_phase()

		if (leg is None):
			try:
				self.np = self.np + self.gd
				self.cs = self.phases[self.np]
			except IndexError:
				# print 'index error'
				self.np = 0 if (self.gd == 1) else 5
				self.cs = self.phases[self.np]
		else:
			for i in range(0, len(self.phases)):
				if (self.phases[i][leg] == 1):
					self.cs = self.phases[i]
					self.np = i
					# print 'found: ', gait.phase[i]

	def support_mode(self):
		""" Sets robot to full support mode and
			updates previous state for continuation """

		self.update_phase()
		self.cs = [0]*6

	def walk_mode(self):
		""" Restore gait to previous state if
			robot was in full support mode 		"""

		if (self.cs == [0]*6):
			self.cs = copy.deepcopy(self.ps)

	def reverse_gait(self):
		""" reverses the gait """
		if (self.gd == 1):
			self.gd = -1
		elif (self.gd == -1):
			self.gd = 1
	
	def set_step_stroke(self, leg_stance, d_clear, stroke_default=0.1):
		""" Sets the size of step stroke """
		
		p_base_X_foot_0 = np.array([TRN_BASE_X_LEG[0][0], TRN_BASE_X_LEG[0][1], 0.]) + mX(rot_Z(np.deg2rad(ROT_BASE_X_LEG[0])),leg_stance[0])
		p_base_X_foot_1 = np.array([TRN_BASE_X_LEG[1][0], TRN_BASE_X_LEG[1][1], 0.]) + mX(rot_Z(np.deg2rad(ROT_BASE_X_LEG[1])),leg_stance[1])
		
		# Compute pitch
		nrp_pitch = p_base_X_foot_0 - p_base_X_foot_1
		x_pitch = (nrp_pitch[0] - d_clear)/2.
		
		# Set stroke
		self.step_stroke = 2*x_pitch if (stroke_default/2 > x_pitch) else stroke_default

## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##

gait = GaitClass(3)
# print gait.cs
leg = 1
# for i in range(0, len(gait.phases)):
# 	print 'checking: ', gait.phases[i]
	
# 	if (gait.phases[i][leg] == 1):
# 		print 'found: ', gait.phases[i]
# for i in range(0,3):
# 	gait.change_phase()
# 	print gait.cs

# gait.reverse_gait()

# print 'now reversed'
# for i in range(0,12):
# 	gait.change_phase()
# 	print gait.cs, gait.np
# print gait.phase[-3]
# print '============='
# print gait.cs
# gait.support_mode()
# print gait.cs

# gait.walk_mode()
# print '============='
# for i in range(0,3):
# 	print gait.cs
# 	gait.change_phase()