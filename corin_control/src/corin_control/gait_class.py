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
		self.type  = gselect 		# gait type: 1=wave, 2=ripple, 3=tetrapod, 4=wave
		self.phases = [] 			# list of gait phases
		self.np = 0 				# counter for gait phase
		self.tphase = GAIT_TPHASE	# timing per gait phase
		self.gd = 1 				# gait direction
		self.tcycle = GAIT_TPHASE	# timing per gait cycle
		self.duty_factor = 0

		self.step_stroke = STEP_STROKE#self.set_step_stroke(LEG_STANCE, LEG_CLEAR, STEP_STROKE)
		self.step_height = STEP_HEIGHT

		# self.fault = []		# walk mode either 'normal' or 'fault'; latter discontinuous phase gait

		# States
		self.cs = [0]*6	# current 
		self.ns = [0]*6	# next
		self.ps = [0]*6	# previous

		self.__initialise__()

	def __initialise__(self):
		self.set_gait_type()

	def set_gait_type(self, gait_type=None):
		""" Set gait phases """

		if (gait_type is not None):
			self.type = gait_type

		# Reset gait parameters
		self.np = 0
		self.phases = []

		# Wave Gait
		if self.type == 1:
			# wall walking
			# self.phases.append([0,0,1,0,0,0])
			# self.phases.append([0,1,0,0,0,0])
			# self.phases.append([1,0,0,0,0,0])
			# self.phases.append([0,0,0,0,0,1])
			# self.phases.append([0,0,0,0,1,0])
			# self.phases.append([0,0,0,1,0,0])
			self.phases.append([0,0,0,0,0,1])
			self.phases.append([0,0,0,0,1,0])
			self.phases.append([0,0,0,1,0,0])
			self.phases.append([0,0,1,0,0,0])
			self.phases.append([0,1,0,0,0,0])
			self.phases.append([1,0,0,0,0,0])
			self.duty_factor = Fraction(5,6)
		# Ripple Gait
		elif (self.type == 2):
			self.phases.append([1,0,0,0,0,1])
			self.phases.append([0,0,0,0,1,0])
			self.phases.append([0,0,1,1,0,0])
			self.phases.append([0,1,0,0,0,0])
			self.duty_factor = Fraction(3,4)
		# Tetrapod Gait
		elif (self.type == 3):
			self.phases.append([1,0,0,0,0,1])
			self.phases.append([0,0,1,0,1,0])
			self.phases.append([0,1,0,1,0,0])
			self.duty_factor = Fraction(2,3)
		# Tripod Gait
		elif (self.type == 4):
			self.phases.append([0,1,0,1,0,1])
			self.phases.append([1,0,1,0,1,0])
			self.duty_factor = Fraction(1,2)
		# Custom: Reset
		elif (self.type == 5):
			self.phases.append([0,1,0,0,1,0])
			self.phases.append([1,0,0,0,0,1])
			self.phases.append([0,0,1,1,0,0])
		# Tripod Gait
		elif (self.type == 6):
			self.phases.append([0,0,0,0,0,1])
			self.phases.append([0,0,0,0,0,0])
		
		self.tcycle = len(self.phases)*self.tphase

		self.cs = self.phases[self.np]
		self.ns = self.phases[self.np+1]
		
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

		try:
			self.ns = self.phases[self.np+1]
		except IndexError:
			self.ns = self.phases[0]

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

gait = GaitClass(1)
leg = 1
for i in range(0, len(gait.phases)):
	gait.change_phase()

# gait_list = []
# for i in range(2):
# 	gait_list.append(gait.phases)
# gait_list = [item for i in gait_list for item in i]
# print gait_list
# for item in gait_list:
# 	print item
