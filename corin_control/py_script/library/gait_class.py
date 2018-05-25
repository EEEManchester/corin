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
		self.type  = gselect 	# gait type
		self.phase = [] 		# list of gait phases
		self.np = 0 			# counter for gait phase
		self.tp = GAIT_TPHASE	# timing per gait phase
		self.gd = 1 			# gait direction

		# States
		self.cs = [0]*6	# current 
		self.ds = [0]*6	# desired 
		self.ps = [0]*6	# previous

		self.__initialise__()

	def __initialise__(self):
		""" Set gait phases """

		# Reset list
		self.phase = []

		# Wave Gait
		if self.type == 1:
			self.phase.append([0,0,0,0,0,1])
			self.phase.append([0,0,0,0,1,0])
			self.phase.append([0,0,0,1,0,0])
			self.phase.append([0,0,1,0,0,0])
			self.phase.append([0,1,0,0,0,0])
			self.phase.append([1,0,0,0,0,0])
		# Ripple Gait
		elif (self.type == 2):
			self.phase.append([1,0,0,0,0,1])
			self.phase.append([0,0,0,0,1,0])
			self.phase.append([0,0,1,1,0,0])
			self.phase.append([0,1,0,0,0,0])
		# Tetrapod Gait
		elif (self.type == 3):
			self.phase.append([1,0,0,0,0,1])
			self.phase.append([0,0,1,0,1,0])
			self.phase.append([0,1,0,1,0,0])
		# Tripod Gait
		elif (self.type == 4):
			self.phase.append([0,1,0,1,0,1])
			self.phase.append([1,0,1,0,1,0])

		self.cs = self.phase[self.np]

	def update_phase(self):
		self.ps = copy.deepcopy(self.cs)

	def change_phase(self):
		""" Change to next gait phase """

		self.update_phase()

		try:
			self.np = self.np + self.gd
			self.cs = self.phase[self.np]
		except IndexError:
			# print 'index error'
			self.np = 0 if (self.gd == 1) else 5
			self.cs = self.phase[self.np]

	def change_exceeded_phase(self, leg=None):
		""" Change the input leg to transfer """

		self.update_phase()

		if (leg is None):
			try:
				self.np = self.np + self.gd
				self.cs = self.phase[self.np]
			except IndexError:
				# print 'index error'
				self.np = 0 if (self.gd == 1) else 5
				self.cs = self.phase[self.np]
		else:
			for i in range(0, len(gait.phase)):
				if (gait.phase[i][leg] == 1):
					self.cs = gait.phase[i]
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
		
## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##
gait = GaitClass(3)
# print gait.cs
leg = 1
for i in range(0, len(gait.phase)):
	print 'checking: ', gait.phase[i]
	
	if (gait.phase[i][leg] == 1):
		print 'found: ', gait.phase[i]
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