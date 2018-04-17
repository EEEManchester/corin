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
		self.gait_type = gselect 	# gait type
		self.gait = [] 				# matrix for gait phases
		self.gdic = {} 				# dictionary for gait: 'beta'-duty factor, 'dphase'-fraction for displacement per phase i.e. distance = Stroke/dphase
		self.gphase = 0 			# counter for gait phases
		self.tphase = GAIT_TPHASE	# timing per gait phase
		self.phase_trans = 0 		# distance travelled in each phase

		self.direction 	= True 		# True: forward, False: reversed
		self.toggle 	= True 		# checks if direction toggled

		self.cs = [1,0,1,0,1,0] 	# current state
		self.ds = [1,0,1,0,1,0]		# desired state
		self.ps = [0,1,0,1,0,1] 	# previous state
		self.es = [0,0,0,0,0,0] 	# error state

		self.initiate_gait(gselect)

	def gait_dutyFactor(self):
		if self.gait_type == 1:
			self.gait = np.matrix([	[Fraction(5, 6), Fraction(5, 6), Fraction(5, 6), Fraction(5, 6), Fraction(5, 6), Fraction(5, 6)],
									[Fraction(6, 6), Fraction(5, 6), Fraction(4, 6), Fraction(3, 6), Fraction(2, 6), Fraction(1, 6)]])
			self.gdic = {'name': "wave",
						'beta': Fraction(5, 6),
						'dphase': Fraction(1, 5),
						'phase':np.matrix([ Fraction(6, 6), Fraction(5, 6), Fraction(4, 6), Fraction(3, 6), Fraction(2, 6), Fraction(1, 6) ])}
			

		elif self.gait_type == 2:	
			self.gait = np.matrix([	[Fraction(3, 4), Fraction(3, 4), Fraction(3, 4), Fraction(3, 4), Fraction(3, 4), Fraction(3, 4)],
									[Fraction(4, 4), Fraction(3, 4), Fraction(2, 4), Fraction(2, 4), Fraction(1, 4), Fraction(4, 4)]])
			self.gdic = {'name': "ripple",
						'beta': Fraction(3, 4),
						'dphase': Fraction(1, 3),
						'phase':np.matrix([ Fraction(4, 4), Fraction(3, 4), Fraction(2, 4), Fraction(2, 4), Fraction(1, 4), Fraction(4, 4) ])}
		
		elif self.gait_type == 3:
			self.gait = np.matrix([	[Fraction(2, 3), Fraction(2, 3), Fraction(2, 3), Fraction(2, 3), Fraction(2, 3), Fraction(2, 3)],
									[Fraction(3, 3), Fraction(2, 3), Fraction(1, 3), Fraction(2, 3), Fraction(1, 3), Fraction(3, 3)]])
			self.gdic = {'name': "tetrapod",
						'beta': Fraction(2, 3),
						'dphase': Fraction(1, 2),
						'phase':np.matrix([ Fraction(3, 3), Fraction(2, 3), Fraction(1, 3), Fraction(2, 3), Fraction(1, 3), Fraction(3, 3) ])}
		
		elif self.gait_type == 4:
			self.gait = np.matrix([	[Fraction(1, 2), Fraction(1, 2), Fraction(1, 2), Fraction(1, 2), Fraction(1, 2), Fraction(1, 2)],
									[Fraction(1, 1), Fraction(1, 2), Fraction(1, 1), Fraction(1, 2), Fraction(1, 1), Fraction(1, 2)]])
			self.gdic = {'name': "tripod",
						'beta': Fraction(1, 2),
						'dphase': Fraction(1, 1),
						'phase':np.matrix([ Fraction(1, 1), Fraction(1, 2), Fraction(1, 1), Fraction(1, 2), Fraction(1, 1), Fraction(1, 2)])}

	def initiate_gait(self, gait_selected):
		if (gait_selected > 0 and gait_selected <= 4):
			self.gait_type = gait_selected
			self.gait_dutyFactor()
			self.change_phase()
			self.ps = copy.deepcopy(self.cs)
			print '>> INITIALISED GAIT CLASS - ', self.gdic['name']
		else:
			print 'Invalid gait selected'

	def update_phase(self):
		self.ps = copy.deepcopy(self.cs)

	def change_phase(self):
		# update previous state prior to update
		self.update_phase()

		# modify gait sequence if direction changed
		if (self.direction==True and self.toggle==False):
			self.gphase += 1
			self.toggle = self.direction
		elif (self.direction==False and self.toggle==True):
			self.gphase -= 1
			self.toggle = self.direction

		# reset phase
		if (self.gphase == self.gdic['beta'].denominator):
			self.gphase = 0
		elif (self.gphase < 0):
			self.gphase = self.gdic['beta'].denominator - 1

		# number of phases in a cycle
		i = self.gphase 		

		for n in range(0,6): 												# number of legs
			# Condition for Leg 6 to start first - used for tetrapod and ripple due to fraction equals to 1 for leg 6
			##### Runs once at the start 
			if (self.gait.item(1,5)==1):	
				for k in range(0,6):
					self.gait[1,k] = self.gait.item(1,k) + Fraction(1,self.gdic['beta'].denominator)
					if (self.gait[1,k] > 1.0): 															# modifies for phase to be fraction 
						self.gait[1,k] = self.gait[1,k] - Fraction(1,1)

			## Determine Transfer or Support phase
			if (self.gait.item(1,n).denominator < self.gdic['beta'].denominator): 									
				temp_num = self.gait[1,n].numerator*self.gdic['beta'].denominator/self.gait.item(1,n).denominator
			else:
				temp_num = self.gait[1,n].numerator

			if ( (i+1) == temp_num ): 	# Transfer Phase
				self.cs[n] = 1
			else: 						# Support Phase
				self.cs[n] = 0
		
		gait_phase = [i,self.gdic['beta'].denominator]
		
		# increment/decrement counter for next gait phase
		if (self.direction==True):
			self.gphase += 1
		else:
			self.gphase -= 1

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

## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##

# gait = GaitClass(3)
# print gait.cs, gait.ps

# for z in range(0,3):
# 	print gait.cs, gait.ps
# 	gait.change_phase()
# print '--------------'

# gclass.direction = False 	# change gait direction
# for z in range(0,3):
# 	gclass.change_phase()

# gclass.direction = True 	# change gait direction
# for z in range(0,3):
# 	gclass.change_phase()