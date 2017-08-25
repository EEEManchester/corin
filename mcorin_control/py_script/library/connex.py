#!/usr/bin/env python

## Connex class for 6D axis controller

class Connexion:
	def __init__(self):
		self.b1_state = 0 			# left button
		self.b2_state = 0 			# right button
		self.b1_disable = False
		self.b2_disable = False

	def displayState(self):
		if (self.b1_state == 0):
			print 'Connex state: Walking Mode'
		if (self.b1_state == 1):
			print 'Connex state: Support Mode' 