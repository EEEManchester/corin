#!/usr/bin/env python

""" Class for state: - 	cs: current state
						ds: desired state
						es: error state		 """ 

import Twist
import Wrench
import numpy as np

__all__ = ['StateClass']

class StateClass:
	def __init__(self, stype):
		if (stype == 'Twist'):
			self.cs = Twist.TwistClass()
			self.ds = Twist.TwistClass()
			self.es = Twist.TwistClass()
		elif (stype == 'Wrench'):
			self.cs = Wrench.WrenchClass()
			self.ds = Wrench.WrenchClass()
			self.es = Wrench.WrenchClass()