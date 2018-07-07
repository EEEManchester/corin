#!/usr/bin/env python

## Class for 2D arrays for wrench

import numpy as np

__all__ = ['Wrench']

class Wrench:
	def __init__(self):
		self.force  = np.zeros((3,1))
		self.torque = np.zeros((3,1))