#!/usr/bin/env python

## Class for 2D arrays for wrench

import numpy as np

__all__ = ['WrenchClass']

class WrenchClass:
	def __init__(self):
		# force components
		self.f = np.zeros((3,1))
		# moment components
		self.m = np.zeros((3,1))