#!/usr/bin/env python

## Class for 2D arrays for twist

import numpy as np

__all__ = ['Twist']

class Twist:
	def __init__(self):
		self.linear  = np.zeros((3,1))
		self.angular = np.zeros((3,1))
