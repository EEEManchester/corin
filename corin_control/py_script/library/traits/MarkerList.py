#!/usr/bin/env python

## Class for list for points

import numpy as np

__all__ = ['MarkerList']

class MarkerList():
	def __init__(self,data=None):
		if data is None:
			self.t  = []
			self.xp = []
		else:
			self.t  = data[0]
			self.xp = data[1]

	def append(self, x):
		pass