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

	def append(self, data):
		self.t  += data.t 
		self.xp += data.xp

	def reverse(self):
		self.xp = list(reversed(self.xp))