#!/usr/bin/env python

## Class for 2D arrays for twist
## actually, not accurate as twist refers to linear and angular velocity only
import numpy as np

class TwistClass:
	def __init__(self):
		# linear components
		self.xp = np.zeros((3,1))
		self.xv = np.zeros((3,1))
		self.xa = np.zeros((3,1))
		# angular components
		self.wp = np.zeros((3,1))
		self.wv = np.zeros((3,1))
		self.wa = np.zeros((3,1))