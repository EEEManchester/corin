#!/usr/bin/env python

## Class for common robotic terminologies

import numpy as np

class Wrench:
	def __init__(self):
		self.force  = np.zeros((3,1))
		self.torque = np.zeros((3,1))

class Twist:
	def __init__(self):
		self.linear  = np.zeros((3,1))
		self.angular = np.zeros((3,1))

class Pose:
	def __init__(self):
		self.position    = np.zeros((3,1))
		self.orientation = np.zeros((3,1))	