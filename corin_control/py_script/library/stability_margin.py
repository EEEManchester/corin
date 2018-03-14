#!/usr/bin/env python

""" Evaluates the stability margin of the robot """
import sys; sys.dont_write_bytecode = True
import numpy as np
from scipy import linalg

import transformations as tf
from constant import *

class StabilityMargin():
	def __init__(self):
		pass

	def LSM(self, q=None):
		""" longitudinal stability margin """

		pass