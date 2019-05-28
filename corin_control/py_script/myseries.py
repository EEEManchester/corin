#!/usr/bin/env python

## State estimation for the robot
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'library'))
sys.dont_write_bytecode = True

import time
import warnings
import numpy as np

## Personal libraries
from library import *			# library modules to include

class StateEstimator:

	def __init__(self):

		w = np.array([1, 2, 3])
		T= 0.002
		for i in range(100000):
			A = self.mySeries(w, T, 2)


	# Implements auxiliary quantity defined in State Estimation paper by Bloesch
	@staticmethod
	def mySeries(w, T, n, N=3):
		w_skew = skew(w)
		mysw = StateEstimator.my_skew(w)
		# S = np.zeros((3, 3))
		# for i in range(N):
		# 	temp = ( T**(i+n) / math.factorial(i+n) ) * np.linalg.matrix_power(w_skew, i)
		# 	S += temp

		S = ( (T**n / math.factorial(n) ) * np.eye(3)
			 +(T**(n+1) / math.factorial(1+n) ) * w_skew  )
		return S

	@staticmethod
	def my_skew(x):
		return np.array([[0, -x[2], x[1]],
			[x[2], 0, x[0]],
			[x[1], x[0], 0]])


if __name__ == "__main__":

	state = StateEstimator()

	"Complete"
