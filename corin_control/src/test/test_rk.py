#!/usr/bin/env python

import sys
sys.path.insert(0, '/home/wilson/catkin_ws/src/corin/corin_control/src/corin_control')
sys.dont_write_bytecode = True

import numpy as np

from enum import Enum 
import yaml
from scipy import signal

if __name__ == "__main__":

	Fz = 10
	force_desired = 7

	fs = 50 # Sampling frequency (Hz)

	D = 1.4 # Damping ratio 0.7 1.5
	fn = 1 # Natural frequency (Hz) 0.5 3
	wn = 2*3.142*fn # in rad
	G = 0.004 # Gain

	H = ([G*wn*wn], [1, 2*D*wn, wn*wn])

	num, den, dt = signal.cont2discrete(H, 1.0/fs, method='bilinear') #zoh, euler
	
	num = num[0]
	print num, den, dt

	
	den = den[1:]
	filter_states = []
	forward_filters = []
	reverse_filters = []
	for j in range(6):
		# filter_states.append(signal.lfilter_zi(num, den))
		filt = [0] * len(num)
		filt2 = [0] * (len(den))
		forward_filters.append(filt)
		reverse_filters.append(filt2)

	# print num, den, dt
	print "len(forward)", len(forward_filters)
	print "len(reverse)", len(reverse_filters)
	# exit()
	# for i in range(100):
		# offset = np.array([0, 0, i/1000.0])

	for j in range(1):
		# force = np.linalg.norm(self.robot.Leg[j].F6c.tibia_X_foot[:3])
		force = Fz - force_desired# z-axis

		forward_filters[j].pop()
		forward_filters[j].insert(0,force)

		output = 0
		for k in range(len(forward_filters[j])):
			output += num[k]*forward_filters[j][k]

		for k in range(len(reverse_filters[j])):
			output -= den[k]*reverse_filters[j][k]

		if j == 1:
			print output

		reverse_filters[j].pop()
		reverse_filters[j].insert(0,output)

		offset = np.array([0, 0, output])
		# manager.Robot.Leg[j].XHd.coxa_X_foot[0:3,3] = positions[j] + offset