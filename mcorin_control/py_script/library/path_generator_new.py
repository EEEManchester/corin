#!/usr/bin/env python

## Class to plan a path for the robot given arbitrary points

# sys.dont_write_bytecode = True

import time
from fractions import Fraction
import numpy as np
import math
from math import modf
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# import robot_class
# import param_gait
import plotgraph as Plot
from constant import *

import transformations as tf
import pspline_generator as Pspline
import bspline_generator as Bspline
from TrajectoryPoints import TrajectoryPoints

class PathGenerator():
	def __init__(self):
		self.spline = Bspline.SplineGenerator()
		self.spline = Pspline.SplineGenerator()
		self.x_com = np.array([ [.0,.0,BODY_HEIGHT] ]) 	# output linear spline points
		self.w_com = np.array([ [0.0, 0.0, 0.0] ]) 		# output angular spline points
		self.t_com = np.zeros(0) 						# time interval for output spline points
		
		self.gait = {} 		# dictionary for gait parameters

		self.x_length = 0 	# output linear spline length
		self.w_length = 0	# output angular spline length

	def get_path_details(self):
		""" display details of path generated """

		gait_phases = len(self.x_com)-1
		gait_cycles = gait_phases/self.gait['beta'].denominator
		total_time	= (self.x_length-1.)*TRAC_INTERVAL

		print 'Gait  type: ', self.gait['name']
		print 'No. phases: ', gait_phases
		print 'No. cycles: ', gait_cycles
		print 'Total time: ', total_time

	def compute_no_via_points(self, via_points):
		""" generate unit time interval for spline if unspecified or modified	"""
		
		point_size = len(via_points) 	# determine number of via points
		t_com = np.zeros(point_size) 	# create an array of via points size

		for i in range(0,point_size):
			t_com[i] = i
		return t_com

	def auto_generate_points(self, x_com, w_com):
		""" generate linear or angular points if unspecified """

		## no. of items in array
		xsize = x_com.size 	
		wsize = w_com.size

		## Generate empty array if either not defined by user
		# w_com not defined by user
		if (wsize < xsize):
			w_com = np.array([.0,.0,.0])
			for i in range(0,xsize/3-1):
				w_com = np.vstack((w_com,np.array([0., 0., 0.])))

		# x_com not defined by user
		elif (xsize < wsize):
			x_com = np.array([.0,.0,.0])
			for i in range(0,wsize/3-1):
				x_com = np.vstack((x_com,np.array([0., 0., 0.])))

		# create array of time for via points
		t_com = self.compute_no_via_points(x_com)
		return x_com, w_com, t_com

	def generate_path(self, x_com, w_com):
		""" Generate body trajectory and modify to be within velocity limit """

		# generate linear or angular via points if size mismatch
		x_com, w_com, t_com = self.auto_generate_points(x_com, w_com)

		# generate spline based on unit time interval between via points
		x_out = self.spline.generate_body_spline(x_com, t_com, TRAC_INTERVAL)
		w_out = self.spline.generate_body_spline(w_com, t_com, TRAC_INTERVAL)

		# check if base linear velocity exceeds limit
		v_max = np.amax(x_out[2]);
		if (v_max > BASE_MAX_VELOCITY):
			# print 'max linear velocity exceeded'
			ndiv  = v_max/BASE_MAX_VELOCITY			# determine number of divisions to reduce by
			t_com = np.round(t_com*ndiv,1) 			# set new time interval
			# regenerate spline
			x_out = self.spline.generate_body_spline(x_com, t_com, TRAC_INTERVAL)
			w_out = self.spline.generate_body_spline(w_com, t_com, TRAC_INTERVAL)

		# check if base angular velocity exceeds limit
		w_max = np.amax(w_out[2]);
		if (w_max > BASE_MAX_VELOCITY):
			# print 'max angular velocity exceeded'
			ndiv  = w_max/BASE_MAX_VELOCITY			# determine number of divisions to reduce by
			t_com = np.round(t_com*ndiv,1) 			# set new time interval
			# regenerate spline
			x_out = self.generate_body_spline(x_com, t_com, TRAC_INTERVAL)
			w_out = self.generate_body_spline(w_com, t_com, TRAC_INTERVAL)

		# return x_out, w_out 		# python 2D array
		return TrajectoryPoints(x_out), TrajectoryPoints(w_out)	# convert to TrajectoryPoints format


## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##
gait = {'name': "wave",
		'beta': Fraction(5, 6),
		'dphase': Fraction(1, 5),
		'phase':np.matrix([ Fraction(6, 6), Fraction(5, 6), Fraction(4, 6), Fraction(3, 6), Fraction(2, 6), Fraction(1, 6) ])}

x_com = np.array([.0,.0,BODY_HEIGHT])
x_com = np.vstack((x_com,np.array([0.2, 0.0, BODY_HEIGHT+0.05])))
x_com = np.vstack((x_com,np.array([0.3, 0.5, BODY_HEIGHT])))
x_com = np.vstack((x_com,np.array([0.1, 0.0, BODY_HEIGHT-0.05])))
# x_com = np.vstack((x_com,np.array([0.7, 0.0, BODY_HEIGHT+0.12])))

w_com = np.array([.0,.0,.0])

planner = PathGenerator()
planner.heading = (1,0,0)
planner.gait = gait
x_out, w_out = planner.generate_path(x_com, w_com)
print x_out.t.shape
print x_out.t[1]
# Plot.plot_2d(x_out.t,x_out.xp)
# Plot.plot_2d_multiple(2,x_out[0],x_out[1],x_out[3])
## wall transition
# cq = 0.
# t_com = np.array([0.0])

# for q in range(0,91,15):
# 	qr = q*np.pi/180
# 	xd = np.array([0.0, (1-np.cos(qr))*COXA_Y, (np.sin(qr))*COXA_Y + BODY_HEIGHT])

# 	x_com = np.vstack(( x_com, xd ))
# 	w_com = np.vstack(( w_com, np.array([qr,0.0,0.0]) ))
# 	t_com = np.hstack(( t_com, cq ))
# 	cq += 1
# # clean up first row
# x_com = np.delete(x_com, 1, 0) # clean up first row of matrix
# w_com = np.delete(w_com, 1, 0) # clean up first row of matrix
# t_com = np.delete(t_com, 0)


# nt,nx,nv,na = planner.generate_path(x_com, w_com)
# planner.get_path_details()
# Plot.plot_3d(nx,nv,na)
# Plot.plot_2d_multiple(3,nt,nx,nv,na)

# print planner.x_com
# print '======================'
# print planner.w_com
# print planner.t_com
