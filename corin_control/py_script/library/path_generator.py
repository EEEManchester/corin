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
import TrajectoryPoints as TP

class PathGenerator():
	def __init__(self):
		self.base_path = TP.BaseTrajectory() 		# class for 6D path storage
		

	def get_path_details(self):
		""" display details of path generated """
		pass
		# gait_phases = len(self.x_cob)-1
		# gait_cycles = gait_phases/self.gait['beta'].denominator
		# total_time	= (self.x_length-1.)*tn

		# print 'Gait  type: ', self.gait['name']
		# print 'No. phases: ', gait_phases
		# print 'No. cycles: ', gait_cycles
		# print 'Total time: ', total_time

	def compute_no_via_points(self, via_points):
		""" generate unit time interval for spline if unspecified or modified	"""
		
		point_size = len(via_points) 	# determine number of via points
		t_cob = np.zeros(point_size) 	# create an array of via points size

		for i in range(0,point_size):
			t_cob[i] = i
		return t_cob

	def auto_generate_points(self, x_cob, w_cob):
		""" generate linear or angular points if unspecified """

		## no. of items in array
		xsize = x_cob.size 	
		wsize = w_cob.size

		## Generate empty array if either not defined or unequal size
		# w_cob undefined or mismatch
		if (wsize < xsize):
			w_cob = np.array([.0,.0,.0])
			for i in range(0,xsize/3-1):
				w_cob = np.vstack((w_cob,np.array([0., 0., 0.])))

		# x_cob undefined or mismatch
		elif (xsize < wsize):
			x_cob = np.array([.0,.0,.0])
			for i in range(0,wsize/3-1):
				x_cob = np.vstack((x_cob,np.array([0., 0., 0.])))

		# create array of time for via points
		t_cob = self.compute_no_via_points(x_cob)
		
		return x_cob, w_cob, t_cob

	def generate_path(self, x_cob, w_cob, tn):
		""" Generate body trajectory and modify to be within velocity limit """
		""" Size of x_cob & w_cob needs to be the same otherwise the array
			with the smaller size will be ignored and set to zero 			"""
		""" Input: 	1) x_cob -> array of via points for linear translation
					2) w_cob -> array of via points for angular rotations 	
			Output:	1) array of linear translation points
					2) array of angular rotation points 					"""

		# SplineGenerator = Bspline.SplineGenerator()	# biezer spline generator
		SplineGenerator = Pspline.SplineGenerator() 	# cubic polynomial spline generator 

		# generate linear or angular via points if size mismatch
		x_cob, w_cob, t_cob = self.auto_generate_points(x_cob, w_cob)
		
		# generate spline based on unit time interval between via points
		x_out = SplineGenerator.generate_body_spline(x_cob, t_cob, tn)
		w_out = SplineGenerator.generate_body_spline(w_cob, t_cob, tn)

		# check if base linear velocity exceeds limit
		v_max = np.amax(x_out[2]);
		if (v_max > BASE_MAX_LINEAR_VELOCITY):
			print 'max linear velocity exceeded'
			ndiv  = v_max/BASE_MAX_LINEAR_VELOCITY	# determine number of divisions to reduce by
			t_cob = np.round(t_cob*ndiv,1) 			# set new time interval
			# regenerate spline
			x_out = SplineGenerator.generate_body_spline(x_cob, t_cob, tn)
			w_out = SplineGenerator.generate_body_spline(w_cob, t_cob, tn)

		# check if base angular velocity exceeds limit
		w_max = np.amax(w_out[2]);
		if (w_max > BASE_MAX_ANGULAR_VELOCITY):
			print 'max angular velocity exceeded'
			ndiv  = w_max/BASE_MAX_ANGULAR_VELOCITY	# determine number of divisions to reduce by
			t_cob = np.round(t_cob*ndiv,1) 			# set new time interval
			# regenerate spline
			x_out = SplineGenerator.generate_body_spline(x_cob, t_cob, tn)
			w_out = SplineGenerator.generate_body_spline(w_cob, t_cob, tn)

		# return x_out, w_out 		# python 2D array
		self.base_path = TP.BaseTrajectory((x_out,w_out))
		# return TP.TrajectoryPoints(x_out), TP.TrajectoryPoints(w_out)	# convert to TrajectoryPoints format
		return TP.BaseTrajectory((x_out,w_out))

## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##
gait = {'name': "wave",
		'beta': Fraction(5, 6),
		'dphase': Fraction(1, 5),
		'phase':np.matrix([ Fraction(6, 6), Fraction(5, 6), Fraction(4, 6), Fraction(3, 6), Fraction(2, 6), Fraction(1, 6) ])}

x_cob = np.array([.0,.0,BODY_HEIGHT])
w_cob = np.array([.0,.0,.0])
# x_cob = np.vstack((x_cob,np.array([0.2, 0.0, BODY_HEIGHT+0.05])))
# x_cob = np.vstack((x_cob,np.array([0.3, 0.5, BODY_HEIGHT])))
# x_cob = np.vstack((x_cob,np.array([0.1, 0.0, BODY_HEIGHT-0.05])))
# x_cob = np.vstack((x_cob,np.array([0.7, 0.0, BODY_HEIGHT+0.12])))


# w_cob = np.vstack((w_cob,np.array([1.0,.0,.0])))
# w_cob = np.vstack((w_cob,np.array([1.0,.0,.0])))
# w_cob = np.vstack((w_cob,np.array([1.0,.0,.0])))

x_cob = np.vstack((x_cob,np.array([0. , -0.04, BODY_HEIGHT])))
w_cob = np.vstack((w_cob,np.array([-0.2, -0.1, 0.])))
x_cob = np.vstack((x_cob,np.array([0.03,  0.04, BODY_HEIGHT])))
w_cob = np.vstack((w_cob,np.array([0.2, -0.1, 0.])))
x_cob = np.vstack((x_cob,np.array([.0,.0,BODY_HEIGHT])))
w_cob = np.vstack((w_cob,np.array([0., 0., 0.])))

planner = PathGenerator()
planner.gait = gait
# path = planner.generate_path(x_cob, w_cob, 0.1)
# Plot.plot_2d(path.W.t,path.W.xp)
# Plot.plot_2d_multiple(2,x_out[0],x_out[1],x_out[3])
## wall transition
# cq = 0.
# t_cob = np.array([0.0])

# for q in range(0,91,15):
# 	qr = q*np.pi/180
# 	xd = np.array([0.0, (1-np.cos(qr))*COXA_Y, (np.sin(qr))*COXA_Y + BODY_HEIGHT])

# 	x_cob = np.vstack(( x_cob, xd ))
# 	w_cob = np.vstack(( w_cob, np.array([qr,0.0,0.0]) ))
# 	t_cob = np.hstack(( t_cob, cq ))
# 	cq += 1
# # clean up first row
# x_cob = np.delete(x_cob, 1, 0) # clean up first row of matrix
# w_cob = np.delete(w_cob, 1, 0) # clean up first row of matrix
# t_cob = np.delete(t_cob, 0)


# nt,nx,nv,na = planner.generate_path(x_cob, w_cob)
# planner.get_path_details()
# Plot.plot_3d(nx,nv,na)
# Plot.plot_2d_multiple(3,nt,nx,nv,na)

# print planner.x_cob
# print '======================'
# print planner.w_cob
# print planner.t_cob
