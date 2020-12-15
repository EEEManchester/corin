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
from traits import *
from constant import *

from matrix_transforms import *
import pspline_generator as Pspline
import bspline_generator as Bspline

import plotgraph as Plot

class PathGenerator():
	V_MAX = BASE_MAX_LINEAR_VELOCITY
	W_MAX = BASE_MAX_ANGULAR_VELOCITY

	def __init__(self):
		self.base_path = Trajectory6D() 		# class for 6D path storage

	def reset_static_elements(self, v_max=None, w_max=None):
		if (v_max is None and w_max is None):
			self.V_MAX = BASE_MAX_LINEAR_VELOCITY
			self.W_MAX = BASE_MAX_ANGULAR_VELOCITY
		else:
			self.V_MAX = v_max
			self.W_MAX = w_max

	def compute_no_via_points(self, via_points):
		""" generate unit time interval for spline if unspecified or modified	"""

		point_size = len(via_points) 		# determine number of via points
		time_array = np.zeros(point_size) 	# create an array of via points size

		for i in range(0,point_size):
			time_array[i] = i
		return time_array

	def auto_generate_points(self, x_cob, w_cob, t_cob):
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
		if (t_cob is None):
			t_cob = self.compute_no_via_points(x_cob)
		elif (len(t_cob) != len(x_cob)):
			t_cob = self.compute_no_via_points(x_cob)

		return x_cob, w_cob, t_cob

	def generate_base_path(self, x_cob, w_cob, tn, t_cob=None):
		""" Generate body trajectory (Re^6) and modify to be within velocity limit 	"""
		""" Size of x_cob & w_cob needs to be the same otherwise the array with
			the smaller size will be ignored and set to zero 						"""
		""" Input: 	1) x_cob -> array of via points for linear translation
					2) w_cob -> array of via points for angular rotations
					3) tn -> path sampling rate
			Output:	BaseTrajectory() array of linear translation and angular
					rotation points 												"""

		SplineGenerator = Bspline.SplineGenerator()	# biezer spline generator
		# SplineGenerator = Pspline.SplineGenerator() # cubic polynomial spline generator

		# generate linear or angular via points if size mismatch
		x_cob, w_cob, t_cob = self.auto_generate_points(x_cob, w_cob, t_cob)

		# generate spline based on unit time interval between via points
		x_out = SplineGenerator.generate_spline(x_cob, t_cob, tn)
		w_out = SplineGenerator.generate_spline(w_cob, t_cob, tn)

		# check if base linear velocity exceeds limit
		v_max = np.amax(x_out[2])
		v_min = np.amin(x_out[2])
		v_max = v_max if (abs(v_max) > abs(v_min)) else abs(v_min)

		if (v_max > self.V_MAX):
			print 'Maximum linear velocity exceeded, modifying... '
			ndiv  = v_max/self.V_MAX		# determine number of divisions to reduce by
			t_cob = np.round(t_cob*ndiv,1) 	# set new time interval
			# regenerate spline
			x_out = SplineGenerator.generate_spline(x_cob, t_cob, tn)
			w_out = SplineGenerator.generate_spline(w_cob, t_cob, tn)

		# check if base angular velocity exceeds limit
		w_max = np.amax(w_out[2])
		w_min = np.amin(w_out[2])
		w_max = w_max if (abs(w_max) > abs(w_min)) else abs(w_min)

		if (w_max > self.W_MAX):
			print 'Maximum angular velocity exceeded, modifying... '
			ndiv  = w_max/self.W_MAX		# determine number of divisions to reduce by
			t_cob = np.round(t_cob*ndiv,1) 	# set new time interval
			# regenerate spline
			x_out = SplineGenerator.generate_spline(x_cob, t_cob, tn)
			w_out = SplineGenerator.generate_spline(w_cob, t_cob, tn)


		self.base_path = Trajectory6D((x_out,w_out))
		# return TrajectoryPoints(x_out), TrajectoryPoints(w_out)	# convert to TrajectoryPoints format

		return Trajectory6D((x_out,w_out))

	def old_interpolate_leg_path(self, sp, ep, snorm, snorm2, phase=1, reflex=False, ctime=2.0, type='parabolic'):
		""" Generate via points for leg trajectory (Re^3) 				"""
		""" Input: 	1) sp -> starting position (Re^3)
					2) ep -> end position (Re^3)
					3) snorm -> surface normal in unit vector (Re^3)
								wrt to leg frame
					4) phase -> leg phase: 1 = transfer, 0 = support
					5) reflex -> boolean for triggering reflex
					6) ctime -> duration of trajectory
					7) type -> spline shape
			Output:	Array of via points and time intervals				"""

		## Define Variables ##
		cpx = sp.copy()		# cartesian position
		sh  = STEP_HEIGHT  	# step height for transfer phase

		## 1) Set via points according to phase
		if (phase==1):
			## Transfer phase

			# vector from origin to travel path midpoint
			if (type=='parabolic'):
				pdiv = np.array([0.1, 0.5, 1.0]) 		# point division
				hdiv = np.array([0.6*sh, sh, 0.6*sh]) 		# height division
				tdiv = np.array([0., 0.24*ctime, 0.5*ctime, 0.76*ctime, ctime ])
				# pdiv = np.array([0.125, 0.5, 0.875]) 		# point division
				# hdiv = np.array([0.6*sh, sh, 0.6*sh]) 		# height division
				# tdiv = np.array([0., 0.24*ctime, 0.5*ctime, 0.76*ctime, ctime ])
			elif (type == 'trapezoidal'):
				pdiv = np.array([0., 0.12, 0.5, 0.88, 1.])
				hdiv = np.array([0.9*sh, sh, sh, sh, 0.9*sh])
				tdiv = np.array([0., 0.32*ctime, 0.4*ctime, 0.5*ctime, 0.6*ctime, 0.68*ctime, ctime ])

			A = np.zeros((1,3))

			for i in range(0,len(pdiv)):
				## Compute via points along the vector from the start to end point
				if (reflex and i==0):
					print 'TRIGGER REFLEX'
					m3 = sp + (ep - sp)*pdiv[i] + np.array([0., 0.03, 0.])
				else:
					v3 = sp + (ep - sp)*pdiv[i]

				# Compute the height clearance in surface normal direction
				h3 = snorm*hdiv[i]

				# Compute the via point with height clearance and stack into array
				vn  = v3 + h3
				cpx = np.vstack((cpx, vn))

			# stack final point into array
			cpx = np.vstack((cpx, ep))

		elif (phase==0):
			## Support phase - direct interpolation
			tdiv = np.array([0, ctime])
			cpx  = np.array([sp, ep])

		else:
			print "Invalid phase" 	# have a way to exit function if no phase selected
			return None

		return cpx, tdiv

	def interpolate_leg_path(self, sp, ep, sn1, sn2, phase=1, reflex=False, ctime=2.0, type='parabolic'):
		""" Generate via points for leg trajectory (Re^3) 				"""
		""" Input: 	1) sp -> starting position (Re^3)
					2) ep -> end position (Re^3)
					3) snorm -> surface normal in unit vector (Re^3)
								wrt to leg frame
					4) phase -> leg phase: 1 = transfer, 0 = support
					5) reflex -> boolean for triggering reflex
					6) ctime -> duration of trajectory
					7) type -> spline shape
			Output:	Array of via points and time intervals				"""

		## Define Variables ##
		cpx = sp.copy()		# cartesian position
		sh  = STEP_HEIGHT  	# step height for transfer phase
		hl  = 0.5

		## 1) Set via points according to phase
		if (phase==1):
			## Transfer phase

			# vector from origin to travel path midpoint
			if (type=='parabolic'):
				# pdiv = np.array([0.1, 0.5, 0.9]) 		# point division
				# hdiv = np.array([0.4, 1.0, 0.4])*sh		# height division
				# tdiv = np.array([0.0, 0.05,0.1,0.15, 0.2, 0.3, 0.5, 0.7, 0.8,0.85, 0.9,0.95, 1.0])*ctime
				# pdiv = np.array([0.1, 0.5, 0.9]) 		# point division
				# hdiv = np.array([0.4, 1.0, 0.4])*sh		# height division
				# tdiv = np.array([0.0, 0.06,0.1,0.16, 0.2, 0.5, 0.8, 0.84, 0.9,0.94, 1.0])*ctime
				tdiv = np.array([0.0, 0.12,0.2,0.32, 0.4, 0.5, 0.6, 0.68, 0.8,0.88, 1.0])*ctime

			elif (type == 'trapezoidal'):
				pdiv = np.array([0., 0.12, 0.5, 0.88, 1.])
				hdiv = np.array([0.9*sh, sh, sh, sh, 0.9*sh])
				tdiv = np.array([0., 0.32*ctime, 0.4*ctime, 0.5*ctime, 0.6*ctime, 0.68*ctime, ctime ])

			A = np.zeros((1,3))

			## Unloading phase
			for i in range(1,5):
				pu = sp + sn1*(i*sh*hl/4)
				cpx = np.vstack((cpx, pu))

			# Apex point: Compute the via point with height clearance and stack into array
			v3 = sp + (ep - sp)/2
			h3 = sh*(sn1+sn2)/2
			vn  = v3 + h3
			cpx = np.vstack((cpx, vn))
			# print v3, h3, vn

			## Loading phase
			for i in range(4,0,-1):
				pl = ep + sn2*(i*sh*hl/4)
				cpx = np.vstack((cpx, pl))

			# stack final point into array
			cpx = np.vstack((cpx, ep))

		elif (phase==0):
			## Support phase - direct interpolation
			tdiv = np.array([0, ctime])
			cpx  = np.array([sp, ep])

		else:
			print "Invalid phase" 	# have a way to exit function if no phase selected
			return None

		return cpx, tdiv

	def generate_leg_path(self, cp, td, tn):
		""" Generate spline based on provided via points and time intervals """
		""" Input: 	1) cp -> array of via points
					2) td -> time intervals for via points
					3) tn -> trajectory interval
			Output:	List of trajectory (position, velocity, acceleration)	"""

		SpGen = Pspline.SplineGenerator()
		# SpGen = Bspline.SplineGenerator()
		x_out = SpGen.generate_spline(cp, td, tn)

		return x_out

## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##
## Test leg path generation ##
# Ground walking:
# sn1 = np.array([0.,0.,1.])
# sn2 = np.array([0.,0.,1.])
# sp = np.array([0.0, -0.30, -0.1])
# ep = np.array([0.1, -0.30, -0.1 ])
# Ground to wall
sn1 = np.array([0.,0.,1.])
sn2 = np.array([0.,-0.5,0.5])
sp = np.array([0.0, 0.30, -0.1])
ep = np.array([0.0, 0.35,  0.1 ])

phase = 1
if phase ==2 :
	sn1 = np.array([0.,0.,1.])
	sn2 = np.array([0.,-5,5])
	sp = np.array([0.0, 30, -1])
	ep = np.array([0.0, 35,  1 ])
### Test scripts
planner = PathGenerator()
# cxp, tdiv = planner.interpolate_leg_path(sp, ep, (sn1+sn2)/2, phase, False, GAIT_TPHASE)
# cxp, tdiv = planner.new_interpolate_leg_path(sp, ep, sn1, sn2, phase, False, GAIT_TPHASE)
# print cxp
# data = planner.generate_leg_path(cxp, tdiv, CTR_INTV)
# path = TrajectoryPoints(data)
# Plot.plot_2d(path.xp[:,1], path.xp[:,2])
# Plot.plot_2d(data[0],data[1])
# Plot.plot_3d(path.xp[:,0], path.xp[:,1], path.xp[:,2])
# print len(data[0])


x_cob = np.array([.0,.0,BODY_HEIGHT])
w_cob = np.array([.0,.0,.0])
if phase ==2 :
	x_cob = np.vstack((x_cob,np.array([0.2, 0.0, BODY_HEIGHT+0.05])))
	x_cob = np.vstack((x_cob,np.array([0.3, 0.5, BODY_HEIGHT])))
	x_cob = np.vstack((x_cob,np.array([0.1, 0.0, BODY_HEIGHT-0.05])))
	x_cob = np.vstack((x_cob,np.array([0.7, 0.0, BODY_HEIGHT+0.12])))

	w_cob = np.vstack((w_cob,np.array([1.0,.0,.0])))
	w_cob = np.vstack((w_cob,np.array([1.0,.0,.0])))
	w_cob = np.vstack((w_cob,np.array([1.0,.0,.0])))

x_cob = np.vstack((x_cob,np.array([0. , -0.04, BODY_HEIGHT])))
# w_cob = np.vstack((w_cob,np.array([-0.2, -0.1, 0.])))
# x_cob = np.vstack((x_cob,np.array([0.03,  0.04, BODY_HEIGHT])))
# w_cob = np.vstack((w_cob,np.array([0.2, -0.1, 0.])))
# x_cob = np.vstack((x_cob,np.array([.0,.0,BODY_HEIGHT])))
# w_cob = np.vstack((w_cob,np.array([0., 0., 0.])))
t_cob = np.array([0.,2.])
# path_n = planner.generate_base_path(x_cob, w_cob, 0.04, t_cob)
# path_m = planner.generate_base_path(x_cob, w_cob, 0.04, t_cob)
# path_n.insert(6,10,path_m)
# print type(path_n.X.t)
# print np.round(path_n.X.xp,3)
# Plot.plot_2d(path_n.X.t,path_n.X.xp)
# path_n.reverse()
# Plot.plot_2d(path_n.X.t,path_n.X.xp)
# path_o = planner.generate_base_path_old(x_cob, w_cob, 0.1)
# Plot.plot_2d(path_o.W.t,path_o.W.xp,False)

# Plot.plot_2d(new_path.X.t,new_path.X.xp)
# plt.show()
# Plot.plot_2d_multiple(2,x_out[0],x_out[1],x_out[3])

## Joint Recovery =============================================================
qsp = np.zeros((0,3))
for i in range(0,46,5):
	qsp = np.vstack(( qsp, np.array([0.0, np.deg2rad(i), np.deg2rad(i)]) ))

tscale = np.array([range(0,len(qsp))])
rmax = float(np.amax(tscale))
tsp = (tscale/rmax).flatten()

# path_n = planner.generate_leg_path(qsp, tsp, 0.1)
# Plot.plot_2d(path_n[0], path_n[1])

## Wall Transition =============================================================
tran_y = 0.1
tran_z = 0.3

## Ground to Wall
x_cob  = np.array([.0,.0,.0])
w_cob  = np.array([.0,.0,.0])

for q in range(10,91,10):
	qr = np.deg2rad(q)
	xd = np.array([0.0, (1.-np.cos(qr))*tran_y, (np.sin(qr))*tran_z])

	x_cob = np.vstack(( x_cob, xd ))
	w_cob = np.vstack(( w_cob, np.array([qr,0.0,0.0]) ))

# path_n = planner.generate_base_path(x_cob, w_cob, 0.1)
# Plot.plot_2d(path_n.X.t,path_n.X.xp)

## ----------------------------------------------------------------------- ##
## Wall to Ground
x_cob = np.array([0.,tran_y,tran_z])
w_cob = np.array([0., 0., 0.])

for q in range(80,-1,-10):
	qr = np.deg2rad(q)
	xd = np.array([0.0, tran_y-(np.cos(qr))*tran_y, tran_z-(1-np.sin(qr))*tran_z])

	x_cob = np.vstack(( x_cob, xd ))
	w_cob = np.vstack(( w_cob, np.array([qr-np.pi/2,0.0,0.0]) ))

# path_n = planner.generate_base_path(x_cob, w_cob, 0.1)

# Plot.plot_2d(path_n.X.t,path_n.X.xp)
# Plot.plot_3d(path_n.X.xp[:,0], path_n.X.xp[:,1], path_n.X.xp[:,2])
# Plot.plot_3d(path_n.X.xp[:,0], tran_y+path_n.X.xp[:,1], tran_z+path_n.X.xp[:,2])

## write to csv file
# import csv
# with open('trajectory.csv', 'wb') as csvfile:
# 	csvwriter = csv.writer(csvfile, delimiter=',')#, quotechar='|', quoting=csv.QUOTE_MINIMAL)
# 	for i in range(0,len(path.t)):
# 		data_row = np.hstack((path.t[i], path.xp[i], path.xv[i], path.xa[i]))
# 		csvwriter.writerow(data_row)
