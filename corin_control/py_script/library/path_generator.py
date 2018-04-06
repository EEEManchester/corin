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
	def __init__(self):
		self.base_path = Trajectory6D() 		# class for 6D path storage
		
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

	def generate_base_path(self, x_cob, w_cob, tn):
		""" Generate body trajectory (Re^6) and modify to be within velocity limit 	"""
		""" Size of x_cob & w_cob needs to be the same otherwise the array with 
			the smaller size will be ignored and set to zero 						"""
		""" Input: 	1) x_cob -> array of via points for linear translation
					2) w_cob -> array of via points for angular rotations 	
			Output:	BaseTrajectory() array of linear translation and angular
					rotation points 												"""

		SplineGenerator = Bspline.SplineGenerator()	# biezer spline generator
		# SplineGenerator = Pspline.SplineGenerator() # cubic polynomial spline generator 

		# generate linear or angular via points if size mismatch
		x_cob, w_cob, t_cob = self.auto_generate_points(x_cob, w_cob)
		
		# generate spline based on unit time interval between via points
		x_out = SplineGenerator.generate_spline(x_cob, t_cob, tn)
		w_out = SplineGenerator.generate_spline(w_cob, t_cob, tn)

		# check if base linear velocity exceeds limit
		v_max = np.amax(x_out[2])
		v_min = np.amin(x_out[2])
		v_max = v_max if (abs(v_max) > abs(v_min)) else abs(v_min)
		if (v_max > BASE_MAX_LINEAR_VELOCITY):
			print 'max linear velocity exceeded'
			ndiv  = v_max/BASE_MAX_LINEAR_VELOCITY	# determine number of divisions to reduce by
			t_cob = np.round(t_cob*ndiv,1) 			# set new time interval
			# regenerate spline
			x_out = SplineGenerator.generate_spline(x_cob, t_cob, tn)
			w_out = SplineGenerator.generate_spline(w_cob, t_cob, tn)

		# check if base angular velocity exceeds limit
		w_max = np.amax(w_out[2])
		w_min = np.amin(w_out[2])
		w_max = w_max if (abs(w_max) > abs(w_min)) else abs(w_min)
		if (w_max > BASE_MAX_ANGULAR_VELOCITY):
			print 'max angular velocity exceeded'
			ndiv  = w_max/BASE_MAX_ANGULAR_VELOCITY	# determine number of divisions to reduce by
			t_cob = np.round(t_cob*ndiv,1) 			# set new time interval
			# regenerate spline
			x_out = SplineGenerator.generate_spline(x_cob, t_cob, tn)
			w_out = SplineGenerator.generate_spline(w_cob, t_cob, tn)

		self.base_path = Trajectory6D((x_out,w_out))
		# return TrajectoryPoints(x_out), TrajectoryPoints(w_out)	# convert to TrajectoryPoints format
		return Trajectory6D((x_out,w_out))

	def interpolate_leg_path(self, sp, ep, snorm, phase=1, reflex=False, ctime=2.0, tn=0.1, type='parabolic'):
		## Define Variables ##
		cpx = np.zeros((1,3))	# cartesian position in matrix
		sh  = STEP_HEIGHT  		# step height for transfer phase
		
		## 1) Set via points according to phase
		if (phase==1):						
			## Transfer phase 

			# vector from origin to travel path midpoint
			if (type=='parabolic'):
				pdiv = np.array([0.125, 0.5, 0.875]) 		# time division
				sdiv = np.array([0.6*sh, sh, 0.6*sh])
				td   = np.array([0, 0.25*ctime, 0.5*ctime, 0.75*ctime, ctime ])
			elif (type == 'trapezoidal'):
				pdiv = np.array([0., 0.12, 0.5, 0.88, 1.])
				sdiv = np.array([0.9*sh, sh, sh, sh, 0.9*sh])
				td   = np.array([0, 0.3*ctime, 0.4*ctime, 0.5*ctime, 0.6*ctime, 0.7*ctime, ctime ])

			A = np.zeros((1,3))

			for i in range(0,len(pdiv)):
				if (reflex and i==0):
					print 'TRIGGER REFLEX'
					ch = np.array([0.0, +0.03, 0.0])
					m3 = v3_X_m(sp + (ep - sp)*pdiv[i] + ch)	# via point 
				else:
					m3 = v3_X_m(sp + (ep - sp)*pdiv[i]) 		# via point

				# surface orientation; offset due to surface orientation = Rot(x,theta)*[0, 0, z]'
				nm = rotation_matrix(snorm[0],[1, 0, 0])
				zh = v3_X_m(np.array([0.,0.,sdiv[i]]))

				# compute clearance point in SCS, SE(3) & stack into array
				mp = mX(m3,nm,zh)
				A  = np.vstack( (A, np.array([ mp[0,3],mp[1,3],mp[2,3] ]) ) )
				# print nm

			A  = np.delete(A, 0, 0) # clean up first row of matrix

			cpx = sp
			cpx = np.vstack( (cpx, A ))
			cpx = np.vstack( (cpx, ep ))
			
		elif (phase==0):
			## support phase - direct interpolation NOT USED ACTUALLY
			td  = np.array( [0, ctime] )
			cpx = np.array([ sp, ep])

		else:
			print "No phase" 	# have a way to exit function if no phase selected

		return cpx, td

	def generate_new_leg_path(self, cpx, td, tn):
		SpGen = Bspline.SplineGenerator()
		x_out = SpGen.generate_spline(cpx, td, tn)

		return x_out

	def generate_leg_path(self, sp, ep, snorm, phase=1, reflex=False, ctime=2.0, tn=0.1, type='parabolic'):
		""" Generate leg trajectory (Re^3) based on phase and type 			"""
		""" First:  introduce via points for transfer phase trajectory
			Second: generate the spline based on the new array of points 	"""
		""" Input: 	1) sp -> starting position (Re^3)
					2) ep -> end position (Re^3)
					3) snorm -> surface normal in unit vector (Re^3) 
								wrt to leg frame
					4) phase -> leg phase: 1 = transfer, 0 = support
					5) reflex -> boolean for triggering reflex
					6) ctime -> duration of trajectory
					7) tn -> trajectory interval
					8) type -> spline shape 
			Output:	List of trajectory (position, velocity, acceleration)	"""

		## Define Variables ##
		cpx = np.zeros((1,3))	# cartesian position in matrix
		sh  = STEP_HEIGHT  		# step height for transfer phase
		
		## 1) Set via points according to phase
		if (phase==1):						
			## Transfer phase 

			# vector from origin to travel path midpoint
			if (type=='parabolic'):
				pdiv = np.array([0.125, 0.5, 0.875]) 		# time division
				sdiv = np.array([0.6*sh, sh, 0.6*sh])
				td   = np.array([0, 0.25*ctime, 0.5*ctime, 0.75*ctime, ctime ])
			elif (type == 'trapezoidal'):
				pdiv = np.array([0., 0.12, 0.5, 0.88, 1.])
				sdiv = np.array([0.9*sh, sh, sh, sh, 0.9*sh])
				td   = np.array([0, 0.3*ctime, 0.4*ctime, 0.5*ctime, 0.6*ctime, 0.7*ctime, ctime ])

			A = np.zeros((1,3))

			for i in range(0,len(pdiv)):
				if (reflex and i==0):
					print 'TRIGGER REFLEX'
					v3 = sp + (ep - sp)*pdiv.item(i) + np.array([0.0, +0.03, 0.0]) 	# via point 
				else:
					v3 = sp + (ep - sp)*pdiv.item(i) 	# via point

				# vector to SE(3)
				m3 = np.matrix([ [1, 0, 0, v3.item(0)], [0, 1, 0, v3.item(1)], [0, 0, 1, v3.item(2)], [0, 0, 0, 1] ])
				
				# surface orientation; offset due to surface orientation = R(y,theta)*[0, 0, z]'
				ry = snorm.item(1) 	# rotation about y (leg frame pitch, world frame roll)
				nm = np.matrix([ [np.cos(ry), 0., np.sin(ry), 0.],[0., 1., 0., 0.],
								 [-np.sin(ry), 0., np.cos(ry), 0.],[0.,0.,0.,1.] ])
				zh = np.matrix([ [1, 0, 0, 0.], [0, 1, 0, 0], [0, 0, 1, sdiv.item(i)], [0, 0, 0, 1] ]) 	
				
				# compute clearance point in SCS, SE(3) & stack into array
				mp = m3*nm*zh
				A  = np.vstack( (A, np.array([ mp[0,3],mp[1,3],mp[2,3] ]) ) )

			A  = np.delete(A, 0, 0) # clean up first row of matrix

			cpx = sp
			cpx = np.vstack( (cpx, A ))
			cpx = np.vstack( (cpx, ep ))
			
		elif (phase==0):
			## support phase - direct interpolation NOT USED ACTUALLY
			td  = np.array( [0, ctime] )
			cpx = np.array([ sp, ep])

		else:
			print "No phase" 	# have a way to exit function if no phase selected
		# print np.round(cpx,4)
		## 2) Generate spline
		# SpGen = Pspline.SplineGenerator()
		SpGen = Bspline.SplineGenerator()
		x_out = SpGen.generate_spline(cpx, td, tn)

		return x_out

## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##
## Test leg path generation ##
sp = np.array([0.50, -0.10, -0.05])
ep = np.array([0.50,  0.10, -0.05])
sn = np.array([0., 0., 0.])
phase = 1

sp = np.array([-0.115,  -0.3027, -0.068 ])
ep = np.array([-0.115,  -0.3368, -0.0666])
sn = np.array([0.,     0.0998, 0.995])

### Test scripts
planner = PathGenerator()
cxp, td = planner.interpolate_leg_path(sp, ep, sn, phase)
print np.round(cxp,4)
# for bp in cxp:
# 	print bp
for i in range (4):
	print i
# Plot.plot_2d(xout[0],xout[1])
# cx = np.zeros(len(xout[0]))
# cy = np.zeros(len(xout[0]))
# cz = np.zeros(len(xout[0]))
# for i in range(0,len(xout[0])):
# 	data  = xout[1][i]
# 	cx[i] = data[0]
# 	cy[i] = data[1] 
# 	cz[i] = data[2] 
	
# Plot.plot_2d(xout[0],xout[1])
# Plot.plot_3d(cx,cy,cz)

# print x_out

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
# x_cob = np.vstack((x_cob,np.array([0.03,  0.04, BODY_HEIGHT])))
# w_cob = np.vstack((w_cob,np.array([0.2, -0.1, 0.])))
# x_cob = np.vstack((x_cob,np.array([.0,.0,BODY_HEIGHT])))
# w_cob = np.vstack((w_cob,np.array([0., 0., 0.])))

# planner = PathGenerator()
# planner.gait = gait
# path_n = planner.generate_base_path(x_cob, w_cob, 0.1)
# print type(path_n.X.t)
# path_o = planner.generate_base_path_old(x_cob, w_cob, 0.1)
# Plot.plot_2d(path_o.W.t,path_o.W.xp,False)
# Plot.plot_2d(path_n.X.t,path_n.X.xp)
# plt.show()
# Plot.plot_2d_multiple(2,x_out[0],x_out[1],x_out[3])
## wall transition
# cq = 0.
# t_cob = np.array([0.0])

x_cob  = np.array([.0,.0,.0])
w_cob  = np.array([.0,.0,.0])
tran_y = 0.1
for q in range(5,91,5):
	qr = np.deg2rad(q)
	xd = np.array([0.0, (1.-np.cos(qr))*tran_y, (np.sin(qr))*tran_y])
	
	x_cob = np.vstack(( x_cob, xd ))
	w_cob = np.vstack(( w_cob, np.array([qr,0.0,0.0]) ))
# path_n = planner.generate_base_path(x_cob, w_cob, 0.1)
# Plot.plot_2d(path_n.X.t,path_n.X.xp)

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


# nt,nx,nv,na = planner.generate_base_path(x_cob, w_cob)
# planner.get_path_details()
# Plot.plot_3d(nx,nv,na)
# Plot.plot_2d_multiple(3,nt,nx,nv,na)