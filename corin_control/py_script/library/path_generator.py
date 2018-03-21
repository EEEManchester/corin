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

from matrix_transforms import *
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
		v_max = np.amax(x_out[2]);
		if (v_max > BASE_MAX_LINEAR_VELOCITY):
			print 'max linear velocity exceeded'
			ndiv  = v_max/BASE_MAX_LINEAR_VELOCITY	# determine number of divisions to reduce by
			t_cob = np.round(t_cob*ndiv,1) 			# set new time interval
			# regenerate spline
			x_out = SplineGenerator.generate_spline(x_cob, t_cob, tn)
			w_out = SplineGenerator.generate_spline(w_cob, t_cob, tn)

		# check if base angular velocity exceeds limit
		w_max = np.amax(w_out[2]);
		if (w_max > BASE_MAX_ANGULAR_VELOCITY):
			print 'max angular velocity exceeded'
			ndiv  = w_max/BASE_MAX_ANGULAR_VELOCITY	# determine number of divisions to reduce by
			t_cob = np.round(t_cob*ndiv,1) 			# set new time interval
			# regenerate spline
			x_out = SplineGenerator.generate_spline(x_cob, t_cob, tn)
			w_out = SplineGenerator.generate_spline(w_cob, t_cob, tn)

		# return x_out, w_out 		# python 2D array
		self.base_path = TP.BaseTrajectory((x_out,w_out))
		# return TP.TrajectoryPoints(x_out), TP.TrajectoryPoints(w_out)	# convert to TrajectoryPoints format
		return TP.BaseTrajectory((x_out,w_out))

	def generate_leg_path(self, sp, ep, snorm, phase=1, reflex=False, ctime=2.0, tn=0.1, type='trapezoidal'):
		""" Generate leg trajectory (Re^3) based on phase and type 			"""
		""" First:  introduce via points for transfer phase trajectory
			Second: generate the spline based on the new array of points 	"""
		""" Input: 	1) sp -> starting position (Re^3)
					2) ep -> end position (Re^3)
					3) snorm -> surface normal in unit vector (Re^3)
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
		
		## 2) Generate spline
		# SpGen = Pspline.SplineGenerator()
		SpGen = Bspline.SplineGenerator()
		x_out = SpGen.generate_spline(cpx, td, tn)

		return x_out

	def generate_base_path_old(self, x_com, w_com, tn):

		## Define variables ##
		SplineGenerator = Pspline.SplineGenerator() 	# cubic polynomial spline generator 
		self.x_com = np.array([ [.0,.0,BODY_HEIGHT] ]) 	# output linear spline points
		self.w_com = np.array([ [0.0, 0.0, 0.0] ]) 		# output angular spline points
		self.vorig = np.array([1.0, 0.0, 0.0]) 			# world frame vector
		self.heading = (1,0,0)
		self.com_w_qc = np.array([ [0.0, 0.0, 0.0] ]) 		# base angular: current state
		self.com_w_iq = np.array([ [0.0, 0.0, 0.0] ]) 		# base angular: instantenous rotation (within instance i to i+1)
		self.point_skipped = np.zeros((1,3))

		x_com, w_com, t_com = self.auto_generate_points(x_com, w_com)

		x_out = SplineGenerator.generate_spline( x_com, t_com , 1)
		# Plot.plot_3d(x,y,z)
		# Plot.plot_2d(t,x)
		# travel distance per phase in a gait cycle
		d_lambda = STEP_STROKE*self.gait['dphase']; 	print 'lambda: ', d_lambda

		# counter for new stacked array
		nc = 0

		## robot leg default position parameters
		inst_lm = np.array([ -0.05, STANCE_WIDTH, -BODY_HEIGHT]) 	# left mid leg nominal position
		inst_rm = np.array([ 0.0, -STANCE_WIDTH, -BODY_HEIGHT]) 	# right mid leg nominal position
		tf_world_X_lm = np.array([ [-0.05, STANCE_WIDTH, 0.0] ]) 		# array of left mid foot positions
		tf_world_X_rm = np.array([ [0.0, -STANCE_WIDTH, 0.0] ])			# array of right mid foot positions

		x_prv = x_com[0] 	# previous x_com
		t_prv = t_com[0] 	# previous t_com

		ts = 0 	# sampling time

		# Generate linear trajectory for each segment
		for io in range(1,len(t_com)):
			# print '== ', io, ' ===================='
			# compute magnitude difference between start and end point of this segment
			vd = x_com[io] - x_prv
			si = np.sqrt( vd.dot(vd) )

			# determine number of points to sample within segment
			nk = int(np.round(si/d_lambda))
			# print 'nk  : ', nk
			# determine sampling time interval
			if (nk > 0):
				td = t_com[io] - t_prv 	# time difference between each segment
				ts = td/nk 					# sampling time

				# if valid point exist
				if (ts > 0):
					# normalize time
					ts = ts/td
					td = td/td
					# set t counter
					t = ts

					# generate spline coefficient
					xnew = SplineGenerator.generate_spline( np.array([ x_prv, x_com[io] ]), np.array([0.0,1.0]) , 1)
					# print 't ', t, ' td: ', td, ' ts  : ', ts
					# print 'xcom ', np.round(x_prv,3), np.round(x_com[io],3)
					# print 'xnc  ', np.round(self.x_com[nc],3)

					## loop for each increment
					# while (np.round(t,3) < td):
					tinterval = True
					while (tinterval == True):

						# CoM selection -----------------------------------------------------
						# loop until difference threshold met within certain iteration
						for i in range(0,40):
							# get point at t
							x_cur = xnew[1][int(t)] #self.spline.get_point(t)
							# print 'x_cur', x_cur
							# vector and its magnitude between current and previous point
							m_dif = x_cur - self.x_com[nc]
							x_mag = np.sqrt( m_dif.dot(m_dif) )
							# print 'x_mag: ', x_mag
							# magnitude difference
							x_dif = x_mag-d_lambda
							# print 'xdif: ', x_dif
							# prevent end goal reverse
							g_dif = x_cur - x_com[-1]
							g_mag = np.sqrt( g_dif.dot(g_dif) )

							# check if within end goal range
							if (abs(g_mag) > 0.01):

								# threshold not met, increase/decrease size
								if (abs(x_dif) > 0.001):
									if (x_dif < 0.0):
										t += 0.005
									else:
										t -= 0.005

								# terminate when differece threshold met
								elif (abs(x_dif) < 0.01):
									# print 'loop'
									break

							# end goal reached, terminate
							else:
								print 'target reached'
								break
							# print '-------------------------------'

						## Stack to linear CoM array
						self.x_com = np.vstack(( self.x_com, x_cur ))

						# Body angular movement ---------------------------------------------------

						## change in vector
						inst_linear = m_dif

						## direction
						uvec = inst_linear/(np.linalg.norm(inst_linear)) 	# normalised direction vector

						# axis angle: rotation
						th = -np.arccos(np.dot(uvec,self.vorig));

						# axis angle: unit axis of rotation
						qt = np.cross(uvec,self.vorig);
						if (np.linalg.norm(qt) == 0.0):
							qc = np.array([.0, .0, 1.])
						else:
							try:
								qc = qt/np.linalg.norm(qt)
							except:
								qc = np.array([.0, .0, 1.])

						## determine angular movement: global body rotation, fr_world_X_base (angular rotation), n+1
						sorth_R  = axisAngle_to_SO3(qc, th) 				# SO(3) from axis angle
						euler_R  = euler_from_matrix(sorth_R, 'sxyz')	# euler from SO(3)

						# vector projected onto xy-plane
						a = np.sqrt(uvec.item(0)**2+uvec.item(1)**2)

						# Rotation about z dependant on heading direction
						if (self.heading == (1,0,0)):
							ry = np.arctan2(uvec.item(2),a) 				# rotation about y-axis
							rz = np.arctan2(uvec.item(1),uvec.item(0))  	# rotation about z-axis
							euler_R = (0.0, ry, rz)
						elif (self.heading == (0,1,0)):
							rx = np.arctan2( uvec.item(2),uvec.item(1))
							rz = np.arctan2(-uvec.item(0),uvec.item(1))
							euler_R = (rx, 0.0, rz)

						## change in rotation between instance n to n+1
						self.com_w_iq = np.vstack(( self.com_w_iq, euler_R - self.com_w_qc ))

						## Stack to angular base array - if no angular movement specified
						self.w_com = np.vstack((self.w_com, euler_R))

						# Foothold selection ---------------------------------------------------
						## Compute foothold & stack to foothold array
						# self.tf_world_X_lm = np.vstack((  self.tf_world_X_lm, self.x_com[nc] + (inst_linear + np.dot(sorth_R, inst_lm))  ))  # delta_x_com + R_com*bpi_nom
						# self.tf_world_X_rm = np.vstack((  self.tf_world_X_rm, self.x_com[nc] + (inst_linear + np.dot(sorth_R, inst_rm))  ))

						## increment counters and update world_X_base rotation
						t += ts
						nc += 1
						self.com_w_qc = self.w_com[nc]

						# check if loop should run
						if (np.round(t,3) > td):
							tinterval = False

					# Finally, update x_prv and t_com if valid point exist
					x_prv = x_com[io]
					t_prv = t_com[io]

			else:
				self.point_skipped = np.vstack(( self.point_skipped, x_com[io] ))
				# print 'point skipped'

		print '=========================================='

		# clean up
		self.point_skipped = np.delete(self.point_skipped,0,0)

		# determine linear spline timing
		self.t_com  = np.zeros(len(self.x_com))
		self.tw_com = np.zeros(len(self.x_com))
		for i in range(1,len(self.x_com)):
			self.t_com[i]  = i*TRAC_PERIOD 	#*self.gait['dphase'].denominator
			self.tw_com[i] = i*TRAC_PERIOD	#*self.gait['dphase'].denominator

		# Generate angular trajectory IF specified
		try:
			ssize = w_com.size
			self.w_com = w_com

			# interpolate angular spline timing according to linear timing
			wseg = self.t_com[-1]/(len(self.w_com)-1) 		# time interval for each angular segment
			self.tw_com = np.zeros(len(self.w_com))  		# angular timing size

			for i in range(1,len(self.w_com)):
				self.tw_com[i] = wseg*i
		except:
			print "exception raised"
			pass

		# temporary fix for angular rotation in place
		if (len(self.tw_com)==2 and self.tw_com.item(1) == 0.):
			self.tw_com = np.array([0.0,3.0])

		print self.t_com
		print self.x_com
		# Regenerate linear and angular spline with updated CoM position
		x_out = SplineGenerator.generate_spline(self.x_com, self.t_com,  tn)
		w_out = SplineGenerator.generate_spline(self.w_com, self.tw_com, tn)

		# print self.x_com
		# Plot.plot_3d(nx, nv, na)
		# Plot.plot_2d(nt,na)
		# Plot.plot_2d_multiple(3,wt,wx,wv,wa)

		self.base_path = TP.BaseTrajectory((x_out,w_out))
		# return TP.TrajectoryPoints(x_out), TP.TrajectoryPoints(w_out)	# convert to TrajectoryPoints format
		return TP.BaseTrajectory((x_out,w_out))
## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##
## Test leg path generation ##
sp = np.array([0.50, -0.10, -0.05])
ep = np.array([0.50,  0.10, -0.05])
snorm = np.array([0., 0., 0.])
phase = 1

### Test scripts
spliner = PathGenerator()
# xout = spliner.generate_leg_path(sp, ep, snorm, phase)

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

planner = PathGenerator()
# planner.gait = gait
path_n = planner.generate_base_path(x_cob, w_cob, 0.1)
# path_o = planner.generate_base_path_old(x_cob, w_cob, 0.1)
# Plot.plot_2d(path_o.W.t,path_o.W.xp,False)
# Plot.plot_2d(path_n.X.t,path_n.X.xp)
# plt.show()
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


# nt,nx,nv,na = planner.generate_base_path(x_cob, w_cob)
# planner.get_path_details()
# Plot.plot_3d(nx,nv,na)
# Plot.plot_2d_multiple(3,nt,nx,nv,na)

# print planner.x_cob
# print '======================'
# print planner.w_cob
# print planner.t_cob
