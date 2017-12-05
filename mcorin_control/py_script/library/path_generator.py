#!/usr/bin/env python

## Class to plan a path for the robot given arbitrary points

# sys.dont_write_bytecode = True

import time
from fractions import Fraction
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# import robot_class
# import param_gait
import plotgraph as Plot
from constant import *

import transformations as tf
import pspline_generator as Pspline
import bspline_generator as Bspline


class PathGenerator(Pspline.SplineGenerator):
	def __init__(self):
		Pspline.SplineGenerator.__init__(self)

		self.x_com = np.array([ [.0,.0,BODY_HEIGHT] ]) 	# output linear spline points
		self.w_com = np.array([ [0.0, 0.0, 0.0] ]) 		# output angular spline points
		self.t_com = np.zeros(0) 						# time interval for output spline points
		self.tw_com = np.zeros(0)

		self.x_length = 0 	# output linear spline length
		self.w_length = 0	# output angular spline length

		self.com_w_qc = np.array([ [0.0, 0.0, 0.0] ]) 		# base angular: current state 
		self.com_w_iq = np.array([ [0.0, 0.0, 0.0] ]) 		# base angular: instantenous rotation (within instance i to i+1)

		# foot placements - can be removed
		self.inst_lm = np.array([ 0.0, STANCE_WIDTH, -BODY_HEIGHT]) 	# left mid leg nominal position
		self.inst_rm = np.array([ 0.0, -STANCE_WIDTH, -BODY_HEIGHT]) 	# right mid leg nominal position
		self.tf_world_X_lm = np.array([ [-0.0, STANCE_WIDTH, 0.0] ]) 	# array of left mid foot positions
		self.tf_world_X_rm = np.array([ [0.0, -STANCE_WIDTH, 0.0] ])	# array of right mid foot positions

		self.vorig = np.array([1.0, 0.0, 0.0]) 		# world frame vector
		self.heading = (1,0,0)

		self.gait = {} 		# dictionary for gait parameters

		self.point_skipped = np.zeros((1,3))

		self.count = 1;

	def get_path_details(self):
		gait_phases = len(self.x_com)-1
		gait_cycles = gait_phases/self.gait['beta'].denominator
		total_time	= (self.x_length-1.)*TRAC_INTERVAL

		print 'Gait  type: ', self.gait['name']
		print 'No. phases: ', gait_phases
		print 'No. cycles: ', gait_cycles
		print 'Total time: ', total_time
		
	def compute_path_length(self, via_points):
		point_size = len(via_points) 	# determine number of via points
		t_com = np.zeros(point_size) 	# create an array of via points size

		for i in range(0,point_size):
			t_com[i] = i
		return t_com

	def auto_generate_points(self, x_com, w_com):
		xsize = x_com.size 	# no. of items in array
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

		return x_com, w_com

	def generate_path(self, x_com, w_com=0):
		x_com, w_com = self.auto_generate_points(x_com, w_com)
		print w_com
		tx_com = self.compute_path_length(x_com)

		x, y, z, t = self.generate_body_spline( x_com, tx_com , 1)
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
		t_prv = tx_com[0] 	# previous t_com

		ts = 0 	# sampling time

		# Generate linear trajectory for each segment
		for io in range(1,len(tx_com)):
			# print '== ', io, ' ====================' 
			# compute magnitude difference between start and end point of this segment
			vd = x_com[io] - x_prv
			si = np.sqrt( vd.dot(vd) ) 
			
			# determine number of points to sample within segment
			nk = int(np.round(si/d_lambda))
			# print 'nk  : ', nk
			# determine sampling time interval
			if (nk > 0):
				td = tx_com[io] - t_prv 	# time difference between each segment
				ts = td/nk 					# sampling time
				
				# if valid point exist
				if (ts > 0):
					# normalize time
					ts = ts/td
					td = td/td
					# set t counter
					t = ts
										
					# generate spline coefficient
					xp, yp, zp, tp = self.generate_body_spline( np.array([ x_prv, x_com[io] ]), np.array([0.0,1.0]) , 1)

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
							x_cur = self.get_point(t) 
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
						sorth_R  = tf.axisAngle_to_SO3(qc, th) 				# SO(3) from axis angle
						euler_R  = tf.euler_from_matrix(sorth_R, 'sxyz')	# euler from SO(3)

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
						self.tf_world_X_lm = np.vstack((  self.tf_world_X_lm, self.x_com[nc] + (inst_linear + np.dot(sorth_R, inst_lm))  ))  # delta_x_com + R_com*bpi_nom
						self.tf_world_X_rm = np.vstack((  self.tf_world_X_rm, self.x_com[nc] + (inst_linear + np.dot(sorth_R, inst_rm))  ))

						## increment counters and update world_X_base rotation
						t += ts
						nc += 1
						self.com_w_qc = self.w_com[nc]
						
						# check if loop should run
						if (np.round(t,3) > td):
							tinterval = False

					# Finally, update x_prv and t_com if valid point exist
					x_prv = x_com[io]
					t_prv = tx_com[io]

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
		print self.w_com
		print self.tw_com
		## Regenerate linear and angular spline with updated CoM position
		nx, nv, na, nt = self.generate_body_spline(self.x_com, self.t_com,  1)
		wx, wv, wa, wt = self.generate_body_spline(self.w_com, self.tw_com, 1)
		# print self.x_com
		# print self.point_skipped
		self.x_length = len(nt)
		self.w_length = len(wt)
		Plot.plot_3d(nx, nv, na)
		Plot.plot_2d(nt,na)
		# Plot.plot_2d_multiple(3,wt,wx,wv,wa)

		return self.x_com, self.w_com, self.t_com, self.tw_com, self.x_length, self.w_length
		
gait = {'name': "wave",
		'beta': Fraction(5, 6),
		'dphase': Fraction(1, 5),
		'phase':np.matrix([ Fraction(6, 6), Fraction(5, 6), Fraction(4, 6), Fraction(3, 6), Fraction(2, 6), Fraction(1, 6) ])}

x_com = np.array([.0,.0,BODY_HEIGHT])
w_com = np.array([.0,.0,.0])
## short V spline
# x_com = np.vstack((x_com,np.array([0.0, 0.4, BODY_HEIGHT])))
# x_com = np.vstack((x_com,np.array([0.0, 0.6, BODY_HEIGHT])))
# x_com = np.vstack((x_com,np.array([0.0, 1.0, BODY_HEIGHT+0.12])))

x_com = np.vstack((x_com,np.array([0.0, 0.0, BODY_HEIGHT+0.06])))
# x_com = np.vstack((x_com,np.array([0.0, 0.0, BODY_HEIGHT-0.06])))
# x_com = np.vstack((x_com,np.array([0.7, 0.0, BODY_HEIGHT+0.12])))

# w_com = np.vstack((x_com,np.array([0.0, 0.0, 0.0])))
# w_com = np.vstack((x_com,np.array([0.0, 0.0, 0.0])))
# w_com = np.vstack((x_com,np.array([0.0, 0.0, 0.0])))

planner = PathGenerator()
planner.heading = (1,0,0)
planner.gait = gait
# planner.generate_path(x_com, w_com)

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