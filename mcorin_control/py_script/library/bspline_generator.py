#!/usr/bin/env python
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'class'))

from constant import *
from TrajectoryPoints import TrajectoryPoints
import plotgraph as Plot

import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy import linalg
from scipy import arange

class SplineGenerator:
	def __init__(self):
		self.p  = 4; 			# order of spline
		self.n  = 2; 			# derivative degree
		self.C  = np.matrix([0])
		self.t0 = np.array([0.0])
		self.U  = np.array([0.0])
		self.n_knot = 0

	## For a start and end point, interpolate accordingly
	def LegPhase_Interpolation(self, phase, sp, ep, qsurface, reflex, ctime=2.0, p=4):
		vi = 0.0; 	vf = 0.0;
		ai = 0.0;	af = 0.0;

		sh = STEP_HEIGHT #param_gait.get_step_height()		
		
		# reject displacements less than 1mm
		pd = np.array([0, 0, 0])
		for i in range (0,3):
			pd[i] = ep[i]-sp[i]
			if (phase > 1):
				if (abs(pd[i]) < 0.001):
					pd[i] = 0

		# transfer phase 
		if (phase==1):						
			# vector from origin to travel path midpoint
			pdiv = np.array([0.125,0.5,0.875]) 		# intervals for via points, in unity form
			sdiv = np.array([0.6*sh,sh,0.6*sh])		# height clearance at each via points
			A = np.array([ [0],[0],[0]])

			for i in range(0,len(pdiv)):
				if (reflex and i==0):
					print 'TRIGGER REFLEX'
					v3 = sp + (ep - sp)*pdiv.item(i) + np.array([0.0, +0.03, 0.0]) 	# via point 
				else:
					v3 = sp + (ep - sp)*pdiv.item(i) 	# via point
				
				# vector to SE(3)
				m3 = np.matrix([ [1, 0, 0, v3.item(0)], [0, 1, 0, v3.item(1)], [0, 0, 1, v3.item(2)], [0, 0, 0, 1] ])
				
				# # surface normal; offset due to surface orientation = R(y,theta)*[0, 0, z]'
				# nm = np.matrix([ [1, 0, 0, -sdiv.item(i)*np.sin(qsurface[0])], [0, 1, 0, 0], [0, 0, 1, sdiv.item(i)*np.cos(qsurface[0])], [0, 0, 0, 1] ]) 	

				# # compute clearance point in SCS, SE(3)
				# mp = m3*nm
				##============================================
				# surface orientation; offset due to surface orientation = R(y,theta)*[0, 0, z]'
				ry = qsurface.item(1) 	# rotation about y (leg frame pitch, world frame roll)
				nm = np.matrix([ [np.cos(ry), 0., np.sin(ry), 0.],[0., 1., 0., 0.],[-np.sin(ry), 0., np.cos(ry), 0.],[0.,0.,0.,1.] ])
				zh = np.matrix([ [1, 0, 0, 0.], [0, 1, 0, 0], [0, 0, 1, sdiv.item(i)], [0, 0, 0, 1] ]) 	
				
				# compute clearance point in SCS, SE(3)
				mp = m3*nm*zh

				A = np.hstack( (A, np.array([ [mp[0,3]],[mp[1,3]],[mp[2,3]] ]) ) )
			
			A = np.delete(A, 0, 1) # remove first column of matrix

			self.t0 = np.array( [0, 0.25*ctime, 0.5*ctime, 0.75*ctime, ctime ])
			self.C  = np.zeros((3,len(self.t0)+4))
			
			for i in range(0,3):
				k = 0
				for j in range(self.p-1,self.C.shape[1]-self.p+1):
					self.C[i][j] = A.item(i,k)
					k += 1

		# support phase
		elif (phase==0):					
			self.t0 = np.array( [0, ctime] )
			self.C = np.zeros((3,len(self.t0)+4))

		for i in range(0,3):
			self.C[i][0] = sp[i]
			self.C[i][-1] = ep[i]

		self.C = np.matrix(self.C)

	def point_Interpolation(self, cpath, ctime):
		vi = 0.0; 	vf = 0.0;
		ai = 0.0;	af = 0.0;

		self.t0 = ctime
		self.C  = np.zeros((3,len(self.t0)+4))
		# print self.p-1,self.C.shape[1]-self.p+1
		# print cpath
		for i in range(0,3):
			k = 0
			for j in range(self.p-1,self.C.shape[1]-self.p+1):
				self.C[i][j] = cpath.item(j-(self.p-2),i)
				# print cpath.item(j-(self.p-2),i), j-(self.p-2), j
				k += 1
		
		for i in range(0,3):
			self.C[i][0] = cpath[0][i]
			self.C[i][-1] = cpath[-1][i]
		
		self.C = np.matrix(self.C)
		# print self.C

	## Determine knots required
	def knotFunction(self):
		self.U = np.zeros(len(self.t0)+2*self.p+1)

		for i in range(0,len(self.t0)+2*self.p+1):
			if (i<=self.p):
				self.U[i] = self.t0.item(0) 
			elif (i>=self.p and i<len(self.t0)+self.p):
				self.U[i] = (self.t0.item(i-self.p)+self.t0.item(i-self.p-1))/2.0 
			else:
				self.U[i] = self.t0.item(-1)

		self.n_knot = len(self.U) - 1

	## Compute i for a given u
	def whichSpan(self, u, U, n_knot, p):
		if (u==U[-1]):
			u = u - 0.001

		high = n_knot - p;
		low = p;
		if (u == U[high]):
			mid = high;
		else:
			mid = (high+low)/2;
			while ((u<U[mid]) or (u>=U[mid+1])):
				# print u, U[mid], U[mid+1]
				if (u==U[mid+1]):
					mid = mid+1; 			# knot with multiplicity >1 */
				else:
					if (u > U[mid]):
						low = mid;
					else:
						high=mid;
					mid = (high+low)/2;
		return mid

	## Basis functions with derivatives
	def DersBasisFunction(self, u, U, p, i, n=2):
		# initialize variables
		MAX_P = 6
		matrix_size = 5
		DR = np.zeros(MAX_P+1)
		DL = np.zeros(MAX_P+1)
		Du = np.zeros((matrix_size,matrix_size))
		a  = np.zeros((matrix_size,matrix_size))
		Ders = np.zeros((3,matrix_size))
		
		Du[0][0] = 1.0;

		for j in range(1,p+1):

			DL[j] = u - U[i+1-j];
			DR[j] = U[i+j]-u;
			acc = 0.0;
			for r in range(0,j):
				Du[j][r] = DR[r+1] + DL[j-r];
				try:
					temp = float(Du[r][j-1]) / float(Du[j][r]);
				except ZeroDivisionError:
					temp = 0.0
				Du[r][j] = acc + DR[r+1] * temp;
				acc = DL[j-r] * temp;

			Du[j][j] = acc;

		for j in range(0,p+1):
			Ders[0][j] = Du[j][p];

		## Derivatives of basis functions
		for r in range(0,p+1):

			s1=0;
			s2=1;
			a[0][0] = 1.0;
			for k in range(1,n+1):

				d = 0.0;
				rk = r - k;
				pk = p - k;
				if (r >= k):
					try:
						a[s2][0] = float(a[s1][0]) / float(Du[pk+1][rk]);
					except ZeroDivisionError:
						a[s2][0] = 0.0
					d = a[s2][0] * Du[rk][pk];

				if (rk >= -1):
					j1 = 1;
				else:
					j1 = -rk;

				if (r-1 <= pk):
					j2 = k - 1;
				else:
					j2 = p - r;

				for j in range(j1,j2+1):
					try:
						a[s2][j] = float((a[s1][j] - a[s1][j-1])) / float(Du[pk+1][rk+j]);
					except ZeroDivisionError:
						a[s2][j] = 0.0
					d += a[s2][j] * Du[rk+j][pk];
					
				if (r <= pk):
					try:
						a[s2][k] = float(-a[s1][k-1]) / float(Du[pk+1][r]);
					except ZeroDivisionError:
						a[s2][k] = 0.0
					d += a[s2][k] * Du[r][pk];

				Ders[k][r] = d;
				j = s1; s1 = s2; s2 = j;
			
		r = p;

		for k in range(1,n+1):
			for j in range(0,p+1):
				Ders[k][j] *= r;
			r *= (p-k);

		return Ders

	## Calculate control points
	def controlPoint(self, d=1):
		A = np.zeros((self.n_knot-self.p,self.n_knot-self.p))
		row_counter = 0
		z_max = len(self.t0)

		for z in range(0,z_max):
			# check to prevent u = umax
			if (self.U[-1]==self.t0[z]):		
				u = self.t0[z] - 0.001	
			else:
				u = self.t0[z]

			i = self.whichSpan(u, self.U, self.n_knot, self.p) 		# compute i

			# order of derivatives & offset for stacking
			if (z==0.0 or z==z_max-1):
				n = 2;	offset = 3; 			
			else:
				n = 0;	offset = 1;

			temp = self.DersBasisFunction(u, self.U, self.p, i, n) 	# compute control point
			
			# Stack into matrix A
			if (z==z_max-1):
				for y in range(offset-1,0,-1):
					for q in range(0,5):
						A[row_counter][z+q] = temp[y][q]
					
					row_counter+=1
			else:
				for y in range(0,offset):
					for q in range(0,5):
						A[row_counter][z+q] = temp[y][q]
					
					row_counter+=1

		A[-1][-1] = 1.0 	# first and last element always unity
		# np.set_printoptions(formatter={'float': lambda A: "{0:0.3f}".format(A)})
		# print A.shape, C.shape
		P = linalg.solve(A,self.C.getT()) 											# control points

		return P
	
	## Spline function for given time t
	def evaluateSpline(self, u, P, d=1):
		s   = np.zeros(d);
		sv  = np.zeros(d);
		sa  = np.zeros(d);

		i = self.whichSpan(u, self.U, self.n_knot, self.p);
		A =	self.DersBasisFunction(u, self.U, self.p, i)
		
		for k in range(0,d): 	#/* For each components of the B-spline*/
			s[k] = 0;
			sv[k] = 0;
			sa[k] = 0;
			for j in range(0,self.p+1):
				s[k]  = s[k]  + P[i-self.p+j][k]*A[0][j];
				sv[k] = sv[k] + P[i-self.p+j][k]*A[1][j];
				sa[k] = sa[k] + P[i-self.p+j][k]*A[2][j];

		return s, sv, sa

	def spline_generation(self, return_type=0):
		self.knotFunction() 									# determine U, n_knot
		P = self.controlPoint() 								# calculate control point P

		x_com = TrajectoryPoints()

		## Evaluate spline
		t_inc = TRAC_INTERVAL 	
		for i in arange(0,self.t0[-1]+t_inc,t_inc):
			t1, t2, t3 = self.evaluateSpline(i, P, 3)
			
			x_com.xp = np.vstack( (x_com.xp,t1) )
			x_com.xv = np.vstack( (x_com.xv,t2) )
			x_com.xa = np.vstack( (x_com.xa,t3) )
			x_com.t  = np.hstack( (x_com.t ,i ) )

		# print np.around(qp,decimals=3)
		if (return_type==0):
			return x_com
		elif (return_type==1):
			return x_com.xp[:,0], x_com.xp[:,1], x_com.xp[:,2], qt

	def generate_leg_spline(self, sp, ep, qsurface, phase, reflex=False, td=2.0, return_type=0):
		# print 'td: ', td
		self.LegPhase_Interpolation(phase, sp, ep, qsurface, reflex, td) 	# determine C
		
		return self.spline_generation(return_type)

	def generate_body_spline(self, com_path, time, return_type=0):
		self.point_Interpolation(com_path, time) 					# determine C
		
		return self.spline_generation(return_type)
		

## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##
sp = np.array([0.50, -0.10, -0.05])
ep = np.array([0.50,  0.10, -0.05])
qsurface = np.array([0, -1.571/2., 0.])
phase = 1

qpoints = TrajectoryPoints()
### Test scripts
spliner = SplineGenerator()
# qpoints = spliner.generate_leg_spline(sp, ep, qsurface, phase, 0, TRAC_PERIOD, 0)

# print (qpoints.t)
# Plot.plot_3d(x,y,z)
# Plot.plot_2d(t,z)
# fig = plt.figure()
# ax = fig.gca(projection='3d')
# ax.plot(x, y, z, label='parametric curve')
# plt.show()

### CoM linear path ###
# com_path = np.array([.0,.0,0.15])
# point_1  = np.array([0.05,0.0,0.15])
# com_path = np.vstack((com_path,point_1))
# point_1  = np.array([0.1,0.0,0.15])
# com_path = np.vstack((com_path,point_1))
# # point_1  = np.array([3.0,3.0,0.15])
# # com_path = np.vstack((com_path,point_1))
# # point_1  = np.array([4.0,4.0,0.15])
# # com_path = np.vstack((com_path,point_1))
# com_time = np.array([0.0,1.0,2.0])

# x, y, z, t  = spliner.generate_body_spline(com_path, com_time)

# fig = plt.figure()
# ax = fig.gca(projection='3d')
# ax.plot(x, y, z, label='parametric curve')
# plt.xlabel('x-axis');plt.ylabel('y-axis');
# plt.show()
# fig = plt.figure()
# plt.plot(t,x)
# plt.xlabel('x-axis');plt.ylabel('y-axis');
# plt.show()

# Path3D_msg(com_path, com_time)
# def Path3D_msg(linear, angular=0, t=0):
# 	com_path = Path3D()

# 	com_t = Point();	com_r = Quaternion();
# 	com_t.x = 1.0;		com_r.x = 0.0;
# 	com_t.y = 2.0;		com_r.y = 2.0;
# 	com_t.z = 3.0;		com_r.z = 0.0;

# 	g_poses  = Pose()
# 	g_poses.position = com_t
# 	com_path.poses.append(g_poses)
# 	com_path.time_from_start.append(1.1)

# 	g_poses  = Pose()
# 	com_t = Point();com_t.x = 2.5
# 	g_poses.position = com_t
# 	com_path.poses.append(g_poses)
# 	com_path.time_from_start.append(1.7)

	# print com_path.poses[1]
	# print com_path.time_from_start[1]
