#!/usr/bin/env python

import sys; sys.dont_write_bytecode = True

from constant import *
import plotgraph as Plot
import time
import numpy as np
from scipy import linalg
from scipy import arange

class SplineGenerator:
	def __init__(self):
		self.p  = 4; 			# order of spline
		self.n  = 2; 			# derivative degree
		self.C  = np.matrix([0])	# matrix of via points
		self.t0 = np.array([0.0])	# array of time
		self.U  = np.array([0.0])
		self.n_knot = 0

	def knotFunction(self):
		""" Determine knots required """

		self.U = np.zeros(len(self.t0)+2*self.p+1)

		for i in range(0,len(self.t0)+2*self.p+1):
			if (i<=self.p):
				self.U[i] = self.t0.item(0) 
			elif (i>=self.p and i<len(self.t0)+self.p):
				self.U[i] = (self.t0.item(i-self.p)+self.t0.item(i-self.p-1))/2.0 
			else:
				self.U[i] = self.t0.item(-1)

		n_knot = len(self.U) - 1

		return self.U, n_knot

	def whichSpan(self, u, U, n_knot, p):
		""" Compute i for a given u """

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

	def DersBasisFunction(self, u, U, p, i, n=2):
		""" Basis functions with derivatives """

		## Define Variables ##
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

	def controlPoint(self, U, n_knot, d=1):
		""" Calculate control points """

		A = np.zeros((n_knot-self.p,n_knot-self.p))
		row_counter = 0
		z_max = len(self.t0)

		for z in range(0,z_max):
			# check to prevent u = umax
			if (U[-1]==self.t0[z]):		
				u = self.t0[z] - 0.001	
			else:
				u = self.t0[z]

			i = self.whichSpan(u, U, n_knot, self.p) 		# compute i

			# order of derivatives & offset for stacking
			if (z==0.0 or z==z_max-1):
				n = 2;	offset = 3; 			
			else:
				n = 0;	offset = 1;

			temp = self.DersBasisFunction(u, U, self.p, i, n) 	# compute control point
			
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
		try:
			P = linalg.solve(A,self.C.getT(),check_finite=False) 		# control points
		except: 
			print 'no points found!'
		return P
	
	def evaluate_spline(self, u, P, n_knot, d=1):
		""" Spline function for given time t """

		sp  = []
		sv  = []
		sa  = []

		# print 'going into whichSpan ', u
		i = self.whichSpan(u, self.U, n_knot, self.p);
		# print 'going into basis func ', u
		A =	self.DersBasisFunction(u, self.U, self.p, i)
		
		for k in range(0,d): 	#/* For each components of the B-spline*/
			sp.append(0)
			sv.append(0)
			sa.append(0)
			for j in range(0,self.p+1):
				sp[k] = sp[k] + P[i-self.p+j][k]*A[0][j];
				sv[k] = sv[k] + P[i-self.p+j][k]*A[1][j];
				sa[k] = sa[k] + P[i-self.p+j][k]*A[2][j];
		
		return sp, sv, sa

	def spline_generation(self, x, t, tn):
		""" computes b-spline given via points and time interval 	"""
		""" Input:  a) C  -> 2D array - positions (x,y,z) 					
			 		b) t  -> Time interval for each via point
					c) tn -> Output spline time interval			""" 
		""" Ouput: Tuple of 2D list - 
								(time,position,velocity, acceleration)	"""
		self.t0 = t
		self.C  = x
		U, n_knot = self.knotFunction() 		# determine U, n_knot
		P = self.controlPoint(U, n_knot) 			# calculate control point P

		ct = [] 	# output timing array
		cp = [] 	# output position array
		cv = [] 	# output velocity array
		ca = [] 	# output array array
		
		## Evaluate spline
		for i in np.linspace(0,t[-1],(t[-1]-0)/tn+1):
			t1, t2, t3 = self.evaluate_spline(i, P, n_knot, 3)

			ct.append(i) 
			cp.append(t1) 
			cv.append(t2) 
			ca.append(t3) 
		
		return ct,cp,cv,ca

	def point_interpolation(self, cpath, ctime):
		""" expand input array size for bspline computation """

		C  = np.zeros((3,len(cpath)+4))
		
		for i in range(0,3):
			k = 0
			for j in range(self.p-1,C.shape[1]-self.p+1):
				C[i][j] = cpath.item(j-(self.p-2),i)
				k += 1
		
		for i in range(0,3):
			C[i][0] = cpath[0][i]
			C[i][-1] = cpath[-1][i]
		
		return np.matrix(C)

	def compute_time_intervals(self, q):
		""" Compute time interval for spline if not specified 	"""
		""" Intervals are at unit time (1)						"""

		return np.arange(0,len(q),1)

	def generate_spline(self, x, t=None, tn=0.1):
		""" 	Compute spline using via points only	"""
		
		## Set t if undefined
		if (t is None):
			t = self.compute_time_intervals(x)

		C = self.point_interpolation(x, t)		# determine C
		
		return self.spline_generation(C, t, tn)
		

## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##

### CoM linear path ###
x_com = np.array([.0,.0,BODY_HEIGHT])
x_com = np.vstack((x_com,np.array([0.04, 0.0, BODY_HEIGHT])))
x_com = np.vstack((x_com,np.array([0.06, 0.0, BODY_HEIGHT+0.025])))
x_com = np.vstack((x_com,np.array([0.01, 0.2, BODY_HEIGHT])))
x_com = np.vstack((x_com,np.array([0.30, 0.05, 0.15])))
x_com = np.vstack((x_com,np.array([1.35, 0.05, 0.15])))
x_com = np.vstack((x_com,np.array([1.0, 0.5, BODY_HEIGHT+0.05])))
t_com = np.array([0.0,1,2,3,4,8,10]) 

# spliner = SplineGenerator()
# x_out  = spliner.generate_spline(x_com)

# Plot.plot_2d(x_out[0],x_out[1])


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
