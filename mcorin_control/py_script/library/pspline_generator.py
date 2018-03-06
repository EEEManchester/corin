#!/usr/bin/env python

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'class'))

import numpy as np
from numpy.linalg import inv
from fractions import Fraction
import math
import matplotlib.pyplot as plt
from scipy import linalg

from constant import *
import plotgraph as Plot
from TrajectoryPoints import TrajectoryPoints

class SplineGenerator:
	def __init__(self):
		# self.point 	= JointTrajectoryPoint()
		self.nc 	= 4 	# Define the number of coefficients (order + 1) Quintic Polynomial: 6

	def LegPhase_Interpolation(self, sp, ep, snorm ,phase=2, ctime=2.0):
		""" set via points for leg transfer phase """

		## declare variables ##
		cpx = np.zeros((1,3))	# cartesian position in matrix
		sh  = STEP_HEIGHT  		# step height for transfer phase

		print 'phase param: ', phase
		# reject displacements less than 1mm
		pd = np.array([0, 0, 0])
		for i in range (0,3):
			pd[i] = ep[i]-sp[i]
			if (phase>1):
				if (abs(pd[i]) < 0.001):
					pd[i] = 0
		#print 'sp: ', sp, '  ep: ', ep
		
		# set via points according to phase
		if (phase==1):						
			## Transfer phase 

			# vector from origin to travel path midpoint
			pdiv = np.array([0.125,0.5,0.875])
			sdiv = np.array([0.6*sh, sh, 0.6*sh])
			A = np.zeros((1,3))

			for i in range(0,len(pdiv)):
				v3 = sp + (ep - sp)*pdiv.item(i)
				m3 = np.matrix([ [1, 0, 0, v3.item(0)], [0, 1, 0, v3.item(1)], [0, 0, 1, v3.item(2)], [0, 0, 0, 1] ])
				
				# surface normal
				nm = np.matrix([ [1, 0, 0, -sdiv.item(i)*np.sin(snorm[0])], [0, 1, 0, 0], [0, 0, 1, sdiv.item(i)*np.cos(snorm[0])], [0, 0, 0, 1] ])
				
				# compute clearance point in SCS
				mp = m3*nm
				A  = np.vstack( (A, np.array([ mp[0,3],mp[1,3],mp[2,3] ]) ) )
			A  = np.delete(A, 0, 0) # clean up first row of matrix

			td = np.array( [0, 0.25*ctime, 0.5*ctime, 0.75*ctime, ctime ])
			
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
	
	def velocity_heuristics(self, cx, td):
		""" compute intermediate velocity using heuristics """

		## define variables
		k  = len(td) - 1
		cv = np.zeros((len(td),3))
		ca = np.zeros((len(td),3))

		## cycle through each element of 3D trajectory
		for j in range(0,3):
			# cycle through each via points
			for i in range(1,k):
				try:
					dkx  = (cx.item(i,j)-cx.item(i-1,j))/(td.item(i)  -td.item(i-1))
					dkx1 = (cx.item(i+1,j)-cx.item(i,j))/(td.item(i+1)-td.item(i))
				except:
					dkx  = 0.0
					dkx1 = 0.0

				if (np.sign(dkx) != np.sign(dkx1)):
					cv[i][j] = 0
				else:
					cv[i][j] = 0.5*(dkx + dkx1)

		return cv, ca

	def spline_3D_heuristic(self, cx, td, tn):
		""" computes cubic spline given via points and time interval 	
			using velocity heuristics 									"""
		""" Input:  a) 2D array - positions (x,y,z) 					
			 		b) Time interval for each via point
					c) Output spline time interval					 	""" 
		""" Ouput: Tuple of 2D array - 
								(time,position,velocity, acceleration)	"""

		## declare variables ##
		ct = [] 	# output timing array
		cp = [] 	# output position array
		cv = [] 	# output velocity array
		ca = [] 	# output array array

		# Define the number of segments, k
		k = len(td) - 1

		## compute intermediate velocity using heuristics
		cvh, cah = self.velocity_heuristics(cx,td)
		
		## Update the empty coefficient matrix
		Ax = np.zeros((self.nc,3))

		for i in range(0,k): 			# loop the number of via points
			# set matrix variables
			t0 = float(td.item(i))
			t1 = float(td.item(i+1))
			
			#Calcualte the polynomial coefficients
			if (self.nc == 4):
				## Define the polynomial equations
				M = np.matrix([	[1,  t0,  pow(t0,2),    pow(t0,3)		],
		     					[0,  1,   2*t0,    		3*(pow(t0,2))	],
						    	[1,  t1,  pow(t1,2),    pow(t1,3)		],
						    	[0,  1,   2*t1,    		3*(pow(t1,2))	]])

				B = np.matrix([ [cx.item(i,0),cx.item(i,1),cx.item(i,2)], 
								[cvh.item(i,0),cvh.item(i,1),cvh.item(i,2)],
								[cx.item(i+1,0),cx.item(i+1,1),cx.item(i+1,2)], 
								[cvh.item(i+1,0),cvh.item(i+1,1),cvh.item(i+1,2)] ])

			elif (self.nc == 6):
				M = np.matrix(	[	[1,  t0,  pow(t0,2),    pow(t0,3),      pow(t0,4),        pow(t0,5)],
		     						[0,  1,   2*t0,    		3*(pow(t0,2)),  4*(pow(t0,3)),    5*(pow(t0,4))],
						    		[0,  0,   2,       		6*t0,       	12*(pow(t0,2)),   20*(pow(t0,2))],
						    		[1,  t1,  pow(t1,2),    pow(t1,3),      pow(t1,4),        pow(t1,5)],
						    		[0,  1,   2*t1,    		3*(pow(t1,2)),  4*(pow(t1,3)),    5*(pow(t1,4))],
						    		[0,  0,   2,       		6*t1,       	12*(pow(t1,2)),   20*(pow(t1,3))] ])

				B = np.matrix([ [cx.item(i,0),cx.item(i,1),cx.item(i,2)], 
								[cvh.item(i,0),cvh.item(i,1),cvh.item(i,2)],
								[cah.item(i,0),cah.item(i,1),cah.item(i,2)],
								[cx.item(i+1,0),cx.item(i+1,1),cx.item(i+1,2)], 
								[cvh.item(i+1,0),cvh.item(i+1,1),cvh.item(i+1,2)],
								[cah.item(i+1,0),cah.item(i+1,1),cah.item(i+1,2)] ])

			Ax = linalg.solve(M,B)
			
			## determine trajectory timing phases and total number of points
			if(i+1 != k):															# intermediate segments
				T = np.arange(t0, t1, tn)
			else: 																	# final segment
				T = np.arange(t0, (t1+tn), tn)
				
			Tl   = T.shape[0] 	# determine size of matrix
			t_t  = np.zeros(Tl)
			cx_t = np.zeros((Tl,3))
			vx_t = np.zeros((Tl,3))
			ax_t = np.zeros((Tl,3))

			## Generate the trajectories
			for j in range(0,Tl): 		# loop number of transitionary points
				qp = [0,0,0]
				qv = [0,0,0]
				qa = [0,0,0]

				if (self.nc == 4):
					for h in range(0,3):
						qp[h] = Ax[0][h] + (Ax[1][h]*T[j]) + (Ax[2][h]*(T[j]**2)) + (Ax[3][h]*(T[j]**3)) 
						qv[h] = Ax[1][h] + (2*Ax[2][h]*T[j]) + (3*Ax[3][h]*(T[j]**2)) 
						qa[h] = (2*Ax[2][h]) + (6*Ax[3][h]*(T[j])) 						

				elif (self.nc == 6):
						qp[h] = Ax[0][h] + (Ax[1][h]*T[j]) + (Ax[2][h]*(T[j]**2)) + (Ax[3][h]*(T[j]**3)) + + (Ax[4][h]*(T[j]**4)) + (Ax[5][h]*(T[j]**5))
						qv[h] = Ax[1][h] + (2*Ax[2][h]*T[j]) + (3*Ax[3][h]*(T[j]**2)) + (4*Ax[4][h]*(T[j]**3)) + (5*Ax[5][h]*(T[j]**4))
						qa[h] = (2*Ax[2][h]) + (6*Ax[3][h]*(T[j])) + (12*Ax[4][h]*(T[j]**2)) + (20*Ax[5][h]*(T[j]**3))

				ct.append(T[j])
				cp.append(qp) 
				cv.append(qv) 
				ca.append(qa) 
		
		return ct, cp, cv, ca
		
	def spline_1D_acc(self, xi, t):
		""" Cubic spline with assigned initial & final velolcity & acceleration """
		""" First computes intermediatry points then compute spline as before 	"""
		""" (sec 4.4.4, pg.177)													""" 

		## declare variables ##
		n  = len(xi)+1
		ns = len(xi)-1 			# number of splines
		sz = len(xi)-2			# number of via points excl. start & final
		A  = np.zeros((n-2,n-2))	# time
		v  = np.zeros(n-2) 		# intermediate velocity
		C  = np.zeros(n-2)		# intermediate position & time
		X  = np.zeros(n+1) 		# extended array of points including intermediate
		v0 = 0. 				# initial velocity
		vf = 0.					# final velocity
		a0 = 0.					# initial acceleration
		af = 0.					# final acceleration
		t0 = (t[1]-t[0])/2.				# interval for initial time
		tf = t[n-3]+ (t[-1]-t[n-3])/2.	# interval for final time
		T  = np.zeros(n) 		# vector of time difference
		
		# update time and points interval
		tn = np.array([0.,t0])
		tn = np.append(tn,t[1:len(t)-1])
		tn = np.append(tn,tf)
		tn = np.append(tn,t[len(t)-1])
		X[0] = xi[0]
		X[2:n-1] = xi[1:n-2]
		X[-1] = xi[-1]
		
		# time diff between intervals
		for i in range(0,len(tn)-1):
			T[i] = float(tn[i+1]) - float(tn[i])

		# Populate matrix for calculating intermediate velocity (pg. 171)
		A[0][0] = 2.*T[1] + T[0]*(3.+T[0]/T[1])
		A[0][1] = T[1]
		A[1][0] = T[1] - (T[0]**2)/T[1]
		A[1][1] = 2*(T[1]+T[2])
		A[1][2] = T[2]
		A[n-4][n-5] = T[n-3]
		A[n-4][n-4] = 2*(T[n-3]+T[n-2])
		A[n-4][n-3] = T[n-2] - ((T[n-1])**2/(T[n-2]))
		A[n-3][n-4] = T[n-2]
		A[n-3][n-3] = 2*T[n-2]+T[n-1]*(3+T[n-1]/T[n-2])
		
		C[0] = 6*((X[2]-X[0])/T[1] - v0*(1+T[0]/T[1]) - a0*(0.5+T[0]/(3*T[1]))*T[0])
		C[1] = 6*((X[3]-X[2])/T[2] - (X[2]-X[0])/T[1] + v0*(T[0]/T[1]) + a0*(T[0]/(3*T[1])))
		C[ns-2] = 6*((X[n]-X[n-2])/T[n-2] - (X[n-2]-X[n-3])/T[n-3] - vf*(T[n-1]/T[n-2]) + af*(((T[n-1])**2)/(3*T[n-2])))
		C[ns-1] = 6*((X[n-2]-X[n])/T[n-2] + vf*(1+ T[n-1]/T[n-2]) - af*(0.5+ (T[n-1])/(3*T[n-2]))*T[n-1])
		
		for i in range(2,n-4): 		# exclude first two and last rows
			A[i][i-1] 	= T[i]
			A[i][i] 	= 2*(T[i]+T[i+1])
			A[i][i+1] 	= T[i+2] 	# skip first item
						
			# Compute C - change to 3D
			C[i] = 6*( (X[i+2]-X[i+1])/T[n-3] - (X[i+1]-X[i])/T[n-4] ) 
		
		## Solve for w
		w = linalg.solve(A,C)
		# compute extra points using (4.26) & (4.27)
		X[1]   = X[0] + T[0]*v0 + (a0*T[0]**2)/3 + (w[0]*T[0]**2)/6 
		X[n-1] = X[n] - T[-1]*vf + (af*T[-1]**2)/3 + (w[-1]*T[-1]**2)/6

		return X, tn

	def spline_1D(self, x, t, tint):
		""" computes cubic spline given via points and time interval 	"""
		""" Input:  a) 1D array - points 							 
			 		b) Time interval for each via point
					c) Output spline time interval						""" 
		""" Ouput: Tuple of 2D array - 
								(time,position,velocity, acceleration)	"""

		## declare variables ##
		ns = len(x)-1 			# number of splines
		sz = len(x)-2			# number of via points excl. start & final
		A  = np.zeros((sz,sz))	# time
		v  = np.zeros(sz) 		# intermediate velocity
		C  = np.zeros(sz)		# intermediate position & time
		v0 = 0.					# initial velocity
		vf = 0.					# final velocity
		cx = [] 				# cartesian position array
		cv = [] 				# cartesian velocity array
		ca = [] 				# cartesian array array
		ct = [] 				# cartesian timing array

		# Populate matrix for calculating intermediate velocity (pg. 171)
		for i in range(0,sz):
			T0 = t[i+1]-t[i]
			T1 = t[i+2]-t[i+1]

			# Compute A
			if (i-1+0 >= 0): A[i][i-1+0] = T1 	# skip first item
			A[i][i-1+1] = 2*(T0+T1)
			try:	A[i][i-1+2] = T0
			except: pass 	# skip last item

			# Compute C - change to 3D
			C[i] = 3*( T0**2*(x[i+2]-x[i+1]) + T1**2*(x[i+1]-x[i]) )/(T0*T1) 
			if (i==0):		C[i] = C[i] - T1*v0
			elif(i==sz-1):	C[i] = C[i] - T0*vf

		# Solve for v
		v = linalg.solve(A,C)
		v = np.concatenate([[v0],v,[vf]])
		
		## Compute coefficients
		for i in range(0,sz+1):
			tk	 = t[i+1]-t[i]
			qk   = x[i]
			qk_1 = x[i+1]
			vk   = v[i]
			vk_1 = v[i+1]
			
			## compute spline coefficients
			a0 = qk
			a1 = vk
			a2 = (1/tk)*( 3*(qk_1-qk)/tk - 2*vk - vk_1 )
			a3 = (1/(tk**2))*( 2*(qk-qk_1)/tk + vk + vk_1 )

			## compute values
			lv = tint
			nv = 0.
			if (t[i+1]==t[-1]):	
				lv = 0.
				nv = 1

			for tk in np.linspace(t[i],t[i+1]-lv,(t[i+1]-t[i])/tint+nv):
				td = tk - t[i]
				qp = a0 + a1*td + a2*td**2 + a3*td**3
				qv = a1 + 2*a2*td + 3*a3*td**2
				qa = 2*a2 + 6*a3*td

				ct.append(tk)
				cx.append(qp)
				cv.append(qv)
				ca.append(qa)	
		
		return np.around(ct,4),cx,cv,ca

	def spline_3D(self, x, t, tint):
		""" computes cubic spline given via points and time interval 	"""
		""" Input:  a) 2D array - positions (x,y,z) 					
			 		b) Time interval for each via point
					c) Output spline time interval					 	""" 
		""" Ouput: Tuple of 2D array - 
								(time,position,velocity, acceleration)	"""

		## declare variables ##
		ns = len(x)-1 			# number of splines
		sz = len(x)-2			# number of via points excl. start & final
		A  = np.zeros((sz,sz))	# time
		v  = np.zeros((sz,3))	# intermediate velocity
		C  = np.zeros((sz,3))	# intermediate position & time
		v0 = np.zeros((1,3))	# initial velocity
		vf = np.zeros((1,3))	# final velocity
		
		ct = [] 	# output timing array
		cp = [] 	# output position array
		cv = [] 	# output velocity array
		ca = [] 	# output array array

		## Populate matrix for calculating intermediate velocity with via points (pg. 171)
		if (sz>0):
			for i in range(0,sz):
				T0 = t[i+1]-t[i]
				T1 = t[i+2]-t[i+1]

				# Compute A
				if (i-1+0 >= 0): A[i][i-1+0] = T1 	# skip first item
				A[i][i-1+1] = 2*(T0+T1)
				try:	A[i][i-1+2] = T0
				except: pass 	# skip last item

				# Compute C - change to 3D
				# C[i] = 3*( T0**2*(x[i+2]-x[i+1]) + T1**2*(x[i+1]-x[i]) )/(T0*T1) 

				for j in range(0,3):
					C[i][j] = 3*( T0**2*(x[i+2][j]-x[i+1][j]) + T1**2*(x[i+1][j]-x[i][j]) )/(T0*T1) 

					if (i==0):		C[i][j] = C[i][j] - T1*v0[0][j]
					elif(i==sz-1):	C[i][j] = C[i][j] - T0*vf[0][j]
			
			## Solve for v
			v = linalg.solve(A,C)
		v = np.concatenate((v0,v,vf),axis=0)
		
		## Compute coefficients
		for i in range(0,sz+1):
			tk	  = t[i+1]-t[i]
			qkx   = x[i][0]		;qky   = x[i][1]	;qkz   = x[i][2]
			qkx_1 = x[i+1][0]	;qky_1 = x[i+1][1]	;qkz_1 = x[i+1][2]
			vkx   = v[i][0]		;vky   = v[i][1]	;vkz   = v[i][2]
			vkx_1 = v[i+1][0]	;vky_1 = v[i+1][1]	;vkz_1 = v[i+1][2]
			
			# compute spline coefficients for each dimension
			a0x = qkx
			a1x = vkx
			a2x = (1/tk)*( 3*(qkx_1-qkx)/tk - 2*vkx - vkx_1 )
			a3x = (1/(tk**2))*( 2*(qkx-qkx_1)/tk + vkx + vkx_1 )

			a0y = qky
			a1y = vky
			a2y = (1/tk)*( 3*(qky_1-qky)/tk - 2*vky - vky_1 )
			a3y = (1/(tk**2))*( 2*(qky-qky_1)/tk + vky + vky_1 )

			a0z = qkz
			a1z = vkz
			a2z = (1/tk)*( 3*(qkz_1-qkz)/tk - 2*vkz - vkz_1 )
			a3z = (1/(tk**2))*( 2*(qkz-qkz_1)/tk + vkz + vkz_1 )

			# compute trajectory values
			lv = tint
			nv = 0.
			if (t[i+1]==t[-1]):	
				lv = 0.
				nv = 1

			for tk in np.linspace(t[i],t[i+1]-lv,(t[i+1]-t[i])/tint+nv):
				td = tk - t[i]
				qpx = a0x + a1x*td + a2x*td**2 + a3x*td**3
				qvx = a1x + 2*a2x*td + 3*a3x*td**2
				qax = 2*a2x + 6*a3x*td

				qpy = a0y + a1y*td + a2y*td**2 + a3y*td**3
				qvy = a1y + 2*a2y*td + 3*a3y*td**2
				qay = 2*a2y + 6*a3y*td

				qpz = a0z + a1z*td + a2z*td**2 + a3z*td**3
				qvz = a1z + 2*a2z*td + 3*a3z*td**2
				qaz = 2*a2z + 6*a3z*td
				
				## concatenate segments together - size is <no_rows> by 3			
				ct.append( td+t[i] ) 
				cp.append( [qpx,qpy,qpz]) 
				cv.append( [qvx,qvy,qvz]) 
				ca.append( [qax,qay,qaz]) 
			
		return np.around(ct,4),cp,cv,ca

	def compute_time_intervals(self, q):
		""" Compute time interval for spline if not specified 	"""
		""" Intervals are at unit time (1)						"""

		return np.arange(0,len(q),1)

	def generate_leg_spline(self, sp, ep, snorm , phase=2, ctime=2.0, tn=0.1):
		""" Generate transfer phase trajectory """

		x, t = self.LegPhase_Interpolation(sp, ep, snorm ,phase)
		
		## use zero initial and final acceleration
		cx = np.zeros((len(x)+2,3))	# via point 2D array
		tx = np.zeros(len(x)+2)		# time interval array
		for i in range(0,3):
			cx[:,i], tx = self.spline_1D_acc(x[:,i].flatten(), t)

		# return self.spline_3D_heuristic(cx, tx, tn)	# zero initial & final velocity
		return self.spline_3D(cx, tx, tn)	# zero initial & final velocity & acceleration

	def generate_body_spline(self, x, t=None, tn=0.1):
		""" 	Compute spline using via points only	"""
		
		## Set t if undefined
		if (t is None):
			t = self.compute_time_intervals(x)
		
		## use zero initial and final acceleration
		cx = np.zeros((len(x)+2,3))	# via point 2D array
		tx = np.zeros(len(x)+2)		# time interval array
		# for i in range(0,3):
		# 	cx[:,i], tx = self.spline_1D_acc(x[:,i].flatten(), t)

		return self.spline_3D(x, t, tn)	# zero initial & final velocity
		# return self.spline_3D(cx, tx, tn) 	# zero initial & final velocity & acceleration

	
## ================================================================================================ ##
## 												TESTING 											##
## ================================================================================================ ##

# sample variable
sp = np.array([0.214,0.025, -0.1451])
ep = np.array([0.214,-0.025,-0.1451])
snorm = (0, 0, 1)

phase = 1

x_com = np.array([.0,.0,BODY_HEIGHT])

### short V spline
x_com = np.vstack((x_com,np.array([0.04, 0.0, BODY_HEIGHT])))
# x_com = np.vstack((x_com,np.array([0.06, 0.0, BODY_HEIGHT+0.025])))
# x_com = np.vstack((x_com,np.array([0.01, 0.2, BODY_HEIGHT])))
# x_com = np.vstack((x_com,np.array([0.30, 0.05, 0.15])))
# x_com = np.vstack((x_com,np.array([1.35, 0.05, 0.15])))
# x_com = np.vstack((x_com,np.array([1.0, 0.5, BODY_HEIGHT+0.05])))

t_com = np.array([0.0,1]) 
## 1D spline - textbook points
test_points = np.array([3.,-2.,-5.,0.,6.,12.,8.])
test_times  = np.array([0.,5.,7.,8.,10.,15.,18.])
## e.g. transfer phase
test_points = np.array([0.025,0.01875,0,-0.01875,-0.025])	
test_times  = np.array([0.,0.5,1.,1.5,2.])

spliner = SplineGenerator()
# x_out = spliner.spline_1D(test_points,test_times,TRAC_INTERVAL)
# spoints = spliner.spline_1D_acc(test_points,t_com)
# spoints = spliner.spline_3D(x_com,t_com)

# x_out = spliner.generate_leg_spline(sp,ep,snorm,phase)
x_out = spliner.generate_body_spline(x_com)

# Plot.plot_2d(x_out[0],x_out[3])
# Plot.plot_2d_multiple(2,x_out.t,x_out.xv,x_out.xa)



## Stack using 2D array
# ct = np.hstack( (ct,td+t[i] ) )
# cp = np.vstack( (cp,[qpx,qpy,qpz]) )
# cv = np.vstack( (cv,[qvx,qvy,qvz]) )
# ca = np.vstack( (ca,[qax,qay,qaz]) )