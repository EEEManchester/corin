#!/usr/bin/env python
# -*- coding: utf-8 -*-

## QP for trajectory optimization ##
import sys; sys.dont_write_bytecode = True
import numpy as np
from scipy import linalg
from scipy import integrate
from scipy.integrate import odeint

import matrix_transforms as mtf
from constant import *
import plotgraph as Plot

from cvxopt import matrix
from cvxopt import solvers

## cvxopt input form: # P, q, G, h, None, A, b
class QPTrajectoryOptimization():
	def __init__(self):
		self.spline_coeff = np.zeros((18,1))
		self.phase_interv = 2

	def solve_example(self):
		# setting via numpy
		Pn = cvxmatrix(np.diag([1,0]), tc='d')
		qn = matrix(np.array([3,4]), tc='d')
		Gn = matrix(np.array([[-1,0],[0,-1],[-1,-3],[2,5],[3,4]]), tc='d')
		hn = matrix(np.array([0,0,-15,100,80]), tc='d')

		# direct setting
		Pn = matrix([[1.0,0.0],[0.0,0.0]])
		qn = matrix([3.0,4.0])
		Gn = matrix([[-1.0,0.0,-1.0,2.0,3.0],[0.0,-1.0,-3.0,5.0,4.0]])
		hn = matrix([0.0,0.0,-15.0,100.0,80.0])

		sol = solvers.qp(Pn,qn,Gn,hn)

		print sol['x']
		# return sol

	def solve_line_equation(self,x1,y1,x2,y2):
		""" Solves for the equation of a line given two
			points. The solution is in the general form.  """

		a1 = np.array([[x1,-1],[x2,-1] ])
		b1 = np.array([-y1,-y2])
		return np.linalg.solve(a1,b1)

	# def solve(self, x_cog_0, x_cog_1, footholds):
	# 	""" QP problem to optimise trajectory 
	# 		Input: 	x_cog_0: Re(3) initial CoG
	# 				x_cog_1: Re(3) goal CoG
	# 				footholds: Re(3x3) array of foot position
	# 		Output:  							"""

	# 	## Remap variables
	# 	T = self.phase_interv;  # Gait timing per phase
	# 	c1x = footholds[0][0]; c2x = footholds[1][0]; c3x = footholds[2][0]; 
	# 	c1y = footholds[0][1]; c2y = footholds[1][1]; c3y = footholds[2][1];
		
	# 	## Stability polygon
	# 	k1, p1 = self.solve_line_equation(c1x,c1y,c3x,c3y)
	# 	k2, p2 = self.solve_line_equation(c1x,c1y,c2x,c2y)
	# 	k3, p3 = self.solve_line_equation(c2x,c2y,c3x,c3y)

	# 	# [ -k1*(- t^3 + (3*t)/(5*g)), -k1*(- t^2 + 1/(5*g)), k1*t, k1, t^3 - (3*t)/(5*g), t^2 - 1/(5*g), t, 1]

	# 	A1 = np.array([ -k1*(- T**3 + (3*T)/(5*G)), -k1*(-T**2 + 1/(5*G)), k1*T, k1, T**3 - (3*T)/(5*G), T**2 - 1/(5*G), T, 1])
	# 	A2 = np.array([ -k2*(- T**3 + (3*T)/(5*G)), -k2*(-T**2 + 1/(5*G)), k2*T, k2, T**3 - (3*T)/(5*G), T**2 - 1/(5*G), T, 1])
	# 	A3 = np.array([ -k3*(- T**3 + (3*T)/(5*G)), -k3*(-T**2 + 1/(5*G)), k3*T, k3, T**3 - (3*T)/(5*G), T**2 - 1/(5*G), T, 1])
	# 	B1 = B2 = B3 = 0

	# 	if (-k1*x_cog_0[0] + p1 > x_cog_0[1]):
	# 		sign1=1;
	# 	else:
	# 		sign1=-1;

	# 	if (-k2*x_cog_0[0] + p2 > x_cog_0[1]):
	# 		sign2=1;
	# 	else:
	# 		sign2=-1;

	# 	if (-k3*x_cog_0[0] + p3 > x_cog_0[1]):
	# 		sign3=1;
	# 	else:
	# 		sign3=-1;

	# 	## Hessian matrix
	# 	H1 = np.array([ [24*(T**3), 12*(T**2), 0, 0], [12*(T**2), 8*T, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0] ]);
	# 	H = np.zeros((8,8))
	# 	H[0:4,0:4] = H1.copy()
	# 	H[4:8,4:8] = H1.copy()
	# 	H = matrix(H, (8,8), tc='d')
		
	# 	## Regularization matrix
	# 	f = matrix(np.zeros((8,1)), (8,1), tc='d')

	# 	## Inequality constraints
	# 	inq_C = matrix(np.array([ sign1*A1, sign2*A2, sign3*A3 ]), (3,8), tc='d')
	# 	inq_D = matrix([ sign1*p1, sign2*p2, sign3*p3]);
	# 	# print np.round(inq_C,4)
	# 	# print np.round(inq_D,4)
	# 	## Equality Constraints
	# 	Aeq = np.array([ 	[0, 0, 0, 1, 0, 0, 0, 0],
	# 						[0, 0, 1, 0, 0, 0, 0, 0],
	# 						[0, 2, 0, 0, 0, 0, 0, 0],
	# 						[8, 4, 2, 1, 0, 0, 0, 0],
	# 						[0, 0, 0, 0, 0, 0, 0, 1],
	# 						[0, 0, 0, 0, 0, 0, 1, 0],
	# 						[0, 0, 0, 0, 0, 2, 0, 0],
	# 						[0, 0, 0, 0, 8, 4, 2, 1] ])
	# 	Aeq = matrix(Aeq, (8,8), tc='d')
		
	# 	Beq = np.array([ [x_cog_0[0]],
	# 						[12*self.spline_coeff.item(0)+4*self.spline_coeff.item(1)+self.spline_coeff.item(2)],
	# 						[12*self.spline_coeff.item(0)+2*self.spline_coeff.item(1)],
	# 						[x_cog_1[0]],
 #    						[x_cog_0[1]],
 #    						[12*self.spline_coeff.item(4)+4*self.spline_coeff.item(5)+self.spline_coeff.item(6)],
 #    						[12*self.spline_coeff.item(4)+2*self.spline_coeff.item(5)],
 #    						[x_cog_1[1]] ])
	# 	Beq = matrix(Beq, (8,1), tc='d')
		
	# 	## Set Solver parameters
	# 	solvers.options['abstol']  = 1e-10
	# 	solvers.options['reltol']  = 1e-10
	# 	solvers.options['feastol'] = 1e-10
	# 	solvers.options['show_progress'] = False

	# 	## Solve QP problem
	# 	sol = solvers.qp(H, f, inq_C, inq_D, Aeq, Beq) 
	# 	# print np.round(sol['x'],4)
	# 	# print np.round(Beq,4)
	# 	## z-axis trajectory
	# 	AZ = np.array([	[0, 0, 0, 1],
	# 					[0, 0, 1, 0],
	# 					[0,-2, 0, 0],
	# 					[8, 4, 2, 1] ]);
	# 	BZ = np.array([ [x_cog_0[2]],
	# 					[12*self.spline_coeff.item(8)+4*self.spline_coeff.item(9)+self.spline_coeff.item(10)],
	# 					[12*self.spline_coeff.item(8)+2*self.spline_coeff.item(9)],
	# 					[x_cog_1[2]] ]);
	# 	xn = np.linalg.solve(AZ,BZ)
		
	# 	## Store coefficient
	# 	self.spline_coeff[0:8]  = sol['x']
	# 	self.spline_coeff[8:12] = xn
	# 	# print (np.round(self.spline_coeff,3))
	# 	return None

	# def interpolate_spline(self, tint):
	# 	""" Interpolates the spline """

	# 	ct = [] 	# output timing array
	# 	cp = [] 	# output position array
	# 	cv = [] 	# output velocity array
	# 	ca = [] 	# output array array
		
	# 	for t in np.linspace(0, self.phase_interv, np.round(self.phase_interv/tint+1)):
	# 		## Compute for each instance
	# 		px = self.spline_coeff.item(0)*t**3 + self.spline_coeff.item(1)*t**2 + self.spline_coeff.item(2)*t + self.spline_coeff.item(3);
	# 		py = self.spline_coeff.item(4)*t**3 + self.spline_coeff.item(5)*t**2 + self.spline_coeff.item(6)*t + self.spline_coeff.item(7);
	# 		pz = self.spline_coeff.item(8)*t**3 + self.spline_coeff.item(9)*t**2 + self.spline_coeff.item(10)*t+ self.spline_coeff.item(11);
			
	# 		vx = 3*self.spline_coeff.item(0)*t**2 + 2*self.spline_coeff.item(1)*t + self.spline_coeff.item(2)
	# 		vy = 3*self.spline_coeff.item(4)*t**2 + 2*self.spline_coeff.item(5)*t + self.spline_coeff.item(6)
	# 		vz = 3*self.spline_coeff.item(8)*t**2 + 2*self.spline_coeff.item(9)*t + self.spline_coeff.item(10)

	# 		ax = 6*self.spline_coeff.item(0)*t + 2*self.spline_coeff.item(1)
	# 		ay = 6*self.spline_coeff.item(4)*t + 2*self.spline_coeff.item(5)
	# 		az = 6*self.spline_coeff.item(8)*t + 2*self.spline_coeff.item(9)
			
	# 		## Append to list
	# 		ct.append( t ) 
	# 		cp.append( [px, py, pz]) 
	# 		cv.append( [vx, vy, vz]) 
	# 		ca.append( [ax, ay, az]) 

	# 	return ct, cp, cv, ca

	def solve_quintic(self, x_cog_0, x_cog_1, footholds):
		""" QP problem to optimise trajectory 
			Input: 	x_cog_0: Re(3) initial CoG
					x_cog_1: Re(3) goal CoG
					footholds: Re(3x3) array of foot position
			Output:  							"""
		
		## Remap variables
		T = self.phase_interv;  # Gait timing per phase
		c1x = footholds[0][0]; c2x = footholds[1][0]; c3x = footholds[2][0]; 
		c1y = footholds[0][1]; c2y = footholds[1][1]; c3y = footholds[2][1];
		z = x_cog_1[2]

		## Stability polygon
		k1, p1 = self.solve_line_equation(c1x,c1y,c3x,c3y)
		k2, p2 = self.solve_line_equation(c1x,c1y,c2x,c2y)
		k3, p3 = self.solve_line_equation(c2x,c2y,c3x,c3y)

		A1 = np.array([ k1*(T**5 - (20*z*T**3)/G), k1*(T**4 - (12*z*T**2)/G), k1*(T**3 - (6*z*T)/G), -k1*(- T**2 + (2*z)/G), k1*T, k1, T**5 - (20*z*T**3)/G, T**4 - (12*z*T**2)/G, T**3 - (6*z*T)/G, T**2 - (2*z)/G, T, 1])
		A2 = np.array([ k2*(T**5 - (20*z*T**3)/G), k2*(T**4 - (12*z*T**2)/G), k2*(T**3 - (6*z*T)/G), -k2*(- T**2 + (2*z)/G), k2*T, k2, T**5 - (20*z*T**3)/G, T**4 - (12*z*T**2)/G, T**3 - (6*z*T)/G, T**2 - (2*z)/G, T, 1]) 
		A3 = np.array([ k3*(T**5 - (20*z*T**3)/G), k3*(T**4 - (12*z*T**2)/G), k3*(T**3 - (6*z*T)/G), -k3*(- T**2 + (2*z)/G), k3*T, k3, T**5 - (20*z*T**3)/G, T**4 - (12*z*T**2)/G, T**3 - (6*z*T)/G, T**2 - (2*z)/G, T, 1])
		
		# A1 = np.array([ -k1*(- T**3 + (3*T)/(5*G)), -k1*(-T**2 + 1/(5*G)), k1*T, k1, T**3 - (3*T)/(5*G), T**2 - 1/(5*G), T, 1])
		# A2 = np.array([ -k2*(- T**3 + (3*T)/(5*G)), -k2*(-T**2 + 1/(5*G)), k2*T, k2, T**3 - (3*T)/(5*G), T**2 - 1/(5*G), T, 1])
		# A3 = np.array([ -k3*(- T**3 + (3*T)/(5*G)), -k3*(-T**2 + 1/(5*G)), k3*T, k3, T**3 - (3*T)/(5*G), T**2 - 1/(5*G), T, 1])
		B1 = B2 = B3 = 0

		if (-k1*x_cog_0[0] + p1 > x_cog_0[1]):
			sign1=1;
		else:
			sign1=-1;

		if (-k2*x_cog_0[0] + p2 > x_cog_0[1]):
			sign2=1;
		else:
			sign2=-1;

		if (-k3*x_cog_0[0] + p3 > x_cog_0[1]):
			sign3=1;
		else:
			sign3=-1;

		## Hessian matrix
		H1 = np.array([ [	(800./7)*(T**7), 80.*(T**6), 48.*(T**5), 20.*(T**4), 0., 0.],
					       [80.*(T**6), (288./5)*(T**5), 36.*(T**4), 16.*(T**3), 0., 0.],
					       [48.*(T**5), 36.*(T**4), 24.*(T**3), 12.*(T**2), 0., 0.],
					       [20.*(T**4), 16.*(T**3), 12.*(T**2), 8.*T, 0., 0.],
					       [0., 0., 0., 0., 0., 0.],
					       [0., 0., 0., 0., 0., 0.] ])
		H = np.zeros((12,12))
		H[0:6,0:6] = H1.copy()
		H[6:12,6:12] = H1.copy()
		H = matrix(H, (12,12), tc='d')
		
		## Regularization matrix
		f = matrix(np.zeros((12,1)), (12,1), tc='d')

		## Inequality constraints
		inq_C = matrix(np.array([ sign1*A1, sign2*A2, sign3*A3 ]), (3,12), tc='d')
		inq_D = matrix([ sign1*p1, sign2*p2, sign3*p3]);
		# print np.round(inq_C,4)
		# print np.round(inq_D,4)
		## Equality Constraints
		Aeq = np.array([ [   0,  0, 0, 0, 0, 1,  0,  0, 0, 0, 0, 0],
					        [ 0,  0, 0, 0, 1, 0,  0,  0, 0, 0, 0, 0],
					        [ 0,  0, 0, 2, 0, 0,  0,  0, 0, 0, 0, 0],
					        [ 0,  0, 6, 0, 0, 0,  0,  0, 0, 0, 0, 0],
					        [32, 16, 8, 4, 2, 1,  0,  0, 0, 0, 0, 0],
					        [ 0,  0, 0, 0, 0, 0,  0,  0, 0, 0, 0, 1],
					        [ 0,  0, 0, 0, 0, 0,  0,  0, 0, 0, 1, 0],
					        [ 0,  0, 0, 0, 0, 0,  0,  0, 0, 2, 0, 0],
					        [ 0,  0, 0, 0, 0, 0,  0,  0, 6, 0, 0, 0],
					        [ 0,  0, 0, 0, 0, 0, 32, 16, 8, 4, 2, 1] ])
		
		Aeq = matrix(Aeq, (10,12), tc='d')
		
		Beq = np.array([[x_cog_0[0]],
						[ 80*self.spline_coeff.item(0)+32*self.spline_coeff.item(1)+12*self.spline_coeff.item(2)+4*self.spline_coeff.item(3)+self.spline_coeff.item(4)],
						[160*self.spline_coeff.item(0)+48*self.spline_coeff.item(1)+12*self.spline_coeff.item(2)+2*self.spline_coeff.item(3)],
						[240*self.spline_coeff.item(0)+48*self.spline_coeff.item(1)+ 6*self.spline_coeff.item(2)],
						[x_cog_1[0]],
						[x_cog_0[1]],
						[ 80*self.spline_coeff.item(6)+32*self.spline_coeff.item(7)+12*self.spline_coeff.item(8)+4*self.spline_coeff.item(9)+self.spline_coeff.item(10)],
						[160*self.spline_coeff.item(6)+48*self.spline_coeff.item(7)+12*self.spline_coeff.item(8)+2*self.spline_coeff.item(9)],
						[240*self.spline_coeff.item(6)+48*self.spline_coeff.item(7)+ 6*self.spline_coeff.item(8)],
						[x_cog_1[1]] ])
		
		Beq = matrix(Beq, (10,1), tc='d')
		
		## Set Solver parameters
		solvers.options['abstol']  = 1e-10
		solvers.options['reltol']  = 1e-10
		solvers.options['feastol'] = 1e-10
		solvers.options['show_progress'] = False

		# ## Solve QP problem
		sol = solvers.qp(H, f, inq_C, inq_D, Aeq, Beq) 
		# print np.round(sol['x'],4)
		# print np.round(Beq,4)
		# ## z-axis trajectory
		AZ = np.array([ [ 0,  0, 0, 0, 0, 1],
						[ 0,  0, 0, 0, 1, 0],
						[ 0,  0, 0, 2, 0, 0],
						[ 0,  0, 6, 0, 0, 0],
						[ 0, 24, 0, 0, 0, 0],
						[32, 16, 8, 4, 2, 1] ])
		BZ = np.array([ [x_cog_0[2]],
						[ 80*self.spline_coeff.item(12)+32*self.spline_coeff.item(13)+12*self.spline_coeff.item(14)+4*self.spline_coeff.item(15)+self.spline_coeff.item(16)],
						[160*self.spline_coeff.item(12)+48*self.spline_coeff.item(13)+12*self.spline_coeff.item(14)+2*self.spline_coeff.item(15)],
						[240*self.spline_coeff.item(12)+48*self.spline_coeff.item(13)+ 6*self.spline_coeff.item(14)],
						[240*self.spline_coeff.item(12)+24*self.spline_coeff.item(13)],
						[x_cog_1[2]] ])
		xn = np.linalg.solve(AZ,BZ)
		
		## Store coefficient
		self.spline_coeff[0:12]  = sol['x']
		self.spline_coeff[12:18] = xn
		# print (np.round(self.spline_coeff,3))
		return None

	def interpolate_spline_quintic(self, tint):
		""" Interpolates the spline """

		ct = [] 	# output timing array
		cp = [] 	# output position array
		cv = [] 	# output velocity array
		ca = [] 	# output array array
		
		for t in np.linspace(0, self.phase_interv, np.round(self.phase_interv/tint+1)):
			## Compute for each instance
			px = self.spline_coeff.item(0)*t**5  + self.spline_coeff.item(1)*t**4  + self.spline_coeff.item(2)*t**3 + self.spline_coeff.item(3)*t**2  + self.spline_coeff.item(4)*t  + self.spline_coeff.item(5);
			py = self.spline_coeff.item(6)*t**5  + self.spline_coeff.item(7)*t**4  + self.spline_coeff.item(8)*t**3 + self.spline_coeff.item(9)*t**2  + self.spline_coeff.item(10)*t + self.spline_coeff.item(11);
			pz = self.spline_coeff.item(12)*t**5 + self.spline_coeff.item(13)*t**4 + self.spline_coeff.item(14)*t**3+ self.spline_coeff.item(15)*t**2 + self.spline_coeff.item(16)*t + self.spline_coeff.item(17);
			
			vx = 5*self.spline_coeff.item(0)*t**4  + 4*self.spline_coeff.item(1)*t**3  + 3*self.spline_coeff.item(2)*t**2  + 2*self.spline_coeff.item(3)*t  + self.spline_coeff.item(4);
			vy = 5*self.spline_coeff.item(6)*t**4  + 4*self.spline_coeff.item(7)*t**3  + 3*self.spline_coeff.item(8)*t**2  + 2*self.spline_coeff.item(9)*t  + self.spline_coeff.item(10);
			vz = 5*self.spline_coeff.item(12)*t**4 + 4*self.spline_coeff.item(13)*t**3 + 3*self.spline_coeff.item(14)*t**2 + 2*self.spline_coeff.item(15)*t + self.spline_coeff.item(16);

			ax = 20*self.spline_coeff.item(0)*t**3  + 12*self.spline_coeff.item(1)*t**2  + 6*self.spline_coeff.item(2)*t  + 2*self.spline_coeff.item(3)
			ay = 20*self.spline_coeff.item(6)*t**3  + 12*self.spline_coeff.item(7)*t**2  + 6*self.spline_coeff.item(8)*t  + 2*self.spline_coeff.item(9)
			az = 20*self.spline_coeff.item(12)*t**3 + 12*self.spline_coeff.item(13)*t**2 + 6*self.spline_coeff.item(14)*t + 2*self.spline_coeff.item(15)
			
			## Append to list
			ct.append( t ) 
			cp.append( [px, py, pz]) 
			cv.append( [vx, vy, vz]) 
			ca.append( [ax, ay, az]) 

		return ct, cp, cv, ca
	

## ============================================================ ##
## 								Test 							##
## ============================================================ ##
qprog = QPTrajectoryOptimization()

## Segment 1
x_cog_0 = np.array([0.30, 0.39, 0.1]);
x_cog_1 = np.array([0.3522, 0.39, 0.1]);
c1x = 0.549; c1y = 0.634;    # Leg 2
c2x = 0.051; c2y = 0.634;    # Leg 4
c3x = 0.300; c3y = 0.092;    # Leg 6
p1 = np.array([c1x,c1y,0])
p2 = np.array([c2x,c2y,0])
p3 = np.array([c3x,c3y,0])
footholds = [p1,p2,p3]

new_trajectory = qprog.solve_quintic(x_cog_0, x_cog_1, footholds);
ti, cpi, cvi, cai = qprog.interpolate_spline_quintic(CTR_INTV)

t = []; cp = []; cv = []; ca = []
t += [x+(0*GAIT_TPHASE) for x in ti]
cp += cpi
cv += cvi
ca += cai

## Segment 2
x_cog_0 = np.array([0.3522, 0.39, 0.1]);
x_cog_1 = np.array([0.4538, 0.39, 0.1]);
c1x = 0.402; c1y = 0.690;    # Leg 1
c2x = 0.651; c2y = 0.141;    # Leg 3
c3x = 0.154; c3y = 0.141;    # Leg 5
p1 = np.array([c1x,c1y,0])
p2 = np.array([c2x,c2y,0])
p3 = np.array([c3x,c3y,0])
footholds = [p1,p2,p3]

new_trajectory = qprog.solve_quintic(x_cog_0, x_cog_1, footholds);
ti, cpi, cvi, cai = qprog.interpolate_spline_quintic(CTR_INTV)
ti.pop(0)
cpi.pop(0)
cvi.pop(0)
cai.pop(0)
t += [x+(1*GAIT_TPHASE) for x in ti]
cp += cpi
cv += cvi
ca += cai

## Segment 3
x_cog_0 = np.array([0.4538, 0.39, 0.1]);
x_cog_1 = np.array([0.4800, 0.39, 0.1]);
c1x = 0.7526; c1y = 0.6393;    # Leg 1
c2x = 0.2552; c2y = 0.6393;    # Leg 3
c3x = 0.5039; c3y = 0.092;    # Leg 5
p1 = np.array([c1x,c1y,0])
p2 = np.array([c2x,c2y,0])
p3 = np.array([c3x,c3y,0])
footholds = [p1,p2,p3]

new_trajectory = qprog.solve_quintic(x_cog_0, x_cog_1, footholds);
ti, cpi, cvi, cai = qprog.interpolate_spline_quintic(CTR_INTV)
ti.pop(0)
cpi.pop(0)
cvi.pop(0)
cai.pop(0)
t += [x+(2*GAIT_TPHASE) for x in ti]
cp += cpi
cv += cvi
ca += cai
# print t
# print len(t), len(cp), len(cv), len(ca)
# for i in range(0, len(t)):
# 	print t[i], cp[i]
# Plot.plot_2d(t, cp)
# Plot.plot_2d(t, cv)
# Plot.plot_2d(t, ca)