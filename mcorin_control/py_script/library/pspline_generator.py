#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint

import numpy as np
from numpy.linalg import inv
from fractions import Fraction
import math
import matplotlib.pyplot as plt
from scipy import linalg
	
import constant as UDC

class SplineGenerator:
	def __init__(self):
		# self.point 	= JointTrajectoryPoint()
		self.sh 	= UDC.STEP_HEIGHT
		self.nc 	= 4 					# Define the number of coefficients (order + 1) Quintic Polynomial: 6
		self.td 	= np.matrix([0]) 		# time interval
		self.cpx 	= np.zeros((1,3))		# cartesian position in matrix
		self.t_samp = UDC.TRAC_INTERVAL
		self.Ax 	= np.zeros((self.nc,3)) # coefficient matrix

	def LegPhase_Interpolation(self, sp, ep, snorm ,phase=2, ctime=2.0):
		#set waypoints for trajectory	
		self.sh = UDC.STEP_HEIGHT  	#param_gait.get_step_height()		
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
		if (phase==1):						# transfer phase 
			
			# vector from origin to travel path midpoint
			pdiv = np.array([0.125,0.5,0.875])
			sdiv = np.array([0.6*self.sh, self.sh, 0.6*self.sh])
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

			self.td = np.matrix( [0, 0.25*ctime, 0.5*ctime, 0.75*ctime, ctime ])
			
			self.cpx = sp
			self.cpx = np.vstack( (self.cpx, A ))
			self.cpx = np.vstack( (self.cpx, ep ))

		elif (phase==0):					# support phase
			self.td  = np.matrix( [0, ctime] )
			self.cpx = np.array([ sp, ep])

		else:
			print "No phase" 	# have a way to exit function if no phase selected
		
	def point_Interpolation(self, parray=0, ptime=0):
		self.td  = np.matrix([ptime])
		self.cpx = parray

	def velocity_heuristics(self, k):
		
		self.cvx = np.zeros((self.td.shape[1],3))
		self.cax = np.zeros((self.td.shape[1],3))

		for j in range(0,3):
			for i in range(1,k):
				try:
					dkx  = (self.cpx.item(i,j)-self.cpx.item(i-1,j))/(self.td.item(i)-self.td.item(i-1))
					dkx1 = (self.cpx.item(i+1,j)-self.cpx.item(i,j))/(self.td.item(i+1)-self.td.item(i))
				except:
					dkx  = 0.0
					dkx1 = 0.0

				if (np.sign(dkx) != np.sign(dkx1)):
					self.cvx[i][j] = 0
				else:
					self.cvx[i][j] = 0.5*(dkx + dkx1)

	def polynomial_spline(self, return_type=0):
		
		# Define the number of segments, k
		sshape = self.td.shape 		# determine size of matrix
		k = sshape[1] - 1

		self.velocity_heuristics(k)

		## Update the empty coefficient matrix
		self.Ax = np.zeros((self.nc,3))

		tt  = []
		ctp = np.zeros((0,3))
		ctv = np.zeros((0,3))
		cta = np.zeros((0,3))
		
		for i in range(0,k): 			# loop the number of via points
			# set matrix variables
			t0 = float(self.td.item(i))
			t1 = float(self.td.item(i+1))
			
			#Calcualte the polynomial coefficients
			if (self.nc == 4):
				## Define the polynomial equations
				M = np.matrix([	[1,  t0,  pow(t0,2),    pow(t0,3)		],
		     				[0,  1,   2*t0,    		3*(pow(t0,2))	],
						    [1,  t1,  pow(t1,2),    pow(t1,3)		],
						    [0,  1,   2*t1,    		3*(pow(t1,2))	]])

				B = np.matrix([ [self.cpx.item(i,0),self.cpx.item(i,1),self.cpx.item(i,2)], 
								[self.cvx.item(i,0),self.cvx.item(i,1),self.cvx.item(i,2)],
								[self.cpx.item(i+1,0),self.cpx.item(i+1,1),self.cpx.item(i+1,2)], 
								[self.cvx.item(i+1,0),self.cvx.item(i+1,1),self.cvx.item(i+1,2)] ])

			elif (self.nc == 6):
				M = np.matrix(	[	[1,  t0,  pow(t0,2),    pow(t0,3),      pow(t0,4),        pow(t0,5)],
		     					[0,  1,   2*t0,    		3*(pow(t0,2)),  4*(pow(t0,3)),    5*(pow(t0,4))],
						    	[0,  0,   2,       		6*t0,       	12*(pow(t0,2)),   20*(pow(t0,2))],
						    	[1,  t1,  pow(t1,2),    pow(t1,3),      pow(t1,4),        pow(t1,5)],
						    	[0,  1,   2*t1,    		3*(pow(t1,2)),  4*(pow(t1,3)),    5*(pow(t1,4))],
						    	[0,  0,   2,       		6*t1,       	12*(pow(t1,2)),   20*(pow(t1,3))] ])

				B = np.matrix([ [self.cpx.item(i,0),self.cpx.item(i,1),self.cpx.item(i,2)], 
								[self.cvx.item(i,0),self.cvx.item(i,1),self.cvx.item(i,2)],
								[self.cax.item(i,0),self.cax.item(i,1),self.cax.item(i,2)],
								[self.cpx.item(i+1,0),self.cpx.item(i+1,1),self.cpx.item(i+1,2)], 
								[self.cvx.item(i+1,0),self.cvx.item(i+1,1),self.cvx.item(i+1,2)],
								[self.cax.item(i+1,0),self.cax.item(i+1,1),self.cax.item(i+1,2)] ])

			self.Ax = linalg.solve(M,B)
			
			## determine trajectory timing phases and total number of points
			if(i+1 != k):															# intermediate segments
				T = np.arange(t0, t1, self.t_samp)
			else: 																	# final segment
				T = np.arange(t0, (t1+self.t_samp), self.t_samp)
				
			Tl   = T.shape[0] 	# determine size of matrix
			t_t  = np.zeros(Tl)
			cx_t = np.zeros((Tl,3))
			vx_t = np.zeros((Tl,3))
			ax_t = np.zeros((Tl,3))

			## Generate the trajectories
			for h in range(0,3):
				for j in range(0,Tl): 		# loop number of transitionary points
					t_t[j]	= T[j]

					if (self.nc == 4):
						cx_t[j][h] = self.Ax[0][h] + (self.Ax[1][h]*T[j]) + (self.Ax[2][h]*(T[j]**2)) + (self.Ax[3][h]*(T[j]**3)) 
						vx_t[j][h] = self.Ax[1][h] + (2*self.Ax[2][h]*T[j]) + (3*self.Ax[3][h]*(T[j]**2)) 
						ax_t[j][h] = (2*self.Ax[2][h]) + (6*self.Ax[3][h]*(T[j])) 

					elif (self.nc == 6):
						cx_t[j][h] = self.Ax[0][h] + (self.Ax[1][h]*T[j]) + (self.Ax[2][h]*(T[j]**2)) + (self.Ax[3][h]*(T[j]**3)) + + (self.Ax[4][h]*(T[j]**4)) + (self.Ax[5][h]*(T[j]**5))
						vx_t[j][h] = self.Ax[1][h] + (2*self.Ax[2][h]*T[j]) + (3*self.Ax[3][h]*(T[j]**2)) + (4*self.Ax[4][h]*(T[j]**3)) + (5*self.Ax[5][h]*(T[j]**4))
						ax_t[j][h] = (2*self.Ax[2][h]) + (6*self.Ax[3][h]*(T[j])) + (12*self.Ax[4][h]*(T[j]**2)) + (20*self.Ax[5][h]*(T[j]**3))

			## concatenate segments together
			tt 	= np.hstack( (tt,t_t) )
			ctp = np.vstack( (ctp,cx_t) )
			ctv = np.vstack( (ctv,vx_t) )
			cta = np.vstack( (cta,ax_t) )

		if (return_type==0):
			# put cartesian points into Joint Trajectory Point, clears previous items
			point = JointTrajectoryPoint()
			#point.positions 	= [ np.around(ctx,decimals=4), np.around(cty,decimals=4), np.around(ctz,decimals=4) ]
			point.positions 		= [ ctp[:,0], ctp[:,1], ctp[:,2] ]
			point.velocities 		= [ ctv[:,0], ctv[:,1], ctv[:,2] ]
			point.accelerations	= [ cta[:,0], cta[:,1], cta[:,2] ]
			point.time_from_start 	= [tt]

			return point
		elif (return_type==1):
			return ctp[:,0], ctp[:,1], ctp[:,2], tt
		elif (return_type==2):
			return ctp, ctv, cta, tt

	def generate_leg_spline(self, sp, ep, snorm , phase=2, ctime=2.0, spline_degree=4):
		self.nc = spline_degree
		self.LegPhase_Interpolation(sp, ep, snorm ,phase)
		
		return self.polynomial_spline()
		

	def generate_body_spline(self, parray, ptime, return_type=0, spline_degree=4):
		self.nc = spline_degree
		self.point_Interpolation(parray, ptime)
		
		return self.polynomial_spline(return_type)


	def get_point(self, t):
		cx_t = np.zeros(3)
		vx_t = np.zeros(3)
		ax_t = np.zeros(3)

		for h in range(0,3):
			if (self.nc == 4):
				cx_t[h] = self.Ax[0][h] + (self.Ax[1][h]*t) + (self.Ax[2][h]*(t**2)) + (self.Ax[3][h]*(t**3)) 
				vx_t[h] = self.Ax[1][h] + (2*self.Ax[2][h]*t) + (3*self.Ax[3][h]*(t**2)) 
				ax_t[h] = (2*self.Ax[2][h]) + (6*self.Ax[3][h]*(t)) 

			elif (self.nc == 6):
				cx_t[h] = self.Ax[0][h] + (self.Ax[1][h]*t) + (self.Ax[2][h]*(t**2)) + (self.Ax[3][h]*(t**3)) + + (self.Ax[4][h]*(t**4)) + (self.Ax[5][h]*(t**5))
				vx_t[h] = self.Ax[1][h] + (2*self.Ax[2][h]*t) + (3*self.Ax[3][h]*(t**2)) + (4*self.Ax[4][h]*(t**3)) + (5*self.Ax[5][h]*(t**4))
				ax_t[h] = (2*self.Ax[2][h]) + (6*self.Ax[3][h]*(t)) + (12*self.Ax[4][h]*(t**2)) + (20*self.Ax[5][h]*(t**3))

		return cx_t

# sample variable
sp = np.array([0.214,0.025, -0.1451])
ep = np.array([0.214,-0.025,-0.1451])
snorm = (0, 0, 0)

phase = 1
### CoM linear path ###
# x_com = np.array([.0,.0,0.15])
# x_com = np.vstack((x_com,np.array([0.025, 0.00, 0.15])))
# x_com = np.vstack((x_com,np.array([0.05, 0.00, 0.15])))
# x_com = np.vstack((x_com,np.array([0.075, 0.00, 0.15])))
# x_com = np.vstack((x_com,np.array([0.10, 0.00, 0.15])))
# x_com = np.vstack((x_com,np.array([0.15, 0.00, 0.15])))
# x_com = np.vstack((x_com,np.array([0.20, 0.00, 0.15])))
# x_com = np.vstack((x_com,np.array([0.25, 0.10, 0.15])))
# x_com = np.vstack((x_com,np.array([0.30, 0.05, 0.15])))
# x_com = np.vstack((x_com,np.array([0.35, 0.05, 0.15])))

# com_time = np.array([0.0,0.5,1.0,1.5,2.0,2.5,3.0,4.0,5.0, 6.0])
x_com = np.array([.0,.0,UDC.BODY_HEIGHT])

# ## short V spline
x_com = np.vstack((x_com,np.array([0.4, 0.0, UDC.BODY_HEIGHT])))
x_com = np.vstack((x_com,np.array([0.6, 0.0, UDC.BODY_HEIGHT+0.025])))
x_com = np.vstack((x_com,np.array([1.0, 0.0, UDC.BODY_HEIGHT+0.05])))
t_com = np.array([0.0,1.0,2.0,3.0]) 

spliner = SplineGenerator()
# spliner.generate_leg_spline(sp,ep,snorm,phase)
ctp, ctv, cta, tt = spliner.generate_body_spline(x_com, t_com, 1)
# print ctp

# fig = plt.figure()
# plt.plot(tt,ctp[:,1])
# plt.xlabel('x-axis');plt.ylabel('y-axis');plt.grid('on');
# plt.show()

