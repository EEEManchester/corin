#!/usr/bin/env python

## VDP oscillator
import numpy as np
from scipy import linalg
from scipy import integrate
from scipy.integrate import odeint
import matplotlib.pyplot as plt

from constant import *
import kdl

class VDP():
	def __init__(self):
		
		self.a = np.array([0.,0.,0.,0.,0.,0.]) 	# global variables for a1-a6, also z1, y1, x1, w1, v1, u1
		#Tripod Gait mode 1		
		self.trigait = np.matrix([ 	[0,    -0.2,   -0.2,     0.2,   0.2,   -0.2],
									[-0.2,    0,    0.2,    -0.2,  -0.2,    0.2],
									[-0.2,  0.2,      0,    -0.2,  -0.2,    0.2],
									[0.2,  -0.2,   -0.2,     0,     0.2,   -0.2],
									[0.2,  -0.2,   -0.2,     0.2,   0,     -0.2],
									[-0.2,  0.2,    0.2,    -0.2,  -0.2,    0  ],
									[1./2.,    12,   9,       0,     0,      0] ])

		#self.trigait = np.matrix([ 	[0     , -0.2,   -0.2,     0.  ,   0.  ,   -0.2], 
		#							[-0.2  ,  0.  ,    0.  ,    -0.2,  -0.2,    0.  ],
		#							[-0.2  ,  0.  ,    0.  ,    -0.2,  -0.2,    0.  ],
		#							[0.    ,  -0.2,   -0.2,     0.,     0.  ,   -0.2],
		#							[0.    ,  -0.2,   -0.2,     0.  ,   0.,     -0.2],
		#							[-0.2  ,  0  ,    0.  ,    -0.2,  -0.2,    0.  ],
		#							[1./2.,  10.,     11.,      5.,     0.,      0.] ])

		#Tetrapod Gait mode 2
		self.tetgait = np.matrix([  [0.   ,0.,-0.2,0.,0.,-0.2 ],
									[-0.2 ,0.,0.,-0.2,0.,0.],
									[0.    ,-0.2,0.,   0.,-0.2,0.],
									[0.,0. ,-0.2,0.,0.,-0.2],
									[-0.2  ,0.,0.,-0.2,0.,0.],
									[0.    ,-0.2,0., 0.,-0.2, 0.],
									[3./4. ,25.,42., 0., 0.,0.] ])
		#Meta wave Gait mode 3
		self.mwavegait = np.matrix([  [0.,0.,-0.2,0.,0.,-0.2 ],
									  [-0.2,0.,0.,-0.2,0.,0.],
									  [0.,-0.2,0.,0.,-0.2,0.],
									  [0.,0.,-0.2,0.,0., -0.2],
									  [-0.2,0.,0.,-0.2,0.,0.],
									  [0.,-0.2,0.,0.,-0.2,0.],
								 	  [3./4.,25.,42.,0.,0.,0.] ])
		self.gait_selection = 1  # 0 = tri, 1 = tet, 2 = mwave
		#self.lam = 0


		self.mu = 1.;	#shape
		self.Ps = 6.;	#amplitude
		self.Ws = 20.;	#frequency

		## Mapping function parameters
		self.r = 12; 			# step size
		self.ang = 0; 			# robot orientation angle 
		self.kz = 1; 			# amplitude gain for mapping
		self.b = np.ones(6) 	# b = ones (6); # Swing phase counter
		self.c = np.ones(6) 	# c = ones (6); # Support phase counter

		## create KDL class
		self.KDL = kdl.corin_kinematics()

	def alpha1(self, Y, t):
		## variable mapping - convenience
		lam11 = self.lam[0,0];
		lam12 = self.lam[0,1]; 	# lam12 = lam(1,2);
		lam13 = self.lam[0,2];
		lam14 = self.lam[0,3];
		lam15 = self.lam[0,4];
		lam16 = self.lam[0,5];

		x 	 = self.a[0]+ (lam12*self.a[1]) +(lam13*self.a[2]) + (lam14*self.a[3])+(lam15*self.a[4])+(lam16*self.a[5]);
		return [Y[1], self.mu*((self.Ps)-x**2)*Y[1]-(self.Ws)*x]


	def alpha2(self, Y, t):

		lam21 = self.lam[1,0];
		lam22 = self.lam[1,1]; 	# lam12 = lam(1,2);
		lam23 = self.lam[1,2];
		lam24 = self.lam[1,3];
		lam25 = self.lam[1,4];
		lam26 = self.lam[1,5];
		x 	 = self.a[1]+ (lam21*self.a[0]) +(lam23*self.a[2]) + (lam24*self.a[3])+(lam25*self.a[4])+(lam26*self.a[5]);
		return [Y[1], self.mu*((self.Ps)-x**2)*Y[1]-(self.Ws)*x]



	def alpha3(self, Y, t):

		lam31 = self.lam[2,0];
		lam32 = self.lam[2,1]; 	# lam12 = lam(1,2);
		lam33 = self.lam[2,2];
		lam34 = self.lam[2,3];
		lam35 = self.lam[2,4];
		lam36 = self.lam[2,5];

		x 	 = self.a[2]+ (lam31*self.a[0]) +(lam32*self.a[1]) + (lam34*self.a[3])+(lam35*self.a[4])+(lam36*self.a[5]);
		return [Y[1], self.mu*((self.Ps)-x**2)*Y[1]-(self.Ws)*x]


	def alpha4(self, Y, t):

		lam41 = self.lam[3,0];
		lam42 = self.lam[3,1]; 	# lam12 = lam(1,2);
		lam43 = self.lam[3,2];
		lam44 = self.lam[3,3];
		lam45 = self.lam[3,4];
		lam46 = self.lam[3,5];


		x 	 = self.a[3]+ (lam41*self.a[0]) +(lam42*self.a[1]) + (lam43*self.a[2])+(lam45*self.a[4])+(lam46*self.a[5]);
		return [Y[1], self.mu*((self.Ps)-x**2)*Y[1]-(self.Ws)*x]


	def alpha5(self, Y, t):

		lam51 = self.lam[4,0];
		lam52 = self.lam[4,1]; 	# lam12 = lam(1,2);
		lam53 = self.lam[4,2];
		lam54 = self.lam[4,3];
		lam55 = self.lam[4,4];
		lam56 = self.lam[4,5];

		x 	 = self.a[4]+ (lam51*self.a[0]) +(lam52*self.a[1]) + (lam53*self.a[2])+(lam54*self.a[3])+(lam56*self.a[5]);
		return [Y[1], self.mu*((self.Ps)-x**2)*Y[1]-(self.Ws)*x]


	def alpha6(self, Y, t):

		lam61 = self.lam[5,0];
		lam62 = self.lam[5,1]; 	# lam12 = lam(1,2);
		lam63 = self.lam[5,2];
		lam64 = self.lam[5,3];
		lam65 = self.lam[5,4];
		lam66 = self.lam[5,5];

		x 	 = self.a[5]+ (lam61*self.a[0]) +(lam62*self.a[1]) + (lam63*self.a[2])+(lam64*self.a[3])+(lam65*self.a[4]);
		return [Y[1], self.mu*((self.Ps)-x**2)*Y[1]-(self.Ws)*x]

	def compute(self):
		## declare & initialise variables
		tint = 402 # number of samples
		z = np.zeros((tint,2)) # array for CPG output
		z[0] = np.array([1.,0.]) # set initial value of CPG
		zx 	 = np.zeros((tint,3)) 	# array for leg cartesian output

		y = np.zeros((tint,2))
		y[0] = np.array([1.,0.])
		yx 	 = np.zeros((tint,3))

		x = np.zeros((tint,2))
		x[0] = np.array([1.,0.])
		xx 	 = np.zeros((tint,3))

		w = np.zeros((tint,2))
		w[0] = np.array([1.,0.])
		wx 	 = np.zeros((tint,3))

		v = np.zeros((tint,2))
		v[0] = np.array([1.,0.])
		vx 	 = np.zeros((tint,3))

		u = np.zeros((tint,2))
		u[0] = np.array([1.,0.])
		ux 	 = np.zeros((tint,3))

		tspan = np.arange(0, 0.05, 0.0001)

				## gait selection - to change gait modify the value self.gait_selection
		if (self.gait_selection == 0):
			self.lam = self.trigait
		elif (self.gait_selection == 1):
			self.lam = self.tetgait
		elif (self.gait_selection == 2):
			self.lam = self.mwavegait


		Tsw = self.lam[6,1]; 	# Tsw = lam(7,2);
		Tsu = self.lam[6,2]; 	# Tsu = lam(7,3);
		sh = self.lam[6,3];     # to find threshold of

		for a in range(0,tint-1):
			z[0] = np.array([1.,0.])
			
			# solve using runge-kutta 4th order 	
			#Oscillator 1
			self.ode_solution = integrate.odeint(self.alpha1, z[a], tspan) 	# [t,Y] = ode45('alpha1',[0 0.05],z(a,:)); 
			z1 = self.ode_solution[-1][0] 	# z1 = Y(length(Y),1);
			z2 = self.ode_solution[-1][1] 	# z2 = Y(length(Y),2);
			z[a+1] = np.array([z1, z2]) 	## z(a+1,:) = [z1 z2];
			self.a[0] = z1 					## coupleda1(z1); 

			#mapping function
			am = tint 			# different variable, same name
			if (a <= am/2):  # As CPG is not stable at the beginning 
				zx[a,:] = np.array([0.,0.,0.])


			elif ( (a > am/2) and (z[a,0] >= 10*self.lam[6,0] - 5) ):
				zx[a,0]    = self.r * np.sin(self.ang)*((self.b[0]/Tsw));	# gx1(a) = r * sin (ang) *((b(1))/Tsw);
				zx[a,1]    = self.r * np.cos(self.ang)*((self.b[0]/Tsw));	# gy1(a) = r * cos (ang) *((b(1))/Tsw);
				zx[a,2] 	  = self.kz*z[a,0] 									# gz1(a) = kz.*z(a,1);
				self.c[0] = 0;												# c(1) = 0;
				self.b[0] = self.b[0]+1 									# b(1) = b(1)+1

			else:
				zx[a,0]    = self.r * np.sin(self.ang)*( (Tsu-self.c[0])/Tsu ) 	# gx1(a) = r * sin (ang) *((Tsu-c(1))/(Tsu));
				zx[a,1]    = self.r * np.cos(self.ang)*( (Tsu-self.c[0])/Tsu )	# gy1(a) = r * cos (ang) *((Tsu-c(1))/(Tsu));
				zx[a,2] 	  = 0; 				# gz1(a) = 0; 
				self.b[0] = 0 				# b(1) = 0;  
				self.c[0] = self.c[0] + 1 	# c(1) = c(1)+1
			##===============================================================================================================##
				
			#Oscillator 2
			self.ode_solution = integrate.odeint(self.alpha2, y[a], tspan) 	# [t,Y] = ode45('alpha1',[0 0.05],z(a,:)); 
			y1 = self.ode_solution[-1][0] 	# z1 = Y(length(Y),1);
			y2 = self.ode_solution[-1][1] 	# z2 = Y(length(Y),2);
			y[a+1] = np.array([y1, y2]) 	## z(a+1,:) = [z1 z2];
			self.a[1] = y1 			

						#mapping function
			am = tint 			# different variable, same name
			if (a <= am/2):  # As CPG is not stable at the beginning 
				yx[a,:] = np.array([0.,0.,0.])


			elif ( (a > am/2) and (y[a,0] >= 10*self.lam[6,0] - 5) ):
				yx[a,0]    = self.r * np.sin(self.ang)*((self.b[1]/Tsw));	# gx1(a) = r * sin (ang) *((b(1))/Tsw);
				yx[a,1]    = self.r * np.cos(self.ang)*((self.b[1]/Tsw));	# gy1(a) = r * cos (ang) *((b(1))/Tsw);
				yx[a,2] 	  = self.kz*y[a,0] 									# gz1(a) = kz.*z(a,1);
				self.c[1] = 0;												# c(1) = 0;
				self.b[1] = self.b[1]+1 									# b(1) = b(1)+1

			else:
				yx[a,0]    = self.r * np.sin(self.ang)*( (Tsu-self.c[1])/Tsu ) 	# gx1(a) = r * sin (ang) *((Tsu-c(1))/(Tsu));
				yx[a,1]    = self.r * np.cos(self.ang)*( (Tsu-self.c[1])/Tsu )	# gy1(a) = r * cos (ang) *((Tsu-c(1))/(Tsu));
				yx[a,2] 	  = 0; 				# gz1(a) = 0; 
				self.b[1] = 0 				# b(1) = 0;  
				self.c[1] = self.c[1] + 1 	# c(1) = c(1)+1
			##===============================================================================================================##

			#Oscillator 3	
			self.ode_solution = integrate.odeint(self.alpha3, x[a], tspan) 	# [t,Y] = ode45('alpha1',[0 0.05],z(a,:)); 
			x1 = self.ode_solution[-1][0] 	# z1 = Y(length(Y),1);
			x2 = self.ode_solution[-1][1] 	# z2 = Y(length(Y),2);
			x[a+1] = np.array([x1, x2]) 	## z(a+1,:) = [z1 z2];
			self.a[2] = x1 			

						#mapping function
			am = tint 			# different variable, same name
			if (a <= am/2):  # As CPG is not stable at the beginning 
				xx[a,:] = np.array([0.,0.,0.])

			elif ( (a > am/2) and (x[a,0] >= 10*self.lam[6,0] - 5) ):
				xx[a,0]    = self.r * np.sin(self.ang)*((self.b[2]/Tsw));	# gx1(a) = r * sin (ang) *((b(1))/Tsw);
				xx[a,1]    = self.r * np.cos(self.ang)*((self.b[2]/Tsw));	# gy1(a) = r * cos (ang) *((b(1))/Tsw);
				xx[a,2] 	  = self.kz*x[a,0] 									# gz1(a) = kz.*z(a,1);
				self.c[2] = 0;												# c(1) = 0;
				self.b[2] = self.b[2]+1 									# b(1) = b(1)+1

			else:
				xx[a,0]    = self.r * np.sin(self.ang)*( (Tsu-self.c[2])/Tsu ) 	# gx1(a) = r * sin (ang) *((Tsu-c(1))/(Tsu));
				xx[a,1]    = self.r * np.cos(self.ang)*( (Tsu-self.c[2])/Tsu )	# gy1(a) = r * cos (ang) *((Tsu-c(1))/(Tsu));
				xx[a,2] 	  = 0; 				# gz1(a) = 0; 
				self.b[2] = 0 				# b(1) = 0;  
				self.c[2] = self.c[2] + 1 	# c(1) = c(1)+1
			##===============================================================================================================##

			#Oscillator 4
			self.ode_solution = integrate.odeint(self.alpha4, w[a], tspan) 	# [t,Y] = ode45('alpha1',[0 0.05],z(a,:)); 
			w1 = self.ode_solution[-1][0] 	# z1 = Y(length(Y),1);
			w2 = self.ode_solution[-1][1] 	# z2 = Y(length(Y),2);
			w[a+1] = np.array([w1, w2]) 	## z(a+1,:) = [z1 z2];
			self.a[3] = w1 			

						#mapping function
			am = tint 			# different variable, same name
			if (a <= am/2):  # As CPG is not stable at the beginning 
				wx[a,:] = np.array([0.,0.,0.])


			elif ( (a > am/2) and (w[a,0] >= 10*self.lam[6,0] - 5) ):
				wx[a,0]    = self.r * np.sin(self.ang)*((self.b[3]/Tsw));	# gx1(a) = r * sin (ang) *((b(1))/Tsw);
				wx[a,1]    = self.r * np.cos(self.ang)*((self.b[3]/Tsw));	# gy1(a) = r * cos (ang) *((b(1))/Tsw);
				wx[a,2] 	  = self.kz*w[a,0] 									# gz1(a) = kz.*z(a,1);
				self.c[3] = 0;												# c(1) = 0;
				self.b[3] = self.b[3]+1 									# b(1) = b(1)+1

			else:
				wx[a,0]    = self.r * np.sin(self.ang)*( (Tsu-self.c[3])/Tsu ) 	# gx1(a) = r * sin (ang) *((Tsu-c(1))/(Tsu));
				wx[a,1]    = self.r * np.cos(self.ang)*( (Tsu-self.c[3])/Tsu )	# gy1(a) = r * cos (ang) *((Tsu-c(1))/(Tsu));
				wx[a,2] 	  = 0; 				# gz1(a) = 0; 
				self.b[3] = 0 				# b(1) = 0;  
				self.c[3] = self.c[3] + 1 	# c(1) = c(1)+1
			##===============================================================================================================##

			#Oscillator 5
			self.ode_solution = integrate.odeint(self.alpha5, v[a], tspan) 	# [t,Y] = ode45('alpha1',[0 0.05],z(a,:)); 
			v1 = self.ode_solution[-1][0] 	# z1 = Y(length(Y),1);
			v2 = self.ode_solution[-1][1] 	# z2 = Y(length(Y),2);
			v[a+1] = np.array([v1, v2]) 	## z(a+1,:) = [z1 z2];
			self.a[4] = v1 			

						#mapping function
			am = tint 			# different variable, same name
			if (a <= am/2):  # As CPG is not stable at the beginning 
				vx[a,:] = np.array([0.,0.,0.])


			elif ( (a > am/2) and (v[a,0] >= 10*self.lam[6,0] - 5) ):
				vx[a,0]    = self.r * np.sin(self.ang)*((self.b[4]/Tsw));	# gx1(a) = r * sin (ang) *((b(1))/Tsw);
				vx[a,1]    = self.r * np.cos(self.ang)*((self.b[4]/Tsw));	# gy1(a) = r * cos (ang) *((b(1))/Tsw);
				vx[a,2] 	  = self.kz*v[a,0] 									# gz1(a) = kz.*z(a,1);
				self.c[4] = 0;												# c(1) = 0;
				self.b[4] = self.b[4]+1 									# b(1) = b(1)+1

			else:
				vx[a,0]    = self.r * np.sin(self.ang)*( (Tsu-self.c[4])/Tsu ) 	# gx1(a) = r * sin (ang) *((Tsu-c(1))/(Tsu));
				vx[a,1]    = self.r * np.cos(self.ang)*( (Tsu-self.c[4])/Tsu )	# gy1(a) = r * cos (ang) *((Tsu-c(1))/(Tsu));
				vx[a,2] 	  = 0; 				# gz1(a) = 0; 
				self.b[4] = 0 				# b(1) = 0;  
				self.c[4] = self.c[4] + 1 	# c(1) = c(1)+1
			##===============================================================================================================##

			#Oscillator 6
			self.ode_solution = integrate.odeint(self.alpha6, u[a], tspan) 	# [t,Y] = ode45('alpha1',[0 0.05],z(a,:)); 
			u1 = self.ode_solution[-1][0] 	# z1 = Y(length(Y),1);
			u2 = self.ode_solution[-1][1] 	# z2 = Y(length(Y),2);
			u[a+1] = np.array([u1, u2]) 	## z(a+1,:) = [z1 z2];
			self.a[5] = u1 			
						#mapping function
			am = tint 			# different variable, same name
			if (a <= am/2):  # As CPG is not stable at the beginning 
				ux[a,:] = np.array([0.,0.,0.])

			elif ( (a > am/2) and (u[a,0] >= 10*self.lam[6,0] - 5) ):
				ux[a,0]    = self.r * np.sin(self.ang)*((self.b[5]/Tsw));	# gx1(a) = r * sin (ang) *((b(1))/Tsw);
				ux[a,1]    = self.r * np.cos(self.ang)*((self.b[5]/Tsw));	# gy1(a) = r * cos (ang) *((b(1))/Tsw);
				ux[a,2] 	  = self.kz*u[a,0] 									# gz1(a) = kz.*z(a,1);
				self.c[5] = 0;												# c(1) = 0;
				self.b[5] = self.b[5]+1 									# b(1) = b(1)+1

			else:
				ux[a,0]    = self.r * np.sin(self.ang)*( (Tsu-self.c[5])/Tsu ) 	# gx1(a) = r * sin (ang) *((Tsu-c(1))/(Tsu));
				ux[a,1]    = self.r * np.cos(self.ang)*( (Tsu-self.c[5])/Tsu )	# gy1(a) = r * cos (ang) *((Tsu-c(1))/(Tsu));
				ux[a,2] 	  = 0; 				# gz1(a) = 0; 
				self.b[5] = 0 				# b(1) = 0;  
				self.c[5] = self.c[5] + 1 	# c(1) = c(1)+1
			##===============================================================================================================##

		## ================================== Inverse Kinematic ================================== ## 
		## Sample use of inverse kinematic
		position = np.array([ [0.3], [0.0], [-0.14] ]) 	# positions are [x, y, z]
		output 	 = self.KDL.IK(position)
		print 'Positions are reachable by leg, the output: '
		print output
		print self.KDL.FK(output)

		# the following is an example where it is out of range
		position = np.array([ [1.3], [0.0], [-0.14] ]) 	# positions are [x, y, z]
		print 'Positions are UN-reachable by leg, error occurs: '
		output = self.KDL.IK(position)
		
		## ================================== ================== ================================== ##

		t_range = np.arange(0,tint,1)

		disable_plot = True
		if (disable_plot != True):
			fig = plt.figure(1)
			plt.plot(t_range,z[:,0], 'r--',label="LF")
			plt.plot(t_range,y[:,0], 'b-',label="RF")
			plt.plot(t_range,x[:,0], 'g--',label="LM")
			plt.plot(t_range,w[:,0], 'm-',label="RM")
			plt.plot(t_range,v[:,0], 'c--',label="LR")
			plt.plot(t_range,u[:,0], 'k--',label="RR")#, marker='x')
			plt.xlim((300,400))
			plt.xlabel('x-axis');plt.ylabel('y-axis');plt.grid('on');
			plt.legend(bbox_to_anchor=(1.05,1),loc = 2,borderaxespad=0.)
			plt.show()


			fig = plt.figure(2)
			plt.subplot(6,2,1)
			plt.plot(t_range,zx); 					# plot for end-effector trajectory
			plt.xlabel('x-axis');plt.ylabel('y-axis');plt.grid('on');

			plt.subplot(6,2,2)
			plt.plot(zx[:,1],zx[:,2]); 					# plot for end-effector trajectory
			plt.xlabel('x-axis');plt.ylabel('y-axis');plt.grid('on');
			plt.xlim([-2,14])
			plt.ylim([0,7])


			plt.subplot(6,2,3)
			plt.plot(t_range,yx); 					# plot for end-effector trajectory
			plt.xlabel('x-axis');plt.ylabel('y-axis');plt.grid('on');


			plt.subplot(6,2,4)
			plt.plot(yx[:,1],yx[:,2]); 					# plot for end-effector trajectory
			plt.xlabel('x-axis');plt.ylabel('y-axis');plt.grid('on');
			plt.xlim([-2,14])
			plt.ylim([0,7])

			plt.subplot(6,2,5)
			plt.plot(t_range,xx); 					# plot for end-effector trajectory
			plt.xlabel('x-axis');plt.ylabel('y-axis');plt.grid('on');


			plt.subplot(6,2,6)
			plt.plot(xx[:,1],xx[:,2]); 					# plot for end-effector trajectory
			plt.xlabel('x-axis');plt.ylabel('y-axis');plt.grid('on');
			plt.xlim([-2,14])
			plt.ylim([0,7])

			plt.subplot(6,2,7)
			plt.plot(t_range,wx); 					# plot for end-effector trajectory
			plt.xlabel('x-axis');plt.ylabel('y-axis');plt.grid('on');


			plt.subplot(6,2,8)
			plt.plot(wx[:,1],wx[:,2]); 					# plot for end-effector trajectory
			plt.xlabel('x-axis');plt.ylabel('y-axis');plt.grid('on');
			plt.xlim([-2,14])
			plt.ylim([0,7])

			plt.subplot(6,2,9)
			plt.plot(t_range,vx); 					# plot for end-effector trajectory
			plt.xlabel('x-axis');plt.ylabel('y-axis');plt.grid('on');


			plt.subplot(6,2,10)
			plt.plot(vx[:,1],vx[:,2]); 					# plot for end-effector trajectory
			plt.xlabel('x-axis');plt.ylabel('y-axis');plt.grid('on');
			plt.xlim([-2,14])
			plt.ylim([0,7])

			plt.subplot(6,2,11)
			plt.plot(t_range,ux); 					# plot for end-effector trajectory
			plt.xlabel('x-axis');plt.ylabel('y-axis');plt.grid('on');


			plt.subplot(6,2,12)
			plt.plot(ux[:,1],ux[:,2]); 					# plot for end-effector trajectory
			plt.xlabel('x-axis');plt.ylabel('y-axis');plt.grid('on');
			plt.xlim([-2,14])
			plt.ylim([0,7])

			plt.show()


		return z

if __name__ == "__main__":
	cpg = VDP()
	cpg.compute()

