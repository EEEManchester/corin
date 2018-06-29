#!/usr/bin/env python

from library import *
import copy
footholds = [[1],[2,3],[4],[None],[None],[None]]


a = np.array([1,2,3]).reshape(3,1)
b = np.array([4,5,6]).reshape(3,1)
c = np.array([0,0,0,0,0,0]).reshape(6,1)

c = np.array([a,b]).reshape(6,1)

# def func(a,b=[0]*6):
# 	print a
# 	print a+b[0]
# 	return
a = [[1,2,-7],[4,5,1]]
v_max = np.amax(a)
v_min = np.amin(a)
v_max = v_max if (abs(v_max) > abs(v_min)) else abs(v_min)

x_cob = np.array([.0,.0,.0])
w_cob = np.array([.0,.0,.0])

for q in range(0,91,10):
	qr = np.deg2rad(q)
	xd = np.array([0.0, (1-np.cos(qr))*COXA_Y, (np.sin(qr))*COXA_Y])
	
	x_cob = np.vstack(( x_cob, xd ))
	w_cob = np.vstack(( w_cob, np.array([qr,0.0,0.0]) ))

# Plot.plot_3d(x_cob[:,0],x_cob[:,1],x_cob[:,2])
# Plot.plot_2d(x_cob[1],x_cob[2])

def func1(a, xlist):
	def in_func1():
		print 'nested function'
		print a, xlist[a]

	print 'in func 1'
	if (a == 1):
		in_func1()
	else:
		print 'returning'

a = np.array([0.2,-0.1,-0.1])
b = np.array([0.2, 0.1,-0.1])

initial_foothold  = [np.zeros(3)]*3
# initial_foothold[0] = np.array([1,2,3])
# initial_foothold[1] = np.array([4,5,6])
# initial_foothold[2] = np.array([7,8,9])

# initial_foothold[0][0] = 2
# initial_foothold[1][2] = 15
# initial_foothold[0][0] = 6

initial_foothold[0] = a.copy()
# print initial_foothold[0]
initial_foothold[0][0] = 6

v3_dv = np.array([0.5,0.2,0.1])
surface_normal = np.array([0.,-1.,0.])

plane_vector = np.dot(rot_X(np.deg2rad(90)), surface_normal)
# vector projection
v3_pv = ( np.dot(v3_dv,plane_vector)/np.linalg.norm(plane_vector)**2 )*plane_vector
# print np.round(v3_pv,4)

# print v3_dv - (np.dot(v3_dv,surface_normal))*surface_normal
al = [1,2,3]
a = np.array([1,2,3,4,5,])
b = np.array([3,4])


arr = np.arange(6) #.reshape(2, 3)

# np.place(arr, arr>2, [44, 55])
arr = np.insert(arr,2,np.array([0,1,0]))

a = True
b = True
c = True
a = np.array([1.11,2.321])

a = ([1,2,3],[6,3,2])# 'seom'
a = [1]


world_ground_X_base = np.array([[ 0. ,    -0.9918 , 0.1275 , 0.    ],
								 [ 1.,      0.    , -0.    ,  0.    ],
								 [-0.,      0.1275,  0.9918,  0.1143],
								 [ 0.,      0.    ,  0.    ,  1.    ]])
world_ground_X_base[:2,3:4] = np.zeros((2,1))
Leg_base_X_femur = np.array([[-0.342,   0.,     -0.9397, -0.1355],
	 							[-0.9397,  0.,      0.342,  -0.1464],
	 							[ 0.,      1.,      0.   ,   0.    ],
	 							[ 0.,      0.,      0.   ,   1.    ]])
world_ground_X_femur = mX(world_ground_X_base, Leg_base_X_femur)

# print np.round(world_ground_X_base,4)
# print np.round(world_ground_X_femur,4)
qr = 0.128
hy = world_ground_X_femur[2,3] - L3 - 0. 		# h_femur_X_tibia
yy = np.sqrt(L2**2 - hy**2) 					# world horizontal distance from femur to foot
by = np.cos(qr)*(COXA_Y + L1) 				# world horizontal distance from base to femur 
sy = by + yy									# y_base_X_foot - leg frame
py = sy*np.sin(np.deg2rad(ROT_BASE_X_LEG[5]+LEG_OFFSET[5])) 	# y_base_X_foot - world frame

# pbas = np.array([[-0.19],[-0.28],[-0.1]])
temp = np.array([[-0.19],[py],[0.5]])
# pby = np.round(mX(rot_X(qr),temp),4)
# print pby
a = (1,1)
b = (2,1)

q = 0
d = 0
p = (13,15)
body_area = (10,6)

bx = int(math.floor(body_area[0])/2) # longitudinal distance to check
by = int(math.floor(body_area[1])/2) # lateral distance to check

for y in range(0,by+1): 		# cycle lateral - inflated if body_area[1] is even
	for x in range(0,2*bx+1):		# cycle longitudinal
		# displacement from centre rotated about q
		nx1 = p[0] + int(np.round((-bx+x)*np.cos(q) - (-y)*np.sin(q)))
		ny1 = p[1] + int(np.round((-bx+x)*np.sin(q) + (-y)*np.cos(q)))
		nx2 = p[0] + int(np.round((-bx+x)*np.cos(q) - (+y)*np.sin(q)))
		ny2 = p[1] + int(np.round((-bx+x)*np.sin(q) + (+y)*np.cos(q)))

	# 	print p, body_area, (nx1,ny1), (nx2,ny2)
	# print '----------------------------'

a = [(1,2),(5,5),(1,2)]

qi = 0.24
qf = 0.0
xi = np.array([1.02,0.39,0.15])
xf = np.array([1.02,0.36,0.10])
qrange = range(90,-1,-10) # (0,91,10)
delta_q = (qf-qi)/(len(qrange)-1)
delta_x = xf - xi
x_cob = xi.copy()
w_cob = np.array([qi, 0., 0.])

for i in range(1, len(qrange)):
	qr = np.deg2rad(qrange[i])
	xd = xi + np.array([(np.cos(qr))*delta_x[0],
									(np.cos(qr))*delta_x[1], 
									(1-np.sin(qr))*delta_x[2]])

	x_cob = np.vstack(( x_cob, np.round(xd,3) ))
	w_cob = np.vstack(( w_cob, np.array([qi+i*delta_q, 0., 0.]) ))

PathGenerator = Pathgenerator.PathGenerator()
base_path = PathGenerator.generate_base_path(x_cob, w_cob, 0.1)
# Plot.plot_3d(base_path.X.xp[:,0],base_path.X.xp[:,1],base_path.X.xp[:,2])
# Plot.plot_2d(base_path.X.t, base_path.W.xp)

def fu(radius=None):
	# if (radius is None):
	# 	radius = STEP_STROKE/2.
	# else: 
	# 	pass
	radius = STEP_STROKE/2. if (radius is None) else radius
	# print 'radius ', radius

# fu(0.01)
a = np.array([1.,0.,0.])
b = 1.
c = a/b
d = np.nan_to_num(a/b)

class Class1:
	def __init__(self):
		self.a = 1
		self.b = 2
		self.c = 3

class PathPlanner:
	def __init__(self, Map):
		self.name = 'This'

	def testf(self):
		print Map.c

import numpy as np
from scipy.optimize import linprog

def in_hull(points, x):
    n_points = len(points)
    n_dim = len(x)
    c = np.zeros(n_points)
    A = np.r_[points.T,np.ones((1,n_points))]
    b = np.r_[x, np.ones(1)]
    # print A
    # print b
    # print c
    # lp = linprog(c, A_ub=A, b_ub=b)
    # # lp = linprog(c, A_eq=A, b_eq=b)
    # print lp 
    # return lp.x

# n_points = 4#10000
# n_dim = 2#10
# Z = np.random.rand(n_points,n_dim)
# x = np.random.rand(n_dim)

Z = np.array([[0.,0.],[2.,0.],[0.,2.],[2,2]])
x = np.array([1,0,5,6,4,1])
# print x[:3]
t_cob = np.zeros(1)

t_cob = np.hstack((t_cob,1))

# x_out = (in_hull(Z, x))
# print x_out
# print np.sum(x_out)
# print np.dot(Z.T,x_out.reshape(4,1))

a = ['test', 'corin']
a = [1,2,3]
b = [4,5,6]

a = b
b = [9,9,9]


ti1 = 0.4
ti2 = 0.50
lv = 0.02
nv = 0.
tint = 0.02

xi = np.array([0., 2.0, 0.0])
xf = np.array([0., 4.0, 1.0])

qi = 0.0
qf = 20.0

qrange = range(0,int(20.),4)

delta_q = (qf-qi)/(len(qrange)-1)
delta_x = (xf-xi)/(len(qrange)-1)

x_cob = xi.copy()
w_cob = np.array([qi, 0., 0.])
t_cob = np.zeros(2*len(qrange)-1)

total_time = 1.2*6

for i in range(1, len(qrange)):
	qr = np.deg2rad(qrange[i])
	xd = xi + delta_x*i
	wd = qi+i*delta_q

	x_cob = np.vstack(( x_cob, xd ))
	w_cob = np.vstack(( w_cob, np.array([wd, 0., 0.]) ))
	t_cob[i] = i*total_time/(2*(len(qrange)-1))


for i in range(len(qrange),2*len(qrange)-1):
	x_cob = np.vstack(( x_cob, xd ))
	w_cob = np.vstack(( w_cob, np.array([wd, 0., 0.]) ))
	t_cob[i] = i*total_time/(2*(len(qrange)-1))

a = np.array([1,0,0])

a = [1,5,3]

sh = 0.1
sn1 = np.array([0.,0.,1.])
sn2 = np.array([0.,-1.,0.])
sp = np.array([0.0, -0.30, -0.1])
ep = np.array([0.0, 0.35,  0.1 ])
cpx = sp.copy()

for i in range(1,5):
	pu = sp + sn1*np.array([0.,0.,i*0.05*sh])
	cpx = np.vstack((cpx, pu))

qsp = np.zeros((0,3))
for i in range(0,46,5):
	qsp = np.vstack(( qsp, np.array([0.0, np.deg2rad(i), np.deg2rad(i)]) ))

tscale = np.array([range(0,len(qsp))])
rmax = float(np.amax(tscale))
tsp = tscale/rmax
# print 'sizeL', tsp.shape

a = np.nan_to_num(float('NaN')) 


sp = (0,0)
point_list = []
end = 2*3 - 2
for i in range(1,end):
	if (i%2):
		# move right
		for x in range(0,i):
            # skip last cell - ends in square shape
			if (end-i==1 and i-x==1):
				pass
			else:
				sp = (sp[0]+1,sp[1])
				point_list.append(sp)
		# move up
		if (i!=end-1):
			for x in range(0,i):
				sp = (sp[0],sp[1]+1)
				point_list.append(sp)
	else:
		# move left
		for x in range(0,i):
            # skip last cell - ends in square shape
			if (end-i==1 and i-x==1):
				pass
			else:
				sp = (sp[0]-1,sp[1])
				point_list.append(sp)
		# move down
		if (i!=end-1):
			for x in range(0,i):
				sp = (sp[0],sp[1]-1)
				point_list.append(sp)


a = (1,2,3)
b = np.asarray(a)
a = np.array([[1,2],[3,4]])
print a[0,0]
a[0,0] = 2
