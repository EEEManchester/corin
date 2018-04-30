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
if a:
	print 'not empty'
elif (not a):
	print 'empty'


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

print py
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
print bx, by
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

a = map(lambda x: (x[0],-1) , a)
print a

from collections import OrderedDict
print list(OrderedDict.fromkeys(a))