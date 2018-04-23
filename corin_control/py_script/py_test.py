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
print arr
# np.place(arr, arr>2, [44, 55])
arr = np.insert(arr,2,np.array([0,1,0]))
print (arr)

a = True
b = True
c = True
a = np.array([1.11,2.321])

a = ([1,2,3],[6,3,2])# 'seom'
print a[1]
print type(a)
if (isinstance(a,str)):
	print 'a string!'
elif (isinstance(a,tuple)):
	print 'a tuple!'
else:
	print 'not a string'

def testf(**options):
	print options
	if options.get("start"):
		s = options.get("start")
		print 'start!', s

# testf(start=[1,2,3],end=2)