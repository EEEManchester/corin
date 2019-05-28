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

'''for q in range(0,91,10):
	qr = np.deg2rad(q)
	xd = np.array([0.0, (1-np.cos(qr))*COXA_Y, (np.sin(qr))*COXA_Y])

	x_cob = np.vstack(( x_cob, xd ))
	w_cob = np.vstack(( w_cob, np.array([qr,0.0,0.0]) ))
'''
#Plot.plot_3d(x_cob[:,0],x_cob[:,1],x_cob[:,2])
# Plot.plot_2d(x_cob[1],x_cob[2])

x2 = x_cob

x_cob[0] = 10

print(x2)

class wilson:

	a = 99
	b = 1

	def __init__(self):
		pass

	def __getitem__(self, index):
		print(self.a)

W = wilson()
W[1]
