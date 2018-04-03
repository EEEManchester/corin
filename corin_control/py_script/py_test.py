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

print np.array([1,2,3]).reshape(3,1)
