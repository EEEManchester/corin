#!/usr/bin/env python

from library import *
import copy
footholds = [[1],[2,3],[4],[None],[None],[None]]


a = np.array([1,2,3]).reshape(3,1)
b = np.array([4,5,6]).reshape(3,1)
c = np.array([0,0,0,0,0,0]).reshape(6,1)

c = np.array([a,b]).reshape(6,1)

def func(a,b=[0]*6):
	print a
	print a+b[0]
	return

# print func(1)
# a = raw_input('Enter some value: ')
# print a
# if (a == 'Y'):
# 	print 'Yes'
