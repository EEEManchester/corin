#!/usr/bin/env python

import numpy as np
import copy
from termcolor import colored
import scipy
a = np.array([1,2,3])
b = np.array([4,5,6])
c = np.array([a,b]).reshape(6,1)

def seq(start, stop, step=1):
	n = int(round((stop - start)/float(step)))
	if n > 1:
		return([start + step*i for i in range(n+1)])
	elif n == 1:
		return([start])
	else:
		return([])

tphase = 1.8
lenp = seq(0, tphase*5, tphase)
# print len(lenp), lenp

a = []
b = np.array([1,2])
b = np.vstack((b, np.array([3,4])))
b = np.vstack((b, np.array([10,4])))
# print b
# print np.sum(b[:,0])

a = [1,2]
b = [0,1]
# print np.round([1,2],3)

def skew(v):
    if len(v) == 4: v = v[:3]/v[3]
    skv = np.roll(np.roll(np.diag(v.flatten()), 1, 1), -1, 0)
    return skv - skv.T

def tangential(v):
	a = np.zeros((3,3))
	for i in range(3):
		for j in range(3):
			a[i][j] = v[i]*v[j]	
	return a

force = np.array([ 10.05, -0.068,  7.684])
pos = np.array([-0.25,  -0.251,  0.   ])
snorm = np.array([0., 0., 1.])

fz = np.dot(snorm.T, force)
ft = np.dot(np.eye(3) - tangential(snorm), force)
print ft
print np.linalg.norm(ft), ' < ', fz