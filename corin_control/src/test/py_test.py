#!/usr/bin/env python

import numpy as np
import copy
import math
print math.tan(0.2)
print math.atan(0.2)
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
print b
print np.sum(b[:,0])

a = [1,2]
b = [0,1]
print np.round([1,2],3)