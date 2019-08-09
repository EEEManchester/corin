#!/usr/bin/env python

import numpy as np
import copy
import math
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

a = [np.array([1,2,3]), 0.5, [11,12], np.array([100,200]).reshape((2,1))]

data = []
for i in a:
	try:
		temp = iter(i)
		try:
			for item in i.flatten():
				data.append(item) 
		except:
			for item in i:
				data.append(item)
	except:
		data.append(i)
		
print data#[item for i in data for item in i]