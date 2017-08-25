#!/usr/bin/env python

# import sys
# sys.path.insert(0, '/home/wilson/catkin_ws/src/mcorin/mcorin_control/py_script/library')
# sys.dont_write_bytecode = True

import numpy as np
# import connex

dglobe = 2

def func1():
	x_com = np.array([.0,.0,0.1])
	w_com = np.array([1.0,.0,.0])
	x_com = np.vstack((x_com,np.array([0.2, 0.0, 0.2])))
	w_com = np.vstack((w_com,np.array([2.,0.,0.])))
	x_com = np.vstack((x_com,np.array([0.2, 0.0, 0.2])))
	w_com = np.vstack((w_com,np.array([2.,0.,0.])))
	x_com = np.vstack((x_com,np.array([0.2, 0.0, 0.2])))
	w_com = np.vstack((w_com,np.array([2.,0.,0.])))
	x_com = np.vstack((x_com,np.array([0.2, 0.0, 0.2])))
	w_com = np.vstack((w_com,np.array([2.,0.,0.])))
	t_com = np.array([0.0])
	mode  = 2 	
	## determine length of array
	return (x_com, w_com, mode)

def compute_path_length(via_points):
	point_size = len(via_points) 	# determine number of via points
	t_com = np.zeros(point_size) 	# create an array of via points size

	for i in range(0,point_size):
		t_com[i] = i

	return t_com

if __name__ == "__main__":
	test1 = 1

	data = func1()
	print type(data)
	x_com, w_com, mode = data
	
	# if 'test1' in globals():
	# 	print 'manager is global'

	# print globals()