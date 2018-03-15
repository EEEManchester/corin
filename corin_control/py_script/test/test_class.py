#!/usr/bin/env python

import sys
sys.path.insert(0, '/home/wilson/catkin_ws/src/corin/corin_control/py_script/library')
sys.dont_write_bytecode = True

import numpy as np
from matrix_transforms import *
from enum import Enum 


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

def test_func(game_type):
	return 'white' if game_type == 'home' else 'green'

def bar(first, second, third, **options):
    if options.get("action") == "sum":
        print("The sum is: %d" %(first + second + third))

    if options.get("number") == "first":
        return first

def update_state(**options):
	if options.get("reset") == True:
		print 'Resetting'

	print 'Running as normal'

if __name__ == "__main__":
	a = True
	bar(1,2,3, action='sum') if a else None
	
	# print bar(1,2,3, action='sum')
	# update_state(reset=True)

	f = np.array([1,2])
	
	a = np.array([[1,2],[2,2]])
	b = np.array([[1,2],[2,2]])
	
	
	data = test_func('home1')
	# print data
	test1 = 1
	Animal = Enum('Animal', 'ant bee cat dog')
	data = func1()
	# print type(data)
	x_com, w_com, mode = data

	# if 'test1' in globals():
	# 	print 'manager is global'

	# print globals()
	radian = -0.9
	## Direct Control
	value_of_0_radian_position_      = 2048
	value_of_min_radian_position_    = 0
	value_of_max_radian_position_    = 4095
	min_radian_                      = -3.14159265
	max_radian_                      =  3.14159265

	# angles = tf.euler_from_quaternion(quat)
	
	xp = np.array([ [1,2,3],[4,5,6],[7,8,9] ])
	
	tlen = np.array([1,2,3,4,5.4,6])
	
	Column6D = (1,1)
	
	a = 1
	try:
		len(a)
	except TypeError, e:
		print 'Value ', e
	except Exception, e:
		print e