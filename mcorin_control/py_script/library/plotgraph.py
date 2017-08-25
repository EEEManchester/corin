#!/usr/bin/env python

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_3d(x=0,y=0,z=0):
	fig = plt.figure()
	ax = fig.gca(projection='3d')
	ax.plot(x, y, z, label='parametric curve', marker='x')
	ax.set_xlabel('X axis')
	ax.set_ylabel('Y axis')
	ax.set_zlabel('Z axis')
	plt.grid('on');
	plt.show()

def plot_2d(x=0,y=0):
	fig = plt.figure()
	plt.plot(x,y, marker='x')
	plt.xlabel('x-axis');plt.ylabel('y-axis');plt.grid('on');
	plt.show()

def plot_2d_multiple(no_plots=0, t=0, x=0, y=0, z=0):
	fig, ax = plt.subplots()

	if (no_plots==1):
		ax.plot(t, x, label='x');	
	elif (no_plots==2):
		ax.plot(t, x, label='x');
		ax.plot(t, y, label='y');	
	elif (no_plots==3):
		ax.plot(t, x, label='x');
		ax.plot(t, y, label='y');	
		ax.plot(t, z, label='z')

	legend = ax.legend(loc='upper center', shadow=True)
	plt.grid('on');
	plt.show()