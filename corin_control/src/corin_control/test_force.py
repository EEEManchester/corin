#!/usr/bin/env python

import numpy as np
import stabilipy
import force_distribution
import time
from scipy.spatial import ConvexHull, Delaunay
from constant import *

## Test Data 01
# normals = [[[0.0, -1.0, 0.0]], [[0.0, -1.0, 0.0]], [[0.0, 1.0, 0.0]], [[0.0, 1.0, 0.0]]]
# normals = [[[0.0, -0.0, 1.0]], [[0.0, 1.0, 0.0]], [[0.0, 0.0, 1.0]], [[0.0, 0.0, 1.0]], [[0.0, 0.0, 1.0]]]
# normals = [[[0.0, -1.0, 0.0]], [[0.0, -1.0, 0.0]], [[0.0, 1.0, 0.0]], [[0.0, 1.0, 0.0]], [[0.0, 1.0, 0.0]]]
# pos = [	[-0.22561972,  0.30945383, -0.00034246], 
# 			[-0.31952697,  0.3094684 , -0.00034287], 
# 			[ 4.60182659e-01,  4.04583972e-02, -3.42660774e-04], 
# 			[ 2.66755852e-01, -3.50555585e-01, -3.42615948e-04], 
# 			[ 1.27600822e-01, -3.50571928e-01, -3.42023479e-04]]
# pos = [[0.117, 0.322, 0.602], [0.002, 0.323, 0.602], [-0.113, 0.323, 0.602], [-0.001, -0.02, -0.005], [-0.116, -0.02, -0.005]]
# normals = [[[0.0, -1.0, 0.0]], [[0.0, -1.0, 0.0]], [[0.0, -1.0, 0.0]], [[0.0, 0.0, 1.0]], [[0.0, 0.0, 1.0]]]
gphase = [0,0,0,1,0,0]



## Test Data 02
# normals = [[[0.0, -1.0, 0.0]], [[0.0, -1.0, 0.0]], [[0.0, 1.0, 0.0]], [[0.0, 1.0, 0.0]]]
# pos = [ [0.1, 0.1,0.0],
# 		[-0.1,0.1,0.0],
# 		[0.1,-0.1,0.0],
# 		[-0.1,-0.1,0.0] ]
# contacts = [0,1,0,0,1,0]

## Thesis test data:
## Set 1
friction = 0.5
pos = [[0.13, 0.33, 0.58], [0.02, 0.32, 0.58], [-0.1, 0.32, 0.59], [-0.0, -0.02, -0.02], [-0.12, -0.02, -0.02]]
normals = [[[0.0, -1.0, 0.0]], [[0.0, -1.0, 0.0]], [[0.0, -1.0, 0.0]], [[0.0, 0.0, 1.0]], [[0.0, 0.0, 1.0]]]
gphase = [0,0,1,0,0,0]
x = [-20, 20]
y = [-10, 40]

## Set 2
# test for friction = 1.0 and 0.8
# x = [-0, 130] # plot axis: plt.xticks(np.arange(0., 1.21, step=0.2), fontname="cmr10", fontsize=fsize)
# y = [-60, 80] # plot axis: plt.yticks(np.arange(-0.6, 0.9, step=0.2), fontname="cmr10", fontsize=fsize)
# pos = [[0.3703, 0.5559, 0.5], [0.266, 0.556, 0.5], [0.9176, 0.1531, 0.5], [0.8954, 0.0108, 0.5], [0.7356, 0.011, 0.5]]
# normals = [[[0.0, -1.0, 0.0]], [[0.0, -1.0, 0.0]], [[-1.0, 0.0, 0.0]], [[0.0, 1.0, 0.0]], [[0.0, 1.0, 0.0]]]
# gphase = [0,0,1,0,0,0]

## Set 4
# pos = [[0.12, 0.3, 0.1], [0.0, 0.3, 0.1], [-0.12, 0.3, 0.1], [0.12, -0.3, 0.1], [0.0, -0.3, 0.1], [-0.12, -0.3, 0.1]]
# normals = [[[0.0, 0.0, 1.0]], [[0.0, 0.0, 1.0]], [[0.0, 0.0, 1.0]], [[0.0, 0.0, 1.0]], [[0.0, 0.0, 1.0]], [[0.0, 0.0, 1.0]]]
# gphase = [0,0,0,0,0,0]
# x = [-20, 20]
# y = [-40, 40]

## Set 5
# pos = [[0.3742, 0.8429, 0.5], [0.3741, 0.7601, 0.5], [0.3202, 0.7067, 0.5], [1.034, 0.69, 0.5], [1.0339, 0.5697, 0.5], [1.0338, 0.4495, 0.5]]
# normals = [[[1.0, 0.0, 0.0]], [[1.0, 0.0, 0.0]], [[0.0, -1.0, 0.0]], [[-1.0, 0.0, 0.0]], [[-1.0, 0.0, 0.0]], [[-1.0, 0.0, 0.0]]]
# gphase = [0,0,0,0,0,0]
# x = [-110, 250]
# y = [-0, 140]

## Set 6
# pos = [[0.3481, 0.7069, 0.5], [0.2542, 0.707, 0.5], [1.0338, 0.4378, 0.5], [0.8403, 0.0468, 0.5], [0.7012, 0.0469, 0.5]]
# normals = [[[0.0, -1.0, 0.0]], [[0.0, -1.0, 0.0]], [[-1.0, 0.0, 0.0]], [[0.0, 1.0, 0.0]], [[0.0, 1.0, 0.0]]]
# gphase = [0,0,0,1,0,0]
# x = [-0, 110] # plt.xticks(np.arange(-0.2, 0.21, step=0.2), fontname="cmr10", fontsize=fsize)
# y = [-40, 80] # plt.yticks(np.arange(-0.4, 0.41, step=0.2), fontname="cmr10", fontsize=fsize)


## Force distribution =========================================================
qprog = force_distribution.QPForceDistribution()
xb_com = np.array([0.,0.,0.])
Ig_com = np.eye(3)
xa_com = np.array([0.,0.,0.]).reshape((3,1))
wa_com = np.array([0.,0.,0.]).reshape((3,1))
farr = [F_MAX,F_MAX,F_MAX,F_MAX,F_MAX,F_MAX]
p_foot = [np.round(np.array(i)-xb_com,3) for i in pos] 	# CoM to foot in world frame
s_norm = [np.array(i).flatten() for i in normals]

# force_vector, torque_vector = qprog.resolve_force(xa_com, wa_com, p_foot, xb_com, Ig_com, gphase, farr, s_norm)
# print np.round(force_vector.flatten(),3)

## Stability ==================================================================
# mass, dimension, gravity, radius, force limit, robust sphere (not enforced)
if True:
	poly = stabilipy.StabilityPolygon(ROBOT_MASS, 2, -G, 10., 1.0, -1, 0.)
	# poly = stabilipy.StabilityPolygon(ROBOT_MASS, 2, -9.81)
	poly.display_final = True
	contacts = [stabilipy.Contact(SURFACE_FRICTION, np.array(p).T,
	                              stabilipy.utils.normalize(np.array(n).T))
	            for p, n in zip(pos, normals)]
	poly.contacts = contacts

	# modes: best, iteration, precision
	now = time.time()
	poly.compute(stabilipy.Mode.best, epsilon=1e-3, 
										maxIter=20, 
										solver='cdd',
										plot_error=False,
										plot_init=False,
										plot_step=False,
										record_anim=False,
										plot_direction=False,
										plot_final=False)
	# print poly.point_in_hull(xb_com)

## Validation ==================================================================
# Generate random set of points
import random
def Rand(num, x, y): 
	res = [] 
	for j in range(num): 
		res.append([1e-2*random.randint(x[0], x[1]),
					1e-2*random.randint(y[0], y[1]), 0.])
	return res
npoints = 1000#1000 	# no. of elements
# x = [-0, 130]
# y = [-60, 80]
com_points = Rand(npoints, x, y)

ptrue  = []
pfalse = []
pfalse_lp = []
pfalse_sr = []
epsilon = 0.001

for p in range(npoints):
	com = com_points[p]

	p_foot = [np.round(np.array(i)-np.array(com),3) for i in pos] 	# CoM to foot in world frame
	force_vector, torque_vector = qprog.resolve_force(xa_com, wa_com, p_foot, xb_com, Ig_com, gphase, farr, s_norm)
	
	valid, sm = poly.point_in_hull(com)

	if (np.linalg.norm(qprog.error_forces) < epsilon and 
	 	np.linalg.norm(qprog.error_moment) < epsilon):
		if valid:
			ptrue.append(com)
			# print 'In  poly: ', com[0:2]
		else:
			pfalse.append(com)
			pfalse_lp.append(com)
			print 'Out poly: ', com[0:2]
			print np.linalg.norm(qprog.error_forces), np.round(qprog.sum_forces,4)
			print np.linalg.norm(qprog.error_moment), np.round(qprog.sum_moments,4)
			print '----------------------------------------------------'

	elif (np.linalg.norm(qprog.error_forces) > epsilon or 
	 	np.linalg.norm(qprog.error_moment) > epsilon):
		pass
		if not valid:
			ptrue.append(com)
		else:
			pfalse.append(com)
			pfalse_sr.append(com)
			# print 'Out poly: ', com[0:2]
			# print np.linalg.norm(qprog.error_forces), np.round(qprog.sum_forces,4)
			# print np.linalg.norm(qprog.error_moment), np.round(qprog.sum_moments,4)
			# print '----------------------------------------------------'
			
print 'True: ', len(ptrue)
# pfalse_lp: solution exists, but Sr says no
# pfalse_sr: no solution, but Sr says yes
print 'False: ', len(pfalse), len(pfalse_lp), len(pfalse_sr)
# print pfalse

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import cdd
from scipy import spatial
plt.rcParams.update({'font.size': 14})

points = np.array(cdd.Polyhedron(poly.inner).get_generators())[:, 1:]
hull = spatial.ConvexHull(points)

ptrue = np.array(ptrue)
pfalse = np.array(pfalse)
if len(ptrue) > 0:
	plt.plot(ptrue[:,0], ptrue[:,1], 'bo', markersize=5)
if len(pfalse) > 0:
	plt.plot(pfalse[:,0], pfalse[:,1], 'rX', markersize=8)
cent = np.mean(points, 0)
pts = []
for pt in points[hull.simplices]:
    pts.append(pt[0].tolist())
    pts.append(pt[1].tolist())

# Footholds
for i in range(len(pos)):
	plt.plot(pos[i][0], pos[i][1], 'ko', markersize=10)

pts.sort(key=lambda p: np.arctan2(p[1] - cent[1],
                                p[0] - cent[0]))
pts = pts[0::2]  # Deleting duplicates
pts.insert(len(pts), pts[0])
k = 1.
color = 'magenta'
poly = Polygon(k*(np.array(pts)- cent) + cent,
               facecolor=color, alpha=0.2)
poly.set_capstyle('round')
plt.gca().add_patch(poly)

axes = plt.gca()
# axes.set_xlim([min(points[:,0]), max(points[:,0])])
# axes.set_ylim([min(points[:,1]), max(points[:,1])])
# axes.set_xlim([1e-2*x[0],1e-2*x[1]])
# axes.set_ylim([1e-2*y[0],1e-2*y[1]])

fsize = 24
plt.xlabel("x-axis, m", fontname="cmr10", fontsize=fsize)
plt.ylabel("y-axis, m", fontname="cmr10", fontsize=fsize)
# Set the font name for axis tick labels to be Comic Sans
plt.rcParams['axes.unicode_minus']=False
# plt.xticks(np.arange(-0.2, 0.21, step=0.2), fontname="cmr10", fontsize=fsize)
# plt.yticks(np.arange(-0.4, 0.41, step=0.2), fontname="cmr10", fontsize=fsize)

fig = plt.matplotlib.pyplot.gcf()
# fig.set_size_inches(6.3, 6.7)
fig.set_tight_layout(True)

plt.grid(True);
plt.show()
# plt.savefig("ww_simple", dpi=200, format='png')