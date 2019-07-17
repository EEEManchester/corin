#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np
from numpy import array
import time
from jet_leg.plotting_tools import Plotter
import random
from jet_leg.math_tools import Math
from jet_leg.computational_dynamics import ComputationalDynamics
from jet_leg.iterative_projection_parameters import IterativeProjectionParameters
# from jet_leg.sequential_iterative_projection import SequentialIterativeProjection

import matplotlib.pyplot as plt
from jet_leg.arrow3D import Arrow3D

plt.close('all')
math = Math()
# number of contacts
#nc = 3
# number of generators, i.e. rays used to linearize the friction cone
ng = 4

# ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION

constraint_mode_IP = ['ONLY_FRICTION',
                      'ONLY_FRICTION',
                      'ONLY_FRICTION',
                      'ONLY_FRICTION',
					  'ONLY_FRICTION',
                      'ONLY_FRICTION']
useVariableJacobian = False
# number of decision variables of the problem
#n = nc*6
comWF = np.array([0.0, 0.0, 0.0])

## contact positions
LF_foot = np.array([ 0.25,  0.25, 0.])
LM_foot = np.array([ 0.00,  0.30, 0.])
LR_foot = np.array([-0.25,  0.25, 0.])
RF_foot = np.array([ 0.25, -0.25, 0.])
RM_foot = np.array([ 0.00, -0.30, 0.])
RR_foot = np.array([-0.25, -0.25, 0.])
contacts = np.vstack((LF_foot,LM_foot,LR_foot,RF_foot,RM_foot,RR_foot))

''' parameters to be tuned'''
g = 9.81
trunk_mass = 5.
mu = 1.0

stanceFeet = [1,1,1,1,1,1]

axisZ= array([[0.0], [0.0], [1.0]])

n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n5 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n6 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
normals = np.vstack([n1, n2, n3, n4, n5, n6])

LF_tau_lim = [50.0, 100.0, 100.0]
LM_tau_lim = [50.0, 100.0, 100.0]
LR_tau_lim = [50.0, 100.0, 100.0]
RF_tau_lim = [50.0, 100.0, 100.0]
RM_tau_lim = [50.0, 100.0, 100.0]
RR_tau_lim = [50.0, 100.0, 100.0]
torque_limits = np.array([LF_tau_lim, LM_tau_lim, LR_tau_lim, RF_tau_lim, RM_tau_lim, RR_tau_lim])

extForceW = np.array([0.0,0.0, 0.0])

nc = np.sum(stanceFeet)
# plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15)
# for j in range(0,nc):
#     ax.scatter(contacts[j,0], contacts[j,1], contacts[j,2],c='b',s=100)
#     a = Arrow3D([contacts[j,0], contacts[j,0]+normals[j,0]/10], [contacts[j,1], contacts[j,1]+normals[j,1]/10],[contacts[j,2], contacts[j,2]+normals[j,2]/10], mutation_scale=20, lw=3, arrowstyle="-|>", color="r")
#     ax.add_artist(a)

comp_dyn = ComputationalDynamics()
params = IterativeProjectionParameters()
params.setContactsPosWF(contacts)
params.setCoMPosWF(comWF)
params.setTorqueLims(torque_limits)
params.setActiveContacts(stanceFeet)
params.setConstraintModes(constraint_mode_IP)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)
params.setTotalMass(trunk_mass + extForceW[2]/9.81)
params.externalForceWF = extForceW

''' compute iterative projection '''
# now = time.time()
IP_points, actuation_polygons, computation_time = comp_dyn.iterative_projection_bretl(params)
# end = time.time()
print 'Computation time: ', computation_time

# sq_iter = SequentialIterativeProjection()
# sq_iter.compute_polygon_variable_constraint('ONLY_FRICTION', comWF, contacts)

# print IP_points
# print actuation_polygons
# if IP_points is not False:
#     #print 'actuation polygons', actuation_polygons
#     #print IP_points
#     ''' plotting Iterative Projection points '''
#     plotter = Plotter()
#     scaling_factor = 1000
      
#     ''' 2D figure '''
#     plt.figure()
#     h1 = plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15, label='feet')
#     h2 = plotter.plot_polygon(np.transpose(IP_points), '-b','Iterative Projection')
    
#     plt.grid()
#     plt.xlabel("X [m]")
#     plt.ylabel("Y [m]")
#     plt.legend()
#     plt.show()

