# -*- coding: utf-8 -*-
"""
Created on Wed Dec 19 09:44:02 2018

@author: rorsolino
"""

import copy
import numpy as np
import os

import rospy as ros
import sys
import time
import threading

from copy import deepcopy

from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Vector3, Wrench
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Point
from dls_msgs.msg import SimpleDoubleArray, StringDoubleArray, Polygon3D, LegsPolygons
from dwl_msgs.msg import WholeBodyState, WholeBodyTrajectory, JointState, ContactState, BaseState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Header
from std_srvs.srv import Empty
from termcolor import colored

from context import jet_leg 
from jet_leg.computational_dynamics import ComputationalDynamics
from jet_leg.computational_geometry import ComputationalGeometry
from jet_leg.math_tools import Math


stderr = sys.stderr
sys.stderr = open(os.devnull, 'w')
sys.stderr = stderr


class FootholdPlanningInterface:
    def __init__(self):      
        
        self.com_position_to_validateW = [0., 0., 0.] #used only for foothold planning
                
        self.orientation0 = [0., 0., 0.]
        self.orientation1 = [0., 0., 0.]
        self.orientation2 = [0., 0., 0.]
        self.orientation3 = [0., 0., 0.]
        self.orientation4 = [0., 0., 0.]
        self.orientationOptions = np.array([self.orientation0,
                                     self.orientation1,
                                     self.orientation2,
                                     self.orientation3,
                                     self.orientation4])
                                     
        #foothold planning 
        self.footOption0 = [0., 0., 0.]
        self.footOption1 = [0., 0., 0.]
        self.footOption2 = [0., 0., 0.]
        self.footOption3 = [0., 0., 0.]
        self.footOption4 = [0., 0., 0.]
        self.footOption5 = [0., 0., 0.]
        self.footOption6 = [0., 0., 0.]
        self.footOption7 = [0., 0., 0.]
        self.footOption8 = [0., 0., 0.]
        self.footOption9 = [0., 0., 0.]
        self.footOption10 = [0., 0., 0.]
        self.footOption11 = [0., 0., 0.]
        self.footOption12 = [0., 0., 0.]
        self.footOption13 = [0., 0., 0.]
        self.footOption14 = [0., 0., 0.]
        self.footOption15 = [0., 0., 0.]
        self.footOption16 = [0., 0., 0.]
        self.footOption17 = [0., 0., 0.]
        self.footOptions = np.array([self.footOption0,
                                     self.footOption1,
                                     self.footOption2,
                                     self.footOption3,
                                     self.footOption4,
                                     self.footOption5,
                                     self.footOption6,
                                     self.footOption7,
                                     self.footOption8,
                                     self.footOption9,
                                     self.footOption10,
                                     self.footOption11,
                                     self.footOption12,
                                     self.footOption13,
                                     self.footOption14,
                                     self.footOption15,
                                     self.footOption16,
                                     self.footOption17
                                     ])
                                    
        self.numberOfFeetOptions = 0
        self.sample_contacts = np.zeros((4,3))
   
        self.minRadius = 0.
        self.optimization_started = False
        #outputs        
        self.option_index = 0
        self.ack_optimization_done = False
        self.TOL = 0.001
        
    def getParamsFromRosDebugTopic(self, received_data):
        
        num_of_elements = np.size(received_data.data)
#        print 'number of elements: ', num_of_elements
        for j in range(0, num_of_elements):
            if str(received_data.name[j]) == str("optimization_started"):
               self.optimization_started = bool(received_data.data[j])

        if self.optimization_started is True:
            print 'Optimization started flag is now true. Reading new data...............'
            for j in range(0, num_of_elements):
                if str(received_data.name[j]) == str("numberOfFootholdOptions"):
                    self.numberOfFeetOptions = received_data.data[j]

            for j in range(0,num_of_elements):
#                print j, received_data.name[j], str(received_data.name[j]), str("footPosLFx")

                        #foothold planning
                if str(received_data.name[j]) == str("com_position_to_validateWx"):
                    self.com_position_to_validateW[0] = received_data.data[j]
                if str(received_data.name[j]) == str("com_position_to_validateWy"):
                    self.com_position_to_validateW[1] = received_data.data[j]
                if str(received_data.name[j]) == str("com_position_to_validateWz"):
                    self.com_position_to_validateW[2] = received_data.data[j]

                for k in range(0, int(self.numberOfFeetOptions)):
                    if str(received_data.name[j]) == str("foothold_option"+str(k)+"x"):
                        self.footOptions[k,0] = received_data.data[j]
                    if str(received_data.name[j]) == str("foothold_option"+str(k)+"y"):
                        self.footOptions[k,1] = received_data.data[j]
                    if str(received_data.name[j]) == str("foothold_option"+str(k)+"z"):
                        self.footOptions[k,2] = received_data.data[j]

                #if str(received_data.name[j]) == str("numberOfFootholdOptions"):
                #        self.numberOfFeetOptions = received_data.data[j]

#                    print self.footOptions

                if str(received_data.name[j]) == str("minRadius"):
                    self.minRadius = received_data.data[j]


                