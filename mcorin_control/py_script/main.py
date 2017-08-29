#!/usr/bin/env python

## Function for manipulating the body pose of the hexapod with legs
## fixed to the spot. Initial stance is the nominal stance
import rospy
import sys
sys.path.insert(0, '/home/wilson/catkin_ws/src/mcorin/mcorin_control/py_script/library')
# sys.dont_write_bytecode = True

import numpy as np
import math

import controller_walking as Control_Framework
import path_generator as Pathgenerator
from constant import *
			

if __name__ == "__main__":

	manager = Control_Framework.CorinManager(True)
		
	while not rospy.is_shutdown():
		manager.action_interface()