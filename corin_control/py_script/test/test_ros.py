#!/usr/bin/env python

## Function for manipulating the body pose of the hexapod with legs
## fixed to the spot. Initial stance is the nominal stance
import sys, os
sys.path.insert(0, '/home/wilson/catkin_ws/src/mcorin/mcorin_control/py_script/library')

import rospy
from constant import *
import numpy as np
import math

from sensor_msgs.msg import JointState

ROBOT_STATE = {}
ROBOT_STATE[0] ='x'
ROBOT_STATE[1] ='y'
ROBOT_STATE[2] ='z'
ROBOT_STATE[3] ='r'
ROBOT_STATE[4] ='p'
ROBOT_STATE[5] ='y'

def joint_state_callback(msg):
	# self.Robot.qc = msg
	print len(msg.name)

if __name__ == "__main__":
	rospy.init_node('main_controller') 		#Initialises node
	rate  = rospy.Rate(1.0/0.05)			# frequency

	# joint_sub_ = rospy.Subscriber('/corin/joint_states', JointState, joint_state_callback, queue_size=5)
	#
	# rospy.spin()
	sp_state = JointState()
	

	print sp_state
