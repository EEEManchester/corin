#!/usr/bin/env python

## Function for manipulating the body pose of the hexapod with legs
## fixed to the spot. Initial stance is the nominal stance
import rospy

import numpy as np
import math

from sensor_msgs.msg import JointState	

def joint_state_callback(msg):
	# self.Robot.qc = msg
	print len(msg.name)

if __name__ == "__main__":
	rospy.init_node('main_controller') 		#Initialises node
	rate  = rospy.Rate(1.0/0.05)			# frequency

	joint_sub_ = rospy.Subscriber('/corin/joint_states', JointState, joint_state_callback, queue_size=5)

	rospy.spin()