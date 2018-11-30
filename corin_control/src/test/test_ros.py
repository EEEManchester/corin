#!/usr/bin/env python

## Function for manipulating the body pose of the hexapod with legs
## fixed to the spot. Initial stance is the nominal stance

import rospy
import numpy as np
import math

from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import matlab.engine

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

def foothold_list_to_marker(footholds, stamp=None, frame_id=None):
	""" Converts custom marker list to nav_msgs/Path """

	## Define Variables ##
	mark = Marker()
	
	if stamp is not None:
		mark.header.stamp = stamp
	if frame_id is not None:
		mark.header.frame_id = frame_id
	
	mark.type = 11
	mark.id = 1

	mark.scale.x = 1
	mark.scale.y = 1
	mark.scale.z = 1
	mark.color.r = 1.0;
	mark.color.g = 0.0;
	mark.color.b = 0.0;
	mark.color.a = 0.5; # transparency level
	# mark.pose.position.x = 0.5
	# mark.pose.position.y = 0.5
	# mark.pose.position.z = 0
	mark.pose.orientation.x = 0.0;
	mark.pose.orientation.y = 0.0;
	mark.pose.orientation.z = 0.0;
	mark.pose.orientation.w = 1.0;
	# mark.lifetime = rospy.Time(2)
	## assumes that footholds array are of the same size
	for j in range(0,6):
		point3d = Point()
		if (j < len(footholds)):		
			point3d.x = footholds[j][0]
			point3d.y = footholds[j][1]
			point3d.z = footholds[j][2]
		else:
			point3d.x = footholds[j-len(footholds)][0]
			point3d.y = footholds[j-len(footholds)][1]
			point3d.z = footholds[j-len(footholds)][2]

		mark.points.append(point3d)
		
	return mark

if __name__ == "__main__":
	rospy.init_node('main_controller') 		#Initialises node
	rate  = rospy.Rate(1.0/0.05)			# frequency
	
	print 'Starting matlab engine.....'
	eng = matlab.engine.start_matlab()
	print 'Engine started!'

	sp_pub_   = rospy.Publisher('corin/support', Marker, queue_size=1)

	rospy.sleep(0.2)
	footholds = [(0.5,0,0),(0.7,0,0),(0.5,0.7,0),(0.4,0.5,0)]

	# options = eng.optimoptions('quadprog','Algorithm','interior-point-convex','Display','none',
 #                            'MaxIterations', 200, 
 #                            'StepTolerance', 1e-12, 
 #                            'ConstraintTolerance', 1e-7, 
 #                            'OptimalityTolerance', 1e-8);
 	# a = matlab.double([1,4;9,16])
 	# b = matlab.double([2,0;1,1])
 	ret = eng.test_matlab(1.0,5.0)
	print ret
	# eng.H = [1 -1; -1 2]; 
	# eng.f = [-2; -6];
	# eng.A = [1 1; -1 2; 2 1];
	# eng.b = [2; 2; 3];

    # [leg_force,fval,exitflag,output,lamb] = 
    # eng.quadprog(eng.H, eng.f, eng.A, eng.b);
    # quadprog(eng.H,q,inq_C,inq_D,[],[],[],[],[],options);


	# while not rospy.is_shutdown():

	# 	# sp_pub_.publish(foothold_list_to_marker(footholds, rospy.Time.now(), 'world'))
	# 	rospy.sleep(2)
	# joint_sub_ = rospy.Subscriber('/corin/joint_states', JointState, joint_state_callback, queue_size=5)
	#
	# rospy.spin()
	sp_state = JointState()
	
	eng.quit()
