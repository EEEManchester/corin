#!/usr/bin/env python

import rospy

from dynamixel_controllers.srv import *
from sensor_msgs.msg import JointState

qstate = JointState()

def JointStateCallback(msg):
	qstate.name = msg.name
	qstate.position = msg.position
	qstate.velocity = msg.velocity

def JointState_server(req):

	return GetJointStateResponse(qstate)

if __name__ == "__main__":
	rospy.init_node('dxl_joint_state_server')

	joint_sub_  = rospy.Subscriber('/corin/joint_state/dynamixel_port', JointState, JointStateCallback)
	joint_serv_ = rospy.Service('/corin/get_jointState', GetJointState, JointState_server)
	print "Ready to return joint states."

	rospy.spin()