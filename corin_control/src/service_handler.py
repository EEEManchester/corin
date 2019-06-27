#!/usr/bin/env python

""" Handler for ROS services """ 
__version__ = '1.0'
__author__  = 'Wei Cheah'

import rospy

class ServiceHandler:
	def __init__(self, srv_name, srv_type):
		self.available = False
		self.srv_name  = srv_name
		self.srv_type  = srv_type
		self.srv_proxy = None

		self.initialise_service(self.srv_name, self.srv_type)

	def initialise_service(self, srv_name, srv_type):
		""" Check if service valid """

		try:
			rospy.wait_for_service(srv_name, 0.5)
			self.srv_proxy = rospy.ServiceProxy(srv_name, srv_type)
			self.available = True
		except rospy.ROSException, e:
			print "Service did not process request: %s"%str(e)

	def call(self, *argv):

		if (self.available):
			try:
				narg = len(argv)
				if (narg == 1):
					output = self.srv_proxy(argv[0])
				elif (narg == 2):
					output = self.srv_proxy(argv[0],argv[1])
				elif (narg == 3):
					output = self.srv_proxy(argv[0],argv[1],argv[2])
				elif (narg == 4):
					output = self.srv_proxy(argv[0],argv[1],argv[2],argv[3])
				return output
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
				raise Exception 