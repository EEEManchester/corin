#!/usr/bin/env python

## Function for manipulating the body pose of the hexapod with legs
## fixed to the spot. Initial stance is the nominal stance
import rospy
import numpy as np
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from robotis_controller_msgs.msg import SyncWriteItem
from robotis_controller_msgs.msg import SyncWriteMulti
# from robotis_controller_msgs import WriteControlTable

class CorinManager:
	def __init__(self):
		rospy.init_node('read_try') 		#Initialises node
		self.freq	= 1 					# frequency
		self.rate 	= rospy.Rate(self.freq)	# control rate

		self.check_joints()

	def check_joints(self):
		#######################################
		## initialise publishers/subscribers ##
		#######################################
		
		robot_file = rospy.get_param("robot_file_path")

		try:
			file = open(robot_file, 'r') 
		except:
			print 'File does not exist!'
			return

		start_joint = False
		while (start_joint != True):
			s = file.readline()
			for char in s:
				s = s.replace('[','')
				s = s.replace(']','')
			s = s.strip()
			
			print s
			if (s=="port info"):
				start_joint = True
		print 'loop ended'

		joint_list  = {}
		joint_count = 0
		end_joints  = False

		while (end_joints != True):
			s = file.readline()
			# check for termination
			for char in s:
				s = s.replace('[','')
				s = s.replace(']','')
				s = s.replace('|','')
			s = s.strip()
			
			sline = s.split()
			if sline:
				if "/dev/" in sline[0]:
					joint_list[joint_count] = sline[2]
					joint_count += 1
			
			if (s=="device info"):
				end_joints = True
		print 'loop ended'
		print joint_list

		file.close()
		
	

if __name__ == "__main__":

	manager = CorinManager()
	tdelay = 0.1

