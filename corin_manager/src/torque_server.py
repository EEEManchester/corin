#!/usr/bin/env python

## service server for torque enable/disable
import rospy
import numpy as np
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from robotis_controller_msgs.msg import SyncWriteItem
from robotis_controller_msgs.msg import SyncWriteMulti
from robotis_controller_msgs.srv import *

class CorinManager:
	def __init__(self):
		rospy.init_node('torque_server') 		# Initialises node
		self.freq	= 1 						# frequency
		self.rate 	= rospy.Rate(self.freq)		# control rate
		self.joint  = self._read_file() 		# read list of joints

		self._start()

	def _read_file(self):
		## read joints from .robot file
		robot_file = rospy.get_param("robot_file_path")

		try:
			file = open(robot_file, 'r') 
		except:
			rospy.logwarn('File does not exist!')
			return

		## skip lines until it reaches start of 'port info'
		start_joint = False
		while (start_joint != True):
			s = file.readline()
			for char in s:
				s = s.replace('[','')
				s = s.replace(']','')
			s = s.strip()
			
			if (s=="port info"):
				start_joint = True

		joint_list  = {}
		joint_count = 0
		end_joints  = False

		## read the joint names from the file
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

		file.close() 		# be nice by closing file
		return joint_list 	# return dictionary of names

	def _start(self):
		#######################################
		## initialise publishers/subscribers ##
		#######################################

		##***************** PUBLISHERS ***************##
		self.control_mode_	= rospy.Publisher('/robotis/set_control_mode', String, queue_size=1)
		self.mm_joint_pub_	= rospy.Publisher('/robotis/set_joint_states', JointState, queue_size=1)

		self.sync_itm_pub_  = rospy.Publisher('/robotis/sync_write_item', SyncWriteItem, queue_size=1)
		self.sync_mlt_pub_  = rospy.Publisher('/robotis/sync_write_multi', SyncWriteMulti, queue_size=1)

		##***************** SUBSCRIBERS ***************##
		# self.joint_sub_	= rospy.Subscriber('/robotis/present_joint_states', JointState, self.joint_state_callback, queue_size=5)
		
		##***************** SERVICES ***************##
		# start only if non-empty
		if (self.joint):
			self.torque_serv_ 	= rospy.Service('dxl_server/torque_enable', SetTorqueEnable, self.set_torque)
			rospy.sleep(0.5)
			rospy.loginfo("Torque Server Initiated")
		else:
			rospy.logwarn("No Joints Found, TERMINATING")
			rospy.signal_shutdown("No Joints Found, TERMINATING")

	def set_torque(self, req):
		torque_enable = 0;
		
		if (req.enable == True):
			torque_enable = 1
			rospy.loginfo("Enabling Torque...")
		elif (req.enable == False):
			torque_enable = 0
			rospy.loginfo("Disabling Torque...")
		else:
			rospy.logwarn("Invalid Command!")

		dqp = SyncWriteItem()
		dqp.item_name = str("torque_enable") 	# Address to start writing to
		
		# set joint name
		for i in self.joint:
			dqp.joint_name.append(self.joint[i])
		
		# set torque value 
		for i in range(0,18):
			dqp.value.append(int(torque_enable))	# value to append

		manager.sync_itm_pub_.publish(dqp) 			# publish topic

		return SetTorqueEnableResponse(True)

if __name__ == "__main__":

	if len(sys.argv) < 2:
		sleep_time = 0.0
	else:
		print 'Delaying by ', sys.argv[1], ' s'
		sleep_time = float(sys.argv[1])

	rospy.sleep(sleep_time)

	manager = CorinManager()
	tdelay = 0.1

	rospy.spin()

	
	