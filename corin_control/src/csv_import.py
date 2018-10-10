#!/usr/bin/env python

""" Snapshot of motion plan imported from csv """

import rospy
from corin_control import *			# library modules to include
from rviz_visual import *
from corin_msgs.msg import LoggingState

import csv
import numpy as np

class CSVimport:
	def __init__(self):
		rospy.init_node('csv_import') 		# Initialises node
		self.rate 	  = rospy.Rate(CTR_RATE)	# Controller rate

		self.CoB = []
		self.footholds = [None]*6
		self.snorm = np.zeros((18,1))
		self.Visualizer = RvizVisualise()

		self.Robot = robot_class.RobotState()
		self.ForceDist = QPForceDistribution()

		self.joint_states = []

		self.__initialise_variables__()
		self.__initialise_topics__()
		
	def __initialise_topics__(self):
		self.joint_pub_ = rospy.Publisher(ROBOT_NS + '/joint_states', JointState, queue_size=1)
		self.log_pub_ = rospy.Publisher('/corin/log_states', LoggingState, queue_size=1)	# LOGGING publisher

	def __initialise_variables__(self):
		for j in range(0,6):
			self.footholds[j] = MarkerList()

	def publish(self, CoB, qp):
		self.Visualizer.publish_robot(CoB)

		dqp = JointState()
		dqp.header.stamp = rospy.Time.now()
		for n in range(0,18): 	# loop for each joint
			dqp.name.append(str(JOINT_NAME[n])) # joint name
			dqp.position.append(qp[n])			# joint angle
		self.joint_pub_.publish(dqp)

		# rospy.sleep(0.4)

	def load_file(self, filename):
		print 'Reading file: ', filename
		
		with open(filename, 'rb') as csvfile:
			motion_data = csv.reader(csvfile, delimiter=',')
			counter = 0
			for row in motion_data:
				# Append for CoB
				self.CoB.append( np.asarray(map(lambda x: float(x), row[0:6])) )
				# Append for footholds
				# self.footholds.append( map(lambda x: float(x), row[6:25]) )
				for j in range(0,6):
					self.footholds[j].t.append(counter)
					# self.footholds[j].xp.append( np.asarray(map(lambda x: float(x), row[6+j*3:6+(j*3)+3])) )
					mod_foothold = np.asarray(map(lambda x: float(x), row[6+j*3:6+(j*3)+3]))# + np.array([0.,0.,0.05])
					self.footholds[j].xp.append(mod_foothold)
				
				self.joint_states.append( np.asarray(map(lambda x: float(x), row[24:42])) )
				
				counter += 1
			
			# print self.CoB
			# print '======================'
			# print (self.footholds[0].xp)
		try:
			if not self.joint_states[0]:
				self.joint_states = []
		except:
			pass

	def visualise_motion_plan(self):

		self.Visualizer.publish_robot(self.CoB[0])
		self.Visualizer.publish_path(self.CoB)
		self.Visualizer.publish_footholds(self.footholds)

		if self.joint_states:
			for xi in range(0,3):
				self.publish(self.CoB[0], self.joint_states[0])
				rospy.sleep(0.5)

	def cycle_states(self):
		""" Publishes joint states directly """

		i = 0
		
		while (i != len(self.CoB) and not rospy.is_shutdown()):
			for xi in range(0,2):
				self.publish(self.CoB[i], self.joint_states[i])
			rospy.sleep(0.1)
				
			i += 1
			# raw_input('cont')
			rospy.sleep(0.5)

if __name__ == "__main__":

	csv_import = CSVimport()

	# rviz.load_file('chimney_new_01.csv')
	# rviz.load_file('wall_highRes_convex.csv')
	csv_import.load_file('chimney_highRes_opt_working.csv')
	# rviz.load_file('wall_medRes_convex.csv')
	
	csv_import.visualise_motion_plan()
	raw_input('Start motion!')
	for i in range(0,5):
		# rviz.cycle_snapshot()
		csv_import.cycle_states()