#!/usr/bin/env python

""" Visualisation topics for rviz 
	and functions for mundane task """

import rospy
import tf
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import WrenchStamped

from grid_planner_core.numpy_to_rosmsg import *
from corin_control.constant import *

LEG_FORCE_NAME = {}
LEG_FORCE_NAME[0] = 'LF_foot_force'
LEG_FORCE_NAME[1] = 'LM_foot_force'
LEG_FORCE_NAME[2] = 'LR_foot_force'
LEG_FORCE_NAME[3] = 'RF_foot_force'
LEG_FORCE_NAME[4] = 'RM_foot_force'
LEG_FORCE_NAME[5] = 'RR_foot_force'

LEG_FORCE_FRAME = {}
LEG_FORCE_FRAME[0] = 'lf_foot'
LEG_FORCE_FRAME[1] = 'lm_foot'
LEG_FORCE_FRAME[2] = 'lr_foot'
LEG_FORCE_FRAME[3] = 'rf_foot'
LEG_FORCE_FRAME[4] = 'rm_foot'
LEG_FORCE_FRAME[5] = 'rr_foot'

class RvizVisualise:
	def __init__(self):
		self.fr_fix = "world"
		self.fr_robot = "trunk"

		##***************** PUBLISHERS ***************##
		self.map_pub_  = rospy.Publisher(ROBOT_NS + '/point_cloud', PointCloud2, queue_size=1)
		self.path_pub_ = rospy.Publisher(ROBOT_NS + '/path', Path, queue_size=1)
		self.mark_pub_ = rospy.Publisher(ROBOT_NS + '/footholds', MarkerArray, queue_size=1)	# marker array
		self.cob_pub_  = rospy.Publisher(ROBOT_NS + '/cob', MarkerArray, queue_size=1)

		self.wrench_pub_ = {}
		for j in range(0,6):
			self.wrench_pub_[j] = rospy.Publisher(ROBOT_NS + '/' + LEG_FORCE_NAME[j], WrenchStamped, queue_size=1)

		self.robot_broadcaster = tf.TransformBroadcaster()	# Transform for robot pose		

		rospy.sleep(0.5)
		self.clear_visualisation()

	def clear_visualisation(self):
		""" Clear visualization components """
		
		clear_marker = MarkerArray()
		mark = Marker()
		mark.action = 3
		clear_marker.markers.append(mark)
		clear_path = Path()
		clear_path.header.frame_id = 'world'
		self.mark_pub_.publish(clear_marker)
		self.cob_pub_.publish(clear_marker)
		self.path_pub_.publish(clear_path)

	def publish_robot(self, qb):
		""" Robot transform publisher """
		# print 'pub: ', np.round(qb[:3].flatten(),4)
		quat = tf.transformations.quaternion_from_euler(qb[3].copy(), qb[4].copy(), qb[5].copy())
		self.robot_broadcaster.sendTransform( (qb[0],qb[1],qb[2]), quat, rospy.Time.now(), self.fr_robot, self.fr_fix);

	def publish_path(self, xb, xoff=None):
		""" Trajectory path publisher 
			Input: 	1) xb: trajectory Re^(6xn)
					2) xoff: offset Re^6 		"""

		self.path_pub_.publish(array_to_path(xb, rospy.Time.now(), self.fr_fix, xoff))

	def publish_footholds(self, footholds):
		""" Foothold array publisher 
			Input:  footholds: list of footholds """
		
		self.mark_pub_.publish(foothold_list_to_marker_array(footholds, rospy.Time.now(), self.fr_fix))

	def publish_cob(self, cob):
		""" Centre of Base array publisher 
			Input:  cob: list of cob along trajectory """
		
		self.cob_pub_.publish(list_to_marker_array(cob, rospy.Time.now(), self.fr_fix))

	def publish_foot_force(self, cstate, forces):
		""" Publishes foot force expressed in 
				world frame BUT located in foot frame 
				Input: 1) cstate: Re^6, list of leg contact state
							 2) forces: Re^3c, list of contact forces 	"""

		counter = 0
		for j in range(0,6):
			if (cstate[j] == 0):
				data = WrenchStamped()
				data.header.frame_id = LEG_FORCE_FRAME[j]
				data.wrench.force.x = forces[counter]
				data.wrench.force.y = forces[counter+1]
				data.wrench.force.z = forces[counter+2]
				self.wrench_pub_[j].publish(data)
				# if j==1:
				# 	print np.round(forces[counter:counter+3].flatten(),3)
				counter += 3
