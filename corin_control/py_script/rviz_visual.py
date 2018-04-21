#!/usr/bin/env python

import rospy
from library import *
from motion_planning import *

class RvizVisualise:
	def __init__(self):
		self.map_pub_  = rospy.Publisher(ROBOT_NS + '/point_cloud', PointCloud2, queue_size=1)
		self.path_pub_ = rospy.Publisher(ROBOT_NS + '/path', Path, queue_size=1)
		self.mark_pub_ = rospy.Publisher(ROBOT_NS + '/footholds', MarkerArray, queue_size=1)	# marker array
		self.robot_broadcaster = tf.TransformBroadcaster()	# Transform for robot pose		

		rospy.sleep(0.5)
		self.clear_visualisation()

	def clear_visualisation(self):
		## Clear visualization components ##
		clear_marker = MarkerArray()
		mark = Marker()
		mark.action = 3
		clear_marker.markers.append(mark)
		clear_path = Path()
		clear_path.header.frame_id = 'world'
		self.mark_pub_.publish(clear_marker)
		self.path_pub_.publish(clear_path)
