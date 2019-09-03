#!/usr/bin/env python
""" Visualize the grip map in rviz """

import os.path
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rospy

from numpy_to_rosmsg import *
from grid_map import *

if __name__ == "__main__":

	rospy.init_node('grid_map')
	namespace = 'corin'
	map_pub_  = rospy.Publisher(namespace + '/point_cloud', PointCloud2, queue_size=1)

	robot_broadcaster = tf.TransformBroadcaster()	# Transform for robot pose
	# rospy.sleep(0.5)

	## Create map and plan path
	GridPlanner = GridMap('wall_hole_demo')
	# GridPlanner = GridMap('wall_transition')
	
	## Convert to ROS point cloud and visualise in RVIZ
	map_arr   = point_cloud_array_mapping(GridPlanner.graph_to_nparray())
	cloud_msg = array_to_pointcloud2(map_arr, rospy.Time.now(), "world")

	while (not rospy.is_shutdown()):
		map_pub_.publish(cloud_msg)
		
		rospy.sleep(2)