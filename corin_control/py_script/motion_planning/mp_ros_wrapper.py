#!/usr/bin/env python

import rospy

from numpy_to_rosmsg import *
from grid_planner import * 	# grid based planner

if __name__ == "__main__":

	rospy.init_node('grid_motion_planner') 		#Initialises node
	namespace = 'corin'
	map_pub_  = rospy.Publisher(namespace + '/point_cloud', PointCloud2, queue_size=1)
	path_pub_ = rospy.Publisher(namespace + '/path', Path, queue_size=1)
	mark_pub_ = rospy.Publisher(namespace + '/footholds', MarkerArray, queue_size=1)	# marker array
	joint_pub_= rospy.Publisher(namespace + '/joint_states', JointState, queue_size=1)

	robot_broadcaster = tf.TransformBroadcaster()	# Transform for robot pose
	# rospy.sleep(0.5)

	## Create map and plan path
	GridPlanner = GridPlanner('wall_demo')
	# GridPlanner = GridPlanner((1.27,5.48)) 	# map size, [x,y] (m,m)
	# grid_path 	= GridPlanner.find_path((15, 10),(10,150))	# define start and end points
	# pp.graph_representation(False, grid_path)

	# GridPlanner = GridPlanner((1.27,5.48)) 	# map size, [x,y] (m,m)
	# GridPlanner.advance_capable = False 		# disable advanced motions
	# grid_path = GridPlanner.find_path((15, 10),(10,150))	# define start and end points
	# footholds = GridPlanner.find_foothold(grid_path)
	
	# pp.graph_representation(True,grid_path)

	# plt.show()

	## Convert to ROS point cloud and visualise in RVIZ
	map_arr  = point_cloud_array_mapping(GridPlanner.graph_to_nparray())
	cloud_msg= array_to_pointcloud2(map_arr, rospy.Time.now(), "world")

	# path_arr = GridPlanner.path_to_nparray(grid_path) 	# raw path
	# path_arr = GridPlanner.post_process_path(grid_path)		# post-processed path
	# path_msg = array_to_path(path_arr, rospy.Time.now(), "world")
	# mark_msg = array_to_marker_array(footholds, rospy.Time.now(), "world")
	
	

	# path_length = len(path_arr.X.t)
	# path_count  = 0
	while (not rospy.is_shutdown()):
		## Update robot's pose
		# px = path_arr.X.xp[path_count][0]
		# py = path_arr.X.xp[path_count][1]
		# pz = path_arr.X.xp[path_count][2]

		# rx = path_arr.W.xp[path_count][0]*(-1.)
		# ry = path_arr.W.xp[path_count][1]
		# rz = path_arr.W.xp[path_count][2] + np.pi/2

		# # transform from world frame to robot's base_link
		# robot_broadcaster.sendTransform( (px,py,pz), tf.transformations.quaternion_from_euler(rx, ry, rz), 
		# 									rospy.Time.now(), "trunk", "world") ;

		# ## Update joint angles
		# q = [None]*18
		# for i in range(0,6):
		# 	q[i*3+0] = 0
		# 	q[i*3+1] = 0.524
		# 	q[i*3+2] = -2.09
		# q_msg = array_to_joint_states(q, rospy.Time.now(), "")

		map_pub_.publish(cloud_msg)
		# path_pub_.publish(path_msg)
		# mark_pub_.publish(mark_msg)
		# joint_pub_.publish(q_msg)

		# rospy.sleep(0.01)
		
		# path_count = path_count+1 if (path_count < path_length-1) else 0

		rospy.sleep(2)