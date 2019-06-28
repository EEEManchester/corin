#!/usr/bin/env python

import os.path
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import yaml

import rospy

from grid_planner_core.grid_map import GridMap 			# grid map class
from grid_planner_core.path_planner import PathPlanner 	# path planning class
from grid_planner_core.numpy_to_rosmsg import *

from corin_control.robot_class import RobotState
from corin_control.constant import *

from grid_planner_msgs.msg import GridMap as RosGridMapMsg
from grid_planner_msgs.srv import *#PlanPath
from std_msgs.msg import *
# from numpy_to_rosmsg import * 	# convert numpy to ros messages

class GridMapRos:
	def __init__(self,map_name):
		rospy.init_node('grid_map_planner')

		self.namespace = 'GridMap'
		self.GridMap = GridMap(map_name)
		self.Planner = PathPlanner(self.GridMap)
		self.Robot 	 = RobotState()
		self.point_cloud = PointCloud2()

		self.__hotstart_initialise__()

		# self.Planner.plot_primitive_graph()

	def __hotstart_initialise__(self):
		self.__initialise_topics__()
		self.__initialise_services__()
		self.__initialise_parameters__()
		self.convert_graph_to_pointcloud()

	def __initialise_topics__(self):
		""" Initializes ROS publishers and subscribers """

		self.map_pub_  = rospy.Publisher(self.namespace + '/point_cloud', PointCloud2, queue_size=1)

	def __initialise_services__(self):
		""" setup service call """

		self.plan_path = rospy.Service(self.namespace + '/query_map', PlanPath, self.serv_path_planner)
		self.set_grid_map  = rospy.Service(self.namespace + '/set_grid_map', GenericString, self.serv_set_grid_map)
		self.grid_map  = rospy.Service(self.namespace + '/grid_map', GetGridMap, self.serv_get_grid_map)
		print "Ready to plan path."

	def __initialise_parameters__(self):
		""" initialise ROS parameters """

		rospy.set_param(self.namespace + '/map_name', self.GridMap.map_name)
		rospy.set_param(self.namespace + '/resolution', self.GridMap.resolution)

	def initialise_robot_state(self, ps, pf):
		## Set robot to starting position in default configuration
		self.Robot.P6c.world_X_base = np.array([ps[0]*self.GridMap.resolution,
												ps[1]*self.GridMap.resolution,
												BODY_HEIGHT,
												0.,0.,0.]).reshape(6,1)
		self.Robot.P6c.world_X_base_offset = np.array([ps[0]*self.GridMap.resolution,
														ps[1]*self.GridMap.resolution,
														0.,0.,0.,0.]).reshape(6,1)
		self.Robot.P6d.world_X_base = self.Robot.P6c.world_X_base.copy()
		self.Robot.XHc.update_world_X_base(self.Robot.P6c.world_X_base)

		self.Robot.init_robot_stance()
		# self.Robot._initialise()

	def serv_set_grid_map(self, req):
		""" Set to selected grid map """

		print "Service - Setting Grid Map"
		self.GridMap = PyGridMap(req.request)
		self.Planner = PathPlanner(self.GridMap)
		self.convert_graph_to_pointcloud()
		return GenericStringResponse(response = "Success")

	def serv_path_planner(self, req):
		""" Plans path given start and end position """

		print "SERVICE - Path Planning"
		# convert to index position
		ps = (int(np.round(req.start.position.x/self.GridMap.resolution)), 
					int(np.round(req.start.position.y/self.GridMap.resolution)))
		pf = (int(np.round(req.goal.position.x/self.GridMap.resolution)), 
					int(np.round(req.goal.position.y/self.GridMap.resolution)))
		print ps, pf
		if (self.GridMap.get_index_exists(ps) and self.GridMap.get_index_exists(pf)):
			self.initialise_robot_state(ps, pf)

			motion_plan = self.Planner.generate_motion_plan(self.Robot, start=ps, end=pf)
			qbp, qbi, gphase, wXf, bXf, bXN = motionplan_to_planpath(motion_plan, "world")
			
			return PlanPathResponse(base_path = qbp, CoB = qbi, 
																gait_phase = gphase, 
																f_world_X_foot = wXf,
																f_base_X_foot = bXf,
																f_world_base_X_NRP = bXN)
		else:
			print "Start or End goal out of bounds!"
			return None

	def serv_get_grid_map(self, req):
		""" Returns grid map """

		print "SERVICE - Get Grid Map"
		gmap = RosGridMapMsg()
		
		gmap.info.header.stamp = rospy.Time.now()
		gmap.info.header.frame_id = "world"
		## Specify map parameters
		gmap.info.resolution = self.GridMap.resolution
		gmap.info.length_x = self.GridMap.map_size_m[0]
		gmap.info.length_y = self.GridMap.map_size_m[1]
		## Specify graph layers to be included
		gmap.layers = ["height","norm"]
		gmap.basic_layers = list(gmap.layers)
		## Convert graph to multiarray
		for i in range(0, len(gmap.layers)):
			gmap.data.append(graph_attr_to_multiarray(self.GridMap, gmap.layers[i]))

		return GridMapResponse(grid_map = gmap)

	def convert_graph_to_pointcloud(self):
		""" converts graph to pointcloud2 """

		arr_mapped_data  = point_cloud_array_mapping(self.GridMap.graph_to_nparray(), self.GridMap.viz_offset)
		self.point_cloud = array_to_pointcloud2(arr_mapped_data, rospy.Time.now(), "world")

	def loop_cycle(self):
		""" keeps the node looping """

		self.map_pub_.publish(self.point_cloud)
		rospy.sleep(2)

	def save_to_yaml(self):
		""" Saves the motion plan in a yaml file """
		
		def path_to_list(path):
			tlist = []; plist = []; vlist = []; alist = [];
			for i in range(len(path.xp)):
				tlist.append(path.t[i].tolist())
				plist.append(path.xp[i].tolist())
				vlist.append(path.xv[i].tolist())
				alist.append(path.xa[i].tolist())

			return tlist, plist, vlist, alist

		def footholds_to_list(footholds):

			footlist = [None]*6
			for j in range(6):
				footlist[j] = [footholds[j].xp[0].tolist()]
				for i in range(1, len(footholds[j].xp)):
					footlist[j].append(footholds[j].xp[i].tolist()) 
			return footlist

		def array_list_to_list(array):
			newlist = [];
			for i in range(len(array)):
				newlist.append(array[i].tolist())
			return newlist

		mapname = self.GridMap.map_name
		print 'Saving to yaml....'

		xt, xp, xv, xa = path_to_list(self.Planner.motion_plan.qb.X)
		wt, wp, wv, wa = path_to_list(self.Planner.motion_plan.qb.W)
		wXf = footholds_to_list(self.Planner.motion_plan.f_world_X_foot)
		bXf = footholds_to_list(self.Planner.motion_plan.f_base_X_foot)
		bXn = footholds_to_list(self.Planner.motion_plan.f_world_base_X_NRP)
		cob = array_list_to_list(self.Planner.motion_plan.qbp)

		data = dict(map=mapname,
					cell_resolution=self.GridMap.resolution,
					start = dict(x = self.Planner.start[0]*RosGridMap.GridMap.resolution,
								 y = self.Planner.start[1]*RosGridMap.GridMap.resolution,
								 z = self.Planner.base_map.nodes[self.Planner.start]['pose'][0]),
					goal = dict(x = self.Planner.goal[0]*RosGridMap.GridMap.resolution,
								y = self.Planner.goal[1]*RosGridMap.GridMap.resolution),
					path_x_t = xt,
					path_x_xp = xp,
					path_x_xv = xv,
					path_x_xa = xa,
					path_w_xp = wp,
					path_w_xv = wv,
					path_w_xa = wa,
					cob = cob,
					wXf = wXf,
					bXf = bXf,
					bXn = bXn,
					gphase = self.Planner.motion_plan.gait_phase
					)
		
		filename = 'data.yaml'
		with open(filename, 'w') as outfile:
			yaml.dump(data, outfile, default_flow_style=None)
		print 'Motion plan saved!'

def call_planner(ps, pf):
	rospy.wait_for_service('GridMap/query_map')
	try:
		start = Pose()
		goal  = Pose()
		start.position.x = ps[0]*RosGridMap.GridMap.resolution
		start.position.y = ps[1]*RosGridMap.GridMap.resolution
		goal.position.x = pf[0]*RosGridMap.GridMap.resolution
		goal.position.y = pf[1]*RosGridMap.GridMap.resolution

		path_planner = rospy.ServiceProxy('GridMap/query_map', PlanPath)
		path_planned = path_planner(start, goal, String())

		# mplan = planpath_to_motionplan(path_planned)
		# print mplan
		
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == "__main__":

	mapname = "flat"

	if len(sys.argv) > 1:
		if sys.argv[1] == 'wall_transition':
			mapname = 'wall_transition'
		elif sys.argv[1] == 'chimney_straight':
			mapname = 'chimney_straight'
		# Full Planning
		elif sys.argv[1] == 'taros':
			mapname = 'wall_hole_demo'
		# Cornering
		elif sys.argv[1] == 'chimney_corner_053':
			mapname = 'chimney_corner_053'
		elif sys.argv[1] == 'chimney_corner_066':
			mapname = 'chimney_corner_066'
		elif sys.argv[1] == 'wall_concave_corner':
			mapname = 'wall_concave_corner'
		elif sys.argv[1] == 'wall_convex_corner':
			mapname = 'wall_convex_corner'
		else:
			print 'No match found, setting to default'
			mapname = "flat"

	RosGridMap = GridMapRos(mapname)
	
	print "ROS Grid Map Planner Initialised"
	
	## Path Planning
	# ps = (10,13); pf = (150,9) 	# IROS
	# ps = (11,13); pf = (76,13) 	# TAROS
	
	# call_planner(ps, pf)

	# RosGridMap.save_to_yaml()

	while (not rospy.is_shutdown()):
		RosGridMap.loop_cycle()
