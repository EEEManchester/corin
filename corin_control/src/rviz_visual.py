#!/usr/bin/env python

""" Visualisation topics for rviz 
	and functions for mundane task """

import rospy
import tf
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import WrenchStamped, PolygonStamped, Point32

from grid_planner_core.numpy_to_rosmsg import *
from corin_control.constant import *
from cone_visual.msg import ConeStamped
from tf.transformations import quaternion_from_euler
import math

class RvizVisualise:
	def __init__(self):
		self.fr_fix = "world"
		self.fr_robot = "trunk"

		##***************** PUBLISHERS ***************##
		self.map_pub_  = rospy.Publisher(ROBOT_NS + '/point_cloud', PointCloud2, queue_size=1) 	# grid map
		self.path_pub_ = rospy.Publisher(ROBOT_NS + '/path', Path, queue_size=1) 				# splined path
		self.mark_pub_ = rospy.Publisher(ROBOT_NS + '/footholds', MarkerArray, queue_size=1)	# fooholds
		self.cob_pub_  = rospy.Publisher(ROBOT_NS + '/cob', MarkerArray, queue_size=1) 			# CoB position
		self.poly_pub_ = rospy.Publisher(ROBOT_NS + '/support_polygon', PolygonStamped, queue_size=1) # Support polygon
		self.com_pub_  = rospy.Publisher(ROBOT_NS + '/centre_of_mass', Marker, queue_size=1) # CoM position
		self.cone_pub_ = rospy.Publisher('cone_visual/cone_arrays', ConeStamped, queue_size=1)
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

	def publish_robot_pose(self, qb):
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

	def publish_support_polygon(self, footholds, offset=None):
		""" Support polygon publisher 
			Input:  footholds: list of footholds
					offset: offsets the polygon """
		
		if offset is None:
			offset = [0,0,0]

		poly = PolygonStamped()
		poly.header.stamp = rospy.Time.now()
		poly.header.frame_id = self.fr_fix
		for f in footholds:
			p = Point32()
			p.x = f[0] + offset[0]
			p.y = f[1] + offset[1]
			p.z = 0. #+ offset[2]
			poly.polygon.points.append(p)
			
		self.poly_pub_.publish(poly)

	def publish_com(self, com):
		""" Publishes the CoM position """
		
		mark = Marker()
		mark.header.stamp = rospy.Time.now()
		mark.header.frame_id = self.fr_fix

		mark.type = 2
		mark.id = 0
		mark.pose.position.x = com[0]
		mark.pose.position.y = com[1]
		mark.pose.position.z = 0.
		mark.pose.orientation.x = 0.0;
		mark.pose.orientation.y = 0.0;
		mark.pose.orientation.z = 0.0;
		mark.pose.orientation.w = 1.0;
		mark.scale.x = 0.03
		mark.scale.y = 0.03
		mark.scale.z = 0.03
		mark.color.a = 1.0; # transparency level

		self.com_pub_.publish(mark)

	def show_motion_plan(self, robot_pose, path, footholds):
		""" Publishes the motion plan
			Input: 	robot_pose: robot's current pose
					path: the path for the robot to travel
					footholds: footholds for all six legs 	"""

		for c in range(0,3):
			self.publish_robot_pose(robot_pose)
			self.publish_path(path)
			self.publish_footholds(footholds)
			rospy.sleep(0.2)

	def publish_friction_cones(self, robot, mu):
		# cones_pos, cones_rpy, cones_angle
		""" Converts custom marker list to ConeStamped """

		cones_pos = []
		cones_rpy = []
		cones_angle = []
		for j in range(6):
			if robot.Gait.cs[j] == 0:
				cones_pos.append(robot.Leg[j].XHc.world_X_foot[:3,3])
				cones_rpy.append(np.array([0.,0.,0.]))
				cones_angle.append(math.atan(mu))

		## Define Variables ##
		cone = ConeStamped()
		cone.header.stamp = rospy.Time.now()
		cone.header.frame_id = "world"
		
		for i in range(len(cones_pos)):
			pose3d = Pose()
			pose3d.position.x = cones_pos[i][0]
			pose3d.position.y = cones_pos[i][1]
			pose3d.position.z = cones_pos[i][2]

			quat = quaternion_from_euler(cones_rpy[i][0], 
										 cones_rpy[i][1],
										 cones_rpy[i][2])
			pose3d.orientation.x = quat[0]
			pose3d.orientation.y = quat[1]
			pose3d.orientation.z = quat[2]
			pose3d.orientation.w = quat[3]

			cone.pose.append(pose3d)
			cone.angles.append(Float32(cones_angle[i])) 	# internal angle

		self.cone_pub_.publish(cone)