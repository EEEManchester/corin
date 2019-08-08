#!/usr/bin/env python

## Publishes robot base pose for RVIZ
import sys
import rospy
import tf
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates

class Node_class:
	def __init__(self, offset):
		rospy.init_node("robot_base_publisher")							# Initiates Ros node
		self.freq = 400
		self.rate = rospy.Rate(self.freq)					# Sets transmit frequency
		self.robot_broadcaster = tf.TransformBroadcaster()	# Transform for robot pose
		self.lidar_broadcaster = tf.TransformBroadcaster()	# Transform for robot pose
		self.robot_offset = offset

		self.start()

	def start(self):
		##***************** SUBSCRIBERS ***************##
		# subscribe to gazebo
		if rospy.has_param('/gazebo/auto_disable_bodies'):
			self.robot_state_sub_ = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback)

		# subscribe to hardware IMU
		else:
			self.imu_sub_ 		  = rospy.Subscriber("/imu_data", Imu, self.imu_callback)

		##***************** PUBLISHERS ***************##


	## Transform from gazebo robot_pose to TF for RVIZ visualization
	def model_callback(self, robot_state):
		rs_size = len(robot_state.name)

		for i in range(0,rs_size):
			if (robot_state.name[i]=='corin'):
				px = robot_state.pose[i].position.x + self.robot_offset[0]
				py = robot_state.pose[i].position.y + self.robot_offset[1]
				pz = robot_state.pose[i].position.z + self.robot_offset[2]

				rx = robot_state.pose[i].orientation.x
				ry = robot_state.pose[i].orientation.y
				rz = robot_state.pose[i].orientation.z
				rw = robot_state.pose[i].orientation.w

				self.robot_broadcaster.sendTransform( (px,py,pz), (rx,ry,rz,rw), rospy.Time.now(), "base_link", "world") ;
				# self.lidar_broadcaster.sendTransform( (px,py,pz), (rx,ry,rz,rw), rospy.Time.now(), "trunk_lidar", "trunk") ;

	def imu_callback(self, imu):
	    self.robot_broadcaster.sendTransform( (0.0,0.0,0.1), tf.transformations.quaternion_from_euler(imu.orientation.y, imu.orientation.x, -imu.orientation.z),
	    							rospy.Time.now(), "trunk", "base_link") ;


if __name__ == "__main__":

	if len(sys.argv) != 6:
		offset = [0,0,0]
	elif len(sys.argv == 6):
		offset = map(lambda x: float(x), sys.argv[1:4])
		
	n = Node_class(offset)
	rospy.loginfo('Robot base transform initiated')
	rospy.spin()
