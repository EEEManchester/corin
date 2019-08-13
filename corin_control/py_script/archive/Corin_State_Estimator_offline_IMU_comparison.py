#!/usr/bin/env python

## Offline State estimation for the robot
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'library'))
sys.dont_write_bytecode = True

import time
import datetime
import warnings
import numpy as np

## Personal libraries
from library import *			# library modules to include
from constant import *
import control_interface 								# action selection from ROS parameter server
import robot_class 										# class for robot states and function
import tf 												# SE(3) transformation library

## ROS messages
import rospy
from sensor_msgs.msg import Imu 						# sub msg for IMU
from sensor_msgs.msg import JointState 					# sub msg for joint states
from trajectory_msgs.msg import JointTrajectoryPoint	# pub msg for Robotis joints
from std_msgs.msg import Float64 						# pub msg for Gazebo joints
from std_msgs.msg import ByteMultiArray 				# foot contact state
from std_msgs.msg import Float32MultiArray				# foot contact force
from geometry_msgs.msg import PoseStamped					# IMU pose
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import matplotlib.pyplot as plt

## Robotis ROS msgs for joint control
from robotis_controller_msgs.msg import SyncWriteItem 	# pub msg for Robotis joints
from robotis_controller_msgs.msg import SyncWriteMulti 	# pub msg for Robotis joints

from State_Estimator import StateEstimator

import rosbag

np.set_printoptions(suppress=True) # suppress scientific notation
np.set_printoptions(precision=5) # number display precision
np.set_printoptions(formatter={'float': '{: 0.7f}'.format})
np.set_printoptions(linewidth=1000)

class CorinStateTester:
    def __init__(self):
        rospy.init_node('Corin_estimator')

        self.IMU = "LORD" # LORD / ODROID

        self.pub_pose = rospy.Publisher('/IMU_pose', PoseStamped, queue_size=1)
        self.pub_pose2 = rospy.Publisher('/IMU_pose2', PoseStamped, queue_size=1)
        t0 = time.time()
        bag = rosbag.Bag('./corin_walking_6.bag')# 

        callback_dict = {   '/imu_LORD/data': self.imu_callback,
                            '/imu/data': self.imu_callback0,}

        i = 0
        for topic, msg, t in bag.read_messages():#topics=['/robotis/present_joint_states', '/imu/data']):
            if topic in callback_dict:
                callback_dict[topic](msg, t.to_time())
                rospy.sleep(0.001)
                # print type(t)
                 #self.counter += 1
                # if topic == '/imu_mod':
                #     error = self.body_r - self.estimator.r
                #     print "e=", error
            # print t, topic, msg
            # i += 1
            # if i > 2000:
            #     break
        bag.close()
        print "done in ", time.time()-t0

    def imu_callback(self, msg, t): 		# robot joint state callback

        o = msg.orientation
        av = msg.angular_velocity
        o = np.array([o.w, o.x, o.y, o.z])
        # only for LORD IMU (to put the orientation from NED to the correct frame)
        o = quaternion_multiply(np.array([0, 1, 0, 0]), o)
        msg.orientation = Quaternion(o[1], o[2], o[3], o[0])

        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "world"
        ps.pose.orientation = Quaternion(o[1], o[2], o[3], o[0])
        self.pub_pose.publish(ps)


    def imu_callback0(self, msg, t):
        o = msg.orientation
        av = msg.angular_velocity
        o = np.array([o.w, o.x, o.y, o.z])
        angular_velocity = np.array([av.x, av.y, av.z])
        self.imu0 = o

        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "world"
        ps.pose.orientation = Quaternion(o[1], o[2], o[3], o[0])
        self.pub_pose2.publish(ps)



if __name__ == "__main__":

    state = CorinStateTester()

    "COMPLETE"
