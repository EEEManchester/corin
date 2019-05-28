#!/usr/bin/env python

## State estimation for the robot
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'library'))
sys.dont_write_bytecode = True

import time
import warnings
import numpy as np

## Personal libraries
from library import *			# library modules to include
from constant import *
import control_interface 								# action selection from ROS parameter server
import robot_class 										# class for robot states and function

## ROS messages
import rospy
from sensor_msgs.msg import Imu 						# sub msg for IMU
from sensor_msgs.msg import JointState 					# sub msg for joint states
from trajectory_msgs.msg import JointTrajectoryPoint	# pub msg for Robotis joints
from std_msgs.msg import Float64 						# pub msg for Gazebo joints
from std_msgs.msg import ByteMultiArray 				# foot contact state
from std_msgs.msg import Float32MultiArray				# foot contact force
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped


## Robotis ROS msgs for joint control
from robotis_controller_msgs.msg import SyncWriteItem 	# pub msg for Robotis joints
from robotis_controller_msgs.msg import SyncWriteMulti 	# pub msg for Robotis joints

class CorinState:
	def __init__(self):
		rospy.init_node('IMU_LORD_estimator') 		#Initialises node

		self.rate 	  = rospy.Rate(100)	# frequency
		self.reset = True
		self.t = 0
		self.T = 0.01

		self.pub_r = rospy.Publisher('imu_LORD_r', Vector3, queue_size=1)

		self.IMU_sub_  = rospy.Subscriber('/imu_LORD/data', Imu, self.Imu_callback, queue_size=5)

		self.pub_pose = rospy.Publisher('IMU_pose', PoseStamped, queue_size=1)


		self.v = np.array([0.0, 0.0, 0.0])
		self.x = np.array([0.0, 0.0, 0.0])
		self.g_vec = [np.zeros(3)] * 1000
		self.w_vec = [np.zeros(3)] * 1000

		rospy.sleep(0.5)

		np.set_printoptions(suppress=True) # suppress scientific notation
		np.set_printoptions(precision=5) # number display precision
		#np.set_printoptions(formatter={'float': '{: 0.7f}'.format})


	def Imu_callback(self, msg): 		# robot joint state callback

		a = msg.linear_acceleration.x
		t = msg.header.stamp.to_sec()

		#self.v += a * self.T
		#self.x += self.v * self.T

		#self.pub_.publish(self.x)

		if self.reset:
			self.t0 = t
			self.samples = 0
			self.reset = False
			self.mag_sum = 0
		else:
			self.samples += 1
			#print("samples", self.samples)
			#print((t-self.t0)/self.samples)

		o = msg.orientation
		orientation = np.array([o.w, o.x, o.y, o.z])
		euler = euler_from_quaternion(orientation, 'rxyz')
		euler_deg = tuple([k * 180/np.pi for k in euler])

		#print np.array(euler)* 180/np.pi

		w = msg.angular_velocity
		w = np.array([w.x, w.y, w.z])

		self.w_vec.pop()
		self.w_vec.insert(0,w)
		print "w stddev:", np.std(self.w_vec, axis=0)

		ps = PoseStamped()
		ps.header.stamp 	 = rospy.Time.now()
		ps.header.frame_id = "trunk"
		#pose.pose.position.x = path_arr.X.xp[i][0] + offset[0]
		ps.pose.orientation = o
		self.pub_pose.publish(ps)

		acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
		g_mag = np.linalg.norm(acc)

		self.g_vec.pop()
		self.g_vec.insert(0,acc)

		self.mag_sum += g_mag
		#mag_avg = self.mag_sum/self.samples
		mag_avg = np.mean(self.g_vec, axis=0)
		print "g stddev:", np.std(self.g_vec, axis=0)
		#print("g_avg", mag_avg)
		#print("g", g_mag)

		#print("mag", g_mag)

		#print(t-self.t)
		#self.t = t


		#gravity = g_mag * np.array([0, 0, 1, 0])
		#gravity = np.linalg.norm(mag_avg) * np.array([0, 0, 1, 0])
		gravity = 9.81 * np.array([0, 0, 1, 0])

		#g = qv_mult(tf.quaternion_inverse(orientation), gravity)
		#g = qv_mult(orientation, gravity)

		#print(g)

 		R = quaternion_matrix(orientation)

		# estimated gravity
		v = np.linalg.inv(R).dot(gravity.T)
		#v = R.dot(gravity.T)
		# print "acc:", acc
		# print "rotated:", v

		# corrected accelerometer data
		acc2 = acc-v[:3]

		'''self.pub_x.publish(acc2[0])
		self.pub_y.publish(acc2[1])
		self.pub_z.publish(acc2[2])'''

		if (self.samples % 400) == 0:
			self.v = np.array([0.0, 0.0, 0.0])
			self.x = np.array([0.0, 0.0, 0.0])
		else:
			'''
			thresh = 1
			if abs(acc2[0]) < thresh:
				acc2[0] = 0
			if abs(acc2[1]) < thresh:
				acc2[1] = 0
			if abs(acc2[2]) < thresh:
				acc2[2] = 0

			if abs(acc2[0]) < thresh:
				self.v = np.array([0.0, 0.0, 0.0])
			else:
				self.v += acc2 * self.T'''
			self.v += acc2 * self.T
			#self.x += self.v * self.T
			self.x += (self.v * self.T) + (0.5 * acc2 * self.T * self.T)

		#self.pub_x.publish(mag_avg)
		#self.pub_y.publish(g_mag)
		#self.pub_r.publish(Vector3(*self.x))

		self.pub_r.publish(Vector3(*acc2))

		#print(v)

# rotate vector v1 by quaternion q1
def qv_mult(q1, v1):
    #v1 = tf.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return quaternion_multiply(
        quaternion_multiply(q1, q2),
        quaternion_conjugate(q1)
    )#[1:4]


if __name__ == "__main__":

	state = CorinState()

	rospy.spin()
