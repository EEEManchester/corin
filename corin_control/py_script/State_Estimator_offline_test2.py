#!/usr/bin/env python

## Offline State estimation for the robot
# Uses IMU data generated from model_states topic during generation
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
        rospy.init_node('IMU_estimator') 		#Initialises node
        self.data_source = "imu2"	# ideal, imu, imu2
        self.control_mode = "normal"
        self.rate   = rospy.Rate(100)	# frequency
        self.reset = True
        self.T_sim = 0.001
        self.T_imu = 0.01
        self.model_states = ModelStates()
        self.robot 	= robot_class.RobotState()
        self.estimator = StateEstimator(self.robot, self.T_imu)
        self.counter = 0
        self.update_i = 0
        self.update_N = 2

        self.cal_N = 10
        self.cal_sum = np.zeros(4)
        self.cal_i = 0.0
        self.level = 10

        self.pub_imu_a = rospy.Publisher('imu_a', Vector3, queue_size=1)
        self.pub_imu_v = rospy.Publisher('imu_v', Vector3, queue_size=1)
        self.pub_imu_r = rospy.Publisher('imu_r', Vector3, queue_size=1)
        self.pub_imu_w = rospy.Publisher('imu_w', Vector3, queue_size=1)
        self.pub_imu_p = rospy.Publisher('imu_p', Vector3, queue_size=1)

        self.pub_body_a = rospy.Publisher('body_a', Vector3, queue_size=1)
        self.pub_body_v = rospy.Publisher('body_v', Vector3, queue_size=1)
        self.pub_body_r = rospy.Publisher('body_r', Vector3, queue_size=1)
        self.pub_body_w = rospy.Publisher('body_w', Vector3, queue_size=1)
        self.pub_body_p = rospy.Publisher('body_p', Vector3, queue_size=1)

        #self.pub_imu2_a = rospy.Publisher('imu2_a', Vector3, queue_size=1)
        #self.pub_imu2_v = rospy.Publisher('imu2_v', Vector3, queue_size=1)
        self.pub_body2_v = rospy.Publisher('body2_v', Vector3, queue_size=1)
        self.pub_body2_r = rospy.Publisher('body2_r', Vector3, queue_size=1)

        self.pub_x = rospy.Publisher('imu_x', Float64, queue_size=1)
        self.pub_y = rospy.Publisher('imu_y', Float64, queue_size=1)
        self.pub_z = rospy.Publisher('imu_z', Float64, queue_size=1)
        self.pub_pose = rospy.Publisher('IMU_pose', PoseStamped, queue_size=1)

        self.pub_update = rospy.Publisher('update', Float64, queue_size=5)


        #self.IMU2_sub_  = rospy.Subscriber('corin/imu/base/data2', Imu, self.Imu2_callback, queue_size=1)

        self.p = None
        self.v = np.array([0.0, 0.0, 0.0])
        self.body_r = np.array([0.0, 0.0, 0.0])
        self.t = time.time()
        self.x = np.array([0.0, 0.0, 0.0])
        self.g_vec = [0.0] * 1000
        self.speed = np.zeros(3)
        self.speed2 = np.zeros(3)
        self.r = np.zeros(3)
        self.p0 = None
        self.v0 = None
        self.dp = None

        x = [1]*5 + [0]*5 + [-1]*5
        y = [j/2.0 for j in x]
        #z = [-9.81]*len(x)
        roll_speed = [10]*len(x)

        callback_dict = {   '/imu_mod': self.imu_callback,
                            '/corin/joint_states': self.joint_state_callback,
                            '/corin/contact_state': self.contact_state_callback,
                            '/body_r': self.body_r_callback}

        t0 = time.time()
        bag = rosbag.Bag('./testfile.bag')
        N = 0.9*bag.get_message_count(topic_filters=['/imu_mod'])
        print "messages:", N
        self.imu_a_z = np.array([])
        self.body_a_z = np.array([])
        self.imu_t = np.array([])
        self.body_t = np.array([])

        # i= 0
        # for topic, msg, t in bag.read_messages(topics=callback_dict.keys()):#topics=['chatter', 'numbers']):
        #     if topic in callback_dict:
        #         callback_dict[topic](msg, t.to_time())
        #         #print type(t)
        #          #self.counter += 1
        #         if topic == '/imu_mod':
        #             error = self.body_r - self.estimator.r
        #             print "e=", error
        #     # print topic
        #     i += 1
        #     if i > 10000:
        #         break
        Q = [0.00001, 0.0001, 0.001, 0.01, 0.1, 1, 10]
        for q in Q:
            self.estimator = StateEstimator(self.robot, self.T_imu)
            self.estimator.Qf = q*np.eye(3)
            i= 0
            e2 = 0
            for topic, msg, t in bag.read_messages(topics=callback_dict.keys()):#topics=['chatter', 'numbers']):
                if topic in callback_dict:
                    callback_dict[topic](msg, t.to_time())
                    #print type(t)
                     #self.counter += 1
                    if topic == '/imu_mod':
                        error = self.body_r - self.estimator.r
                        #print "e=", error
                        e2 += error**2
                # print topic
                i += 1
                if i > 10000:
                    e_rms = np.sqrt(e2/1000)
                    print "rms error:", e_rms
                    break

        bag.close()
        print "done in ", time.time()-t0

        plt.figure(1)
        plt.plot(self.imu_t, self.imu_a_z, label="Estimate")
        plt.plot(self.body_t, self.body_a_z, label="Actual")
        plt.legend()
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.show()

        self.power = 0

    def body_r_callback(self, msg, t):
        """ true value """
        r = msg.vector
        self.body_r = np.array([r.x, r.y, r.z])
        self.body_a_z = np.append(self.body_a_z, msg.vector.x)
        self.body_t = np.append(self.body_t, t)

    def joint_state_callback(self, msg, t):
        """ robot joint state callback """
        self.robot.qc = msg

    def contact_state_callback(self, msg, t):
        """ foot contact binary state """
        self.robot.cstate = msg.data

    def imu_callback(self, msg, t): 		# robot joint state callback

        self.robot.imu = msg

        o = msg.orientation
        av = msg.angular_velocity
        o = np.array([o.w, o.x, o.y, o.z])
        angular_velocity = np.array([av.x, av.y, av.z])
        self.p = 0

        if(True and self.cal_i  < self.cal_N):
            self.cal_sum += o
            self.cal_i += 1
        elif(True and self.cal_i == self.cal_N):
            q_av = self.cal_sum / self.cal_N
            self.estimator.q = q_av / np.sqrt(q_av.dot(q_av))

            # Move on to state estimation only after foot positions are reset
            if self.estimator.reset_foot_positions():
                self.cal_i += 1

        else:
            self.estimator.predict_state()

            self.update_i += 1
            if self.update_i >= self.update_N:
                self.estimator.update_state()
                self.update_i = 0

            self.imu_a_z = np.append(self.imu_a_z, self.estimator.r[0])
            self.imu_t = np.append(self.imu_t, t)


if __name__ == "__main__":

    state = CorinStateTester()

    "COMPLETE"
