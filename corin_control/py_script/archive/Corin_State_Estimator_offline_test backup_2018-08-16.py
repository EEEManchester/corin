#!/usr/bin/env python

## Offline State estimation for the robot
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

        t0 = time.time()
        bag = rosbag.Bag('./corin_run_2.bag')
        N = 0.9*bag.get_message_count(topic_filters=['/imu_mod'])
        print "messages:", N
        self.imu_r = np.empty((0,3))
        self.imu_t = np.array([])
        # print self.imu_r.shape
        # k = np.array([0,1,2])
        # print k
        # self.imu_r = np.vstack([self.imu_r, k])
        # #print np.vstack([self.imu_r, k])
        # raw_input("!")

        self.update_i = 0
        self.update_N = 2
        self.cal_N = 10
        self.cal_sum = np.zeros(4)
        self.cal_i = 0.0

        self.T_imu = 0.01
        self.robot 	= robot_class.RobotState()
        self.estimator = StateEstimator(self.robot, self.T_imu)

        callback_dict = {   '/imu/data': self.imu_callback,
                            '/robotis/present_joint_states': self.joint_state_callback,
                            '/force_vector_0': self.force_0_callback,
                            '/force_vector_1': self.force_1_callback,
                            '/force_vector_2': self.force_2_callback,
                            '/force_vector_3': self.force_3_callback,
                            '/force_vector_4': self.force_4_callback,
                            '/force_vector_5': self.force_5_callback,}

        i = 0
        for topic, msg, t in bag.read_messages():#topics=['/robotis/present_joint_states', '/imu/data']):
            if topic in callback_dict:
                callback_dict[topic](msg, t.to_time())
                #print type(t)
                 #self.counter += 1
                # if topic == '/imu_mod':
                #     error = self.body_r - self.estimator.r
                #     print "e=", error
            print t, topic
            # i += 1
            # if i > 2000:
            #     break
        bag.close()
        print "done in ", time.time()-t0

        plt.figure(1)
        plt.plot(self.imu_t, self.imu_r[:,0], label="x")
        plt.plot(self.imu_t, self.imu_r[:,1], label="y")
        plt.plot(self.imu_t, self.imu_r[:,2], label="z")
        plt.legend()
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.show()

        self.power = 0

    def force_0_callback(self, msg, t):
        self.robot.cstate[0] = self.leg_state(msg)

    def force_1_callback(self, msg, t):
        self.robot.cstate[1] = self.leg_state(msg)

    def force_2_callback(self, msg, t):
        self.robot.cstate[2] = self.leg_state(msg)

    def force_3_callback(self, msg, t):
        self.robot.cstate[3] = self.leg_state(msg)

    def force_4_callback(self, msg, t):
        self.robot.cstate[4] = self.leg_state(msg)

    def force_5_callback(self, msg, t):
        self.robot.cstate[5] = self.leg_state(msg)

    def leg_state(self, msg):
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        return 1 if np.linalg.norm(f) > 10 else 0

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

            # self.imu_r = np.append(self.imu_r, self.estimator.r, axis=0)
            self.imu_r = np.vstack([self.imu_r, self.estimator.r])
            self.imu_t = np.append(self.imu_t, t)


if __name__ == "__main__":

    state = CorinStateTester()

    "COMPLETE"
