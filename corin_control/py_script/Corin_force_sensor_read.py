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
from sensor_msgs.msg import JointState 				#	# sub msg for joint states
from trajectory_msgs.msg import JointTrajectoryPoint	# pub msg for Robotis joints
from std_msgs.msg import Float64 						# pub msg for Gazebo joints
from std_msgs.msg import ByteMultiArray 				# foot contact state
from std_msgs.msg import Float32MultiArray				# foot contact force
from geometry_msgs.msg import PoseStamped				# IMU pose
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

from experiment_numbers import *


class CorinStateTester:
    def __init__(self):
        rospy.init_node('Corin_force')
        t0 = time.time()

        #experiment number
        exp_no = 43
        if exp_no in no_offset_list:
            self.f_sensor4_offset = np.array([0, 0, 0])
        else:
            self.f_sensor4_offset = np.array([3, 0, 4])

        bag = rosbag.Bag(experiment[exp_no])

        self.f_sensor0 = np.empty((0,3))
        self.f_base0 = np.empty((0,3))
        self.f_sensor1 = np.empty((0,3))
        self.f_base1 = np.empty((0,3))
        self.f_sensor2 = np.empty((0,3))
        self.f_base2 = np.empty((0,3))
        self.f_sensor3 = np.empty((0,3))
        self.f_base3 = np.empty((0,3))
        self.f_sensor4 = np.empty((0,3))
        self.f_base4 = np.empty((0,3))
        self.f_sensor5 = np.empty((0,3))
        self.f_base5 = np.empty((0,3))

        self.f_mag0 = np.array([])
        self.f_mag0_t = np.array([])
        self.f_mag1 = np.array([])
        self.f_mag1_t = np.array([])
        self.f_mag2 = np.array([])
        self.f_mag2_t = np.array([])
        self.f_mag3 = np.array([])
        self.f_mag3_t = np.array([])
        self.f_mag4 = np.array([])
        self.f_mag4_t = np.array([])
        self.f_mag5 = np.array([])
        self.f_mag5_t = np.array([])


        self.f_mag4b = np.array([])
        self.f_mag5b = np.array([])


        self.robot 	= robot_class.RobotState()

        callback_dict = {   '/robotis/present_joint_states': self.joint_state_callback,
                            '/force_vector_0': self.force_0_callback,
                            '/force_vector_1': self.force_1_callback,
                            '/force_vector_2': self.force_2_callback,
                            '/force_vector_3': self.force_3_callback,
                            '/force_vector_4': self.force_4_callback,
                            '/force_vector_5': self.force_5_callback}


        i = 0
        print "start"
        for topic, msg, t in bag.read_messages():#topics=['/robotis/present_joint_states', '/imu/data']):
            if topic in callback_dict:
                callback_dict[topic](msg, t.to_time())

            i += 1
            print i
            if i > 100000:
                break
        bag.close()
        print "done in ", time.time()-t0


        plt.figure(3)
        plt.plot(self.f_mag0_t, self.f_mag0, label="0")
        plt.plot(self.f_mag1_t, self.f_mag1, label="1")
        plt.plot(self.f_mag2_t, self.f_mag2, label="2")
        plt.plot(self.f_mag3_t, self.f_mag3, label="3")
        plt.plot(self.f_mag4_t, self.f_mag4, label="4")
        plt.plot(self.f_mag5_t, self.f_mag5, label="5")
        plt.plot(self.f_mag4_t, self.f_mag4b, label="4b")
        # plt.plot(self.f_mag5_t, self.f_mag5b, label="5b")
        plt.legend()
        plt.xlabel('Time (s)')
        plt.ylabel('Force (N)')


        # plt.figure(5)
        # plt.plot(self.f_mag0_t, self.f_sensor0[:,0], label="f_x")
        # plt.plot(self.f_mag0_t, self.f_sensor0[:,1], label="f_y")
        # plt.plot(self.f_mag0_t, self.f_sensor0[:,2], label="f_z")
        # plt.plot(self.f_mag0_t, self.f_base0[:,0], label="b_x")
        # plt.plot(self.f_mag0_t, self.f_base0[:,1], label="b_y")
        # plt.plot(self.f_mag0_t, self.f_base0[:,2], label="b_z")
        # plt.legend()
        # plt.xlabel('Time (s)')
        # plt.ylabel('Foot 0 force (N)')
        #
        #
        # plt.figure(6)
        # plt.plot(self.f_mag1_t, self.f_sensor1[:,0], label="f_x")
        # plt.plot(self.f_mag1_t, self.f_sensor1[:,1], label="f_y")
        # plt.plot(self.f_mag1_t, self.f_sensor1[:,2], label="f_z")
        # plt.plot(self.f_mag1_t, self.f_base1[:,0], label="b_x")
        # plt.plot(self.f_mag1_t, self.f_base1[:,1], label="b_y")
        # plt.plot(self.f_mag1_t, self.f_base1[:,2], label="b_z")
        # plt.legend()
        # plt.xlabel('Time (s)')
        # plt.ylabel('Foot 1 force (N)')
        #
        # plt.figure(7)
        # plt.plot(self.f_mag2_t, self.f_sensor2[:,0], label="f_x")
        # plt.plot(self.f_mag2_t, self.f_sensor2[:,1], label="f_y")
        # plt.plot(self.f_mag2_t, self.f_sensor2[:,2], label="f_z")
        # plt.plot(self.f_mag2_t, self.f_base2[:,0], label="b_x")
        # plt.plot(self.f_mag2_t, self.f_base2[:,1], label="b_y")
        # plt.plot(self.f_mag2_t, self.f_base2[:,2], label="b_z")
        # plt.legend()
        # plt.xlabel('Time (s)')
        # plt.ylabel('Foot 2 force (N)')
        #
        # plt.figure(8)
        # plt.plot(self.f_mag3_t, self.f_sensor3[:,0], label="f_x")
        # plt.plot(self.f_mag3_t, self.f_sensor3[:,1], label="f_y")
        # plt.plot(self.f_mag3_t, self.f_sensor3[:,2], label="f_z")
        # plt.plot(self.f_mag3_t, self.f_base3[:,0], label="b_x")
        # plt.plot(self.f_mag3_t, self.f_base3[:,1], label="b_y")
        # plt.plot(self.f_mag3_t, self.f_base3[:,2], label="b_z")
        # plt.legend()
        # plt.xlabel('Time (s)')
        # plt.ylabel('Foot 3 force (N)')
        #
        plt.figure(9)
        plt.plot(self.f_mag4_t, self.f_sensor4[:,0], label="f_x")
        plt.plot(self.f_mag4_t, self.f_sensor4[:,1], label="f_y")
        plt.plot(self.f_mag4_t, self.f_sensor4[:,2], label="f_z")
        # plt.plot(self.f_mag4_t, self.f_base4[:,0], label="b_x")
        # plt.plot(self.f_mag4_t, self.f_base4[:,1], label="b_y")
        # plt.plot(self.f_mag4_t, self.f_base4[:,2], label="b_z")
        plt.legend()
        plt.xlabel('Time (s)')
        plt.ylabel('Foot 4 force (N)')


        plt.figure(10)
        plt.plot(self.f_mag5_t, self.f_sensor5[:,0], label="f_x")
        plt.plot(self.f_mag5_t, self.f_sensor5[:,1], label="f_y")
        plt.plot(self.f_mag5_t, self.f_sensor5[:,2], label="f_z")
        # plt.plot(self.f_mag5_t, self.f_base5[:,0], label="b_x")
        # plt.plot(self.f_mag5_t, self.f_base5[:,1], label="b_y")
        # plt.plot(self.f_mag5_t, self.f_base5[:,2], label="b_z")
        plt.legend()
        plt.xlabel('Time (s)')
        plt.ylabel('Foot 5 force (N)')
        #
        # plt.figure(11)
        # plt.plot(self.imu_ty, self.imu_y0[:,0], label="x")
        # plt.plot(self.imu_ty, self.imu_y0[:,1], label="y")
        # plt.plot(self.imu_ty, self.imu_y0[:,2], label="x")
        # plt.legend()
        # plt.xlabel('Time (s)')
        # plt.ylabel('Foot error 0 (m)')
        #
        #
        # plt.figure(12)
        # plt.plot(self.imu_ty, self.imu_y1[:,0], label="x")
        # plt.plot(self.imu_ty, self.imu_y1[:,1], label="y")
        # plt.plot(self.imu_ty, self.imu_y1[:,2], label="x")
        # plt.legend()
        # plt.xlabel('Time (s)')
        # plt.ylabel('Foot error 1 (m)')
        #
        #
        # plt.figure(13)
        # plt.plot(self.imu_ty, self.imu_y2[:,0], label="x")
        # plt.plot(self.imu_ty, self.imu_y2[:,1], label="y")
        # plt.plot(self.imu_ty, self.imu_y2[:,2], label="x")
        # plt.legend()
        # plt.xlabel('Time (s)')
        # plt.ylabel('Foot error 2 (m)')
        #
        #
        # plt.figure(14)
        # plt.plot(self.imu_ty, self.imu_y3[:,0], label="x")
        # plt.plot(self.imu_ty, self.imu_y3[:,1], label="y")
        # plt.plot(self.imu_ty, self.imu_y3[:,2], label="x")
        # plt.legend()
        # plt.xlabel('Time (s)')
        # plt.ylabel('Foot error 3 (m)')
        #
        # plt.figure(15)
        # plt.plot(self.imu_ty, self.imu_y4[:,0], label="x")
        # plt.plot(self.imu_ty, self.imu_y4[:,1], label="y")
        # plt.plot(self.imu_ty, self.imu_y4[:,2], label="x")
        # plt.legend()
        # plt.xlabel('Time (s)')
        # plt.ylabel('Foot error 4 (m)')
        #
        # plt.figure(16)
        # plt.plot(self.imu_ty, self.imu_y5[:,0], label="x")
        # plt.plot(self.imu_ty, self.imu_y5[:,1], label="y")
        # plt.plot(self.imu_ty, self.imu_y5[:,2], label="x")
        # plt.legend()
        # plt.xlabel('Time (s)')
        # plt.ylabel('Foot error 5 (m)')

        plt.show()
        self.power = 0

    def joint_state_callback(self, msg, t):
        """ robot joint state callback """
        self.robot.qc = msg


    def force_0_callback(self, msg, t):
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[0:3] = f
        f_mag = np.linalg.norm(f)
        self.f_mag0 = np.append(self.f_mag0, f_mag)
        self.f_mag0_t = np.append(self.f_mag0_t, t)

        # if self.robot.qc is not None:
        #     self.robot.update_state()
        # self.f_sensor0 = np.vstack([self.f_sensor0, self.robot.Leg[0].F6c.tibia_X_foot[:3].flatten()])
        # self.f_base0 = np.vstack([self.f_base0, self.robot.Leg[0].F6c.base_X_foot[:3].flatten()])

    def force_1_callback(self, msg, t):
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[3:6] = f
        f_mag = np.linalg.norm(f)
        self.f_mag1 = np.append(self.f_mag1, f_mag)
        self.f_mag1_t = np.append(self.f_mag1_t, t)

        # if self.robot.qc is not None:
        #     self.robot.update_state()
        # self.f_sensor1 = np.vstack([self.f_sensor1, self.robot.Leg[1].F6c.tibia_X_foot[:3].flatten()])
        # self.f_base1 = np.vstack([self.f_base1, self.robot.Leg[1].F6c.base_X_foot[:3].flatten()])

    def force_2_callback(self, msg, t):
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[6:9] = f
        f_mag = np.linalg.norm(f)
        self.f_mag2 = np.append(self.f_mag2, f_mag)
        self.f_mag2_t = np.append(self.f_mag2_t, t)

        # if self.robot.qc is not None:
        #     self.robot.update_state()
        # self.f_sensor2 = np.vstack([self.f_sensor2, self.robot.Leg[2].F6c.tibia_X_foot[:3].flatten()])
        # self.f_base2 = np.vstack([self.f_base2, self.robot.Leg[2].F6c.base_X_foot[:3].flatten()])

    def force_3_callback(self, msg, t):
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[9:12] = f
        f_mag = np.linalg.norm(f)
        self.f_mag3 = np.append(self.f_mag3, f_mag)
        self.f_mag3_t = np.append(self.f_mag3_t, t)

        # if self.robot.qc is not None:
        #     self.robot.update_state()
        # self.f_sensor3 = np.vstack([self.f_sensor3, self.robot.Leg[3].F6c.tibia_X_foot[:3].flatten()])
        # self.f_base3 = np.vstack([self.f_base3, self.robot.Leg[3].F6c.base_X_foot[:3].flatten()])

    def force_4_callback(self, msg, t):
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[12:15] = f
        f_mag = np.linalg.norm(f)
        self.f_mag4 = np.append(self.f_mag4, f_mag)
        self.f_mag4_t = np.append(self.f_mag4_t, t)

        f2 = f + self.f_sensor4_offset
        f2_mag = np.linalg.norm(f2)
        self.f_mag4b = np.append(self.f_mag4b, f2_mag)

        if self.robot.qc is not None:
            self.robot.update_state()
        self.f_sensor4 = np.vstack([self.f_sensor4, self.robot.Leg[4].F6c.tibia_X_foot[:3].flatten()])
        self.f_base4 = np.vstack([self.f_base4, self.robot.Leg[4].F6c.base_X_foot[:3].flatten()])

    def force_5_callback(self, msg, t):
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[15:18] = f
        f_mag = np.linalg.norm(f)
        self.f_mag5 = np.append(self.f_mag5, f_mag)
        self.f_mag5_t = np.append(self.f_mag5_t, t)

        f2 = f + np.array([2, 0, 2])
        f2_mag = np.linalg.norm(f2)
        self.f_mag5b = np.append(self.f_mag5b, f2_mag)

        if self.robot.qc is not None:
            self.robot.update_state()
        self.f_sensor5 = np.vstack([self.f_sensor5, self.robot.Leg[5].F6c.tibia_X_foot[:3].flatten()])
        self.f_base5 = np.vstack([self.f_base5, self.robot.Leg[5].F6c.base_X_foot[:3].flatten()])

    def contact_state_callback(self, msg, t):
        """ foot contact binary state """
        self.robot.cstate = msg.data


if __name__ == "__main__":

    state = CorinStateTester()

    "COMPLETE"
