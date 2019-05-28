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
        rospy.init_node('Corin_estimator')

        exp_no = 81 #experiment number
        self.IMU = "LORD" # LORD / ODROID
        self.ideal_q = True # Use VICON orientation as estimate?
        self.plot_IMU_bias = True
        self.plot_leg_position = True
        self.plot_residual = False

        if exp_no in no_offset_list:
            self.f_sensor4_offset = np.array([0, 0, 0])
        else:
            self.f_sensor4_offset = np.array([3, 0, 4])

        bag = rosbag.Bag(experiment[exp_no])

        self.pub_pose = rospy.Publisher('/IMU_pose', PoseStamped, queue_size=1)
        self.pub_pose2 = rospy.Publisher('/IMU_pose2', PoseStamped, queue_size=1)
        t0 = time.time()
        N = 0.9*bag.get_message_count(topic_filters=['/imu_mod'])
        print "messages:", N
        self.imu_r = np.empty((0,3))
        self.imu_p2 = np.empty((0,3))
        self.imu_bf = np.empty((0,3))
        self.imu_bw = np.empty((0,3))
        self.imu_y0 = np.empty((0,3))
        self.imu_y1 = np.empty((0,3))
        self.imu_y2 = np.empty((0,3))
        self.imu_y3 = np.empty((0,3))
        self.imu_y4 = np.empty((0,3))
        self.imu_y5 = np.empty((0,3))

        self.imu_angles = np.empty((0,3))
        self.imu_o_angles = np.empty((0,3))
        self.imu_t = np.array([])
        self.imu_ty = np.array([])
        self.q0 = None
        self.q1 = None
        self.imu0 = None
        self.p2_0 = None

        self.vicon_r = np.empty((0,3))
        self.vicon_angles = np.empty((0,3))
        self.vicon_t = np.array([])
        self.vicon_r0 = None
        self.vicon_q0 = None
        self.vicon_p2_0 = None
        self.vicon_q = np.empty((0,4))
         # position of foot in the leg VICON object frame
        self.vicon_foot_offset = 0.001 * np.array([-32, 0, -60])

        self.vicon_leg_r = np.empty((0,3))
        self.vicon_leg_angles = np.empty((0,3))
        self.vicon_leg_t = np.array([])
        # print self.imu_r.shape
        # k = np.array([0,1,2])
        # print k
        # self.imu_r = np.vstack([self.imu_r, k])
        # #print np.vstack([self.imu_r, k])
        # raw_input("!")

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

        self.update_i = 0
        self.update_N = 7
        self.cal_N = 100
        self.cal_sum = np.zeros(4)
        self.a0_sum = np.zeros(3)
        self.w_sum = np.zeros(3)
        self.cal_i = 0.0

        self.robot 	= robot_class.RobotState()
        ################
        # self.robot.imu = Imu()
        # self.robot.qc = JointState()
        # self.robot.qc.name = ['joint']*18
        # self.robot.qc.position = [0]*18
        # self.robot.qc.velocity = [0]*18
        # self.robot.update_state()
        # for j in range(0,self.robot.active_legs):
        #     print self.robot.Leg[j].XHc.base_X_foot#[:3,3]
        # exit()
        ###################

        if self.IMU == "LORD":
            self.T_imu = 0.002
            self.update_N = 10
        else: # ODROID
            self.T_imu = 0.01
            self.update_N = 2

        self.estimator = StateEstimator(self.robot, self.T_imu)
        self.estimator.Qf = 0.000625*np.eye(3)# (0.00025**2)*np.eye(3) # Noise power density (m/s2 / sqrt(Hz))
        self.estimator.Qbf = 0.1*np.eye(3)#(0.001)*np.eye(3)
        self.estimator.Qw = 6.4E-11*np.eye(3)#(0.00008**2)*np.eye(3)
        self.estimator.Qbw = 0.0001*np.eye(3)#(0.001)*np.eye(3)

        if (self.ideal_q == True):
            self.estimator.Qw = 0*np.eye(3)
            self.estimator.Qbw = 0*np.eye(3)

        # self.estimator.Qf = (1**2)*np.eye(3)
        # self.estimator.Qw = (1**2)*np.eye(3)

        self.estimator.Qp = 0.000001*np.eye(3)#
        # self.estimator.Qp = np.diagflat([0.002**2, 0.002**2, 0.001**2])	# [<dx> m, <dy> m, <dz> m]
        self.estimator.Rs = 0.000001*np.eye(3)#(0.0001**2)*np.eye(3) # m^2 #0.0000001
        self.estimator.Ra = 0.0001*np.eye(3)#(0.01**2)*np.eye(3)

        callback_dict = {   '/imu_LORD/data': self.imu_callback0,
                            '/imu/data': self.imu_callback,
                            '/robotis/present_joint_states': self.joint_state_callback,
                            '/force_vector_0': self.force_0_callback,
                            '/force_vector_1': self.force_1_callback,
                            '/force_vector_2': self.force_2_callback,
                            '/force_vector_3': self.force_3_callback,
                            '/force_vector_4': self.force_4_callback,
                            '/force_vector_5': self.force_5_callback,
                            '/vicon/corin/corin': self.vicon_callback,#}#,
                            '/vicon/corin_leg/corin_leg': self.vicon_leg_callback,
                            '/vicon/Corin/Corin': self.vicon_callback,
                            '/vicon/Corin_leg/Corin_leg': self.vicon_leg_callback}

        if self.IMU == "LORD":
            temp = callback_dict['/imu_LORD/data']
            callback_dict['/imu_LORD/data'] = callback_dict['/imu/data']
            callback_dict['/imu/data'] = temp
            self.estimator.IMU_r = 0.001*np.array([-123, 5.25, 24.5]) # Body frame origin relative to IMU frame, described in the IMU frame
            self.estimator.IMU_R = np.array([[-1, 0, 0],
                                            [0, 1, 0],
                                            [0, 0, -1]])	# Rotation matrix mapping vectors from IMU frame to body frame
        else: # ODROID
            self.estimator.IMU_r = 0.001*np.array([-4, -8, -27]) #np.zeros(3)
            self.estimator.IMU_R = np.eye(3)

        errors = np.empty((0,3))
        error_norms =  np.array([])
        error_ratios =  np.array([])
        i = 0
        for topic, msg, t in bag.read_messages():#topics=['/robotis/present_joint_states', '/imu/data']):
            if topic in callback_dict:
                callback_dict[topic](msg, t.to_time())

                if topic == '/vicon/corin/corin' or topic == '/vicon/Corin/Corin':
                    if self.vicon_r.size > 0:
                        error = self.vicon_r[-1] - self.estimator.r
                        errors = np.vstack([errors, error])
                        error_norm = np.linalg.norm(error)
                        error_norms = np.append(error_norms, error_norm)
                        error_ratio = np.linalg.norm(error)/np.linalg.norm(self.vicon_r[-1])
                        error_ratios = np.append(error_ratios, error_ratio)
                #print type(t)
                 #self.counter += 1
                # if topic == '/imu_mod':
                #     error = self.body_r - self.estimator.r
                #     print "e=", error
            # print t, topic
            i += 1
            if i > 40000:
                break
        bag.close()
        print "done in ", time.time()-t0

        e_max_norm = error_norms.max()
        e_max_ratio = error_ratios.max()
        e_max_x = np.abs(errors[:,0]).max()
        e_max_y = np.abs(errors[:,1]).max()
        e_max_z = np.abs(errors[:,2]).max()
        e_rms_norm = np.sqrt(np.mean(np.square(error_norms)))
        e_rms_ratio = np.sqrt(np.mean(np.square(error_ratios)))
        e_rms_x = np.sqrt(np.mean(np.square(errors[:,0])))
        e_rms_y = np.sqrt(np.mean(np.square(errors[:,1])))
        e_rms_z = np.sqrt(np.mean(np.square(errors[:,2])))

        print "max error:", e_max_norm
        # print "max error ratio:", e_max_ratio
        print "max error x:", e_max_x
        print "max error y:", e_max_y
        print "max error z:", e_max_z
        print "rms error:", e_rms_norm
        # print "rms error ratio:", e_rms_ratio
        print "rms error x:", e_rms_x
        print "rms error y:", e_rms_y
        print "rms error z:", e_rms_z

        print "final error ratio:", error_ratios[-1]

        # plt.figure(4)
        # plt.plot(self.vicon_t, errors[:,0], label="x")
        # plt.plot(self.vicon_t, errors[:,1], label="y")
        # plt.plot(self.vicon_t, errors[:,2], label="z")
        # plt.plot(self.vicon_t, error_norms, label="norm")
        # # plt.plot(self.vicon_t, error_ratios, label="ratio")
        # plt.legend()
        # plt.xlabel('Time (s)')
        # plt.ylabel('Error (m)')

        plt.figure(1)
        plt.plot(self.imu_t, self.imu_r[:,0], label="Estimate-x")
        plt.plot(self.imu_t, self.imu_r[:,1], label="Estimate-y")
        plt.plot(self.imu_t, self.imu_r[:,2], label="Estimate-z")
        plt.plot(self.vicon_t, self.vicon_r[:,0], label="True-x")
        plt.plot(self.vicon_t, self.vicon_r[:,1], label="True-y")
        plt.plot(self.vicon_t, self.vicon_r[:,2], label="True-z")
        plt.legend()
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')

        plt.figure(2)
        plt.plot(self.imu_t, self.imu_angles[:,0], label="imu-r")
        plt.plot(self.imu_t, self.imu_angles[:,1], label="imu-p")
        plt.plot(self.imu_t, self.imu_angles[:,2], label="imu-y")
        plt.plot(self.vicon_t, self.vicon_angles[:,0], label="vicon-r")
        plt.plot(self.vicon_t, self.vicon_angles[:,1], label="vicon-p")
        plt.plot(self.vicon_t, self.vicon_angles[:,2], label="vicon-y")
        # plt.plot(self.imu_t, self.imu_o_angles[:,0], label="imu_o-r")
        # plt.plot(self.imu_t, self.imu_o_angles[:,1], label="imu_o-p")
        # plt.plot(self.imu_t, self.imu_o_angles[:,2], label="imu_o-y")
        plt.legend()
        plt.xlabel('Time (s)')
        plt.ylabel('Angles (rad)')

        if self.plot_leg_position:
            plt.figure(99)
            plt.plot(self.imu_t, self.imu_p2[:,0], label="Estimate-x")
            plt.plot(self.imu_t, self.imu_p2[:,1], label="Estimate-y")
            plt.plot(self.imu_t, self.imu_p2[:,2], label="Estimate-z")
            plt.plot(self.vicon_leg_t, self.vicon_leg_r[:,0], label="True-x")
            plt.plot(self.vicon_leg_t, self.vicon_leg_r[:,1], label="True-y")
            plt.plot(self.vicon_leg_t, self.vicon_leg_r[:,2], label="True-z")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot Position (m)')

        if self.plot_IMU_bias:
            plt.figure(100)
            plt.plot(self.imu_t, self.imu_bf[:,0], label="Estimate-x")
            plt.plot(self.imu_t, self.imu_bf[:,1], label="Estimate-y")
            plt.plot(self.imu_t, self.imu_bf[:,2], label="Estimate-z")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Acc bias (m/s^2)')
            plt.figure(101)
            plt.plot(self.imu_t, self.imu_bw[:,0], label="Estimate-x")
            plt.plot(self.imu_t, self.imu_bw[:,1], label="Estimate-y")
            plt.plot(self.imu_t, self.imu_bw[:,2], label="Estimate-z")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Gyro bias (rad)')

        plt.figure(3)
        plt.plot(self.f_mag0_t, self.f_mag0, label="0")
        plt.plot(self.f_mag1_t, self.f_mag1, label="1")
        plt.plot(self.f_mag2_t, self.f_mag2, label="2")
        plt.plot(self.f_mag3_t, self.f_mag3, label="3")
        plt.plot(self.f_mag4_t, self.f_mag4, label="4")
        plt.plot(self.f_mag5_t, self.f_mag5, label="5")
        plt.legend()
        plt.xlabel('Time (s)')
        plt.ylabel('Force (N)')


            # plt.figure(19)
            # plt.plot(self.vicon_leg_t, self.vicon_leg_r[:,0], label="vicon-x")
            # plt.plot(self.vicon_leg_t, self.vicon_leg_r[:,1], label="vicon-y")
            # plt.plot(self.vicon_leg_t, self.vicon_leg_r[:,2], label="vicon-z")
            # plt.legend()
            # plt.xlabel('Time (s)')
            # plt.ylabel('Position (m)')
        #
        # plt.figure(20)
        # plt.plot(self.vicon_leg_t, self.vicon_leg_angles[:,0], label="vicon-r")
        # plt.plot(self.vicon_leg_t, self.vicon_leg_angles[:,1], label="vicon-p")
        # plt.plot(self.vicon_leg_t, self.vicon_leg_angles[:,2], label="vicon-y")
        # plt.legend()
        # plt.xlabel('Time (s)')
        # plt.ylabel('Angles (rad)')

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
        # plt.figure(9)
        # plt.plot(self.f_mag4_t, self.f_sensor4[:,0], label="f_x")
        # plt.plot(self.f_mag4_t, self.f_sensor4[:,1], label="f_y")
        # plt.plot(self.f_mag4_t, self.f_sensor4[:,2], label="f_z")
        # plt.plot(self.f_mag4_t, self.f_base4[:,0], label="b_x")
        # plt.plot(self.f_mag4_t, self.f_base4[:,1], label="b_y")
        # plt.plot(self.f_mag4_t, self.f_base4[:,2], label="b_z")
        # plt.legend()
        # plt.xlabel('Time (s)')
        # plt.ylabel('Foot 4 force (N)')
        #
        # plt.figure(10)
        # plt.plot(self.f_mag5_t, self.f_sensor5[:,0], label="f_x")
        # plt.plot(self.f_mag5_t, self.f_sensor5[:,1], label="f_y")
        # plt.plot(self.f_mag5_t, self.f_sensor5[:,2], label="f_z")
        # plt.plot(self.f_mag5_t, self.f_base5[:,0], label="b_x")
        # plt.plot(self.f_mag5_t, self.f_base5[:,1], label="b_y")
        # plt.plot(self.f_mag5_t, self.f_base5[:,2], label="b_z")
        # plt.legend()
        # plt.xlabel('Time (s)')
        # plt.ylabel('Foot 5 force (N)')

        if self.plot_residual:
            plt.figure(11)
            plt.plot(self.imu_ty, self.imu_y0[:,0], label="x")
            plt.plot(self.imu_ty, self.imu_y0[:,1], label="y")
            plt.plot(self.imu_ty, self.imu_y0[:,2], label="x")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot error 0 (m)')


            plt.figure(12)
            plt.plot(self.imu_ty, self.imu_y1[:,0], label="x")
            plt.plot(self.imu_ty, self.imu_y1[:,1], label="y")
            plt.plot(self.imu_ty, self.imu_y1[:,2], label="x")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot error 1 (m)')


            plt.figure(13)
            plt.plot(self.imu_ty, self.imu_y2[:,0], label="x")
            plt.plot(self.imu_ty, self.imu_y2[:,1], label="y")
            plt.plot(self.imu_ty, self.imu_y2[:,2], label="x")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot error 2 (m)')


            plt.figure(14)
            plt.plot(self.imu_ty, self.imu_y3[:,0], label="x")
            plt.plot(self.imu_ty, self.imu_y3[:,1], label="y")
            plt.plot(self.imu_ty, self.imu_y3[:,2], label="x")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot error 3 (m)')

            plt.figure(15)
            plt.plot(self.imu_ty, self.imu_y4[:,0], label="x")
            plt.plot(self.imu_ty, self.imu_y4[:,1], label="y")
            plt.plot(self.imu_ty, self.imu_y4[:,2], label="x")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot error 4 (m)')

            plt.figure(16)
            plt.plot(self.imu_ty, self.imu_y5[:,0], label="x")
            plt.plot(self.imu_ty, self.imu_y5[:,1], label="y")
            plt.plot(self.imu_ty, self.imu_y5[:,2], label="x")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot error 5 (m)')

        plt.show()
        self.power = 0

    def force_0_callback(self, msg, t):
        self.robot.cstate[0] = self.leg_state(msg)
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
        self.robot.cstate[1] = self.leg_state(msg)
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
        self.robot.cstate[2] = self.leg_state(msg)
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
        self.robot.cstate[3] = self.leg_state(msg)
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
        self.robot.cstate[4] = self.leg_state(msg)
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        f += self.f_sensor4_offset
        self.robot.cforce[12:15] = f
        f_mag = np.linalg.norm(f)
        self.f_mag4 = np.append(self.f_mag4, f_mag)
        self.f_mag4_t = np.append(self.f_mag4_t, t)

        # if self.robot.qc is not None:
        #     self.robot.update_state()
        # self.f_sensor4 = np.vstack([self.f_sensor4, self.robot.Leg[4].F6c.tibia_X_foot[:3].flatten()])
        # self.f_base4 = np.vstack([self.f_base4, self.robot.Leg[4].F6c.base_X_foot[:3].flatten()])

    def force_5_callback(self, msg, t):
        self.robot.cstate[5] = self.leg_state(msg)
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[15:18] = f
        f_mag = np.linalg.norm(f)
        self.f_mag5 = np.append(self.f_mag5, f_mag)
        self.f_mag5_t = np.append(self.f_mag5_t, t)

        # if self.robot.qc is not None:
        #     self.robot.update_state()
        # self.f_sensor5 = np.vstack([self.f_sensor5, self.robot.Leg[5].F6c.tibia_X_foot[:3].flatten()])
        # self.f_base5 = np.vstack([self.f_base5, self.robot.Leg[5].F6c.base_X_foot[:3].flatten()])

    def leg_state(self, msg):
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        return 1 if np.linalg.norm(f) > 5 else 0

    def joint_state_callback(self, msg, t):
        """ robot joint state callback """
        self.robot.qc = msg
        # if self.cal_i > self.cal_N:
        #     self.estimator.update_state()



    def contact_state_callback(self, msg, t):
        """ foot contact binary state """
        self.robot.cstate = msg.data

    def vicon_callback(self, msg, t):
        """ Corin position in Vicon frame """
        r = msg.transform.translation
        r = np.array([r.x, r.y, r.z])
        q = msg.transform.rotation
        q = np.array([q.w, q.x, q.y, q.z])

        if self.vicon_q0 is None:
            if self.q0 is not None:
                v_q0_conj = q.copy()
                v_q0_conj[1:4] = - v_q0_conj[1:4]
                self.vicon_q0 = quaternion_multiply(self.q0, v_q0_conj) # Hamilton
                #print quaternion_multiply_JPL(v_q0_conj, self.q0) # JPL should result in same quaternion
                # if self.vicon_r0 is None:
                self.vicon_r0 = r.copy()

                q2 = quaternion_multiply(self.vicon_q0, q)

                # print "q0"
                # print quaternion_matrix(self.q0)
                # print "q"
                # print quaternion_matrix(q)
                # print "q_conj"
                # print quaternion_matrix(v_q0_conj)
                # print "vicon_q0"
                # print quaternion_matrix(self.vicon_q0)
                # print "q2"
                # print quaternion_matrix(q2)

                # exit()

        else:
            r_v = r - self.vicon_r0 # Relative position in Vicon frame
            r_quat = np.insert(r_v, 0, 0)
            r = quaternion_matrix(self.vicon_q0)[0:3,0:3].dot(r_v) # Hamilton
            # print r
            # print quaternion_multiply(quaternion_multiply(self.vicon_q0, r_quat), quaternion_conjugate(self.vicon_q0))[1:4] # Hamilton
            # print quaternion_matrix_JPL(self.vicon_q0)[0:3,0:3].T.dot(r_v) # JPL
            # print quaternion_multiply_JPL(quaternion_multiply_JPL(quaternion_conjugate(self.vicon_q0), r_quat),self.vicon_q0)[1:4] # JPL
            self.vicon_r = np.vstack([self.vicon_r, r])
            self.vicon_t = np.append(self.vicon_t, t)

            q = quaternion_multiply(self.vicon_q0, q) # Hamilton
            angles_tuple = euler_from_quaternion_JPL(q, 'rxyz') # relative: rxyz
            angles = np.asarray(angles_tuple).tolist()
            # print "vic", q
            self.vicon_q = np.vstack([self.vicon_q, q])
            self.vicon_angles = np.vstack([self.vicon_angles, angles])


    def vicon_leg_callback(self, msg, t):
        """ Corin position in Vicon frame """
        r = msg.transform.translation
        r = np.array([r.x, r.y, r.z])
        q = msg.transform.rotation
        q = np.array([q.w, q.x, q.y, q.z])


        if self.vicon_p2_0 is None:
            if self.p2_0 is not None and self.vicon_q0 is not None:
                self.vicon_p2_0 = r + quaternion_matrix(q)[0:3, 0:3].dot(self.vicon_foot_offset)

        else:

            p2 = r + quaternion_matrix(q)[0:3, 0:3].dot(self.vicon_foot_offset)
            p2 -= self.vicon_p2_0 # Calculate position wrt first measurement
            p2 = quaternion_matrix(self.vicon_q0)[0:3, 0:3].dot(p2) # VICON to world frame
            p2 += self.p2_0 # Set the starting position to match that of the EKF

            self.vicon_leg_r = np.vstack([self.vicon_leg_r, p2])
            self.vicon_leg_t = np.append(self.vicon_leg_t, t)


        # if self.vicon_q0 is not None:
        #     r_v = r - self.vicon_r0 # Relative position in Vicon frame
        #     r_quat = np.insert(r_v, 0, 0)
        #     r = quaternion_matrix(self.vicon_q0)[0:3,0:3].dot(r_v) # Hamilton
        #     self.vicon_leg_r = np.vstack([self.vicon_leg_r, r])
        #     self.vicon_leg_t = np.append(self.vicon_leg_t, t)
        #
        #     q = quaternion_multiply(self.vicon_q0, q) # Hamilton
        #     angles_tuple = euler_from_quaternion_JPL(q, 'rxyz') # relative: rxyz
        #     angles = np.asarray(angles_tuple).tolist()
        #     # print "vic", q
        #     self.vicon_leg_angles = np.vstack([self.vicon_leg_angles, angles])


    def imu_callback(self, msg, t): 		# robot joint state callback

        self.robot.imu = msg

        a0 = msg.linear_acceleration
        a0 = np.array([a0.x, a0.y, a0.z])
        w = msg.angular_velocity
        w = np.array([w.x, w.y, w.z])
        o = msg.orientation
        o = np.array([o.w, o.x, o.y, o.z])
        if self.IMU == "LORD":
            # only for LORD IMU (to put the orientation from NED to the correct frame)
            o = quaternion_multiply(np.array([0, 1, 0, 0]), o)
            msg.orientation = Quaternion(o[1], o[2], o[3], o[0])
        # print "q0", o

        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "world"
        ps.pose.orientation = Quaternion(o[1], o[2], o[3], o[0])
        self.pub_pose.publish(ps)

        self.p = 0

        if(True and self.cal_i  < self.cal_N):
            self.cal_sum += o
            self.a0_sum += a0
            self.w_sum += w
            self.cal_i += 1
        elif(True and self.cal_i == self.cal_N):
            q_av = self.cal_sum / self.cal_N
            q_av = q_av / np.sqrt(q_av.dot(q_av)) # = self.q0
            # self.estimator.q = self.q0

            q0 = quaternion_from_matrix(self.estimator.IMU_R.T) # Rotates points from base to IMU
            self.estimator.q = quaternion_multiply(q_av, q0) # Rotate vector from base to IMU then from IMU to world
            # self.estimator.q = self.imu0

            # Set heading to zero
            temp = self.estimator.q
            angles_tuple = euler_from_quaternion(temp, 'sxyz')
            a = np.asarray(angles_tuple).tolist()
            self.estimator.q = quaternion_from_euler(a[0], a[1], 0, 'sxyz')

            self.q0 = self.estimator.q

            # Move on to state estimation only after foot positions are reset
            if self.estimator.reset_foot_positions():
                self.cal_i += 1

            self.p2_0 = self.estimator.p2

            # IMU offsets
            # print "a", msg.linear_acceleration
            # print "w", msg.angular_velocity

            g = np.array([0, 0, -9.80])
            C = quaternion_matrix_JPL(self.q0)[0:3, 0:3]
            a = self.estimator.IMU_R.dot(self.a0_sum / self.cal_N) # Transform to body frame
            ab = a + C.dot(g) # bias in body frame
            # print "ab:", ab

            f = a - ab
            a2 = C.T.dot(f) + g
            # print "a2:", a2 # this should be zero

            self.estimator.bf = ab

            C = quaternion_matrix_JPL(o)[0:3, 0:3]
            ab2 = a0 + C.dot(g)
            # print ab2
            # print self.estimator.IMU_R.dot(ab2)

            bw = self.estimator.IMU_R.dot(self.w_sum / self.cal_N)
            self.estimator.bw = bw

            # exit()

        else:
            # Set IMU orientation to "ideal" (VICON data)
            # if (self.ideal_q == True and self.vicon_q.size > 0):
            #     o = self.vicon_q[-1]
            #     # Can use this orientation directly in State Estimator
            #     msg.orientation = Quaternion(o[1], o[2], o[3], o[0])
            self.estimator.predict_state()

            # use VICON orientation as state estimate with 0 variance
            if (self.ideal_q == True and self.vicon_q.size > 0):
                self.estimator.q = self.vicon_q[-1]
                self.estimator.bw = np.zeros(3)

            self.update_i += 1
            if self.update_i >= self.update_N:
                self.estimator.update_state()
                self.update_i = 0

                if self.plot_residual:
                    self.imu_y0 = np.vstack([self.imu_y0, self.estimator.y[0:3]])
                    self.imu_y1 = np.vstack([self.imu_y1, self.estimator.y[3:6]])
                    self.imu_y2 = np.vstack([self.imu_y2, self.estimator.y[6:9]])
                    self.imu_y3 = np.vstack([self.imu_y3, self.estimator.y[9:12]])
                    self.imu_y4 = np.vstack([self.imu_y4, self.estimator.y[12:15]])
                    self.imu_y5 = np.vstack([self.imu_y5, self.estimator.y[15:18]])
                    self.imu_ty = np.append(self.imu_ty, t)




            # self.imu_r = np.append(self.imu_r, self.estimator.r, axis=0)
            self.imu_r = np.vstack([self.imu_r, self.estimator.r])
            self.imu_angles = np.vstack([self.imu_angles, self.estimator.get_fixed_angles()])
            # print "imu", self.estimator.q
            self.imu_t = np.append(self.imu_t, t)
            self.imu_o_angles = np.vstack([self.imu_o_angles, self.estimator.get_o_fixed_angles()])

            self.imu_p2 = np.vstack([self.imu_p2, self.estimator.p2])
            self.imu_bf = np.vstack([self.imu_bf, self.estimator.bf])
            self.imu_bw = np.vstack([self.imu_bw, self.estimator.bw])


    def imu_callback0(self, msg, t):
        o = msg.orientation
        av = msg.angular_velocity
        o = np.array([o.w, o.x, o.y, o.z])
        if self.IMU != "LORD":
            # only for LORD IMU (to put the orientation from NED to the correct frame)
            o = quaternion_multiply(np.array([0, 1, 0, 0]), o)

        angular_velocity = np.array([av.x, av.y, av.z])
        self.imu0 = o

        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "world"
        ps.pose.orientation = Quaternion(o[1], o[2], o[3], o[0])
        self.pub_pose2.publish(ps)

        if self.q1 is None:
            if self.q0 is not None:
                q1_conj = o.copy()
                q1_conj[1:4] = - q1_conj[1:4]
                self.q1 = quaternion_multiply(self.q0, q1_conj)
        # else:
        #     print "q1", quaternion_multiply(self.q1, o)


if __name__ == "__main__":

    state = CorinStateTester()

    "COMPLETE"
