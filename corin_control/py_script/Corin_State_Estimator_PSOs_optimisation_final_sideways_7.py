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
import random
import string

## Personal libraries
from library import *			# library modules to include
from constant import *
import control_interface 								# action selection from ROS parameter server
import robot_class 										# class for robot states and function
import tf 												# SE(3) transformation library
import csv
import matplotlib.pyplot as plt
from shutil import copyfile
import multiprocess as mp
from joblib import Parallel, delayed

import copy



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

## Robotis ROS msgs for joint control
from robotis_controller_msgs.msg import SyncWriteItem 	# pub msg for Robotis joints
from robotis_controller_msgs.msg import SyncWriteMulti 	# pub msg for Robotis joints

from State_Estimator import StateEstimator

import rospkg
import rosbag

np.set_printoptions(suppress=True) # suppress scientific notation
np.set_printoptions(precision=5) # number display precision
np.set_printoptions(formatter={'float': '{: 0.7f}'.format})
np.set_printoptions(linewidth=1000)

from experiment_numbers import *


class CorinStateTester:
    def __init__(self, exp_no):

        # if len(sys.argv) > 1:
        #     exp_no = int(sys.argv[1])
        # else:
        #     exp_no = 80 #experiment number

        self.exp_no = exp_no

        print 'experiment: ' + str(exp_no)

        data_path = rospkg.RosPack().get_path('data')

        # csv_file = os.path.dirname(__file__) + '/2019_sensitivity_results_'+ str(exp_no) + '.csv'
        # # copyfile(src, dst)
        # src = os.path.dirname(__file__) + '/2019_sensitivity_results_template_3.csv'
        #
        # if os.path.exists(csv_file):
        #     raise Exception("Destination file exists!")
        # else:
        #     # copyfile(src, csv_file)
        #     pass

        self.IMU = "LORD" # LORD / ODROID
        self.initialise_state = True # use IMU reading to initialise orientation and IMU bias states
        self.ideal_q = False # Use VICON orientation as estimate?
        self.ideal_r = False # Use VICON position as estimate?

        if exp_no in no_offset_list:
            self.f_sensor4_offset = np.array([0, 0, 0])
        else:
            self.f_sensor4_offset = np.array([3, 0, 4])

        self.bag = rosbag.Bag(data_path + experiment[exp_no])

        # self.my_messages = []
        # for topic, msg, t, in self.bag.read_messages():
        #     self.my_messages.append((topic, msg, t))

        self.t0 = time.time()

        self.imu_r = np.empty((0,3))
        self.imu_angles = np.empty((0,3))
        self.imu_t = np.array([])

        self.vicon_r_offset = 0.001 * np.array([-160, 0, -28])
        self.vicon_foot_offset = 0.001 * np.array([-31.69, 0, -66])

        # self.robot 	= robot_class.RobotState()

        if self.IMU == "LORD":
            self.T_imu = 0.002
            self.update_N = 10
        else: # ODROID
            self.T_imu = 0.01
            self.update_N = 2

        self.messages_to_read = 100000

        if (self.ideal_q == True):
            Qw = [0]
            Qbw = [0]

        if (self.ideal_r == True):
            Qf = [0]
            Qbf = [0]

    def evaluate(self,position):

        qf = position[0]
        qbf = position[1]
        qw = position[2]
        qbw = position[3]
        qp = position[4]
        rs = position[5]
        ra = 3E-4
        th = position[6]

        p_r  = 1E-6
        p_v  = 1E-6
        p_q  = 1E-6
        p_p  = 1E-6
        p_bf = position[7]#1E-6
        p_bw = position[8]#1E-7

        Q0_list = [1E-7,
                    1E-10, #9
                    1E-8,
                    1E-11, #11
                    1E-6,
                    1E-4,
                    3E-4,
                    3,
                    1E-6,
                    1E-6,
                    1E-6,
                    1E-6,
                    1E-6,
                    1E-6]

        t_trial = time.time()

        self.q0 = None
        self.q1 = None
        self.imu0 = None

        self.vicon_r = None
        self.vicon_q = None
        self.vicon_angles = None
        self.vicon_t = np.array([])
        self.vicon_r0 = None
        self.vicon_q0 = None
        self.update_i = 0
        self.cal_N = 100
        self.cal_sum = np.zeros(4)
        self.a0_sum = np.zeros(3)
        self.w_sum = np.zeros(3)
        self.angles_array = np.empty((0,3))
        self.a0_array = np.empty((0,3))
        self.w_array = np.empty((0,3))

        self.cal_i = 0.0

        self.robot 	= robot_class.RobotState()
        self.estimator = StateEstimator(self.robot, self.T_imu)
        self.estimator.Qf = qf*np.eye(3) # Noise power density (m/s2 / sqrt(Hz))
        self.estimator.Qbf = qbf*np.eye(3)
        self.estimator.Qw = qw*np.eye(3)
        self.estimator.Qbw = qbw*np.eye(3)
        self.estimator.Qp = qp*np.eye(3)
        self.estimator.Rs = rs*np.eye(3)
        self.estimator.Ra = ra*np.eye(3)
        self.estimator.force_threshold = th
        self.P_r = p_r
        self.P_v = p_v
        self.P_q = p_q
        self.P_p = p_p
        self.P_bf = p_bf
        self.P_bw = p_bw

        # callback_dict = {   '/imu_LORD/data': self.imu_callback0,
        #                     '/imu/data': self.imu_callback,
        #                     '/robotis/present_joint_states': self.joint_state_callback,
        #                     '/force_vector_0': self.force_0_callback,
        #                     '/force_vector_1': self.force_1_callback,
        #                     '/force_vector_2': self.force_2_callback,
        #                     '/force_vector_3': self.force_3_callback,
        #                     '/force_vector_4': self.force_4_callback,
        #                     '/force_vector_5': self.force_5_callback,
        #                     '/vicon/corin/corin': self.vicon_callback,
        #                     '/vicon/Corin/Corin': self.vicon_callback}
        callback_dict = {   '/imu_LORD/data': self.imu_callback0,
                            '/imu/data': self.imu_callback,
                            '/robotis/present_joint_states': self.joint_state_callback,
                            '/force_vector_0': self.force_0_callback,
                            '/force_vector_1': self.force_1_callback,
                            '/force_vector_2': self.force_2_callback,
                            '/force_vector_3': self.force_3_callback,
                            '/force_vector_4': self.force_4_callback,
                            '/force_vector_5': self.force_5_callback,
                            '/vicon/corin/corin': self.vicon_callback,
                            '/vicon/Corin/Corin': self.vicon_callback}

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
        errors2 = np.empty((0,3))
        error_norms2 =  np.array([])
        error_ratios2 =  np.array([])

        i = 0
        for topic, msg, t in self.bag.read_messages():#topics=['/robotis/present_joint_states', '/imu/data']):
        # for topic, msg, t in self.my_messages:
            # print t
            if topic in callback_dict:
                callback_dict[topic](msg, t.to_time())
                #self.counter += 1
                if topic == '/vicon/corin/corin' or topic == '/vicon/Corin/Corin':
                    #if self.vicon_r.size > 0:
                    if self.vicon_r is not None:
                        error = self.vicon_r - self.estimator.r
                        # error2 = self.vicon_angles - self.estimator.get_fixed_angles()
                        # error2 = (error2 + np.pi) % (2*np.pi)  - np.pi

                        # errors = np.vstack([errors, error])
                        # error_norm = np.linalg.norm(error)
                        # error_norms = np.append(error_norms, error_norm)
                        error_ratio = np.linalg.norm(error)/np.linalg.norm(self.vicon_r)
                        error_ratios = np.append(error_ratios, error_ratio)


                        # errors2 = np.vstack([errors2, error2])
                        # error_norm2 = np.linalg.norm(error2)
                        # error_norms2 = np.append(error_norms2, error_norm2)
                        # error_ratio2 = np.linalg.norm(error2)/abs(self.vicon_angles[2])#np.linalg.norm(self.vicon_angles)
                        # error_ratios2 = np.append(error_ratios2, error_ratio2)
            # print t, topic
            i += 1
            if i > self.messages_to_read:
                break

        # print "messages read: ", i

        # e_max_norm = error_norms.max()
        # e_max_x = np.abs(errors[:,0]).max()
        # e_max_y = np.abs(errors[:,1]).max()
        # e_max_z = np.abs(errors[:,2]).max()
        # e_rms_norm = np.sqrt(np.mean(np.square(error_norms)))
        # e_rms_x = np.sqrt(np.mean(np.square(errors[:,0])))
        # e_rms_y = np.sqrt(np.mean(np.square(errors[:,1])))
        # e_rms_z = np.sqrt(np.mean(np.square(errors[:,2])))
        #
        e_ratio = error_ratios[-1]

        # print "max position error:", e_max_norm
        # print "rms position error:", e_rms_norm


        # angle error
        # e_max_norm2 = error_norms2.max()
        # e_max_x2 = np.abs(errors2[:,0]).max()
        # e_max_y2 = np.abs(errors2[:,1]).max()
        # e_max_z2 = np.abs(errors2[:,2]).max()
        # e_rms_norm2 = np.sqrt(np.mean(np.square(error_norms2)))
        # e_rms_x2 = np.sqrt(np.mean(np.square(errors2[:,0])))
        # e_rms_y2 = np.sqrt(np.mean(np.square(errors2[:,1])))
        # e_rms_z2 = np.sqrt(np.mean(np.square(errors2[:,2])))
        #
        # e_ratio2 = error_ratios2[-1]

        # print "max angle error:", e_max_norm2
        # print "rms angle error:", e_rms_norm2

        # error_row = []
        # error_row.append(qf)
        # error_row.append(qbf)
        # error_row.append(qw)
        # error_row.append(qbw)
        # error_row.append(qp)
        # error_row.append(rs)
        # error_row.append(ra)
        # error_row.append(th)
        # error_row.append(p_r)
        # error_row.append(p_v)
        # error_row.append(p_q)
        # error_row.append(p_p)
        # error_row.append(p_bf)
        # error_row.append(p_bw)
        # error_row.append(e_max_norm)
        # error_row.append(e_max_x)
        # error_row.append(e_max_y)
        # error_row.append(e_max_z)
        # error_row.append(e_rms_norm)
        # error_row.append(e_rms_x)
        # error_row.append(e_rms_y)
        # error_row.append(e_rms_z)
        # error_row.append(e_ratio)
        # error_row.append("")
        # error_row.append(e_max_norm2)
        # error_row.append(e_max_x2)
        # error_row.append(e_max_y2)
        # error_row.append(e_max_z2)
        # error_row.append(e_rms_norm2)
        # error_row.append(e_rms_x2)
        # error_row.append(e_rms_y2)
        # error_row.append(e_rms_z2)
        # error_row.append(e_ratio2)

        #error_table.append(error_row)
        # print "Time elsapsed: ", time.time()-self.t0
        # print "Trial done in %.1f s" % (time.time()-t_trial)

        return e_ratio #e_rms_norm

        # with open(csv_file, 'ab') as myfile:
        #     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
        #     wr.writerow(error_row)


        # plt.figure(1)
        # plt.plot(self.imu_t, self.imu_r[:,0], label="imu-x")
        # plt.plot(self.imu_t, self.imu_r[:,1], label="imu-y")
        # plt.plot(self.imu_t, self.imu_r[:,2], label="imu-z")
        # plt.plot(self.vicon_t, self.vicon_r[:,0], label="vicon-x")
        # plt.plot(self.vicon_t, self.vicon_r[:,1], label="vicon-y")
        # plt.plot(self.vicon_t, self.vicon_r[:,2], label="vicon-z")
        # plt.legend()
        # plt.xlabel('Time (s)')
        # plt.ylabel('Position (m)')
        #
        # plt.figure(2)
        # plt.plot(self.imu_t, self.imu_angles[:,0], label="imu-r")
        # plt.plot(self.imu_t, self.imu_angles[:,1], label="imu-p")
        # plt.plot(self.imu_t, self.imu_angles[:,2], label="imu-y")
        # plt.plot(self.vicon_t, self.vicon_angles[:,0], label="vicon-r")
        # plt.plot(self.vicon_t, self.vicon_angles[:,1], label="vicon-p")
        # plt.plot(self.vicon_t, self.vicon_angles[:,2], label="vicon-y")
        # plt.legend()
        # plt.xlabel('Time (s)')
        # plt.ylabel('Angles (rad)')
        #
        # plt.show()

        self.power = 0
        # print error_table
        # with open('./optimisation_results.csv', 'wb') as myfile:
        #     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
        #     wr.writerows(error_table)

    def terminate(self):
        self.bag.close()

    def force_0_callback(self, msg, t):
        self.robot.cstate[0] = self.foot_state(msg)
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[0:3] = f

    def force_1_callback(self, msg, t):
        self.robot.cstate[1] = self.foot_state(msg)
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[3:6] = f

    def force_2_callback(self, msg, t):
        self.robot.cstate[2] = self.foot_state(msg)
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[6:9] = f

    def force_3_callback(self, msg, t):
        self.robot.cstate[3] = self.foot_state(msg)
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[9:12] = f

    def force_4_callback(self, msg, t):
        self.robot.cstate[4] = self.foot_state(msg)
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        f = f + self.f_sensor4_offset
        self.robot.cforce[12:15] = f

    def force_5_callback(self, msg, t):
        self.robot.cstate[5] = self.foot_state(msg)
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[15:18] = f

    def foot_state(self, msg):
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        return 1 if np.linalg.norm(f) > 5 else 0

    def joint_state_callback(self, msg, t):
        """ robot joint state callback """
        self.robot.qc = msg

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
                self.vicon_q0 = quaternion_multiply(self.q0, v_q0_conj)
                self.vicon_r0 = r + quaternion_matrix(q)[0:3, 0:3].dot(self.vicon_r_offset)
                q2 = quaternion_multiply(self.vicon_q0, q)

        else:
            r_v = r + quaternion_matrix(q)[0:3, 0:3].dot(self.vicon_r_offset)
            r_v = r_v - self.vicon_r0
            r = quaternion_matrix(self.vicon_q0)[0:3,0:3].dot(r_v)
            self.vicon_r = r.copy() #np.vstack([self.vicon_r, r])
            # self.vicon_t = np.append(self.vicon_t, t)

            q = quaternion_multiply(self.vicon_q0, q)
            angles_tuple = euler_from_quaternion_JPL(q, 'rxyz') # relative: rxyz
            angles = np.asarray(angles_tuple).tolist()
            self.vicon_angles = angles
            self.vicon_q = quaternion_multiply(self.vicon_q0, q) # Hamilton

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

        self.p = 0

        if(True and self.cal_i  < self.cal_N):
            self.cal_sum += o
            self.a0_sum += a0
            self.w_sum += w
            self.cal_i += 1

            angles_tuple = euler_from_quaternion_JPL(o, 'rxyz') # relative: rxyz
            self.angles_array = np.vstack([self.angles_array, np.asarray(angles_tuple)])

            self.w_array = np.vstack([self.w_array, w])
            self.a0_array = np.vstack([self.a0_array, a0])

        elif(True and self.cal_i == self.cal_N):
            q_av = self.cal_sum / self.cal_N
            q_av = q_av / np.sqrt(q_av.dot(q_av)) # = self.q0
            # self.estimator.q = self.q0

            q0 = quaternion_from_matrix(self.estimator.IMU_R.T) # Rotates points from base to IMU
            self.estimator.q = quaternion_multiply(q_av, q0) # Rotate vector from base to IMU then from IMU to world

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

            C = quaternion_matrix_JPL(o)[0:3, 0:3]
            ab2 = a0 + C.dot(g)
            # print ab2
            # print self.estimator.IMU_R.dot(ab2)

            bw = self.estimator.IMU_R.dot(self.w_sum / self.cal_N)
            #

            self.estimator.P = 1E-6 * np.eye(33)
            self.estimator.P[0:3, 0:3] = self.P_r*np.eye(3)
            self.estimator.P[3:6, 3:6] = self.P_v*np.eye(3)
            self.estimator.P[6:9, 6:9] = self.P_q*np.eye(3) #1 deg = 0.01 rad **2 (100 iter)
            self.estimator.P[9:27, 9:27] = self.P_p*np.eye(18) # all feet
            self.estimator.P[27:30, 27:30] = self.P_bf*np.eye(3)
            self.estimator.P[30:33, 30:33] = self.P_bw*np.eye(3)

            if self.initialise_state:
                # initialise IMU bias
                self.estimator.bf = ab
                self.estimator.bw = bw
            else:
                self.estimator.q = np.array([1, 0, 0, 0])
                self.estimator.P[8, 8] = 1E-6 # we known heading = 0

            # Evalute P matrix
            # print "test"
            self.estimator.P[0:3, 0:3] = 1E-10*np.eye(3)
            self.estimator.P[3:6, 3:6] = 1E-10*np.eye(3)
            #
            self.estimator.P[6:9, 6:9] = np.diag( np.std(self.angles_array, axis=0) ** 2 )
            self.estimator.P[8, 8] = 1E-10

            for j in range(0,self.robot.active_legs):
                J = self.robot.Leg[j].XHc.j_base_X_foot # jacobian
                Rs = self.estimator.Rs; Ra = self.estimator.Ra
                P_pi =  C.T.dot(Ra + J.dot(Ra).dot(J.T) ).dot(C)
                i = 9+j*3
                k = 9+(j+1)*3
                self.estimator.P[i:k, i:k] = P_pi

            self.estimator.P[27:30, 27:30] = np.diag( np.std(self.a0_array, axis=0) ** 2  )
            self.estimator.P[30:33, 30:33] = np.diag( np.std(self.w_array, axis=0) ** 2  )


        else:
            self.estimator.predict_state()

            self.update_i += 1
            if self.update_i >= self.update_N:
                self.estimator.update_state()
                self.update_i = 0

            # use VICON orientation as state estimate with 0 variance
            if (self.ideal_q == True and self.vicon_q is not None):
                self.estimator.q = self.vicon_q.copy()
                self.estimator.bw = np.zeros(3)

            if (self.ideal_r == True and self.vicon_r is not None):
                self.estimator.r = self.vicon_r.copy()
                self.estimator.bf = np.zeros(3)


            # self.imu_r = np.append(self.imu_r, self.estimator.r, axis=0)
            # self.imu_r = np.vstack([self.imu_r, self.estimator.r])
            # self.imu_angles = np.vstack([self.imu_angles, self.estimator.get_fixed_angles()])
            # self.imu_t = np.append(self.imu_t, t)

    def imu_callback0(self, msg, t):
        pass
        # o = msg.orientation
        # av = msg.angular_velocity
        # o = np.array([o.w, o.x, o.y, o.z])
        # if self.IMU != "LORD":
        #     # only for LORD IMU (to put the orientation from NED to the correct frame)
        #     o = quaternion_multiply(np.array([0, 1, 0, 0]), o)
        #
        # angular_velocity = np.array([av.x, av.y, av.z])
        # self.imu0 = o
        #
        # if self.q1 is None:
        #     if self.q0 is not None:
        #         q1_conj = o.copy()
        #         q1_conj[1:4] = - q1_conj[1:4]
        #         self.q1 = quaternion_multiply(self.q0, q1_conj)
        # else:
        #     print "q1", quaternion_multiply(self.q1, o)


    def vicon_foot_callback(self, msg, t):
        """ Corin position in Vicon frame """
        pass

class Particle():

    # Class variables (static)
    Qf_lim = [-10, -1]
    Qbf_lim = [-10, -1]
    Qw_lim = [-11, -1]
    Qbw_lim = [-15, -1]
    Qp_lim = [-10, -1]
    Rs_lim = [-10, -1]
    # Ra = []
    Th_lim = [0, 10]
    Pba_lim = [-10, -1]
    Pbw_lim = [-10, -1]

    Q_lims = [Qf_lim, Qbf_lim, Qw_lim, Qbw_lim, Qp_lim, Rs_lim, Th_lim, Pba_lim, Pbw_lim]
    N = len(Q_lims) # Dimensions / number of parameters
    vel_lims = [-0.5, 0.5]

    bound_position = True
    w = 0.5
    c1 = 2
    c2 = 2
    # Speed clamping coefficient
    k = 0.2

    def __init__(self, value_len):
        self.position = np.array([])
        self.velocity = np.array([])
        for i in range(Particle.N):
            p_min = Particle.Q_lims[i][0]
            p_max = Particle.Q_lims[i][1]
            v_max_mag = Particle.k*(p_max - p_min)
            self.position = np.append(self.position, random.uniform(p_min, p_max))
            self.velocity = np.append(self.velocity, random.uniform(-v_max_mag, v_max_mag))
            # self.velocity = np.append(self.velocity, random.uniform(Particle.vel_lims[0], Particle.vel_lims[1]))
        self.best_position = self.position.copy()
        self.best_value = ParticleValue([1000] * value_len)  # very high value

    def __str__(self):
        print("I am at ", self.position, " meu pbest is ", self.pbest_position)

    def evaluate(self, new_value):
        # check to see if the current position is an individual best
        if new_value < self.best_value:
            self.best_position = self.position.copy()
            self.best_value = copy.deepcopy(new_value)

    def move(self, swarm_best_position):
        gb = swarm_best_position
        for i in range(0,Particle.N):
            p_min = Particle.Q_lims[i][0]
            p_max = Particle.Q_lims[i][1]
            v_max_mag = Particle.k*(p_max - p_min)
            r1 = random.random()
            r2 = random.random()
            vel_cognitive = Particle.c1*r1 * (self.best_position[i] - self.position[i])
            vel_social = Particle.c2*r2 * (gb[i] - self.position[i])
            self.velocity[i] = Particle.w*self.velocity[i] + vel_cognitive + vel_social
            # Clamp speed
            self.velocity[i] = max(-v_max_mag, min(v_max_mag, self.velocity[i]))
            new_position = self.position[i] + self.velocity[i]
            if Particle.bound_position:
                # Update position but bound between limits
                p_min = Particle.Q_lims[i][0]
                p_max = Particle.Q_lims[i][1]
                self.position[i] = max(p_min, min(p_max, new_position))
                # If position has hit bounds, reverse velocity
                if self.position[i] != new_position:
                    self.velocity[i] = - self.velocity[i]
            else:
                self.position[i] = new_position

class ParticleValue:

    def __init__(self, errors):
        self.errors = copy.copy(errors)

    # Override less than operator
    def __lt__(self,other):
        self.validate_comparison(other)
        self.errors.sort(reverse=True)
        other.errors.sort(reverse=True)

        for i, j in zip(self.errors, other.errors):
            i_round = round(i*1000) # round to nearest percent (error is a ratio)
            j_round = round(j*1000)
            if i_round < j_round:
                return True
            elif i_round > j_round:
                return False

        return False


    def validate_comparison (self,other):
        if len(self) != len(other):
            raise Exception("Compared error lists are not the same length.")

    def __len__(self):
        return len(self.errors)

    def __str__(self):
        return str(self.errors)

# Some function with a minimum at ideal
def func(position):
    value = 0
    ideal = [-5]*6 + [5]
    for i in range(position.shape[0]):
        value += abs(position[i] - ideal[i]) # reduce value for closeness to -5
    # exit()
    return value

def position_to_cov(p):
    # Change to covariance values because swarm positions are exponents
    amended = np.zeros(len(p))
    amended[0] = 10**p[0] # Qf
    amended[1] = 10**p[1] # Qbf
    amended[2] = 10**p[2] # Qw
    amended[3] = 10**p[3] # Qbw
    amended[4] = 10**p[4] # Qp
    amended[5] = 10**p[5] # Rs
    amended[6] = p[6] # Th
    amended[7] = 10**p[7] # Pbf
    amended[8] = 10**p[8] # Pbw
    return amended


# state_list = [None]*100

def run_estimator(exp_pars):
    exp, pars = exp_pars
    state = CorinStateTester(exp)
    error = state.evaluate(pars)
    state.terminate()
    return error

# def init_estimators(exp_list):
#     global state_list
#     for i in exp_list:
#         state_list[i] = CorinStateTester(i)
#     print state_list
#
# def run_estimator(exp_pars):
#     state, pars = exp_pars
#     error = state.evaluate(pars)
#     return error
#
# def terminate_estimators():
#     global state_list
#     for state in state_list:
#         state.terminate()

if __name__ == "__main__":

    # results1 = [1, 2, 6, 7, 1.5]
    # results2 = [1, 3, 6, 7, 3]
    #
    # pvalue1 = ParticleValue(results1)
    # pvalue2 = ParticleValue(results2)
    #
    # if pvalue1 < pvalue2:
    #     print "pvalue1 is smaller than pvalue2"
    #
    # print pvalue1
    #
    # exit()

    # print mp.cpu_count()



    # pars = np.array([   1E-6,
    #                     1E-6,
    #                     1E-7,
    #                     1E-8,
    #                     1E-5,
    #                     1E-3,
    #                     3])
    # exp_list = [30, 50]
    # pars_list = [pars]*len(exp_list)
    # exp_pars = zip(exp_list, pars_list)
    #
    # pool = mp.Pool(2)
    # results = pool.map(run_estimator, exp_pars)
    # print results
    # exit()

    exp_list = [40, 41, 42, 43, 45, 46, 47, 48, 49, 51]
    exp_no = len(exp_list) # number of experiments

    pool = mp.Pool(11)

    csv_data_path = rospkg.RosPack().get_path('data') + "/PSO/PSO_final_sideways_6.csv"

    # init_estimators(exp_list)

    iterations = 45
    particles = 70
    swarm_best_value = ParticleValue([1000] * exp_no)
    w_max = 0.9
    w_min = 0.4
    swarm_best_position = Particle(exp_no).position


    # list of particles
    swarm = []
    for j in range(particles):
        swarm.append(Particle(exp_no))

    best_positions = np.empty((0,swarm_best_position.shape[0]))
    particle_positions = [np.empty((0,swarm_best_position.shape[0]))]*particles
    best_values = np.array([]) #  for display only
    for i in range(iterations):
        print "iteration", i+1, " of ", iterations
        Particle.w = w_max - (w_max-w_min)*i/iterations
        for j in range(particles):
            t0 = time.time()
            print "particle", j+1, " of ", particles
            print "swarm best position: ", swarm_best_position, "(" + str(swarm_best_value) + ")"

            # new_value = state.evaluate(swarm[j].position)
            amended = position_to_cov(swarm[j].position)
            # new_value = state.evaluate(amended)

            pars_list = [amended]*len(exp_list)
            exp_pars = zip(exp_list, pars_list)
            # estimator_list = filter(None, state_list)

            # exp_pars = zip(estimator_list, pars_list)
            results = pool.map(run_estimator, exp_pars)
            # results = Parallel(n_jobs=2)(delayed(run_estimator)(i) for i in exp_pars)
            # print results
            # print "max:", max(results)
            # print "sum:", sum(results)
            # print "rms:", np.sqrt(np.mean(np.square(results)))
            #new_value = np.sqrt(np.mean(np.square(results)))#max(results)
            new_value = ParticleValue(results)
            swarm[j].evaluate(new_value)

            if swarm[j].best_value < swarm_best_value:
                swarm_best_position = swarm[j].best_position.copy()
                swarm_best_value = copy.deepcopy(swarm[j].best_value)

            swarm[j].move(swarm_best_position)

            particle_positions[j] = np.vstack([particle_positions[j], swarm[j].position])

            print "Particle value evaluated in ", time.time()-t0

        data_row = list(swarm_best_position) + swarm_best_value.errors
        with open(csv_data_path, 'ab') as myfile:
            wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
            wr.writerow(data_row)



        best_values = np.append(best_values, max(swarm_best_value.errors))
        best_positions = np.vstack([best_positions, swarm_best_position])

    plt.figure()
    plt.title(": Best global position")
    plt.plot(best_positions[:,0], label="qf")
    plt.plot(best_positions[:,1], label="qbf")
    plt.plot(best_positions[:,2], label="qw")
    plt.plot(best_positions[:,3], label="qbw")
    plt.plot(best_positions[:,4], label="qp")
    plt.plot(best_positions[:,5], label="rs")
    plt.plot(best_positions[:,6], label="Th")
    plt.plot(best_positions[:,7], label="Pbw")
    plt.legend()
    plt.xlabel('Iteration')
    plt.ylabel('Parameter value')

    plt.figure()
    plt.title(": Best global value")
    plt.plot(best_values)
    plt.xlabel('Iteration')
    plt.ylabel('Value')

    # for j in range(particles):
    #     plt.figure()
    #     plt.title("Particle " + str(j) + " position")
    #     plt.plot(particle_positions[j][:,0], label="qf")
    #     plt.plot(particle_positions[j][:,1], label="qbf")
    #     plt.plot(particle_positions[j][:,2], label="qw")
    #     plt.plot(particle_positions[j][:,3], label="qbw")
    #     plt.plot(particle_positions[j][:,4], label="qp")
    #     plt.plot(particle_positions[j][:,5], label="rs")
    #     plt.plot(particle_positions[j][:,6], label="Th")
    #     plt.legend()
    #     plt.xlabel('Iteration')
    #     plt.ylabel('Parameter value')

    # plt.show()

    print swarm_best_position, swarm_best_value

    np_arr = np.hstack((np.array([best_values]).T, best_positions))
    arr = np.asarray(np_arr)
    # arr = ["Max error", "qf", "qbf", "qw", "qbw", "qp", "rs", "Th"]
    rand = ''.join(random.choice(string.ascii_uppercase + string.digits) for _ in range(6))
    data_path = rospkg.RosPack().get_path('data') + "/PSO/PSO_" + rand + ".csv"
    np.savetxt(data_path, arr, delimiter=",")
    # print state.evaluate(position_to_cov(swarm_best_position))
    # state.terminate()
    # terminate_estimators()
    # print "Experiment: ", state.exp_no
    "COMPLETE"




































#
