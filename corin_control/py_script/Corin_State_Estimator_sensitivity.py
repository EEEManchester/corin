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
import csv
import matplotlib.pyplot as plt
from shutil import copyfile

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

import rosbag

np.set_printoptions(suppress=True) # suppress scientific notation
np.set_printoptions(precision=5) # number display precision
np.set_printoptions(formatter={'float': '{: 0.7f}'.format})
np.set_printoptions(linewidth=1000)

from experiment_numbers import *


class CorinStateTester:
    def __init__(self):

        if len(sys.argv) > 1:
            exp_no = int(sys.argv[1])
        else:
            exp_no = 80 #experiment number

        print 'experiment: ' + str(exp_no)

        csv_file = os.path.dirname(__file__) + '/2019_sensitivity_results_'+ str(exp_no) + '.csv'
        # copyfile(src, dst)
        src = os.path.dirname(__file__) + '/2018_Dec_optimisation_results_template_2.csv'

        if os.path.exists(csv_file):
            raise Exception("Destination file exists!")
        else:
            copyfile(src, csv_file)

        self.IMU = "LORD" # LORD / ODROID
        self.ideal_q = False # Use VICON orientation as estimate?
        self.ideal_r = False # Use VICON position as estimate?

        if exp_no in no_offset_list:
            self.f_sensor4_offset = np.array([0, 0, 0])
        else:
            self.f_sensor4_offset = np.array([3, 0, 4])

        bag = rosbag.Bag(experiment[exp_no])

        t0 = time.time()

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

        # Qf = [0.025**2, 0.0025**2, 0.00025**2, 0.000025**2]
        # Qbf = [0.1, 0.01, 0.001, 0.0001]
        # Qw = [0.008**2, 0.0008**2, 0.00008**2, 0.000008**2]
        # Qbw = [0.1, 0.01, 0.001, 0.0001]
        # Qp = [0.1**2, 0.01**2, 0.001**2, 0.0001**2]
        # Rs = [0.01**2, 0.001**2, 0.0001**2, 0.00001**2]
        # Ra = [0.1**2, 0.01**2, 0.001**2, 0.0001**2]

        # Qf = [0.0025**2] #[0.00025**2]
        # Qbf = [1E-8] #[0.001]
        # Qw = [0.00008**2]
        # Qbw = [1E-9] #[0.001]
        # Qp = [0.000001]  #[0.001**2]
        # Rs = [0.00001] #[0.0001**2]
        # Ra = [0.0001**1, 0.001**1, 0.0001**1]#0.00000001] #[0.0001**2]

        self.messages_to_read = 100000

        # Qf = [1E-1, 1E-2, 1E-3, 1E-4, 1E-5, 1E-6, 1E-7, 1E-8, 1E-9, 1E-10, 1E-11, 1E-12, 1E-13]
        # Qbf = [1E-1, 1E-2, 1E-3, 1E-4, 1E-5, 1E-6, 1E-7, 1E-8, 1E-9, 1E-10, 1E-11, 1E-12, 1E-13]
        # Qw = [1E-1, 1E-2, 1E-3, 1E-4, 1E-5, 1E-6, 1E-7, 1E-8, 1E-9, 1E-10, 1E-11, 1E-12, 1E-13]
        # Qbw = [1E-1, 1E-2, 1E-3, 1E-4, 1E-5, 1E-6, 1E-7, 1E-8, 1E-9, 1E-10, 1E-11, 1E-12, 1E-13]
        # Qp = [1E-3, 1E-4, 1E-5, 1E-6, 1E-7, 1E-8, 1E-9, 1E-10, 1E-11, 1E-12]
        # Rs = [1E-3, 1E-4, 1E-5, 1E-6, 1E-7, 1E-8, 1E-9, 1E-10]
        # Ra = [0.0004]

        # Qf = [7E-7, 7E-8, 7E-9]
        # Qbf = [3E-9, 3E-10, 3E-11]
        Qf = [1E-1, 1E-2, 1E-3, 1E-4, 1E-5, 1E-6, 1E-7, 1E-8, 1E-9, 1E-10, 1E-11, 1E-12]
        Qbf = [1E-4, 1E-5, 1E-6, 1E-7, 1E-8, 1E-9, 1E-10, 1E-11, 1E-12, 1E-13, 1E-14]
        Qw = [1E-1, 1E-2, 1E-3, 1E-4, 1E-5, 1E-6, 1E-7, 1E-8, 1E-9, 1E-10, 1E-11, 1E-12, 1E-13]
        Qbw = [1E-6, 1E-7, 1E-8, 1E-9, 1E-10, 1E-11, 1E-12, 1E-13, 1E-14, 1E-15, 1E-16]
        Qp = [1E-1, 1E-2, 1E-3, 1E-4, 1E-5, 1E-6, 1E-7, 1E-8, 1E-9, 1E-10, 1E-11]
        Rs = [1E-1, 1E-2, 1E-3, 1E-4, 1E-5, 1E-6, 1E-7, 1E-8, 1E-9]
        Ra = [3E-4]
        Th = range(1,11)

        Q0_list = [7E-7,
                    3E-30, #9
                    8E-8,
                    7E-30, #11
                    1E-6,
                    1E-5,
                    3E-4,
                    5]

        Q_list = [Qf, Qbf, Qw, Qbw, Qp, Rs, Ra, Th]

        total_len = 0
        for Q in Q_list:
            total_len += len(Q)

        # total_len = len(Qf) +
        #             len(Qbf) +
        #             len(Qw) +
        #             len(Qbw) +
        #             len(Qp) +
        #             len(Rs) +
        #             len(Ra)

        new_Q_list = []
        counter = 0
        for i in range(len(Q_list)):
            Q = Q_list[i]
            Q0 = Q0_list[i]
            temp = [Q0]*total_len
            temp[counter:counter+len(Q)] = Q
            new_Q_list.append(temp)
            counter += len(Q)

        # for i in new_Q_list:
        #     print i
        # exit()



        # Qp = [0.000001]
        # Rs = [0.000001]
        # Th = [3, 5, 8, 10]


        if (self.ideal_q == True):
            Qw = [0]
            Qbw = [0]

        if (self.ideal_r == True):
            Qf = [0]
            Qbf = [0]

        trial = 0
        #error_table = []

        for i in range (total_len):


            qf = new_Q_list[0][i]
            qbf = new_Q_list[1][i]
            qw = new_Q_list[2][i]
            qbw = new_Q_list[3][i]
            qp = new_Q_list[4][i]
            rs = new_Q_list[5][i]
            ra = new_Q_list[6][i]
            th = new_Q_list[7][i]

            trial += 1
            # print "trial:", trial
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
                                '/vicon/corin/corin': self.vicon_callback,#}#,
                                '/vicon/corin_leg/corin_leg': self.vicon_foot_callback,
                                '/vicon/Corin/Corin': self.vicon_callback,
                                '/vicon/Corin_leg/Corin_leg': self.vicon_foot_callback}

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
            for topic, msg, t in bag.read_messages():#topics=['/robotis/present_joint_states', '/imu/data']):
                # print t
                if topic in callback_dict:
                    callback_dict[topic](msg, t.to_time())
                    #self.counter += 1
                    if topic == '/vicon/corin/corin' or topic == '/vicon/Corin/Corin':
                        #if self.vicon_r.size > 0:
                        if self.vicon_r is not None:
                            error = self.vicon_r - self.estimator.r
                            error2 = self.vicon_angles - self.estimator.get_fixed_angles()
                            error2 = (error2 + np.pi) % (2*np.pi)  - np.pi

                            errors = np.vstack([errors, error])
                            error_norm = np.linalg.norm(error)
                            error_norms = np.append(error_norms, error_norm)
                            error_ratio = np.linalg.norm(error)/np.linalg.norm(self.vicon_r)
                            error_ratios = np.append(error_ratios, error_ratio)


                            errors2 = np.vstack([errors2, error2])
                            error_norm2 = np.linalg.norm(error2)
                            error_norms2 = np.append(error_norms2, error_norm2)
                            error_ratio2 = np.linalg.norm(error2)/abs(self.vicon_angles[2])#np.linalg.norm(self.vicon_angles)
                            error_ratios2 = np.append(error_ratios2, error_ratio2)
                # print t, topic
                i += 1
                if i > self.messages_to_read:
                    break

            print "messages read: ", i

            e_max_norm = error_norms.max()
            e_max_x = np.abs(errors[:,0]).max()
            e_max_y = np.abs(errors[:,1]).max()
            e_max_z = np.abs(errors[:,2]).max()
            e_rms_norm = np.sqrt(np.mean(np.square(error_norms)))
            e_rms_x = np.sqrt(np.mean(np.square(errors[:,0])))
            e_rms_y = np.sqrt(np.mean(np.square(errors[:,1])))
            e_rms_z = np.sqrt(np.mean(np.square(errors[:,2])))

            e_ratio = error_ratios[-1]

            print "max position error:", e_max_norm
            print "rms position error:", e_rms_norm

            # angle error
            e_max_norm2 = error_norms2.max()
            e_max_x2 = np.abs(errors2[:,0]).max()
            e_max_y2 = np.abs(errors2[:,1]).max()
            e_max_z2 = np.abs(errors2[:,2]).max()
            e_rms_norm2 = np.sqrt(np.mean(np.square(error_norms2)))
            e_rms_x2 = np.sqrt(np.mean(np.square(errors2[:,0])))
            e_rms_y2 = np.sqrt(np.mean(np.square(errors2[:,1])))
            e_rms_z2 = np.sqrt(np.mean(np.square(errors2[:,2])))

            e_ratio2 = error_ratios2[-1]

            print "max angle error:", e_max_norm2
            print "rms angle error:", e_rms_norm2

            error_row = []
            error_row.append(qf)
            error_row.append(qbf)
            error_row.append(qw)
            error_row.append(qbw)
            error_row.append(qp)
            error_row.append(rs)
            error_row.append(ra)
            error_row.append(th)
            error_row.append(e_max_norm)
            error_row.append(e_max_x)
            error_row.append(e_max_y)
            error_row.append(e_max_z)
            error_row.append(e_rms_norm)
            error_row.append(e_rms_x)
            error_row.append(e_rms_y)
            error_row.append(e_rms_z)
            error_row.append(e_ratio)
            error_row.append("")
            error_row.append(e_max_norm2)
            error_row.append(e_max_x2)
            error_row.append(e_max_y2)
            error_row.append(e_max_z2)
            error_row.append(e_rms_norm2)
            error_row.append(e_rms_x2)
            error_row.append(e_rms_y2)
            error_row.append(e_rms_z2)
            error_row.append(e_ratio2)

            #error_table.append(error_row)
            print "Time elsapsed: ", time.time()-t0
            print "Trial %d done in %.1f s" % (trial, time.time()-t_trial)

            with open(csv_file, 'ab') as myfile:
                wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
                wr.writerow(error_row)


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

        bag.close()
        # print error_table
        # with open('./optimisation_results.csv', 'wb') as myfile:
        #     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
        #     wr.writerows(error_table)

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
        o = msg.orientation
        av = msg.angular_velocity
        o = np.array([o.w, o.x, o.y, o.z])
        if self.IMU != "LORD":
            # only for LORD IMU (to put the orientation from NED to the correct frame)
            o = quaternion_multiply(np.array([0, 1, 0, 0]), o)

        angular_velocity = np.array([av.x, av.y, av.z])
        self.imu0 = o

        if self.q1 is None:
            if self.q0 is not None:
                q1_conj = o.copy()
                q1_conj[1:4] = - q1_conj[1:4]
                self.q1 = quaternion_multiply(self.q0, q1_conj)
        # else:
        #     print "q1", quaternion_multiply(self.q1, o)


    def vicon_foot_callback(self, msg, t):
        """ Corin position in Vicon frame """
        pass

if __name__ == "__main__":

    state = CorinStateTester()

    "COMPLETE"
