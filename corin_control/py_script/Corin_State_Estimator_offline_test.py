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
from termcolor import colored

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

import rospkg
import rosbag

np.set_printoptions(suppress=True) # suppress scientific notation
np.set_printoptions(precision=10) # number display precision
np.set_printoptions(formatter={'float': '{: 0.7f}'.format})
np.set_printoptions(linewidth=1000)

from experiment_numbers import *

class CorinStateTester:
    def __init__(self):
        #---------------------------------------------------------------------#
        # Analysis parameters                                                 #
        #---------------------------------------------------------------------#
        self.IMU = "LORD" # LORD / ODROID
        self.initialise_state = True # use IMU reading to initialise orientation and IMU bias states
        self.ideal_q = False # Use VICON orientation as estimate?
        self.ideal_r = False # Use VICON position as estimate?
        self.PSO = False # Use optimal values obtained from PSO
        self.position_error = False # Plot position error or orientation error
        self.integrate_total = False # Integrate VICON position to travelled distance
        self.messages_to_read = 10000000
        self.max_angle = 2*np.pi
        self.max_distance = 0.5
        # What to plot?!
        self.plot_position = False #1
        self.plot_position_stddev = False #2
        self.plot_position_error = False #5

        self.plot_angles = False #3
        self.plot_angles_stddev = False #4
        self.plot_angles_error = False #5

        self.plot_foot_position = False #7
        self.plot_p2_stddev = False #7
        self.plot_IMU_bias = False #6
        self.plot_residual = False
        self.plot_forces = False # 8
        self.plot_foot0_force = False
        self.plot_foot1_force = False
        self.plot_foot2_force = False
        self.plot_foot3_force = False
        self.plot_foot4_force = False
        self.plot_foot5_force = False

        self.saveFigure = False
        self.fig_path = "./figures2/"
        self.saveData = False
        #---------------------------------------------------------------------#
        # Tuning parameters                                                   #
        #---------------------------------------------------------------------#
        self.Qf = 3E-7#10**-7.02#1E-6         # (0.00025**2)* # Noise power density (m/s2 / sqrt(Hz))
        self.Qbf = 1E-7#10**-7.15#1E-6       # 9

        self.Qw = 4E-6#10**-5.8#-5.6#1E-7
        self.Qbw = 2E-11#10**-13.3#-2.2#1E-8          # 10

        self.Qp = 3E-6#10**-6.76#-4.3#1E-5
        self.Qp_xy = 1E-5
        self.Qp_z = 1E-5

        self.Rs = 2E-3 #10**-3.356#1E-3
        self.Ra = 3E-4#3E-4 # 0.0003

        self.force_threshold = 2.7#2.9#3.5 #3

        self.P_r  = 1E-6
        self.P_v  = 1E-6
        self.P_q  = 1E-6
        self.P_p  = 1E-6
        self.P_bf = 1E-6
        self.P_bw = 1E-6 # 7
        #---------------------------------------------------------------------#
        #---------------------------------------------------------------------#
        if len(sys.argv) > 1:
            exp_no = int(sys.argv[1])
        else:
            exp_no = 60 #experiment number

        print 'experiment: ' + str(exp_no)
        # rospy.init_node('Corin_estimator_'+str(exp_no))

        # optimal PSO values defined in experiments.py
        if self.PSO:
            (self.Qf,
            self.Qbf,
            self.Qw,
            self.Qbw,
            self.Qp,
            self.Rs,
            self.force_threshold) = \
            tuple(optimal[exp_no])
            self.P_bw = 1E-6

        print colored('Qf, Qbf:', 'red'), self.Qf, self.Qbf
        print colored('Qw, Qbw:', 'red'), self.Qw, self.Qbw
        # print colored('Qp_xy, Qp_z:', 'red'), self.Qp_xy, self.Qp_z
        print colored('Qp:', 'red'), self.Qp
        print colored('Rs, Ra:', 'red'), self.Rs, self.Ra
        print "force threshold:", self.force_threshold



        # max_messages defined in experiments.py
        self.messages_to_read = min(self.messages_to_read, max_messages[exp_no])

        # no_offset_list defined in experiments.py
        if exp_no in no_offset_list:
            self.f_sensor4_offset = np.array([0, 0, 0])
        else:
            self.f_sensor4_offset = np.array([3, 0, 4])

        data_path = rospkg.RosPack().get_path('data')

        # experiment defined in experiments.py
        bag = rosbag.Bag(data_path + experiment[exp_no])

        self.pub_pose = rospy.Publisher('/IMU_pose', PoseStamped, queue_size=1)
        self.pub_pose2 = rospy.Publisher('/IMU_pose2', PoseStamped, queue_size=1)
        t0 = time.time()
        self.imu_r = np.empty((0,3))
        self.imu_r_stddev = np.empty((0,3))
        self.imu_p2 = np.empty((0,3))
        self.imu_p2_stddev = np.empty((0,3))
        self.imu_bf = np.empty((0,3))
        self.imu_bw = np.empty((0,3))
        self.imu_y0 = np.empty((0,3))
        self.imu_y1 = np.empty((0,3))
        self.imu_y2 = np.empty((0,3))
        self.imu_y3 = np.empty((0,3))
        self.imu_y4 = np.empty((0,3))
        self.imu_y5 = np.empty((0,3))


        self.imu_angles = np.empty((0,3))
        self.imu_angles_stddev = np.empty((0,3))
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
        self.vicon_r_last = None
        self.vicon_q0 = None
        self.vicon_p2_0 = None
        self.vicon_q = np.empty((0,4))
        # position of robot body (centre) in the Corin VICON object frame
        # height: 6 cm (centre of sphere to bottom) + 2 mm (thickness of acetal) + 20 mm (half of body height)
        self.vicon_r_offset = 0.001 * np.array([-160, 0, -28])
        # position of actual foot (end-effector) in the foot VICON object frame
        self.vicon_foot_offset = 0.001 * np.array([-31.69, 0, -66])

        self.vicon_foot_r = np.empty((0,3))
        self.vicon_foot_angles = np.empty((0,3))
        self.vicon_foot_t = np.array([])
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
        self.estimator.Qf = self.Qf*np.eye(3)# (0.00025**2)*np.eye(3) # Noise power density (m/s2 / sqrt(Hz))
        self.estimator.Qbf = self.Qbf*np.eye(3)#(0.001)*np.eye(3)
        if (self.ideal_r == True):
            self.estimator.Qf = 0*np.eye(3)
            self.estimator.Qbf = 0*np.eye(3)

        self.estimator.Qw = self.Qw*np.eye(3)#(0.00008**2)*np.eye(3)
        self.estimator.Qbw = self.Qbw*np.eye(3)#(0.001)*np.eye(3)
        if (self.ideal_q == True):
            self.estimator.Qw = 0*np.eye(3)
            self.estimator.Qbw = 0*np.eye(3)

        self.estimator.Qp = self.Qp*np.eye(3)
        # self.estimator.Qp = np.diagflat([self.Qp_xy, self.Qp_xy, self.Qp_z])

        self.estimator.Rs = self.Rs*np.eye(3)#(0.0001**2)*np.eye(3) # m^2 #0.0000001
        self.estimator.Ra = self.Ra*np.eye(3)#(0.01**2)*np.eye(3)

        self.estimator.force_threshold = self.force_threshold

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
        angle_travelled = 1E-5
        for topic, msg, t in bag.read_messages():#topics=['/robotis/present_joint_states', '/imu/data']):
            # print msg
            if topic in callback_dict:
                callback_dict[topic](msg, t.to_time())

                if topic == '/vicon/corin/corin' or topic == '/vicon/Corin/Corin':
                    if self.vicon_r_last is not None:

                        # position error
                        error = self.vicon_r_last - self.estimator.r

                        # angles error
                        error2 = self.vicon_angles[-1] - self.estimator.get_fixed_angles()
                        error2 = (error2 + np.pi) % (2*np.pi)  - np.pi

                        errors = np.vstack([errors, error])
                        error_norm = np.linalg.norm(error)
                        error_norms = np.append(error_norms, error_norm)
                        error_ratio = np.linalg.norm(error)/np.linalg.norm(self.vicon_r_last)
                        error_ratios = np.append(error_ratios, error_ratio)

                        errors2 = np.vstack([errors2, error2])
                        error_norm2 = np.linalg.norm(error2)
                        error_norms2 = np.append(error_norms2, error_norm2)
                        unwrapped = np.unwrap(self.vicon_angles, axis=0)
                        angle_travelled = np.linalg.norm(unwrapped[-1] - unwrapped[0]) + 1E-10
                        # np.abs(np.unwrap(self.vicon_angles[:,2])[-1] )
                        error_ratio2 = np.linalg.norm(error2)/angle_travelled #(self.vicon_angles[-1,2])#np.linalg.norm(self.vicon_angles[-1])
                        error_ratios2 = np.append(error_ratios2, error_ratio2)
                #print type(t)
                 #self.counter += 1
                # if topic == '/imu_mod':
                #     error = self.body_r - self.estimator.r
                #     print "e=", error
            # print t, topic
            i += 1
            if (i > self.messages_to_read or
                np.linalg.norm(self.vicon_r_last) > self.max_distance or
                angle_travelled > self.max_angle):
                break


        print "messages read: ", i
        bag.close()
        print 'Experiment: ' + str(exp_no) + " done in ", time.time()-t0

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


        print "Position error:"
        print "max:", colored(e_max_norm, 'blue'), "[", e_max_x, ",", e_max_y, ",", e_max_z, "]"
        print "rms:", colored(e_rms_norm, 'green'), "[", e_rms_x, ",", e_rms_y, ",", e_rms_z, "]"
        print "final ratio:", colored(error_ratios[-1], 'yellow'), "(", error_norms[-1], " of ", np.linalg.norm(self.vicon_r_last),  ")"
        # print "max error ratio:", e_max_norm/total_r, "of(", total_r, ")"

        if self.integrate_total:
            # downsample VICON data @250Hz to 1 Hz
            dr = np.diff(self.vicon_r[::250*4], axis=0)
            total_r = 0
            for i in dr:
                total_r += np.linalg.norm(i)
                print "ratio of integrated total:", error_norms[-1]/total_r, "of(", total_r, ")"

        e_max_norm2 = error_norms2.max()
        e_max_ratio2 = error_ratios2.max()
        e_max_x2 = np.abs(errors2[:,0]).max()
        e_max_y2 = np.abs(errors2[:,1]).max()
        e_max_z2 = np.abs(errors2[:,2]).max()
        e_rms_norm2 = np.sqrt(np.mean(np.square(error_norms2)))
        e_rms_ratio2 = np.sqrt(np.mean(np.square(error_ratios2)))
        e_rms_x2 = np.sqrt(np.mean(np.square(errors2[:,0])))
        e_rms_y2 = np.sqrt(np.mean(np.square(errors2[:,1])))
        e_rms_z2 = np.sqrt(np.mean(np.square(errors2[:,2])))


        print "Angles error:"
        print "max:", colored(e_max_norm2, 'red'), "[", e_max_x2, ",", e_max_y2, ",", e_max_z2, "]"
        print "rms:", colored(e_rms_norm2, 'yellow'), "[", e_rms_x2, ",", e_rms_y2, ",", e_rms_z2, "]"
        unwrapped = np.unwrap(self.vicon_angles, axis=0)
        final_angle = np.linalg.norm(unwrapped[-1] - unwrapped[0])
        # print "final ratio:", colored(error_ratios2[-1], 'magenta'), "of(", np.linalg.norm(self.vicon_r[-1])
        print "final ratio:", colored(error_norms2[-1]/final_angle, 'magenta'), "(", error_norms2[-1], " of ", final_angle, ")"

        if self.integrate_total:
            da = np.diff(self.vicon_angles[::250*5], axis=0)
            total_a = 0
            for i in da:
                total_a += np.linalg.norm(i)
                print "ratio of integrated total:", error_norms2[-1]/total_a, "of(", total_a, ")"

        # print np.diag(self.estimator.P)[0:3]
        # print np.diag(self.estimator.P)[3:6]
        # print np.diag(self.estimator.P)[6:9]
        # print np.diag(self.estimator.P)[9:12]
        # print np.diag(self.estimator.P)[12:15]
        # print np.diag(self.estimator.P)[15:18]
        # print np.diag(self.estimator.P)[18:21]
        # print np.diag(self.estimator.P)[21:24]
        # print np.diag(self.estimator.P)[24:27]
        # print np.diag(self.estimator.P)[27:30]
        # print np.diag(self.estimator.P)[30:33]

        fig_title = "Experiment " + str(exp_no)

        # self.imu_t = self.imu_t - self.imu_t[0]
        # self.vicon_t = self.vicon_t - self.vicon_t[0]
        #?

        if self.plot_position_error:
            plt.figure()
            plt.title(fig_title)
            plt.plot(self.vicon_t, errors[:,0], label="x")
            plt.plot(self.vicon_t, errors[:,1], label="y")
            plt.plot(self.vicon_t, errors[:,2], label="z")
            plt.plot(self.vicon_t, error_norms, label="norm")
            # plt.plot(self.vicon_t, error_ratios, label="ratio")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Error (m)')
            if self.saveFigure:
                plt.gcf().set_size_inches(16, 9)
                plt.savefig(self.fig_path + str(exp_no) + "_position_error.png")
            if self.saveData:
                np_arr = np.hstack((np.array([self.vicon_t]).T, errors))
                arr = np.asarray(np_arr)
                np.savetxt(self.fig_path + str(exp_no) + "_position_error.csv", arr, delimiter=",")


        if self.plot_angles_error:
            plt.figure()
            plt.title(fig_title)
            plt.plot(self.vicon_t, errors2[:,0], label="raw")
            plt.plot(self.vicon_t, errors2[:,1], label="pitch")
            plt.plot(self.vicon_t, errors2[:,2], label="yaw")
            plt.plot(self.vicon_t, error_norms2, label="norm")
            # plt.plot(self.vicon_t, error_ratios, label="ratio")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Orientation Error (rad)')
            if self.saveFigure:
                plt.gcf().set_size_inches(16, 9)
                plt.savefig(self.fig_path + str(exp_no) + "_angles_error.png")
            if self.saveData:
                np_arr = np.hstack((np.array([self.vicon_t]).T, errors2))
                arr = np.asarray(np_arr)
                np.savetxt(self.fig_path + str(exp_no) + "_angles_error.csv", arr, delimiter=",")

        # np_arr = np.hstack((np.array([self.vicon_t]).T, errors))
        # arr = np.asarray(np_arr)
        # np.savetxt("70_error.csv", arr, delimiter=",")

        if self.plot_position:
            plt.figure()
            plt.title(fig_title)
            plt.plot(self.imu_t, self.imu_r[:,0], label="Estimate-x")
            plt.plot(self.imu_t, self.imu_r[:,1], label="Estimate-y")
            plt.plot(self.imu_t, self.imu_r[:,2], label="Estimate-z")
            plt.plot(self.vicon_t, self.vicon_r[:,0], label="True-x")
            plt.plot(self.vicon_t, self.vicon_r[:,1], label="True-y")
            plt.plot(self.vicon_t, self.vicon_r[:,2], label="True-z")
            if self.plot_position_stddev:
                plt.plot(self.imu_t, self.imu_r[:,0]+self.imu_r_stddev[:,0], '--', label="Estimate-x+")
                plt.plot(self.imu_t, self.imu_r[:,1]+self.imu_r_stddev[:,1], '--', label="Estimate-y+")
                plt.plot(self.imu_t, self.imu_r[:,2]+self.imu_r_stddev[:,2], '--', label="Estimate-z+")
                plt.plot(self.imu_t, self.imu_r[:,0]-self.imu_r_stddev[:,0], '--', label="Estimate-x-")
                plt.plot(self.imu_t, self.imu_r[:,1]-self.imu_r_stddev[:,1], '--', label="Estimate-y-")
                plt.plot(self.imu_t, self.imu_r[:,2]-self.imu_r_stddev[:,2], '--', label="Estimate-z-")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Position (m)')
            if self.saveFigure:
                plt.gcf().set_size_inches(16, 9)
                plt.savefig(self.fig_path + str(exp_no) + "_position.png")
            if self.saveData:
                np_arr = np.hstack((np.array([self.vicon_t]).T, self.vicon_r))
                arr = np.asarray(np_arr)
                np.savetxt(self.fig_path + str(exp_no) + "_position_VICON.csv", arr, delimiter=",")
                np_arr = np.hstack((np.array([self.imu_t]).T, self.imu_r))
                arr = np.asarray(np_arr)
                np.savetxt(self.fig_path + str(exp_no) + "_position_IMU.csv", arr, delimiter=",")

        # np_arr = np.hstack((np.array([self.imu_t]).T, self.imu_r))
        # arr = np.asarray(np_arr)
        # np.savetxt("70_imu.csv", arr, delimiter=",")
        #
        # np_arr = np.hstack((np.array([self.vicon_t]).T, self.vicon_r))
        # arr = np.asarray(np_arr)
        # np.savetxt("70_vicon.csv", arr, delimiter=",")

        if self.plot_angles:
            plt.figure()
            plt.title(fig_title)
            plt.plot(self.imu_t, np.unwrap(self.imu_angles[:,0]), label="estimate-r")
            plt.plot(self.imu_t, np.unwrap(self.imu_angles[:,1]), label="estimate-p")
            plt.plot(self.imu_t, np.unwrap(self.imu_angles[:,2]), label="estimate-y")
            plt.plot(self.vicon_t, np.unwrap(self.vicon_angles[:,0]), label="true-r")
            plt.plot(self.vicon_t, np.unwrap(self.vicon_angles[:,1]), label="true-p")
            plt.plot(self.vicon_t, np.unwrap(self.vicon_angles[:,2]), label="true-y")
            # plt.plot(self.imu_t, self.imu_o_angles[:,0], label="imu_o-r")
            # plt.plot(self.imu_t, self.imu_o_angles[:,1], label="imu_o-p")
            # plt.plot(self.imu_t, self.imu_o_angles[:,2], label="imu_o-y")
            if self.plot_angles_stddev:
                plt.plot(self.imu_t, np.unwrap(self.imu_angles[:,0])+self.imu_angles_stddev[:,0], '--', label="Estimate-r+")
                plt.plot(self.imu_t, np.unwrap(self.imu_angles[:,1])+self.imu_angles_stddev[:,1], '--', label="Estimate-p+")
                plt.plot(self.imu_t, np.unwrap(self.imu_angles[:,2])+self.imu_angles_stddev[:,2], '--', label="Estimate-y+")
                plt.plot(self.imu_t, np.unwrap(self.imu_angles[:,0])-self.imu_angles_stddev[:,0], '--', label="Estimate-r-")
                plt.plot(self.imu_t, np.unwrap(self.imu_angles[:,1])-self.imu_angles_stddev[:,1], '--', label="Estimate-p-")
                plt.plot(self.imu_t, np.unwrap(self.imu_angles[:,2])-self.imu_angles_stddev[:,2], '--', label="Estimate-y-")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Angles (rad)')
            if self.saveFigure:
                plt.gcf().set_size_inches(16, 9)
                plt.savefig(self.fig_path + str(exp_no) + "_angles.png")
            if self.saveData:
                np_arr = np.hstack((np.array([self.vicon_t]).T, self.vicon_angles))
                arr = np.asarray(np_arr)
                np.savetxt(self.fig_path + str(exp_no) + "_angles_VICON.csv", arr, delimiter=",")
                np_arr = np.hstack((np.array([self.imu_t]).T, self.imu_angles))
                arr = np.asarray(np_arr)
                np.savetxt(self.fig_path + str(exp_no) + "_angles_IMU.csv", arr, delimiter=",")

        if self.plot_foot_position:
            imu_t2 = self.imu_t.copy()
            imu_p2_2 = self.imu_p2.copy()
            print "length: ", len(self.imu_p2_stddev)
            for i in range(len(self.imu_p2_stddev)-1, -1, -1):
                if np.linalg.norm(self.imu_p2_stddev[i]) > 1:
                    self.imu_p2_stddev = np.delete(self.imu_p2_stddev, i, 0)
                    imu_t2 = np.delete(imu_t2, i, 0)
                    imu_p2_2 = np.delete(imu_p2_2, i, 0)

            plt.figure()
            plt.title(fig_title)
            plt.plot(self.imu_t, self.imu_p2[:,0], label="Estimate-x")
            plt.plot(self.imu_t, self.imu_p2[:,1], label="Estimate-y")
            plt.plot(self.imu_t, self.imu_p2[:,2], label="Estimate-z")
            plt.plot(self.vicon_foot_t, self.vicon_foot_r[:,0], label="True-x")
            plt.plot(self.vicon_foot_t, self.vicon_foot_r[:,1], label="True-y")
            plt.plot(self.vicon_foot_t, self.vicon_foot_r[:,2], label="True-z")
            if self.plot_p2_stddev:
                plt.plot(imu_t2, imu_p2_2[:,0]+self.imu_p2_stddev[:,0], '--', label="Estimate-x+")
                plt.plot(imu_t2, imu_p2_2[:,1]+self.imu_p2_stddev[:,1], '--', label="Estimate-y+")
                plt.plot(imu_t2, imu_p2_2[:,2]+self.imu_p2_stddev[:,2], '--', label="Estimate-z+")
                plt.plot(imu_t2, imu_p2_2[:,0]-self.imu_p2_stddev[:,0], '--', label="Estimate-x-")
                plt.plot(imu_t2, imu_p2_2[:,1]-self.imu_p2_stddev[:,1], '--', label="Estimate-y-")
                plt.plot(imu_t2, imu_p2_2[:,2]-self.imu_p2_stddev[:,2], '--', label="Estimate-z-")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot Position (m)')
            if self.saveFigure:
                plt.gcf().set_size_inches(16, 9)
                plt.savefig(self.fig_path + str(exp_no) + "_foot_position.png")

        if self.plot_IMU_bias:
            plt.figure()
            plt.title(fig_title)
            plt.plot(self.imu_t, self.imu_bf[:,0], label="Estimate-x")
            plt.plot(self.imu_t, self.imu_bf[:,1], label="Estimate-y")
            plt.plot(self.imu_t, self.imu_bf[:,2], label="Estimate-z")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Acc bias (m/s^2)')
            if self.saveFigure:
                plt.gcf().set_size_inches(16, 9)
                plt.savefig(self.fig_path + str(exp_no) + "_a_bias.png")

            plt.figure()
            plt.title(fig_title)
            plt.plot(self.imu_t, self.imu_bw[:,0], label="Estimate-x")
            plt.plot(self.imu_t, self.imu_bw[:,1], label="Estimate-y")
            plt.plot(self.imu_t, self.imu_bw[:,2], label="Estimate-z")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Gyro bias (rad)')
            if self.saveFigure:
                plt.gcf().set_size_inches(16, 9)
                plt.savefig(self.fig_path + str(exp_no) + "_w_bias.png")

        if self.plot_forces:
            plt.figure()
            plt.title(fig_title)
            plt.plot(self.f_mag0_t, self.f_mag0, label="0")
            plt.plot(self.f_mag1_t, self.f_mag1, label="1")
            plt.plot(self.f_mag2_t, self.f_mag2, label="2")
            plt.plot(self.f_mag3_t, self.f_mag3, label="3")
            plt.plot(self.f_mag4_t, self.f_mag4, label="4")
            plt.plot(self.f_mag5_t, self.f_mag5, label="5")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot Force Magnitude (N)')
            if self.saveFigure:
                plt.gcf().set_size_inches(16, 9)
                plt.savefig(self.fig_path + str(exp_no) + "_forces.png")


            # plt.figure(19)
            # plt.plot(self.vicon_foot_t, self.vicon_foot_r[:,0], label="vicon-x")
            # plt.plot(self.vicon_foot_t, self.vicon_foot_r[:,1], label="vicon-y")
            # plt.plot(self.vicon_foot_t, self.vicon_foot_r[:,2], label="vicon-z")
            # plt.legend()
            # plt.xlabel('Time (s)')
            # plt.ylabel('Position (m)')
        #
        # plt.figure(20)
        # plt.plot(self.vicon_foot_t, self.vicon_foot_angles[:,0], label="vicon-r")
        # plt.plot(self.vicon_foot_t, self.vicon_foot_angles[:,1], label="vicon-p")
        # plt.plot(self.vicon_foot_t, self.vicon_leg_angles[:,2], label="vicon-y")
        # plt.legend()
        # plt.xlabel('Time (s)')
        # plt.ylabel('Angles (rad)')

        if self.plot_foot0_force:
            plt.figure()
            plt.title(fig_title)
            plt.plot(self.f_mag0_t, self.f_sensor0[:,0], label="f_x")
            plt.plot(self.f_mag0_t, self.f_sensor0[:,1], label="f_y")
            plt.plot(self.f_mag0_t, self.f_sensor0[:,2], label="f_z")
            plt.plot(self.f_mag0_t, self.f_base0[:,0], label="b_x")
            plt.plot(self.f_mag0_t, self.f_base0[:,1], label="b_y")
            plt.plot(self.f_mag0_t, self.f_base0[:,2], label="b_z")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot 0 force (N)')

        if self.plot_foot1_force:
            plt.figure()
            plt.title(fig_title)
            plt.plot(self.f_mag1_t, self.f_sensor1[:,0], label="f_x")
            plt.plot(self.f_mag1_t, self.f_sensor1[:,1], label="f_y")
            plt.plot(self.f_mag1_t, self.f_sensor1[:,2], label="f_z")
            plt.plot(self.f_mag1_t, self.f_base1[:,0], label="b_x")
            plt.plot(self.f_mag1_t, self.f_base1[:,1], label="b_y")
            plt.plot(self.f_mag1_t, self.f_base1[:,2], label="b_z")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot 1 force (N)')


        if self.plot_foot2_force:
            plt.figure()
            plt.title(fig_title)
            plt.plot(self.f_mag2_t, self.f_sensor2[:,0], label="f_x")
            plt.plot(self.f_mag2_t, self.f_sensor2[:,1], label="f_y")
            plt.plot(self.f_mag2_t, self.f_sensor2[:,2], label="f_z")
            plt.plot(self.f_mag2_t, self.f_base2[:,0], label="b_x")
            plt.plot(self.f_mag2_t, self.f_base2[:,1], label="b_y")
            plt.plot(self.f_mag2_t, self.f_base2[:,2], label="b_z")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot 2 force (N)')


        if self.plot_foot3_force:
            plt.figure()
            plt.title(fig_title)
            plt.plot(self.f_mag3_t, self.f_sensor3[:,0], label="f_x")
            plt.plot(self.f_mag3_t, self.f_sensor3[:,1], label="f_y")
            plt.plot(self.f_mag3_t, self.f_sensor3[:,2], label="f_z")
            plt.plot(self.f_mag3_t, self.f_base3[:,0], label="b_x")
            plt.plot(self.f_mag3_t, self.f_base3[:,1], label="b_y")
            plt.plot(self.f_mag3_t, self.f_base3[:,2], label="b_z")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot 3 force (N)')

        if self.plot_foot4_force:
            plt.figure()
            plt.title(fig_title)
            plt.plot(self.f_mag4_t, self.f_sensor4[:,0], label="f_x")
            plt.plot(self.f_mag4_t, self.f_sensor4[:,1], label="f_y")
            plt.plot(self.f_mag4_t, self.f_sensor4[:,2], label="f_z")
            plt.plot(self.f_mag4_t, self.f_base4[:,0], label="b_x")
            plt.plot(self.f_mag4_t, self.f_base4[:,1], label="b_y")
            plt.plot(self.f_mag4_t, self.f_base4[:,2], label="b_z")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot 4 force (N)')

        if self.plot_foot5_force:
            plt.figure()
            plt.title(fig_title)
            plt.plot(self.f_mag5_t, self.f_sensor5[:,0], label="f_x")
            plt.plot(self.f_mag5_t, self.f_sensor5[:,1], label="f_y")
            plt.plot(self.f_mag5_t, self.f_sensor5[:,2], label="f_z")
            plt.plot(self.f_mag5_t, self.f_base5[:,0], label="b_x")
            plt.plot(self.f_mag5_t, self.f_base5[:,1], label="b_y")
            plt.plot(self.f_mag5_t, self.f_base5[:,2], label="b_z")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot 5 force (N)')
            # np_arr = np.hstack((np.array([self.f_mag5_t]).T, self.f_sensor5))
            # arr = np.asarray(np_arr)
            # np.savetxt( str(exp_no) + "_force_5.csv" , arr, delimiter=",")


        if self.plot_residual:
            plt.figure()
            plt.title(fig_title)
            plt.plot(self.imu_ty, self.imu_y0[:,0], label="x")
            plt.plot(self.imu_ty, self.imu_y0[:,1], label="y")
            plt.plot(self.imu_ty, self.imu_y0[:,2], label="x")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot error 0 (m)')


            plt.figure()
            plt.title(fig_title)
            plt.plot(self.imu_ty, self.imu_y1[:,0], label="x")
            plt.plot(self.imu_ty, self.imu_y1[:,1], label="y")
            plt.plot(self.imu_ty, self.imu_y1[:,2], label="x")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot error 1 (m)')


            plt.figure()
            plt.title(fig_title)
            plt.plot(self.imu_ty, self.imu_y2[:,0], label="x")
            plt.plot(self.imu_ty, self.imu_y2[:,1], label="y")
            plt.plot(self.imu_ty, self.imu_y2[:,2], label="x")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot error 2 (m)')


            plt.figure()
            plt.title(fig_title)
            plt.plot(self.imu_ty, self.imu_y3[:,0], label="x")
            plt.plot(self.imu_ty, self.imu_y3[:,1], label="y")
            plt.plot(self.imu_ty, self.imu_y3[:,2], label="x")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot error 3 (m)')

            plt.figure()
            plt.title(fig_title)
            plt.plot(self.imu_ty, self.imu_y4[:,0], label="x")
            plt.plot(self.imu_ty, self.imu_y4[:,1], label="y")
            plt.plot(self.imu_ty, self.imu_y4[:,2], label="x")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot error 4 (m)')

            plt.figure()
            plt.title(fig_title)
            plt.plot(self.imu_ty, self.imu_y5[:,0], label="x")
            plt.plot(self.imu_ty, self.imu_y5[:,1], label="y")
            plt.plot(self.imu_ty, self.imu_y5[:,2], label="x")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot error 5 (m)')


        plt.show()
        self.power = 0

    def force_0_callback(self, msg, t):
        self.robot.cstate[0] = self.foot_state(msg)
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[0:3] = f

        if self.plot_forces:
            f_mag = np.linalg.norm(f)
            self.f_mag0 = np.append(self.f_mag0, f_mag)
            self.f_mag0_t = np.append(self.f_mag0_t, t)

        # if self.robot.qc is not None:
        #     self.robot.update_state()
        # self.f_sensor0 = np.vstack([self.f_sensor0, self.robot.Leg[0].F6c.tibia_X_foot[:3].flatten()])
        # self.f_base0 = np.vstack([self.f_base0, self.robot.Leg[0].F6c.base_X_foot[:3].flatten()])

    def force_1_callback(self, msg, t):
        self.robot.cstate[1] = self.foot_state(msg)
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[3:6] = f

        if self.plot_forces:
            f_mag = np.linalg.norm(f)
            self.f_mag1 = np.append(self.f_mag1, f_mag)
            self.f_mag1_t = np.append(self.f_mag1_t, t)

        # if self.robot.qc is not None:
        #     self.robot.update_state()
        # self.f_sensor1 = np.vstack([self.f_sensor1, self.robot.Leg[1].F6c.tibia_X_foot[:3].flatten()])
        # self.f_base1 = np.vstack([self.f_base1, self.robot.Leg[1].F6c.base_X_foot[:3].flatten()])

    def force_2_callback(self, msg, t):
        self.robot.cstate[2] = self.foot_state(msg)
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[6:9] = f

        if self.plot_forces:
            f_mag = np.linalg.norm(f)
            self.f_mag2 = np.append(self.f_mag2, f_mag)
            self.f_mag2_t = np.append(self.f_mag2_t, t)

        # if self.robot.qc is not None:
        #     self.robot.update_state()
        # self.f_sensor2 = np.vstack([self.f_sensor2, self.robot.Leg[2].F6c.tibia_X_foot[:3].flatten()])
        # self.f_base2 = np.vstack([self.f_base2, self.robot.Leg[2].F6c.base_X_foot[:3].flatten()])

    def force_3_callback(self, msg, t):
        self.robot.cstate[3] = self.foot_state(msg)
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[9:12] = f

        if self.plot_forces:
            f_mag = np.linalg.norm(f)
            self.f_mag3 = np.append(self.f_mag3, f_mag)
            self.f_mag3_t = np.append(self.f_mag3_t, t)

        # if self.robot.qc is not None:
        #     self.robot.update_state()
        # self.f_sensor3 = np.vstack([self.f_sensor3, self.robot.Leg[3].F6c.tibia_X_foot[:3].flatten()])
        # self.f_base3 = np.vstack([self.f_base3, self.robot.Leg[3].F6c.base_X_foot[:3].flatten()])

    def force_4_callback(self, msg, t):
        self.robot.cstate[4] = self.foot_state(msg)
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        f = f + self.f_sensor4_offset
        self.robot.cforce[12:15] = f

        if self.plot_forces:
            f_mag = np.linalg.norm(f)
            self.f_mag4 = np.append(self.f_mag4, f_mag)
            self.f_mag4_t = np.append(self.f_mag4_t, t)

        # if self.robot.qc is not None:
        #     self.robot.update_state()
        # self.f_sensor4 = np.vstack([self.f_sensor4, self.robot.Leg[4].F6c.tibia_X_foot[:3].flatten()])
        # self.f_base4 = np.vstack([self.f_base4, self.robot.Leg[4].F6c.base_X_foot[:3].flatten()])

    def force_5_callback(self, msg, t):
        self.robot.cstate[5] = self.foot_state(msg)
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[15:18] = f

        if self.plot_forces:
            f_mag = np.linalg.norm(f)
            self.f_mag5 = np.append(self.f_mag5, f_mag)
            self.f_mag5_t = np.append(self.f_mag5_t, t)

        # if self.robot.qc is not None:
        #     self.robot.update_state()
        # self.f_sensor5 = np.vstack([self.f_sensor5, self.robot.Leg[5].F6c.tibia_X_foot[:3].flatten()])
        # self.f_base5 = np.vstack([self.f_base5, self.robot.Leg[5].F6c.base_X_foot[:3].flatten()])

    def foot_state(self, msg):
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
                #--------------------------------------------------------#
                # how it used to be
                # self.vicon_r0 = r.copy()
                #---------------=
                self.vicon_r0 = r + quaternion_matrix(q)[0:3, 0:3].dot(self.vicon_r_offset)
                #------------------------------------------------
                # mess up orientation to see if roll/pitch converge: They do!
                # self.estimator.q[1:4] =  - self.estimator.q[1:4]

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
            #-------------------------------
            # used to be
            # r_v = r - self.vicon_r0 # Relative position in Vicon frame
            # what it is:
            r_v = r + quaternion_matrix(q)[0:3, 0:3].dot(self.vicon_r_offset)
            r_v = r_v - self.vicon_r0
            # r_quat = np.insert(r_v, 0, 0)
            r = quaternion_matrix(self.vicon_q0)[0:3,0:3].dot(r_v) # Hamilton
            # print r
            # print quaternion_multiply(quaternion_multiply(self.vicon_q0, r_quat), quaternion_conjugate(self.vicon_q0))[1:4] # Hamilton
            # print quaternion_matrix_JPL(self.vicon_q0)[0:3,0:3].T.dot(r_v) # JPL
            # print quaternion_multiply_JPL(quaternion_multiply_JPL(quaternion_conjugate(self.vicon_q0), r_quat),self.vicon_q0)[1:4] # JPL
            self.vicon_r_last = r.copy()

            if self.plot_position or self.integrate_total:
                self.vicon_r = np.vstack([self.vicon_r, r])

            q = quaternion_multiply(self.vicon_q0, q) # Hamilton
            angles_tuple = euler_from_quaternion_JPL(q, 'rxyz') # relative: rxyz
            angles = np.asarray(angles_tuple).tolist()
            # print "vic", q
            self.vicon_t = np.append(self.vicon_t, t)
            self.vicon_angles = np.vstack([self.vicon_angles, angles])

            if self.ideal_q:
                self.vicon_q = np.vstack([self.vicon_q, q])



    def vicon_foot_callback(self, msg, t):
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
            p2 = p2 - self.vicon_p2_0 # Calculate position wrt first measurement
            p2 = quaternion_matrix(self.vicon_q0)[0:3, 0:3].dot(p2) # VICON to world frame
            p2 = p2 + self.p2_0 # Set the starting position to match that of the EKF

            if self.plot_foot_position:
                self.vicon_foot_r = np.vstack([self.vicon_foot_r, p2])
                self.vicon_foot_t = np.append(self.vicon_foot_t, t)
        #

        # if self.vicon_q0 is not None:
        #     p2 = r + quaternion_matrix(q)[0:3, 0:3].dot(self.vicon_foot_offset)
        #     p2 = p2 - self.vicon_r0
        #     p2 = quaternion_matrix(self.vicon_q0)[0:3, 0:3].dot(p2) # VICON to world frame
        #
            # self.vicon_foot_r = np.vstack([self.vicon_foot_r, p2])
            # self.vicon_foot_t = np.append(self.vicon_foot_t, t)

        # if self.vicon_q0 is not None:
        #     r_v = r - self.vicon_r0 # Relative position in Vicon frame
        #     r_quat = np.insert(r_v, 0, 0)
        #     r = quaternion_matrix(self.vicon_q0)[0:3,0:3].dot(r_v) # Hamilton
        #     self.vicon_foot_r = np.vstack([self.vicon_foot_r, r])
        #     self.vicon_foot_t = np.append(self.vicon_foot_t, t)
        #
        #     q = quaternion_multiply(self.vicon_q0, q) # Hamilton
        #     angles_tuple = euler_from_quaternion_JPL(q, 'rxyz') # relative: rxyz
        #     angles = np.asarray(angles_tuple).tolist()
        #     # print "vic", q
        #     self.vicon_foot_angles = np.vstack([self.vicon_foot_angles, angles])


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

        # ps = PoseStamped()
        # ps.header.stamp = rospy.Time.now()
        # ps.header.frame_id = "world"
        # ps.pose.orientation = Quaternion(o[1], o[2], o[3], o[0])
        # self.pub_pose.publish(ps)

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
                # initialise P matrix
            else:
                self.estimator.q = np.array([1, 0, 0, 0])
                self.estimator.P[8, 8] = 1E-6 # we known heading = 0

            # exit()

        else:
            # Set IMU orientation to "ideal" (VICON data)
            # if (self.ideal_q == True and self.vicon_q.size > 0):
            #     o = self.vicon_q[-1]
            #     # Can use this orientation directly in State Estimator
            #     msg.orientation = Quaternion(o[1], o[2], o[3], o[0])
            self.estimator.predict_state()


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



            # use VICON orientation as state estimate with 0 variance
            if (self.ideal_q == True and self.vicon_q.size > 0):
                self.estimator.q = self.vicon_q[-1]
                self.estimator.bw = np.zeros(3)
                # self.estimator.P[6:9, 6:9] = np.zeros((3,3))

            if (self.ideal_r == True and self.vicon_r.size > 0):
                self.estimator.r = self.vicon_r[-1].copy()
                self.estimator.bf = np.zeros(3)
                # self.estimator.P[0:3, 0:3] = np.zeros((3,3))
                # self.estimator.P[27:30, 27:30] = np.zeros((3,3))


            if self.plot_position:
                self.imu_r = np.vstack([self.imu_r, self.estimator.r])

            if self.plot_position_stddev:
                stddev = np.array([ np.sqrt(self.estimator.P[0,0]),
                                    np.sqrt(self.estimator.P[1,1]),
                                    np.sqrt(self.estimator.P[2,2]) ])
                self.imu_r_stddev = np.vstack([self.imu_r_stddev, stddev])

            if self.plot_angles:
                self.imu_angles = np.vstack([self.imu_angles, self.estimator.get_fixed_angles()])

            if self.plot_angles_stddev:
                stddev2 = np.array([np.sqrt(self.estimator.P[6,6]),
                                    np.sqrt(self.estimator.P[7,7]),
                                    np.sqrt(self.estimator.P[8,8]) ])
                self.imu_angles_stddev = np.vstack([self.imu_angles_stddev, stddev2])


            # print "imu", self.estimator.q
            if self.plot_position or self.plot_angles or self.plot_foot_position or self.plot_IMU_bias:
                self.imu_t = np.append(self.imu_t, t)
            # self.imu_o_angles = np.vstack([self.imu_o_angles, self.estimator.get_o_fixed_angles()])

            if self.plot_foot_position:
                self.imu_p2 = np.vstack([self.imu_p2, self.estimator.p2])

            if self.plot_p2_stddev:
                stddev3 = np.array([np.sqrt(self.estimator.P[15,15]),
                                    np.sqrt(self.estimator.P[16,16]),
                                    np.sqrt(self.estimator.P[17,17]) ])
                # if np.linalg.norm(stddev3) < 0.1:
                self.imu_p2_stddev = np.vstack([self.imu_p2_stddev, stddev3])


            if self.plot_IMU_bias:
                self.imu_bf = np.vstack([self.imu_bf, self.estimator.bf])
                self.imu_bw = np.vstack([self.imu_bw, self.estimator.bw])


    def imu_callback0(self, msg, t):
        return
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
