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
        self.messages_to_read = 10000000
        self.max_angle = 2*np.pi
        self.max_distance = 1
        # What to plot?!
        self.plot_position = True
        self.plot_angles = True
        self.plot_foot_position = True
        self.plot_forces = False
        self.plot_foot0_force = False
        self.plot_foot1_force = False
        self.plot_foot2_force = False
        self.plot_foot3_force = False
        self.plot_foot4_force = False
        self.plot_foot5_force = False

        self.saveFigure = False
        self.fig_path = "./figures2/"
        self.saveData = False

        if len(sys.argv) > 1:
            exp_no = int(sys.argv[1])
        else:
            exp_no = 30 #experiment number

        print 'experiment: ' + str(exp_no)

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

        t0 = time.time()

        self.q0 = None
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
        # height: 6 cm (centre of VICON tracking sphere to bottom) + 2 mm (thickness of acetal) + 20 mm (half of body height)
        self.vicon_r_offset = 0.001 * np.array([-160, 0, -28])
        # position of actual foot (end-effector) in the foot VICON object frame
        self.vicon_foot_offset = 0.001 * np.array([-31.69, 0, -66])

        self.vicon_foot_r = np.empty((0,3))
        self.vicon_foot_angles = np.empty((0,3))
        self.vicon_foot_t = np.array([])

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
        self.cal_N = 500
        self.cal_sum = np.zeros(4)
        self.a0_sum = np.zeros(3)
        self.w_sum = np.zeros(3)
        self.cal_i = 0.0

        self.angles_array = np.empty((0,3))
        self.a0_array = np.empty((0,3))
        self.w_array = np.empty((0,3))

        self.robot 	= robot_class.RobotState()

        callback_dict = {   '/imu_LORD/data': self.imu_callback,
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

        angle_travelled = 1E-5
        i=0
        for topic, msg, t in bag.read_messages():
            if topic in callback_dict:
                callback_dict[topic](msg, t.to_time())

            i += 1
            if (i > self.messages_to_read or
                np.linalg.norm(self.vicon_r_last) > self.max_distance or
                angle_travelled > self.max_angle):
                break


        print "messages read: ", i
        bag.close()
        print 'Experiment: ' + str(exp_no) + " done in ", time.time()-t0

        fig_title = "Experiment " + str(exp_no)

        if self.plot_position:
            plt.figure()
            plt.title(fig_title)
            plt.plot(self.vicon_t, self.vicon_r[:,0], label="True-x")
            plt.plot(self.vicon_t, self.vicon_r[:,1], label="True-y")
            plt.plot(self.vicon_t, self.vicon_r[:,2], label="True-z")
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

        if self.plot_angles:
            plt.figure()
            plt.title(fig_title)
            plt.plot(self.vicon_t, np.unwrap(self.vicon_angles[:,0]), label="true-r")
            plt.plot(self.vicon_t, np.unwrap(self.vicon_angles[:,1]), label="true-p")
            plt.plot(self.vicon_t, np.unwrap(self.vicon_angles[:,2]), label="true-y")
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

            plt.figure()
            plt.title(fig_title)
            plt.plot(self.vicon_foot_t, self.vicon_foot_r[:,0], label="True-x")
            plt.plot(self.vicon_foot_t, self.vicon_foot_r[:,1], label="True-y")
            plt.plot(self.vicon_foot_t, self.vicon_foot_r[:,2], label="True-z")
            plt.legend()
            plt.xlabel('Time (s)')
            plt.ylabel('Foot Position (m)')
            if self.saveFigure:
                plt.gcf().set_size_inches(16, 9)
                plt.savefig(self.fig_path + str(exp_no) + "_foot_position.png")


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

            if self.robot.qc is not None:
                self.robot.update_state()
            self.f_sensor0 = np.vstack([self.f_sensor0, self.robot.Leg[0].F6c.tibia_X_foot[:3].flatten()])
            self.f_base0 = np.vstack([self.f_base0, self.robot.Leg[0].F6c.base_X_foot[:3].flatten()])

    def force_1_callback(self, msg, t):
        self.robot.cstate[1] = self.foot_state(msg)
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[3:6] = f

        if self.plot_forces:
            f_mag = np.linalg.norm(f)
            self.f_mag1 = np.append(self.f_mag1, f_mag)
            self.f_mag1_t = np.append(self.f_mag1_t, t)

            if self.robot.qc is not None:
                self.robot.update_state()
            self.f_sensor1 = np.vstack([self.f_sensor1, self.robot.Leg[1].F6c.tibia_X_foot[:3].flatten()])
            self.f_base1 = np.vstack([self.f_base1, self.robot.Leg[1].F6c.base_X_foot[:3].flatten()])

    def force_2_callback(self, msg, t):
        self.robot.cstate[2] = self.foot_state(msg)
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[6:9] = f

        if self.plot_forces:
            f_mag = np.linalg.norm(f)
            self.f_mag2 = np.append(self.f_mag2, f_mag)
            self.f_mag2_t = np.append(self.f_mag2_t, t)

            if self.robot.qc is not None:
                self.robot.update_state()
            self.f_sensor2 = np.vstack([self.f_sensor2, self.robot.Leg[2].F6c.tibia_X_foot[:3].flatten()])
            self.f_base2 = np.vstack([self.f_base2, self.robot.Leg[2].F6c.base_X_foot[:3].flatten()])

    def force_3_callback(self, msg, t):
        self.robot.cstate[3] = self.foot_state(msg)
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[9:12] = f

        if self.plot_forces:
            f_mag = np.linalg.norm(f)
            self.f_mag3 = np.append(self.f_mag3, f_mag)
            self.f_mag3_t = np.append(self.f_mag3_t, t)

            if self.robot.qc is not None:
                self.robot.update_state()
            self.f_sensor3 = np.vstack([self.f_sensor3, self.robot.Leg[3].F6c.tibia_X_foot[:3].flatten()])
            self.f_base3 = np.vstack([self.f_base3, self.robot.Leg[3].F6c.base_X_foot[:3].flatten()])

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

            if self.robot.qc is not None:
                self.robot.update_state()
            self.f_sensor4 = np.vstack([self.f_sensor4, self.robot.Leg[4].F6c.tibia_X_foot[:3].flatten()])
            self.f_base4 = np.vstack([self.f_base4, self.robot.Leg[4].F6c.base_X_foot[:3].flatten()])

    def force_5_callback(self, msg, t):
        self.robot.cstate[5] = self.foot_state(msg)
        f = msg.vector
        f = np.array([f.x, f.y, f.z])
        self.robot.cforce[15:18] = f

        if self.plot_forces:
            f_mag = np.linalg.norm(f)
            self.f_mag5 = np.append(self.f_mag5, f_mag)
            self.f_mag5_t = np.append(self.f_mag5_t, t)

            if self.robot.qc is not None:
                self.robot.update_state()
            self.f_sensor5 = np.vstack([self.f_sensor5, self.robot.Leg[5].F6c.tibia_X_foot[:3].flatten()])
            self.f_base5 = np.vstack([self.f_base5, self.robot.Leg[5].F6c.base_X_foot[:3].flatten()])

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
                self.vicon_q0 = quaternion_multiply(self.q0, v_q0_conj) # Hamilton
                self.vicon_r0 = r + quaternion_matrix(q)[0:3, 0:3].dot(self.vicon_r_offset)

                q2 = quaternion_multiply(self.vicon_q0, q)

        else:
            r_v = r + quaternion_matrix(q)[0:3, 0:3].dot(self.vicon_r_offset)
            r_v = r_v - self.vicon_r0
            r = quaternion_matrix(self.vicon_q0)[0:3,0:3].dot(r_v) # Hamilton
            self.vicon_r_last = r.copy()

            if self.plot_position:
                self.vicon_r = np.vstack([self.vicon_r, r])

            q = quaternion_multiply(self.vicon_q0, q) # Hamilton
            angles_tuple = euler_from_quaternion_JPL(q, 'rxyz') # relative: rxyz
            angles = np.asarray(angles_tuple).tolist()

            self.vicon_t = np.append(self.vicon_t, t)
            self.vicon_angles = np.vstack([self.vicon_angles, angles])



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

            IMU_R = np.array([[-1, 0, 0],
                                            [0, 1, 0],
                                            [0, 0, -1]])	# Rotation matrix mapping vectors from IMU frame to body frame

            q0 = quaternion_from_matrix(IMU_R.T) # Rotates points from base to IMU
            temp = quaternion_multiply(q_av, q0) # Rotate vector from base to IMU then from IMU to world

            # Set heading to zero
            angles_tuple = euler_from_quaternion(temp, 'sxyz')
            a = np.asarray(angles_tuple).tolist()
            self.q0 = quaternion_from_euler(a[0], a[1], 0, 'sxyz')

            self.cal_i += 1

            C = quaternion_matrix_JPL(self.q0)[0:3, 0:3]
            self.robot.update_state() # sets the new foot positions
            s = self.robot.Leg[2].XHc.base_X_foot[:3,3]
            self.p2_0 = C.T.dot(s)


if __name__ == "__main__":

    state = CorinStateTester()

    "COMPLETE"
