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
        self.imu_a_z = np.array([])
        self.body_a_z = np.array([])
        self.imu_t = np.array([])
        self.body_t = np.array([])

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

        self.imu_callback

        callback_dict = {   '/corin/imu/base/data': self.imu_callback,
                            '/corin/joint_states': self.joint_state_callback,
                            '/corin/contact_state': self.contact_state_callback,
                            '/gazebo/model_states': self.m_states_callback}

        t0 = time.time()
        bag = rosbag.Bag('./testfile.bag')
        print "messages:", bag.get_message_count(topic_filters=['/gazebo/model_states'])
        for topic, msg, t in bag.read_messages():#topics=['chatter', 'numbers']):
            if topic in callback_dict:
                callback_dict[topic](msg)
                 #print self.counter#topic
                 #self.counter += 1
            # print topic
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

    def some_callback(self):
        print "called"

    def joint_state_callback(self, msg):
        """ robot joint state callback """
        self.robot.qc = msg

    def contact_state_callback(self, msg):
        """ foot contact binary state """
        self.robot.cstate = msg.data

    def imu_callback(self, msg): 		# robot joint state callback

        self.robot.imu = msg

        if self.data_source == "imu":
            o = msg.orientation
            av = msg.angular_velocity
            o = np.array([o.w, o.x, o.y, o.z])
            angular_velocity = np.array([av.x, av.y, av.z])
            self.p = 0
        else:
            ms = self.model_states
            p = ms.pose[1].position
            o = ms.pose[1].orientation # quaternion
            v = ms.twist[1].linear # linear velocity

            p = np.array([p.x, p.y, p.z])
            o = np.array([o.w, o.x, o.y, o.z])
            v = np.array([v.x, v.y, v.z])
            C = quaternion_matrix_JPL(o)[0:3, 0:3]

            msg.orientation = ms.pose[1].orientation

            if self.p is None:
                #dp = (p - self.p0)/(self.T_sim*self.counter)
                dp = 0
                msg.linear_acceleration = Vector3(0, 0, 0)
                msg.angular_velocity = Vector3(0, 0, 0)
                #if self.dp is None:
            else:
                dp = (p - self.p)/(self.T_imu)
                g = np.array([0, 0, -9.80])
                #a = C.dot( (v - self.dp)/(self.T_sim*self.counter) - g )
                a = C.dot( (dp - self.dp)/(self.T_imu) - g )
                self.counter = 0
                msg.linear_acceleration = Vector3(*a.tolist())

                self.o[1:4] = -self.o[1:4] # inverse
                dq = quaternion_multiply_JPL(o, self.o)
                #dq = dq.round(6)	# ensure dq[0] !> 1
                phi_norm = 2 * np.arccos(dq[0])
                if phi_norm > 0:
                    phi = dq[1:4] * phi_norm / np.sin(phi_norm/2)
                    #print "if"
                else:
                    phi = np.zeros(3)
                    #print "else"

                w = phi/self.T_imu
                msg.angular_velocity = Vector3(*w.tolist())
                #print "model:", w

            self.o = o
            self.dp = dp
            self.p = p

        #print "imu:", angular_velocity#
        #print "imu t: %f" % time.time(),
        #self.pub_x.publish(av.x)

        #print "lin acc:", (msg.linear_acceleration)

        if(True and self.cal_i  < self.cal_N):
            self.cal_sum += o
            self.cal_i += 1
        elif(True and self.cal_i == self.cal_N):
            q_av = self.cal_sum / self.cal_N
            self.estimator.q = q_av / np.sqrt(q_av.dot(q_av))

            # Move on to state estimation only after foot positions are reset
            if self.estimator.reset_foot_positions():
                self.cal_i += 1
                #elif(self.cal_i > self.cal_N + 500)
                #self.cal
        else:
            acc = msg.linear_acceleration
            #acc.x += (1/np.sqrt(self.T_imu)) * np.random.normal(0, 1) #u, std. dev.
            self.estimator.predict_state()

            self.update_i += 1
            if self.update_i >= self.update_N:
                self.estimator.update_state()
                self.update_i = 0

            self.pub_imu_a.publish(Vector3(*self.estimator.a.tolist()))
            self.pub_imu_v.publish(Vector3(*(1.0*self.estimator.v).tolist()))
            self.pub_imu_r.publish(Vector3(*(1.0*self.estimator.r).tolist()))
            self.pub_imu_w.publish(Vector3(*self.estimator.w.tolist()))
            self.pub_imu_p.publish(Vector3(*self.estimator.get_fixed_angles().tolist()))
            #
            # stddev = np.sqrt(self.estimator.P[0,0])
            # self.pub_y.publish(1.0*(self.estimator.r[0])+stddev)
            # self.pub_x.publish(1.0*(self.estimator.r[0])-stddev)

            #a2 = self.estimator.a
            #a_mag = np.sqrt(np.dot(a2, a2))
            #self.pub_y.publish(a_mag)
            #print "a1:", a2
            #print "dr:", self.estimator.dr

            #self.imu_a_z = np.append(self.imu_a_z, msg.angular_velocity.x)
            #print np.std(self.imu_a_z)


            self.imu_a_z = np.append(self.imu_a_z, self.estimator.r[0])
            self.imu_t = np.append(self.imu_t, time.time())
            print len(self.imu_t)



    def m_states_callback(self, msg):
        self.model_states = msg
        # if self.data_source == 'ideal':
        #     self.imu_callback(Imu())
        #     self.counter += 1
        # # contains arrays of states
        # # msg.name = [ground_plane, corin]
        # # The following describe state of 'corin' (indx 1) w.r.t. world frame
        # # msg.pose[1].position.x/y/z
        # # msg.pose[1].orientation.w/y/z/w
        # # msg.twist[1].linear.x/y/z ()
        # # msg.twist[1].angular.x/y/z ()
        # t = time.time()
        # o = msg.pose[1].orientation # quaternion
        # o = np.array([o.w, o.x, o.y, o.z])
        # w = msg.twist[1].angular #angular velocity
        # w = np.array([w.x, w.y, w.z])
        # p = msg.pose[1].position
        # p = np.array([p.x, p.y, p.z])
        # v = msg.twist[1].linear # linear velocity
        # v = np.array([v.x, v.y, v.z])
        # a = (v - self.v)/self.T_sim
        # self.v = v
        # self.t = t
        # #print "m_sta:", w
        #
        #
        # C = quaternion_matrix_JPL(self.estimator.q)[0:3, 0:3]
        # C2 = quaternion_matrix_JPL(o)[0:3, 0:3]
        # #print "bod:", o
        # #print "model:", C.dot(w) # approx 1kHz but variable
        # #self.pub_y.publish(C.dot(w)[0])
        # self.pub_z.publish(C2.dot(v)[0])
        # a2 = C2.dot(a)
        # a_mag = np.sqrt(np.dot(a2, a2))
        # #self.pub_z.publish(a_mag)
        # if self.p0 is None:
        #     if self.p is not None: # initialise starting position when imu loop starts
        #         self.p0 = p.copy()
        # else:
        #     p -= self.p0
        #     self.pub_body_a.publish(Vector3(*a.tolist()))		# m unit
        #     self.pub_body_v.publish(Vector3(*(1.0*v).tolist())) # mm unit
        #     #print "real", 1.0*v[2]
        #     self.pub_body_r.publish(Vector3(*(1.0*p).tolist())) # mm unit
        #     self.pub_body_w.publish(Vector3(*w.tolist()))
        #
        # angles_tuple = euler_from_quaternion_JPL(o, 'rxyz') # relative: rxyz
        # angles = np.asarray(angles_tuple).tolist()
        #
        # self.pub_body_p.publish(Vector3(*angles))

        # self.body_a_z = np.append(self.body_a_z, p[0])
        # self.body_t = np.append(self.body_t, time.time())

if __name__ == "__main__":

    state = CorinStateTester()

    "COMPLETE"
