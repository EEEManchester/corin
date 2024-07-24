#!/usr/bin/env python
import numpy as np
import rospy
from robotis_controller_msgs.msg import SyncWriteMultiFloat, WriteControlTable
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String
from cpp.build.lib_Legged_Robotics_CPP_BINDINGS import DQ_CorinHexapod

class basic_rospy_test_class:
    def __init__(self):
        self.forces = [np.array([0,0,0]) for _ in range(6)]

        rospy.init_node('basic_test_class_node', anonymous=True)
        self.joint_pub = rospy.Publisher('/robotis/sync_write_multi_float', SyncWriteMultiFloat, queue_size=1)
        self.control_mode_pub = rospy.Publisher('/robotis/write_control_table', WriteControlTable, queue_size=1)

        self.force_subs = []
        for i in range(1, 7):
            self.force_subs.append(
                rospy.Subscriber(f'force_sensor_{i}', Vector3Stamped, getattr(self, f'force_sensor_{i}_callback'), queue_size=1)
            )

        self.joint_names = [
            'lf_q1_joint', 'lf_q2_joint', 'lf_q3_joint',
            'lm_q1_joint', 'lm_q2_joint', 'lm_q3_joint',
            'lr_q1_joint', 'lr_q2_joint', 'lr_q3_joint',
            'rf_q1_joint', 'rf_q2_joint', 'rf_q3_joint',
            'rm_q1_joint', 'rm_q2_joint', 'rm_q3_joint',
            'rr_q1_joint', 'rr_q2_joint', 'rr_q3_joint'
        ]

    def force_sensor_callback(self, msg, index):
        self.forces[index] = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

    def force_sensor_1_callback(self, msg): self.force_sensor_callback(msg, 0)
    def force_sensor_2_callback(self, msg): self.force_sensor_callback(msg, 1)
    def force_sensor_3_callback(self, msg): self.force_sensor_callback(msg, 2)
    def force_sensor_4_callback(self, msg): self.force_sensor_callback(msg, 3)
    def force_sensor_5_callback(self, msg): self.force_sensor_callback(msg, 4)
    def force_sensor_6_callback(self, msg): self.force_sensor_callback(msg, 5)

    def broadcast_whole_body_setpoint(self, joint_angles, string):
        rospy.loginfo("Broadcasting motor setpoint:" + str(joint_angles))

        dqp = SyncWriteMultiFloat()
        dqp.item_name = str(string)
        dqp.data_length = 4

        for n in range(18):
            dqp.joint_name.append(str(self.joint_names[n]))
            dqp.value.append(joint_angles[n])

        self.joint_pub.publish(dqp)

    def set_control_mode(self, mode):
        rospy.loginfo("Setting control mode to: " +str(mode))

        dqp = WriteControlTable()
        dqp.start_item_name = str("operating_mode")
        dqp.data_length = 4

        # for n in range(18):
        dqp.joint_name=str(self.joint_names[2])
        dqp.data=[mode]

        self.control_mode_pub.publish(dqp)

if __name__ == '__main__':
    test_class = basic_rospy_test_class()

    rospy.loginfo("Waiting for 5 seconds..")
    rospy.sleep(5)

    test_class.broadcast_whole_body_setpoint([0, 0, 0]*6, "goal_position")
    rospy.loginfo("Moving to all zero...")
    rospy.sleep(2)
    rospy.loginfo("Done. Setting control mode...")

    test_class.set_control_mode(1)
    rospy.sleep(1)  # Give some time for the mode to switch

    rospy.loginfo("Done. moving motor...")

    test_class.broadcast_whole_body_setpoint([0, 0, 1] + [0, 0, 0]*5, "goal_velocity")

    rospy.loginfo("That should be working now?")

    while not rospy.is_shutdown():
        rospy.sleep(1)
