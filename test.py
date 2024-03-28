#!/usr/bin/env python

import rospy
from robotis_controller_msgs.msg import SyncWriteMultiFloat

class basic_rospy_test_class:
    def __init__(self):
        rospy.init_node('basic_test_class_node', anonymous=True)
        self.joint_pub = rospy.Publisher('/robotis/sync_write_multi_float', SyncWriteMultiFloat, queue_size=1)

        self.joint_names    = [None]*18
        self.joint_names[0] = 'lf_q1_joint'
        self.joint_names[1] = 'lf_q2_joint'
        self.joint_names[2] = 'lf_q3_joint'
        self.joint_names[3] = 'lm_q1_joint'
        self.joint_names[4] = 'lm_q2_joint'
        self.joint_names[5] = 'lm_q3_joint'
        self.joint_names[6] = 'lr_q1_joint'
        self.joint_names[7] = 'lr_q2_joint'
        self.joint_names[8] = 'lr_q3_joint'
        self.joint_names[9] = 'rf_q1_joint'
        self.joint_names[10] = 'rf_q2_joint'
        self.joint_names[11] = 'rf_q3_joint'
        self.joint_names[12] = 'rm_q1_joint'
        self.joint_names[13] = 'rm_q2_joint'
        self.joint_names[14] = 'rm_q3_joint'
        self.joint_names[15] = 'rr_q1_joint'
        self.joint_names[16] = 'rr_q2_joint'
        self.joint_names[17] = 'rr_q3_joint'

    def broadcast_whole_body_setpoint(self, joint_angles):
        rospy.loginfo("Broadcasting motor setpoint:"+str(joint_angles))

        dqp = SyncWriteMultiFloat()
        dqp.item_name 	= str("goal_position") 	# register to start first write
        dqp.data_length = 4 					# size of register

        for n in range(0,18): 	# loop for each joint
            dqp.joint_name.append(str(self.joint_names[n])) 	# joint name
            dqp.value.append(joint_angles[n])					# joint angle

        self.joint_pub.publish(dqp)

if __name__ == '__main__':
    test_class=basic_rospy_test_class()
    # while(True):
    rospy.loginfo("Waiting for 5 seconds..")
    rospy.sleep(5)
    test_class.broadcast_whole_body_setpoint([0, 0,  0,
                                              0, 0,  0,
                                              0, 0,  0,
                                              0, 0,  0,
                                              0, 0,  0,
                                              0, 0,  0])
    rospy.loginfo("Done")
    rospy.spin()

    # except rospy.ROSInterruptException:
    #     pass
