#!/usr/bin/env python

""" For backwards compatibility with the old driver files
                Will be DELETED in the future               """

__author__ = 'flier'

import argparse
#import Leap
import rospy
import leap_interface
from leap_motion.msg import leap
from leap_motion.msg import leapros
from sensor_msgs.msg import JointState
#from std_msgs.msg import String
#from geometry_msgs.msg import Twist

FREQUENCY_ROSTOPIC_DEFAULT = 0.01
NODENAME = 'leap_pub'
PARAMNAME_FREQ = 'freq'
PARAMNAME_FREQ_ENTIRE = '/' + NODENAME + '/' + PARAMNAME_FREQ
#controller = Leap.Controller()
#hand=Leap.Hand()

def sender():
    '''
    This method publishes the data defined in leapros.msg to /leapmotion/data
    '''
    rospy.loginfo("Parameter set on server: PARAMNAME_FREQ={}".format(rospy.get_param(PARAMNAME_FREQ_ENTIRE, FREQUENCY_ROSTOPIC_DEFAULT)))

    li = leap_interface.Runner()
    li.setDaemon(True)
    li.start()


    # pub     = rospy.Publisher('leapmotion/raw',leap)
    #pub_ros   = rospy.Publisher('leapmotion/data',leapros, queue_size=2)
    pub_cpr   = rospy.Publisher('CPRMoverJointVel', JointState, queue_size=10)
    #pub_cmd   = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node(NODENAME)

    while not rospy.is_shutdown():
        
        #hand_name = "Left hand" #if hand.is_left else "Right hand"

        #if hand.is_valid:    #hand.is_left or hand.is_right:
        #hand_normal_      = li.get_hand_normal()
        joint_vel = JointState()
        #command = String()
        #vel = Twist()
         
        hand_palm_pos_    = li.get_hand_palmpos()

        #joint_vel = JointState()
        if (hand_palm_pos_[0] > 120) or (hand_palm_pos_[2] > 120.00) or (hand_palm_pos_[0] < -120) or (hand_palm_pos_[2] < -120.00):
            joint_vel.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        elif (hand_palm_pos_[1] > 160.00) & (hand_palm_pos_[1] < 200.00):
            joint_vel.velocity = [hand_palm_pos_[2] * -0.5, hand_palm_pos_[0] * -0.5, hand_palm_pos_[0] * 0.5, 0.0, 0.0, 0.0]
        elif (hand_palm_pos_[1] < 160.00) & (hand_palm_pos_[1] > 10.00):
            joint_vel.velocity = [hand_palm_pos_[2] * -0.5, 0.0, 40.0, -40.0, 0.0, 0.0]
        elif (hand_palm_pos_[1] > 200.00):
            joint_vel.velocity = [hand_palm_pos_[2] * -0.5, 0.0, -40.0, 40.0, 0.0, 0.0]
        else:
            joint_vel = JointState()
            joint_vel.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        pub_cpr.publish(joint_vel)
 
        
        # We don't publish native data types, see ROS best practices
        # pub.publish(hand_direction=hand_direction_,hand_normal = hand_normal_, hand_palm_pos = hand_palm_pos_, hand_pitch = hand_pitch_, hand_roll = hand_roll_, hand_yaw = hand_yaw_)
        
        #pub_cmd.publish(vel)
        
        rospy.sleep(rospy.get_param(PARAMNAME_FREQ_ENTIRE, FREQUENCY_ROSTOPIC_DEFAULT))


if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
