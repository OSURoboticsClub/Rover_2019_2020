#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 14 14:44:13 2018
@author: st-ros
"""

import rospy
import arm
from control_msgs.msg import (FollowJointTrajectoryActionFeedback)
from sensor_msgs.msg import (JointState)
from math import pi

RATIOS = [3640, 8400, 6000, 4000, 4500]  #Joint motor counts corresponding to 90 degree rotation

state = JointState()
arm = arm.Arm()

def joint_to_angle(li):
    '''Reorganise waist-to-wrist ordered joint counts into alphabetical angle values for ros publishing'''
    out = [0, 0, 0, 0, 0]
    out[0] = float(-(li[2]*pi/(RATIOS[2]*2)))
    out[1] = float(-(li[3]*pi/(RATIOS[3]*2)))
    out[2] = float(-(li[1]*pi/(RATIOS[1]*2)))
    out[3] = float(-(li[0]*pi/(RATIOS[0]*2)))
    out[4] = float((li[4]*pi/(RATIOS[4]*2)))
    return(out)

def callback(data):
    '''Upon recieving new position data from feedback, update the position data to be published'''
    state.position = data.feedback.actual.positions     #Sets the joint positions in the JointState message to the joint positions in the action feedback
    
def joint_state_remapper():
    '''Continuously publishes the last heard position from JointTrajectoryActionFeedback to the joint_states topic'''
    state.name = ['elbow_joint','hand_joint','shoulder_joint','waist_joint','wrist_joint']
    #arm.connect()   #Initialises connection to r12 arm
    state.position = [0,0,0,0,0]
    state.position = joint_to_angle(arm.read_pos(state.position))    #Checks current position of arm and sets starting position to it
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)    #Initialises publishing
    rospy.init_node('joint_state_remapper', anonymous=True)
    rospy.loginfo('Successful init')
    rospy.loginfo(state.position)
    rate = rospy.Rate(20)
    rospy.Subscriber("/r12_arm_controller/follow_joint_trajectory/feedback", FollowJointTrajectoryActionFeedback, callback)   #Initialises subscribing
    rospy.loginfo('Ready: now remapping joint feedback')
    while not rospy.is_shutdown():
        state.header.stamp = rospy.Duration.from_sec(rospy.get_time())
        pub.publish(state)   #While running, publish last known joint state
        rate.sleep()
        
if __name__ == '__main__':
    joint_state_remapper()
    
