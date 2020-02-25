#! /usr/bin/env python

import rospy
import actionlib
import arm

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)

from trajectory_msgs.msg import (
    JointTrajectoryPoint
)

from math import pi
import time
import operator
import string

arm = arm.Arm()

GOAL_TOLERANCE = [0.05, 0.05, 0.05, 0.05, 0.05]  #Tolerance joints must be within to report success, in radians

RATIOS = [3640, 8400, 6000, 4000, 4500]  #Joint motor counts corresponding to 90 degree rotation

# Assumption: Strings end with a keyword, followed only by ASCII
# whitespace and >, which is to be stripped off.
OUTPUT_STRIP_CHARS = string.whitespace + '>'


def joint_to_angle(li):
    '''Reorganise waist-to-wrist ordered joint counts into alphabetical angle values for ros publishing'''
    out = [0, 0, 0, 0, 0]
    out[0] = float(-(li[2]*pi/(RATIOS[2]*2)))
    out[1] = float(-(li[3]*pi/(RATIOS[3]*2)))
    out[2] = float(-(li[1]*pi/(RATIOS[1]*2)))
    out[3] = float(-(li[0]*pi/(RATIOS[0]*2)))
    out[4] = float((li[4]*pi/(RATIOS[4]*2)))
    return(out)
    
def angle_to_joint(li):
    '''Reorganise alphabetical angle values into waist-to-wrist ordered joint counts for sending to arm'''
    out = [0, 0, 0, 0, 0]
    out[0] = int(round(-(li[3]*2*RATIOS[0]/pi)))   #joint coordinates come as radians, sorted by joint name in alphabetical order; reoorganise and scale by joint count ratios
    out[1] = int(round(-(li[2]*2*RATIOS[1]/pi)))
    out[2] = int(round(-(li[0]*2*RATIOS[2]/pi)))
    out[3] = int(round(-(li[1]*2*RATIOS[3]/pi)))
    out[4] = int(round(li[4]*2*RATIOS[4]/pi))
    return(out)

class JointTrajectoryActionServer(object):
    '''Joint trajectory action server allowing joint position based control of r12 arm'''
    def __init__(self, controller_name):
        self._action_ns = controller_name + '/follow_joint_trajectory'
        self._as = actionlib.SimpleActionServer(
                self._action_ns,
                FollowJointTrajectoryAction,
                execute_cb=self._execute_cb,
                auto_start = False)
        self._action_name = rospy.get_name()
        self._as.start()
        self._feedback = FollowJointTrajectoryFeedback()
        self._result = FollowJointTrajectoryResult()
        self._rate = rospy.Rate(20)
        rospy.loginfo('Successful init')
        
    def _check_goal(self, pos, final_pos, tolerances):
        '''Checks if all joints are within tolerance of final position, and returns true if so'''
        errors = map(operator.sub,
                     pos,
                     final_pos
                     )  #obtains a list of differences between actual position and final desired position
        errors_tolerances = zip(errors, tolerances)
        for ele in errors_tolerances:
            if (abs(ele[0]) > ele[1]) : #compares absolute angle difference from goal with tolerance for that joint, for each joint
                return False
        return True    #if all joints within tolerance, goal achieved so return true
        
    def _update_feedback(self, joint_names, last_pos, time_elapsed):
        '''Updates feedback with current joint positions and returns current position as angles'''
        new_pos = joint_to_angle(arm.read_pos(angle_to_joint(last_pos)))
        self._feedback.header.stamp = rospy.Duration.from_sec(rospy.get_time())
        self._feedback.joint_names = joint_names
        self._feedback.desired.positions = new_pos   #joint positions are given as a list at the start- no way of checking current command point so just fake by using current position
        self._feedback.desired.time_from_start = rospy.Duration.from_sec(time_elapsed)
        self._feedback.actual.positions = new_pos
        self._feedback.actual.time_from_start = rospy.Duration.from_sec(time_elapsed)
        self._feedback.error.positions = map(operator.sub,
                                         self._feedback.desired.positions,
                                         self._feedback.actual.positions
                                         )
        self._feedback.error.time_from_start = rospy.Duration.from_sec(time_elapsed)
        self._as.publish_feedback(self._feedback)
        return new_pos
        
    def _execute_cb(self, goal):
        '''Initialises and runs route, and updates feedback until complete or time duration violated'''
        rospy.loginfo('Started cb')
        start_time = rospy.get_time()
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points
        if joint_to_angle(trajectory_points[0].positions) == joint_to_angle(trajectory_points[1].positions):   #given trajectory sometimes has two identical points at start of route, which causes errors
            trajectory_points = trajectory_points[1:]
        arm.write('FIND TRAJECTORY .\r\n')    #returns memory location of route TRAJECTORY if it exists in memory, else returns 0
        route_exists_check = arm.read()
        route_exists_check = route_exists_check.strip(OUTPUT_STRIP_CHARS)
        if (route_exists_check[-1]) <> 0:    #if FIND does not return 0, TRAJECTORY already exists in memory
            arm.write('FORGET TRAJECTORY\r\n')
        arm.write('ROUTE TRAJECTORY\r\n')    #create a new route TRAJECTORY
        arm.write('500 RESERVE\r\n')    #reserve space in memory of route points
        joint_values = [0, 0, 0, 0, 0]    #initialise a list to hold joint values at each point in the trajectory
        for point in trajectory_points:    #for each point in the trajectory, create a string of the form a b c d e $JL where variables are joint counts for each joint
            position_input = ''
            joint_values = angle_to_joint(point.positions)    #obtains joint counts from received joint angles
            for i in range (4,-1,-1):
                position_input += str(joint_values[i])
                position_input += ' '
            position_input += '$JL\r\n'
            arm.write(position_input)   #sends created string to arm, teaching point in trajectory
        print(arm.read())
        arm.write('$RUN\r\n')   #run created route
        end_time = trajectory_points[-1].time_from_start.to_sec()    #get expected time for route to complete
        time_elapsed = rospy.get_time() - start_time
        pos = trajectory_points[0].positions    #initialises current position to starting position
        rospy.loginfo('Started route')
        while (time_elapsed < end_time and not rospy.is_shutdown()):    #loop until timeout
            self._rate.sleep()
            if self._as.is_preempt_requested():
                rospy.loginfo("%s: Joint trajectory action preempted, stopping arm" % (self._action_name))
                arm.stop_route()   #sends stop command to arm
                pos = self._update_feedback(joint_names, pos, time_elapsed)   #updates feedback with final position
                self._as.set_preempted()   #cpnfirms action status as prempted and ends action
                break                
            result = self._check_goal(pos, trajectory_points[-1].positions, GOAL_TOLERANCE)
            pos = self._update_feedback(joint_names, pos, time_elapsed)    #updates and publishes feedback, and sets pos to current position
            if result:
                rospy.loginfo("%s: Joint trajectory action succeeded" % (self._action_name))
                self._result.error_code = self._result.SUCCESSFUL
                self._as.set_succeeded(self._result)
                break    #on success, finish

        
if __name__ == '__main__':
    rospy.init_node('r12_interface')
    port = arm.connect()     #Find r12 arm and connect to port where it is located
    rospy.loginfo('Success: Connected to ${0}.'.format(port))
    server = JointTrajectoryActionServer('r12_arm_controller')
    rospy.spin()    
