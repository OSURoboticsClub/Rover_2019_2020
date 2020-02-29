import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import random

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)

group = moveit_commander.MoveGroupCommander('arm')

pose_goal = geometry_msgs.msg.Pose()
pose_goal = group.get_random_pose()
pose_goal.pose.position.x = 0.2
pose_goal.pose.position.y = 0.2
pose_goal.pose.position.z = 0.2
pose_goal.pose.orientation.x = 0
pose_goal.pose.orientation.y = 0
pose_goal.pose.orientation.z = 0
pose_goal.pose.orientation.w = 0

print pose_goal

group.set_pose_target(pose_goal)
group.go(wait=True)
group.stop()
