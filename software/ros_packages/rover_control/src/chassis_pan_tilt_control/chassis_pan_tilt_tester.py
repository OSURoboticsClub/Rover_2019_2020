#!/usr/bin/env python
import rospy
import time

from rover_control.msg import TowerPanTiltControlMessage

DEFAULT_TOWER_PAN_TILT_CONTROL_TOPIC = "chassis/pan_tilt/control"

rospy.init_node("chassis_pan_tilt_tester")

publisher = rospy.Publisher(DEFAULT_TOWER_PAN_TILT_CONTROL_TOPIC, TowerPanTiltControlMessage, queue_size=1)

time.sleep(2)

message = TowerPanTiltControlMessage()
message.should_center = 1

publisher.publish(message)

time.sleep(1)

message = TowerPanTiltControlMessage()
message.relative_pan_adjustment = -100
message.relative_tilt_adjustment = -500

publisher.publish(message)