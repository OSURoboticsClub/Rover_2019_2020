#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rospy

from time import time, sleep

import serial.rs485
import minimalmodbus

# Custom Imports
from rover_control.msg import DriveControlMessage

rospy.init_node("tester")

pub = rospy.Publisher("/drive_control/rear", DriveControlMessage, queue_size=1)

sleep(1)


while True:
	speed = int(input("Enter Speed: "))

	message = DriveControlMessage()
	message.first_motor_speed = speed
	message.first_motor_direction = 1
	message.second_motor_speed = speed

	pub.publish(message)

	sleep(1)

