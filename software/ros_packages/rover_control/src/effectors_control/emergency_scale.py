#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rospy
from time import time, sleep
import serial

from std_msgs.msg import Float64

#####################################
# Global Variables
#####################################
NODE_NAME = "effectors_control"

# ##### Communication Defines #####
DEFAULT_PORT = "/dev/rover/ttyScale"
# DEFAULT_PORT = "/dev/ttyUSB3"
DEFAULT_BAUD = 115200

DEFAULT_TOPIC = "scale/measurement"

CAL_FACTOR = 89500


#####################################
# DriveControl Class Definition
#####################################
class EmergencyScale(object):
    def __init__(self):
        rospy.init_node(NODE_NAME)

        self.scale_serial = serial.Serial(port=DEFAULT_PORT, baudrate=DEFAULT_BAUD, timeout=200)

        self.publisher = rospy.Publisher(DEFAULT_TOPIC, Float64, queue_size=1)

        self.run()

    def run(self):
        sleep(.2)
        self.scale_serial.write(str(CAL_FACTOR))
        sleep(.2)

        while not rospy.is_shutdown():
            value = self.scale_serial.readline()

            try:
                message = Float64()
                message.data = float(value)
                self.publisher.publish(message)
            except:
                print "No data"


if __name__ == "__main__":
    EmergencyScale()
