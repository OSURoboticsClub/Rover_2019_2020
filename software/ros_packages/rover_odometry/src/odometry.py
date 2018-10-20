#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rospy
import serial
from time import time, sleep
import json
import re

from nmea_msgs.msg import Sentence
from sensor_msgs.msg import Imu

#####################################
# Global Variables
#####################################
NODE_NAME = "tower_odometry"

DEFAULT_PORT = "/dev/rover/ttyOdometry"
# DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200


# DEFAULT_GPS_SENTENCE_TOPIC = "gps/sentence"
DEFAULT_GPS_SENTENCE_TOPIC = "gps/sentence"
DEFAULT_IMU_TOPIC = "imu/data"

DEFAULT_HERTZ = 100

ODOM_LAST_SEEN_TIMEOUT = 1  # seconds


#####################################
# DriveControl Class Definition
#####################################
class Odometry(object):
    def __init__(self):
        rospy.init_node(NODE_NAME)

        self.port = rospy.get_param("~port", DEFAULT_PORT)
        self.baud = rospy.get_param("~baud", DEFAULT_BAUD)

        self.gps_sentence_topic = rospy.get_param("~gps_sentence_topic", DEFAULT_GPS_SENTENCE_TOPIC)
        self.imu_data_topic = rospy.get_param("~imu_data_topic", DEFAULT_IMU_TOPIC)

        self.wait_time = 1.0 / rospy.get_param("~hertz", DEFAULT_HERTZ)

        self.odom_serial = serial.Serial(port=self.port, baudrate=self.baud)
        self.odom_serial.setRTS(0)

        self.sentence_publisher = rospy.Publisher(self.gps_sentence_topic, Sentence, queue_size=1)
        self.imu_data_publisher = rospy.Publisher(self.imu_data_topic, Imu, queue_size=1)

        self.odom_last_seen_time = time()

        self.run()

    def run(self):
        while not rospy.is_shutdown():
            start_time = time()

            try:
                self.process_messages()
                self.odom_last_seen_time = time()

            except Exception, error:
                pass

            if (time() - self.odom_last_seen_time) > ODOM_LAST_SEEN_TIMEOUT:
                print "Odometry not seen for", ODOM_LAST_SEEN_TIMEOUT, "seconds. Exiting."
                return  # Exit so respawn can take over

            time_diff = time() - start_time

            sleep(max(self.wait_time - time_diff, 0))

    def process_messages(self):
        if self.odom_serial.inWaiting():
            line = self.odom_serial.readline()

            temp = json.loads(line)

            gps = temp.get('gps', None)
            imu = temp.get('imu', None)
            imu_cal = temp.get('imu_cal', None)

            if gps:
                # ###### THIS IS HERE TO DEAL WITH UBLOX GPS #####
                if "GNGGA" in gps:
                    gps = gps.replace("GNGGA", "GPGGA")
                    gps = gps[:-2] + str(self.chksum_nmea(gps))[2:]
                # print gps
                # #####

                self.broadcast_gps(gps)

            if imu:
                # print imu
                self.broadcast_imu(imu)

            # if imu_cal:
            #     print imu_cal

    @staticmethod
    def chksum_nmea(sentence):
        # String slicing: Grabs all the characters
        # between '$' and '*' and nukes any lingering
        # newline or CRLF
        chksumdata = re.sub("(\n|\r\n)", "", sentence[sentence.find("$") + 1:sentence.find("*")])

        # Initializing our first XOR value
        csum = 0

        # For each char in chksumdata, XOR against the previous
        # XOR'd char.  The final XOR of the last char will be our
        # checksum to verify against the checksum we sliced off
        # the NMEA sentence

        for c in chksumdata:
            # XOR'ing value of csum against the next char in line
            # and storing the new XOR value in csum
            csum ^= ord(c)

        # Do we have a validated sentence?
        return hex(csum)

    def broadcast_gps(self, gps):
        message = Sentence()
        message.header.frame_id = "gps"
        message.header.stamp = rospy.get_rostime()
        message.sentence = gps
        self.sentence_publisher.publish(message)

    def broadcast_imu(self, imu):
        message = Imu()
        message.header.frame_id = "imu"
        message.header.stamp = rospy.get_rostime()

        message.orientation.x = imu["ox"]
        message.orientation.y = imu["oy"]
        message.orientation.z = imu["oz"]
        message.orientation.w = imu["ow"]

        message.angular_velocity.x = imu["avx"]
        message.angular_velocity.y = imu["avy"]
        message.angular_velocity.z = imu["avz"]

        message.linear_acceleration.x = imu["lax"]
        message.linear_acceleration.y = imu["lay"]
        message.linear_acceleration.z = imu["laz"]

        self.imu_data_publisher.publish(message)


if __name__ == "__main__":
    Odometry()
