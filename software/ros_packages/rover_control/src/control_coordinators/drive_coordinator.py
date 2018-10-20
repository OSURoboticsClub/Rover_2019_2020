#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rospy
from PyQt5 import QtCore
from time import time, sleep

# Custom Imports
from rover_control.msg import DriveCommandMessage, DriveControlMessage

#####################################
# Global Variables
#####################################
NODE_NAME = "drive_coordinator"

DEFAULT_IRIS_DRIVE_COMMAND_TOPIC = "command_control/iris_drive"
DEFAULT_GROUND_STATION_DRIVE_COMMAND_TOPIC = "command_control/ground_station_drive"
DEFAULT_REAR_BOGIE_TOPIC = "drive_control/rear"
DEFAULT_LEFT_BOGIE_TOPIC = "drive_control/left"
DEFAULT_RIGHT_BOGIE_TOPIC = "drive_control/right"

UINT16_MAX = 65535

DEFAULT_HERTZ = 30

WATCHDOG_TIMEOUT = 0.3


#####################################
# ControlCoordinator Class Definition
#####################################
class DriveCoordinator(object):
    def __init__(self):

        rospy.init_node(NODE_NAME)

        self.iris_drive_command_topic = rospy.get_param("~iris_drive_command_topic", DEFAULT_IRIS_DRIVE_COMMAND_TOPIC)
        self.ground_station_drive_command_topic = \
            rospy.get_param("~ground_station_drive_command_topic", DEFAULT_GROUND_STATION_DRIVE_COMMAND_TOPIC)
        self.rear_bogie_topic = rospy.get_param("~rear_bogie_control_topic", DEFAULT_REAR_BOGIE_TOPIC)
        self.left_bogie_topic = rospy.get_param("~left_bogie_control_topic", DEFAULT_LEFT_BOGIE_TOPIC)
        self.right_bogie_topic = rospy.get_param("~right_bogie_control_topic", DEFAULT_RIGHT_BOGIE_TOPIC)

        self.wait_time = 1.0 / rospy.get_param("~hertz", DEFAULT_HERTZ)

        # Drive data
        self.drive_command_data = {
            "iris": {
                "message": DriveCommandMessage(),
                "last_time": time()
            },

            "ground_station": {
                "message": DriveCommandMessage(),
                "last_time": time()
            }
        }

        # ########## Class Variables ##########
        self.iris_drive_command_subscriber = rospy.Subscriber(self.iris_drive_command_topic,
                                                              DriveCommandMessage,
                                                              self.iris_drive_command_callback)

        self.ground_station_command_subscriber = rospy.Subscriber(self.ground_station_drive_command_topic,
                                                                  DriveCommandMessage,
                                                                  self.ground_station_drive_command_callback)

        self.rear_bogie_publisher = rospy.Publisher(self.rear_bogie_topic, DriveControlMessage, queue_size=1)
        self.left_bogie_publisher = rospy.Publisher(self.left_bogie_topic, DriveControlMessage, queue_size=1)
        self.right_bogie_publisher = rospy.Publisher(self.right_bogie_topic, DriveControlMessage, queue_size=1)

        self.last_message_time = time()

        # ########## Run the Class ##########
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            start_time = time()

            try:
                self.process_drive_commands()
            except Exception, error:
                print "COORDINATOR: Error occurred:", error

            time_diff = time() - start_time

            sleep(max(self.wait_time - time_diff, 0))

    def process_drive_commands(self):
        if not self.drive_command_data["iris"]["message"].ignore_drive_control:
            self.send_drive_control_command(self.drive_command_data["iris"])
        else:
            self.send_drive_control_command(self.drive_command_data["ground_station"])

    def send_drive_control_command(self, drive_command_data):

        if (time() - drive_command_data["last_time"]) > WATCHDOG_TIMEOUT:
            drive_command = DriveCommandMessage()
        else:
            drive_command = drive_command_data["message"]

        rear_drive = DriveControlMessage()
        left_drive = DriveControlMessage()
        right_drive = DriveControlMessage()

        left = drive_command.drive_twist.linear.x - drive_command.drive_twist.angular.z
        right = drive_command.drive_twist.linear.x + drive_command.drive_twist.angular.z

        left_direction = 1 if left >= 0 else 0
        rear_drive.first_motor_direction = left_direction
        left_drive.first_motor_direction = left_direction
        left_drive.second_motor_direction = left_direction

        right_direction = 1 if right >= 0 else 0
        rear_drive.second_motor_direction = right_direction
        right_drive.first_motor_direction = right_direction
        right_drive.second_motor_direction = right_direction

        left_speed = min(abs(left * UINT16_MAX), UINT16_MAX)
        right_speed = min(abs(right * UINT16_MAX), UINT16_MAX)

        rear_drive.first_motor_speed = left_speed
        left_drive.first_motor_speed = left_speed
        left_drive.second_motor_speed = left_speed

        rear_drive.second_motor_speed = right_speed
        right_drive.first_motor_speed = right_speed
        right_drive.second_motor_speed = right_speed

        self.rear_bogie_publisher.publish(rear_drive)
        self.left_bogie_publisher.publish(left_drive)
        self.right_bogie_publisher.publish(right_drive)

    def iris_drive_command_callback(self, drive_command):
        self.drive_command_data["iris"]["message"] = drive_command
        self.drive_command_data["iris"]["last_time"] = time()

    def ground_station_drive_command_callback(self, drive_command):
        self.drive_command_data["ground_station"]["message"] = drive_command
        self.drive_command_data["ground_station"]["last_time"] = time()


if __name__ == '__main__':
    DriveCoordinator()
