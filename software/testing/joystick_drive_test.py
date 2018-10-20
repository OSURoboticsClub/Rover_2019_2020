"""
    This file contains the live logs page sub-class
"""

#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets, QtGui
import logging
from inputs import devices, GamePad
import sys
import time

import rospy
from rover_drive.msg import RoverMotorDrive


#####################################
# Global Variables
#####################################
GAME_CONTROLLER_NAME = "Logitech Logitech Extreme 3D Pro"

#####################################
# Controller Class Definition
#####################################
class LogitechJoystick(QtCore.QThread):
    def __init__(self):
        super(LogitechJoystick, self).__init__()

        # ########## Thread Flags ##########
        self.run_thread_flag = True
        self.setup_controller_flag = True
        self.data_acquisition_and_broadcast_flag = True
        self.controller_aquired = False

        # ########## Class Variables ##########
        self.gamepad = None  # type: GamePad

        self.controller_states = {
            "left_stick_x_axis": 0,
            "left_stick_y_axis": 0,
            "left_stick_center_pressed": 0,

            "right_stick_x_axis": 0,
            "right_stick_y_axis": 0,
            "right_stick_center_pressed": 0,

            "left_trigger_z_axis": 0,
            "left_bumper_pressed": 0,

            "right_trigger_z_axis": 0,
            "right_bumper_pressed": 0,

            "dpad_x": 0,
            "dpad_y": 0,

            "select_pressed": 0,
            "start_pressed": 0,
            "home_pressed": 0,

            "a_pressed": 0,
            "b_pressed": 0,
            "x_pressed": 0,
            "y_pressed": 0
        }

        self.raw_mapping_to_class_mapping = {
            "ABS_X": "left_stick_x_axis",
            "ABS_Y": "left_stick_y_axis",
            "BTN_THUMBL": "left_stick_center_pressed",

            "ABS_RX": "right_stick_x_axis",
            "ABS_RY": "right_stick_y_axis",
            "BTN_THUMBR": "right_stick_center_pressed",

            "ABS_Z": "left_trigger_z_axis",
            "BTN_TL": "left_bumper_pressed",

            "ABS_RZ": "right_trigger_z_axis",
            "BTN_TR": "right_bumper_pressed",

            "ABS_HAT0X": "dpad_x",
            "ABS_HAT0Y": "dpad_y",

            "BTN_SELECT": "select_pressed",
            "BTN_START": "start_pressed",
            "BTN_MODE": "home_pressed",

            "BTN_SOUTH": "a_pressed",
            "BTN_EAST": "b_pressed",
            "BTN_NORTH": "x_pressed",
            "BTN_WEST": "y_pressed"
        }
        self.ready = False

        self.start()


    def run(self):

        while self.run_thread_flag:
            if self.setup_controller_flag:
                self.controller_aquired = self.__setup_controller()
                self.setup_controller_flag = False
            if self.data_acquisition_and_broadcast_flag:
                self.__get_controller_data()


    def __setup_controller(self):
        for device in devices.gamepads:
            if device.name == GAME_CONTROLLER_NAME:
                self.gamepad = device
                return True
        return False


    def __get_controller_data(self):
        if (self.controller_aquired):
            events = self.gamepad.read()

            for event in events:
                if event.code in self.raw_mapping_to_class_mapping:
                    self.controller_states[self.raw_mapping_to_class_mapping[event.code]] = event.state
            self.ready = True
                    # print "Logitech: %s" % self.controller_states





#####################################
# Controller Class Definition
#####################################
class Publisher(QtCore.QThread):
    def __init__(self):
        super(Publisher, self).__init__()

        self.joystick = LogitechJoystick()
        while not self.joystick.ready:
            self.msleep(100)

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        rospy.init_node("drive_tester")

        self.pub = rospy.Publisher("/drive/motoroneandtwo", RoverMotorDrive, queue_size=1)

        self.last_time = time.time()
        self.drive = RoverMotorDrive()
        self.start()

    def run(self):
        while self.run_thread_flag:
            self.__update_and_publish()
            self.msleep(50)

    def __update_and_publish(self):

        axis = self.joystick.controller_states["left_stick_y_axis"]

        self.drive.first_motor_direction = 1 if axis <= 512 else 0
        self.drive.first_motor_speed = min(abs(self.joystick.controller_states["left_stick_y_axis"] - 512) * 128, 65535)

        self.pub.publish(self.drive)

if __name__ == '__main__':
    qapp = QtCore.QCoreApplication(sys.argv)

    joystick = Publisher()

    qapp.exec_()
