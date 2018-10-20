#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtGui, QtWidgets
import time
import logging
import socket

import rospy

# Custom Imports

#####################################
# Global Variables
#####################################


#####################################
# RoverVideoReceiver Class Definition
#####################################
class ROSMasterChecker(QtCore.QThread):
    def __init__(self):
        super(ROSMasterChecker, self).__init__()

        # ########## Class Variables ##########
        self.ros_master_present = False

        self.start_time = time.time()
        self.start()

    def run(self):
        try:
            master = rospy.get_master()
            master.getPid()
            self.ros_master_present = True
        except socket.error:
            return

    def master_present(self, timeout):
        while self.isRunning() and (time.time() - self.start_time) < timeout:
            self.msleep(100)

        return self.ros_master_present

