#!/usr/bin/env python

"""
    Main file used to launch the Rover Base Station
    No other files should be used for launching this application.
"""

#####################################
# Imports
#####################################
# Python native imports
import sys
from PyQt5 import QtWidgets, QtCore, uic
import signal

# Custom Imports

#####################################
# Global Variables
#####################################
UI_FILE_LEFT = "Resources/UI/RoverGui.ui"
UI_FILE_RIGHT = "Resources/UI/RoverGui2.ui"

FORM_LEFT, BASE_UI_LEFT = uic.loadUiType(UI_FILE_LEFT)
FORM_RIGHT, BASE_UI_RIGHT = uic.loadUiType(UI_FILE_RIGHT)

LEFT_SCREEN_ID = 0
RIGHT_SCREEN_ID = 1


#####################################
# Application Class Definition
#####################################
class LeftWindow(BASE_UI_LEFT, FORM_LEFT):
    kill_threads_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        # noinspection PyArgumentList
        super(BASE_UI_LEFT, self).__init__(parent)
        self.setupUi(self)


#####################################
# Application Class Definition
#####################################
class RightWindow(BASE_UI_RIGHT, FORM_RIGHT):
    kill_threads_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        # noinspection PyArgumentList
        super(BASE_UI_RIGHT, self).__init__(parent)
        self.setupUi(self)


#####################################
# Main Definition
#####################################
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)  # This allows the keyboard interrupt kill to work  properly

    application = QtWidgets.QApplication(sys.argv)  # Create the base qt gui application

    system_desktop = QtWidgets.QDesktopWidget()  # This gets us access to the desktop geometry

    app_window = LeftWindow()  # Make a window in this application
    app_window.setWindowTitle("Rover Control")  # Sets the window title

    app_window.setWindowFlags(app_window.windowFlags() |  # Sets the windows flags to:
                              QtCore.Qt.FramelessWindowHint |  # remove the border and frame on the application,
                              QtCore.Qt.WindowStaysOnTopHint |  # and makes the window stay on top of all others
                              QtCore.Qt.X11BypassWindowManagerHint)  # This is needed to show fullscreen in gnome

    app_window.setGeometry(system_desktop.screenGeometry(LEFT_SCREEN_ID))  # Sets the window to be on the first screen

    app_window.showFullScreen()  # Shows the window in full screen mode

    app_window2 = RightWindow()
    app_window2.setWindowTitle("Rover Video")
    app_window2.setWindowFlags(app_window.windowFlags() |
                               QtCore.Qt.FramelessWindowHint |
                               QtCore.Qt.WindowStaysOnTopHint |
                               QtCore.Qt.X11BypassWindowManagerHint)

    app_window2.setGeometry(system_desktop.screenGeometry(RIGHT_SCREEN_ID))

    app_window2.showFullScreen()

    application.exec_()  # Execute launching of the application
