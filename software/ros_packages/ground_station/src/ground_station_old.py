#####################################
# Imports
#####################################
# Python native imports
import sys
from PyQt5 import QtWidgets, QtCore, QtGui, uic
import signal
import rospy
import logging
import qdarkstyle


# Custom Imports
import Framework.StartupSystems.ROSMasterChecker as ROSMasterChecker
import Framework.LoggingSystems.Logger as Logger
import Framework.VideoSystems.RoverVideoCoordinator as RoverVideoCoordinator
import Framework.MapSystems.RoverMapCoordinator as RoverMapCoordinator
import Framework.ControlSystems.DriveAndCameraControlSender as JoystickControlSender
import Framework.ControlSystems.EffectorsAndArmControlSender as ControllerControlSender
import Framework.NavigationSystems.SpeedAndHeadingIndication as SpeedAndHeading
import Framework.NavigationSystems.WaypointsCoordinator as WaypointsCoordinator
import Framework.ArmSystems.ArmIndication as ArmIndication
import Framework.StatusSystems.StatusCore as StatusCore
import Framework.StatusSystems.UbiquitiStatusCore as UbiquitiStatusCore
import Framework.SettingsSystems.UbiquitiRadioSettings as UbiquitiRadioSettings
import Framework.MiscSystems.MiningCore as MiningCore
import Framework.MiscSystems.BashConsoleCore as BashConsoleCore
import Framework.MiscSystems.MiscArmCore as MiscArmCore
import Framework.MiscSystems.RDFCore as RDFCore

#####################################
# Global Variables
#####################################
UI_FILE_LEFT = "Resources/Ui/left_screen.ui"
UI_FILE_RIGHT = "Resources/Ui/right_screen.ui"

#####################################
# Class Organization
#####################################
# Class Name:
#   "init"
#   "run (if there)" - personal pref
#   "private methods"
#   "public methods, minus slots"
#   "slot methods"
#   "static methods"
#   "run (if there)" - personal pref


#####################################
# ApplicationWindow Class Definition
#####################################
class ApplicationWindow(QtWidgets.QMainWindow):
    exit_requested_signal = QtCore.pyqtSignal()

    kill_threads_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None, ui_file_path=None):
        super(ApplicationWindow, self).__init__(parent)

        uic.loadUi(ui_file_path, self)

        QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+Q"), self, self.exit_requested_signal.emit)


#####################################
# GroundStation Class Definition
#####################################
class GroundStation(QtCore.QObject):
    LEFT_SCREEN_ID = 1
    RIGHT_SCREEN_ID = 0

    exit_requested_signal = QtCore.pyqtSignal()

    start_threads_signal = QtCore.pyqtSignal()
    connect_signals_and_slots_signal = QtCore.pyqtSignal()
    kill_threads_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None,):
        # noinspection PyArgumentList
        super(GroundStation, self).__init__(parent)

        # ##### Setup the Logger Immediately #####
        self.logger_setup_class = Logger.Logger(console_output=True)  # Doesn't need to be shared

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        self.shared_objects = {
            "screens": {},
            "regular_classes": {},
            "threaded_classes": {}
        }

        # ###### Instantiate Left And Right Screens ######
        self.shared_objects["screens"]["left_screen"] = \
            self.create_application_window(UI_FILE_LEFT, "Rover Ground Station Left Screen",
                                           self.LEFT_SCREEN_ID)  # type: ApplicationWindow

        self.shared_objects["screens"]["right_screen"] = \
            self.create_application_window(UI_FILE_RIGHT, "Rover Ground Station Right Screen",
                                           self.RIGHT_SCREEN_ID)  # type: ApplicationWindow

        # ###### Initialize the Ground Station Node ######
        rospy.init_node("ground_station")

        # ##### Instantiate Regular Classes ######
        self.__add_non_thread("Mining System", MiningCore.Mining(self.shared_objects))
        self.__add_non_thread("Arm Indication", ArmIndication.ArmIndication(self.shared_objects))

        # ##### Instantiate Threaded Classes ######
        self.__add_thread("Video Coordinator", RoverVideoCoordinator.RoverVideoCoordinator(self.shared_objects))
        self.__add_thread("Map Coordinator", RoverMapCoordinator.RoverMapCoordinator(self.shared_objects))
        self.__add_thread("Joystick Sender", JoystickControlSender.DriveAndCameraControlSender(self.shared_objects))
        self.__add_thread("Controller Sender", ControllerControlSender.EffectorsAndArmControlSender(self.shared_objects))
        self.__add_thread("Speed and Heading", SpeedAndHeading.SpeedAndHeadingIndication(self.shared_objects))
        self.__add_thread("Rover Status", StatusCore.SensorCore(self.shared_objects))
        self.__add_thread("Ubiquiti Status", UbiquitiStatusCore.UbiquitiStatus(self.shared_objects))
        self.__add_thread("Ubiquiti Radio Settings", UbiquitiRadioSettings.UbiquitiRadioSettings(self.shared_objects))
        self.__add_thread("Waypoints Coordinator", WaypointsCoordinator.WaypointsCoordinator(self.shared_objects))
        self.__add_thread("Bash Console", BashConsoleCore.BashConsole(self.shared_objects))
        self.__add_thread("Misc Arm", MiscArmCore.MiscArm(self.shared_objects))
        self.__add_thread("RDF", RDFCore.RDF(self.shared_objects))

        self.connect_signals_and_slots_signal.emit()
        self.__connect_signals_to_slots()
        self.start_threads_signal.emit()

    def ___ros_master_running(self):
        checker = ROSMasterChecker.ROSMasterChecker()

        if not checker.master_present(5):
            self.logger.debug("ROS Master Not Found!!!! Exiting!!!")
            QtGui.QGuiApplication.exit()
            return False
        return True

    def __add_thread(self, thread_name, instance):
        self.shared_objects["threaded_classes"][thread_name] = instance
        instance.setup_signals(self.start_threads_signal, self.connect_signals_and_slots_signal,
                               self.kill_threads_signal)

    def __add_non_thread(self, name, instance):
        self.shared_objects["regular_classes"][name] = instance

    def __connect_signals_to_slots(self):
        self.shared_objects["screens"]["left_screen"].exit_requested_signal.connect(self.on_exit_requested__slot)
        self.shared_objects["screens"]["right_screen"].exit_requested_signal.connect(self.on_exit_requested__slot)

    def on_exit_requested__slot(self):
        self.kill_threads_signal.emit()

        # Wait for Threads
        for thread in self.shared_objects["threaded_classes"]:
            self.shared_objects["threaded_classes"][thread].wait()

        QtGui.QGuiApplication.exit()

    @staticmethod
    def create_application_window(ui_file_path, title, display_screen):
        system_desktop = QtWidgets.QDesktopWidget()  # This gets us access to the desktop geometry

        app_window = ApplicationWindow(parent=None, ui_file_path=ui_file_path)  # Make a window in this application
        app_window.setWindowTitle(title)  # Sets the window title

        app_window.setWindowFlags(app_window.windowFlags() |  # Sets the windows flags to:
                                  QtCore.Qt.FramelessWindowHint |  # remove the border and frame on the application,
                                  QtCore.Qt.WindowStaysOnTopHint |  # and makes the window stay on top of all others
                                  QtCore.Qt.X11BypassWindowManagerHint)  # This is needed to show fullscreen in gnome

        app_window.setGeometry(
            system_desktop.screenGeometry(display_screen))  # Sets the window to be on the first screen

        app_window.showFullScreen()  # Shows the window in full screen mode

        return app_window


#####################################
# Main Definition
#####################################
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)  # This allows the keyboard interrupt kill to work properly

    # ########## Start the QApplication Framework ##########
    application = QtWidgets.QApplication(sys.argv)  # Create the ase qt gui application
    application.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())

    # ########## Set Organization Details for QSettings ##########
    QtCore.QCoreApplication.setOrganizationName("OSURC")
    QtCore.QCoreApplication.setOrganizationDomain("http://osurobotics.club/")
    QtCore.QCoreApplication.setApplicationName("groundstation")

    # ########## Check ROS Master Status ##########
    master_checker = ROSMasterChecker.ROSMasterChecker()

    if not master_checker.master_present(5):
        message_box = QtWidgets.QMessageBox()
        message_box.setWindowTitle("Rover Ground Station")
        message_box.setText("Connection to ROS Master Failed!!!\n" +
                            "Ensure ROS master is running or check for network issues.")
        message_box.exec_()
        exit()

    # ########## Start Ground Station If Ready ##########
    ground_station = GroundStation()
    application.exec_()  # Execute launching of the application
