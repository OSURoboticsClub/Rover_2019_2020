#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore
import logging
from time import time
import spnav

import rospy

from rover_control.msg import MiningControlMessage

#####################################
# Global Variables
#####################################
THREAD_HERTZ = 100

MINING_CONTROL_TOPIC = "/rover_control/mining/control"

Y_ANGULAR_DEADBAND = 0.05
Z_LINEAR_DEADBAND = 0.15

MINING_LIFT_SCALAR = 5
MINING_TILT_SCALAR = 5


#####################################
# Controller Class Definition
#####################################
class SpaceNavControlSender(QtCore.QThread):
    spacenav_state_update__signal = QtCore.pyqtSignal(object)

    MINING_MODE = 0
    ARM_MODE = 1

    def __init__(self, shared_objects):
        super(SpaceNavControlSender, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.right_screen = self.shared_objects["screens"]["left_screen"]

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.wait_time = 1.0 / THREAD_HERTZ

        self.spnav_states = {
            "linear_x": 0,
            "linear_y": 0,
            "linear_z": 0,

            "angular_x": 0,
            "angular_y": 0,
            "angular_z": 0,

            "panel_pressed": 0,
            "fit_pressed": 0,

            "shift_pressed": 0,
            "alt_pressed": 0,
            "ctrl_pressed": 0,
            "esc_pressed": 0,

            "1_pressed": 0,
            "2_pressed": 0,

            "minus_pressed": 0,
            "plus_pressed": 0,

            "t_pressed": 0,
            "l_pressed": 0,
            "2d_pressed": 0,
            "r_pressed": 0,
            "f_pressed": 0
        }

        self.event_mapping_to_button_mapping = {
            11: "panel_pressed",
            10: "fit_pressed",

            8: "shift_pressed",
            7: "alt_pressed",
            9: "ctrl_pressed",
            6: "esc_pressed",

            0: "1_pressed",
            1: "2_pressed",

            13: "minus_pressed",
            12: "plus_pressed",

            2: "t_pressed",
            3: "l_pressed",
            14: "2d_pressed",
            4: "r_pressed",
            5: "f_pressed"
        }

        # ##### Mining Control #####
        self.mining_control_publisher = rospy.Publisher(MINING_CONTROL_TOPIC, MiningControlMessage, queue_size=1)

        self.current_control_mode = self.MINING_MODE

    def run(self):
        self.logger.debug("Starting SpaceNav Mouse Thread")
        spnav.spnav_open()

        while self.run_thread_flag:

            start_time = time()

            self.process_spnav_events()
            self.check_control_mode_change()
            self.broadcast_control_state()

            time_diff = time() - start_time

            # self.msleep(max(int((self.wait_time - time_diff) * 1000), 0))

        self.logger.debug("Stopping SpaceNav Mouse Thread")

    def process_spnav_events(self):
        event = spnav.spnav_poll_event()

        if event:
            # print event
            if event.ev_type == spnav.SPNAV_EVENT_MOTION:
                self.spnav_states["linear_x"] = event.translation[0] / 350.0
                self.spnav_states["linear_y"] = event.translation[2] / 350.0
                self.spnav_states["linear_z"] = event.translation[1] / 350.0

                self.spnav_states["angular_x"] = event.rotation[2] / 350.0
                self.spnav_states["angular_y"] = -(event.rotation[0] / 350.0)
                self.spnav_states["angular_z"] = -(event.rotation[1] / 350.0)

                # print "x", self.spnav_states["linear_x"], "\t", "y", self.spnav_states["linear_y"], "\t", "z", self.spnav_states["linear_z"]
                # print "x", self.spnav_states["angular_x"], "\t", "y", self.spnav_states["angular_y"], "\t", "z", self.spnav_states["angular_z"]
            else:
                self.spnav_states[self.event_mapping_to_button_mapping[event.bnum]] = event.press

    def check_control_mode_change(self):
        if self.spnav_states["1_pressed"]:
            self.current_control_mode = self.MINING_MODE
        elif self.spnav_states["2_pressed"]:
            self.current_control_mode = self.ARM_MODE

    def broadcast_control_state(self):
        if self.current_control_mode == self.MINING_MODE:
            self.send_mining_commands()
            # self.spacenav_state_update__signal.emit(self.spnav_states)
        elif self.current_control_mode == self.ARM_MODE:
            pass


        # print self.spnav_states["linear_z"], self.spnav_states["angular_y"]

    def connect_signals_and_slots(self):
        pass

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
