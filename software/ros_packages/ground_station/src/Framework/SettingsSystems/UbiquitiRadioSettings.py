# coding=utf-8
#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging
from time import time
import paramiko

#####################################
# Global Variables
#####################################
THREAD_HERTZ = 5

ACCESS_POINT_IP = "192.168.1.20"  # The channel only has to be set on the access point. The staion will adjust.
ACCESS_POINT_USER = "ubnt"
ACCESS_POINT_PASSWORD = "rover4lyfe^"  # We don't care about this password, don't freak out. Wifi is open anyways...

GET_CURRENT_CHANNEL_COMMAND = "iwlist ath0 channel"
SET_CHANNEL_COMMAND = "iwconfig ath0 channel"


#####################################
# UbiquitiRadioSettings Class Definition
#####################################
class UbiquitiRadioSettings(QtCore.QThread):

    show_channel__signal = QtCore.pyqtSignal(int)
    set_gui_elements_enabled__signal = QtCore.pyqtSignal(bool)

    def __init__(self, shared_objects):
        super(UbiquitiRadioSettings, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]

        self.ubiquiti_channel_spin_box = self.left_screen.ubiquiti_channel_spin_box  # type: QtWidgets.QSpinBox
        self.ubiquiti_channel_apply_button = self.left_screen.ubiquiti_channel_apply_button  # type: QtWidgets.QPushButton

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.wait_time = 1.0 / THREAD_HERTZ

        self.channel_change_needed = False
        self.new_channel = 0

        self.ssh_client = None

    def run(self):
        self.set_gui_elements_enabled__signal.emit(False)

        try:
            self.setup_and_connect_ssh_client()
        except Exception:
            return

        self.get_and_show_current_channel()

        while self.run_thread_flag:
            start_time = time()

            self.apply_channel_if_needed()

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

    def setup_and_connect_ssh_client(self):
        self.ssh_client = paramiko.SSHClient()
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh_client.connect(ACCESS_POINT_IP, username=ACCESS_POINT_USER, password=ACCESS_POINT_PASSWORD,
                                compress=True)

    def apply_channel_if_needed(self):
        if self.channel_change_needed:
            self.show_channel__signal.emit(0)
            self.set_gui_elements_enabled__signal.emit(False)
            self.ssh_client.exec_command(SET_CHANNEL_COMMAND + " %02d" % self.new_channel)
            self.get_and_show_current_channel()
            self.channel_change_needed = False

    def get_and_show_current_channel(self):
        channel = 0

        ssh_stdin, ssh_stdout, ssh_stderr = self.ssh_client.exec_command(GET_CURRENT_CHANNEL_COMMAND)
        output = ssh_stdout.read()

        for line in output.split("\n"):
            if "Current Frequency:" in line:
                channel = line.strip("()").split("Channel ")[1]
                break

        self.msleep(500)  # From the gui, this helps show something is actually happening

        self.show_channel__signal.emit(int(channel))
        self.set_gui_elements_enabled__signal.emit(True)

    def on_ubiquiti_channel_apply_pressed__slot(self):
        self.new_channel = self.ubiquiti_channel_spin_box.value()
        self.channel_change_needed = True

    def connect_signals_and_slots(self):
        self.ubiquiti_channel_apply_button.clicked.connect(self.on_ubiquiti_channel_apply_pressed__slot)
        self.show_channel__signal.connect(self.ubiquiti_channel_spin_box.setValue)

        self.set_gui_elements_enabled__signal.connect(self.ubiquiti_channel_spin_box.setEnabled)
        self.set_gui_elements_enabled__signal.connect(self.ubiquiti_channel_apply_button.setEnabled)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
