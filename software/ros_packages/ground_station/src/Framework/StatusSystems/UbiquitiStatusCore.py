# coding=utf-8
#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging
from time import time
import paramiko
from pprint import pprint
import json

#####################################
# Global Variables
#####################################
THREAD_HERTZ = 5

ACCESS_POINT_IP = "192.168.1.20"  # The channel only has to be set on the access point. The staion will adjust.
ACCESS_POINT_USER = "ubnt"
ACCESS_POINT_PASSWORD = "rover4lyfe^"  # We don't care about this password, don't freak out. Wifi is open anyways...

GENERAL_WIRELESS_INFO_COMMAND = "wstalist"


#####################################
# UbiquitiRadioSettings Class Definition
#####################################
class UbiquitiStatus(QtCore.QThread):
    connection_quality_update_ready__signal = QtCore.pyqtSignal(str)
    successful_transmission_update_ready__signal = QtCore.pyqtSignal(str)
    radio_rates_update_ready__signal = QtCore.pyqtSignal(str)
    radio_latency_update_ready__signal = QtCore.pyqtSignal(str)

    def __init__(self, shared_objects):
        super(UbiquitiStatus, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]

        self.connection_quality_label = self.left_screen.connection_quality_label
        self.successful_transmit_label = self.left_screen.successful_transmit_label
        self.radio_rates_label = self.left_screen.radio_rates_label
        self.radio_latency_label = self.left_screen.radio_latency_label

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.wait_time = 1.0 / THREAD_HERTZ

        self.ssh_client = None

    def run(self):
        try:
            self.setup_and_connect_ssh_client()
        except Exception:
            return

        while self.run_thread_flag:
            start_time = time()

            try:
                self.get_and_show_ubiquiti_status()
            except Exception, e:
                print e

            time_diff = time() - start_time

            self.msleep(max(int((self.wait_time - time_diff) * 1000), 0))

    def setup_and_connect_ssh_client(self):
        self.ssh_client = paramiko.SSHClient()
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh_client.connect(ACCESS_POINT_IP, username=ACCESS_POINT_USER, password=ACCESS_POINT_PASSWORD,
                                compress=True)

    def get_and_show_ubiquiti_status(self):
        ssh_stdin, ssh_stdout, ssh_stderr = self.ssh_client.exec_command(GENERAL_WIRELESS_INFO_COMMAND)

        try:
            output_json = json.loads(ssh_stdout.read())[0]

            transmit_percent = output_json["ccq"]
            quality = output_json["airmax"]["quality"]
            # capacity = output_json["airmax"]["capacity"]
            rx_rate = output_json["rx"]
            tx_rate = output_json["tx"]
            ground_tx_latency = output_json["tx_latency"]
            rover_tx_latency = output_json["remote"]["tx_latency"]

        except IndexError:
            transmit_percent = 0
            quality = 0
            # capacity = output_json["airmax"]["capacity"]
            rx_rate = 0
            tx_rate = 0
            ground_tx_latency = "----"
            rover_tx_latency = "----"


        self.connection_quality_update_ready__signal.emit("Connection Quality\n%s %%" % quality)
        self.successful_transmission_update_ready__signal.emit("Successful Transmit\n%s %%" % transmit_percent)
        self.radio_rates_update_ready__signal.emit("TX Rate: %s Mbps\nRX Rate: %s Mbps" % (tx_rate, rx_rate))
        self.radio_latency_update_ready__signal.emit(
            "TX Latency: %s ms\nRX Latency: %s ms" % (ground_tx_latency, rover_tx_latency))

    def connect_signals_and_slots(self):
        self.connection_quality_update_ready__signal.connect(self.connection_quality_label.setText)
        self.successful_transmission_update_ready__signal.connect(self.successful_transmit_label.setText)
        self.radio_rates_update_ready__signal.connect(self.radio_rates_label.setText)
        self.radio_latency_update_ready__signal.connect(self.radio_latency_label.setText)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
