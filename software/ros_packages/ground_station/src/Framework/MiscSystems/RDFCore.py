# coding=utf-8
#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging
import rospy
from time import time
import scipy.fftpack
import numpy

from std_msgs.msg import Float64MultiArray

#####################################
# Global Variables
#####################################
RDF_DATA_TOPIC = "/rover_science/rdf/data"

THREAD_HERTZ = 5


COLOR_GREEN = "background-color:darkgreen;"
COLOR_RED = "background-color:darkred;"


ALLOWED_RDF_VARIANCE = 0.15


#####################################
# UbiquitiRadioSettings Class Definition
#####################################
class RDF(QtCore.QThread):

    rssi_lcd_number_update_ready__signal = QtCore.pyqtSignal(int)
    beacon_lcd_number_update_ready__signal = QtCore.pyqtSignal(float)

    beacon_valid_text_change_ready__signal = QtCore.pyqtSignal(str)
    beacon_valid_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)

    def __init__(self, shared_objects):
        super(RDF, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]

        self.rssi_lcdnumber = self.left_screen.rssi_lcdnumber  # type:QtWidgets.QLCDNumber
        self.beacon_frequency_lcd_number = self.left_screen.beacon_frequency_lcd_number  # type:QtWidgets.QLCDNumber
        self.beacon_frequency_valid_label = self.left_screen.beacon_frequency_valid_label  # type:QtWidgets.QLCDNumber

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.wait_time = 1.0 / THREAD_HERTZ

        self.rdf_subscriber = rospy.Subscriber(RDF_DATA_TOPIC, Float64MultiArray, self.new_rdf_message_received__callback)

        self.moving_average_raw_data = []

        self.raw_data = numpy.array([])
        self.raw_data_timestamps = numpy.array([])
        self.data_window_size = 200

        self.previous_frequencies = []
        self.num_previous_frequencies = 3

    def run(self):
        self.logger.debug("Starting RDF Thread")

        while self.run_thread_flag:
            start_time = time()

            temp = list(self.moving_average_raw_data)
            if temp:
                average = sum(temp) / len(temp)
                self.rssi_lcd_number_update_ready__signal.emit(average)

            if self.raw_data.size >= self.data_window_size and self.raw_data_timestamps.size >= self.data_window_size:

                try:
                    time_step_array = numpy.array([])
                    for n in range(0, self.data_window_size - 1):
                        time_step_array = numpy.append(time_step_array, self.raw_data_timestamps[n + 1] - self.raw_data_timestamps[n])

                    T = numpy.average(time_step_array)

                    yf = scipy.fftpack.fft(self.raw_data)
                    xf = numpy.linspace(0.0, 1.0 / (2.0 * T), self.data_window_size / 2)

                    valid_range = []

                    for n in range(0, len(xf)):
                        if (xf[n] > 0.5) and (xf[n] <= 5.0):
                            valid_range.append(n)

                    yf = numpy.take(yf, valid_range)
                    xf = numpy.take(xf, valid_range)

                    max_index = numpy.argmax(numpy.abs(yf))
                    freq = xf[max_index]

                    if len(self.previous_frequencies) == self.num_previous_frequencies:
                        del self.previous_frequencies[0]

                    self.previous_frequencies.append(freq)

                    if len(self.previous_frequencies) == self.num_previous_frequencies:
                        variance_too_large = False

                        if abs(self.previous_frequencies[0] - self.previous_frequencies[1]) > ALLOWED_RDF_VARIANCE:
                            variance_too_large = True

                        if abs(self.previous_frequencies[0] - self.previous_frequencies[2]) > ALLOWED_RDF_VARIANCE:
                            variance_too_large = True

                        if abs(self.previous_frequencies[1] - self.previous_frequencies[2]) > ALLOWED_RDF_VARIANCE:
                            variance_too_large = True

                        self.beacon_valid_stylesheet_change_ready__signal.emit(COLOR_GREEN if not variance_too_large else COLOR_RED)
                        self.beacon_valid_text_change_ready__signal.emit("Yes" if not variance_too_large else "No")

                    self.beacon_lcd_number_update_ready__signal.emit(freq)
                except Exception, e:
                    print e

                self.raw_data = numpy.array([])
                self.raw_data_timestamps = numpy.array([])

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

        self.logger.debug("Stopping RDF Thread")

    def new_rdf_message_received__callback(self, data):
        if len(self.moving_average_raw_data) >= self.data_window_size:
            del self.moving_average_raw_data[0]

        self.moving_average_raw_data.append(data.data[0])

        if self.raw_data.size != self.data_window_size and self.raw_data_timestamps.size != self.data_window_size:
            self.raw_data = numpy.append(self.raw_data, data.data[0])
            self.raw_data_timestamps = numpy.append(self.raw_data_timestamps, data.data[1])

    def connect_signals_and_slots(self):
        self.rssi_lcd_number_update_ready__signal.connect(self.rssi_lcdnumber.display)
        self.beacon_lcd_number_update_ready__signal.connect(self.beacon_frequency_lcd_number.display)

        self.beacon_valid_text_change_ready__signal.connect(self.beacon_frequency_valid_label.setText)
        self.beacon_valid_stylesheet_change_ready__signal.connect(self.beacon_frequency_valid_label.setStyleSheet)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
