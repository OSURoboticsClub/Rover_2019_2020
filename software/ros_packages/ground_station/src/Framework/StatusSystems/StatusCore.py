#!/usr/bin/env python
# coding=utf-8

import rospy
from rover_status.msg import *
from PyQt5 import QtWidgets, QtCore, QtGui, uic
from std_msgs.msg import Empty
import PIL.Image
from PIL.ImageQt import ImageQt
import time

from std_msgs.msg import UInt16

# import Timer

REQUEST_UPDATE_TOPIC = "/rover_status/update_requested"

CAMERA_TOPIC_NAME = "/rover_status/camera_status"
BOGIE_TOPIC_NAME = "/rover_status/bogie_status"
FRSKY_TOPIC_NAME = "/rover_status/frsky_status"
GPS_TOPIC_NAME = "/rover_status/gps_status"
JETSON_TOPIC_NAME = "/rover_status/jetson_status"
MISC_TOPIC_NAME = "/rover_status/misc_status"
BATTERY_TOPIC_NAME = "/rover_status/battery_status"
CO2_TOPIC_NAME = "/rover_control/tower/status/co2"

COLOR_GREEN = "background-color: darkgreen; border: 1px solid black;"
COLOR_ORANGE = "background-color: orange; border: 1px solid black;"
COLOR_YELLOW = "background-color: rgb(204,204,0); border: 1px solid black; color: black;"
COLOR_RED = "background-color: darkred; border: 1px solid black;"

GPS_BEST_CASE_ACCURACY = 3

LOW_BATTERY_DIALOG_TIMEOUT = 120
CRITICAL_BATTERY_DIALOG_TIMEOUT = 30


class SensorCore(QtCore.QThread):
    # ########## create signals for slots ##########
    jetson_cpu_update_ready__signal = QtCore.pyqtSignal(str)
    jetson_cpu_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)
    jetson_emmc_update_ready__signal = QtCore.pyqtSignal(str)
    jetson_emmc_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)
    jetson_gpu_temp_update_ready__signal = QtCore.pyqtSignal(str)
    jetson_gpu_temp_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)
    jetson_ram_update_ready__signal = QtCore.pyqtSignal(str)
    jetson_ram_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)

    bogie_connection_1_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)
    bogie_connection_2_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)
    bogie_connection_3_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)

    gps_fix_update_ready__signal = QtCore.pyqtSignal(str)
    gps_heading_valid_update_ready__signal = QtCore.pyqtSignal(str)
    gps_num_satellites_update_ready__signal = QtCore.pyqtSignal(str)
    gps_accuracy_update_ready__signal = QtCore.pyqtSignal(str)

    co2_levels_update_ready__signal = QtCore.pyqtSignal(str)

    camera_zed_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)
    camera_under_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)
    camera_chassis_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)
    camera_main_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)

    gps_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)

    frsky_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)

    battery_voltage_update_ready__signal = QtCore.pyqtSignal(str)
    battery_voltage_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)

    show_battery_low__signal = QtCore.pyqtSignal()
    show_battery_critical__signal = QtCore.pyqtSignal()

    def __init__(self, shared_objects):
        super(SensorCore, self).__init__()

        self.run_thread_flag = True

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.screen_main_window = self.shared_objects["screens"]["left_screen"]

        # self.cpu_read = self.screen_main_window.lineEdit  # type: QtWidgets.QLabel
        # self.ram_read = self.screen_main_window.lineEdit_2  # type: QtWidgets.QLabel

        # ########## set vars to gui elements
        self.osurc_logo_label = self.screen_main_window.osurc_logo_label  # type: QtWidgets.QLabel
        self.rover_conn = self.screen_main_window.rover  # type: QtWidgets.QLabel
        self.frsky = self.screen_main_window.frsky  # type: QtWidgets.QLabel
        self.nav_mouse = self.screen_main_window.nav_mouse  # type: QtWidgets.QLabel
        self.joystick = self.screen_main_window.joystick  # type: QtWidgets.QLabel
        self.gps_fix_label = self.screen_main_window.gps_fix_label  # type: QtWidgets.QLabel
        self.gps_heading_valid_label = self.screen_main_window.gps_heading_valid_label  # type: QtWidgets.QLabel
        self.gps_num_satellites_label = self.screen_main_window.gps_num_satellites_label  # type: QtWidgets.QLabel
        self.gps_accuracy_label = self.screen_main_window.gps_accuracy_label  # type: QtWidgets.QLabel
        self.zed = self.screen_main_window.zed  # type: QtWidgets.QLabel
        self.main_cam = self.screen_main_window.main_cam  # type: QtWidgets.QLabel
        self.chassis_cam = self.screen_main_window.chassis_cam  # type: QtWidgets.QLabel
        self.under_cam = self.screen_main_window.under_cam  # type: QtWidgets.QLabel
        self.clock = self.screen_main_window.clock_qlcdnumber  # type: QtWidgets.QLCDNumber
        self.cpu = self.screen_main_window.cpu  # type: QtWidgets.QLabel
        self.ram = self.screen_main_window.ram  # type: QtWidgets.QLabel
        self.gpu_temp = self.screen_main_window.gpu_temp  # type: QtWidgets.QLabel
        self.emmc = self.screen_main_window.emmc  # type: QtWidgets.QLabel
        self.battery = self.screen_main_window.battery_voltage_status_label  # type: QtWidgets.QLabel
        self.co2_levels_label = self.screen_main_window.co2_levels_label  # type: QtWidgets.QLabel

        # ########## subscriptions pulling data from system_statuses_node.py ##########
        self.camera_status = rospy.Subscriber(CAMERA_TOPIC_NAME, CameraStatuses, self.__camera_callback)
        self.frsky_status = rospy.Subscriber(FRSKY_TOPIC_NAME, FrSkyStatus, self.__frsky_callback)
        self.gps_status = rospy.Subscriber(GPS_TOPIC_NAME, GPSInfo, self.__gps_callback)
        self.jetson_status = rospy.Subscriber(JETSON_TOPIC_NAME, JetsonInfo, self.__jetson_callback)
        self.misc_status = rospy.Subscriber(MISC_TOPIC_NAME, MiscStatuses, self.__misc_callback)
        self.battery_status = rospy.Subscriber(BATTERY_TOPIC_NAME, BatteryStatusMessage, self.__battery_callback)
        self.co2_status = rospy.Subscriber(CO2_TOPIC_NAME, UInt16, self.__co2_callback)

        self.camera_msg = CameraStatuses()
        self.bogie_msg = None  # BogieStatuses()
        self.FrSky_msg = FrSkyStatus()
        self.GPS_msg = GPSInfo()
        self.jetson_msg = JetsonInfo()
        self.misc_msg = MiscStatuses()
        self.battery_msg = BatteryStatusMessage()

        self.update_requester = rospy.Publisher(REQUEST_UPDATE_TOPIC, Empty, queue_size=10)

        # Apply OSURC Logo
        self.osurc_logo_pil = PIL.Image.open("Resources/Images/osurclogo.png").resize((210, 75), PIL.Image.BICUBIC)
        self.osurc_logo_pixmap = QtGui.QPixmap.fromImage(ImageQt(self.osurc_logo_pil))
        self.osurc_logo_label.setPixmap(self.osurc_logo_pixmap)  # Init should be in main thread, should be fine

        self.low_battery_warning_dialog = QtWidgets.QMessageBox()
        self.low_battery_warning_dialog.setIcon(QtWidgets.QMessageBox.Warning)
        self.low_battery_warning_dialog.setText("\n\n\n\nRover battery low!\nReturn and charge soon to avoid battery damage!\n\n\n\n")
        self.low_battery_warning_dialog.setWindowTitle("Low Battery")
        self.low_battery_warning_dialog.setStandardButtons(QtWidgets.QMessageBox.Ok)
        self.low_battery_warning_dialog.setWindowFlags(
            QtCore.Qt.WindowStaysOnTopHint | QtCore.Qt.X11BypassWindowManagerHint | QtCore.Qt.WindowTitleHint | QtCore.Qt.WindowCloseButtonHint)
        self.low_battery_warning_dialog.setStyleSheet(COLOR_YELLOW)

        self.critical_battery_warning_dialog = QtWidgets.QMessageBox()
        self.critical_battery_warning_dialog.setIcon(QtWidgets.QMessageBox.Critical)
        self.critical_battery_warning_dialog.setText(
            "\n\n\n\nRover battery critical!\nPower down immediately or battery damage will occur!\n\n\n\n")
        self.critical_battery_warning_dialog.setWindowTitle("Critical Battery")
        self.critical_battery_warning_dialog.setStandardButtons(QtWidgets.QMessageBox.Ok)
        self.critical_battery_warning_dialog.setWindowFlags(
            QtCore.Qt.WindowStaysOnTopHint | QtCore.Qt.X11BypassWindowManagerHint | QtCore.Qt.WindowTitleHint | QtCore.Qt.WindowCloseButtonHint)
        self.critical_battery_warning_dialog.setStyleSheet(COLOR_RED)

        self.low_battery_warning_last_shown = 0
        self.critical_battery_warning_last_shown = 0

    def __camera_callback(self, data):
        self.camera_msg.camera_zed = data.camera_zed
        self.camera_msg.camera_undercarriage = data.camera_undercarriage
        self.camera_msg.camera_chassis = data.camera_chassis
        self.camera_msg.camera_main_navigation = data.camera_main_navigation

        if data.camera_zed is False:
            # self.zed.setStyleSheet("background-color: red;")
            self.camera_zed_stylesheet_change_ready__signal.emit(COLOR_RED)
        else:
            # self.zed.setStyleSheet(COLOR_GREEN)
            self.camera_zed_stylesheet_change_ready__signal.emit(COLOR_GREEN)

        if data.camera_undercarriage is False:
            # self.under_cam.setStyleSheet(COLOR_RED)
            self.camera_under_stylesheet_change_ready__signal.emit(COLOR_RED)
        else:
            # self.under_cam.setStyleSheet(COLOR_GREEN)
            self.camera_under_stylesheet_change_ready__signal.emit(COLOR_GREEN)

        if data.camera_chassis is False:
            # self.chassis_cam.setStyleSheet(COLOR_RED)
            self.camera_chassis_stylesheet_change_ready__signal.emit(COLOR_RED)
        else:
            # self.chassis_cam.setStyleSheet(COLOR_GREEN)
            self.camera_chassis_stylesheet_change_ready__signal.emit(COLOR_GREEN)

        if data.camera_main_navigation is False:
            # self.main_cam.setStyleSheet(COLOR_RED)
            self.camera_main_stylesheet_change_ready__signal.emit(COLOR_RED)
        else:
            # self.main_cam.setStyleSheet(COLOR_GREEN)
            self.camera_main_stylesheet_change_ready__signal.emit(COLOR_GREEN)

    def __frsky_callback(self, data):
        self.FrSky_msg.FrSky_controller_connection_status = data.FrSky_controller_connection_status

        if self.FrSky_msg.FrSky_controller_connection_status is False:
            self.frsky_stylesheet_change_ready__signal.emit(COLOR_RED)
        else:
            self.frsky_stylesheet_change_ready__signal.emit(COLOR_GREEN)

    def __jetson_callback(self, data):
        self.jetson_cpu_update_ready__signal.emit("TX2 CPU\n" + str(data.jetson_CPU) + " %")

        if data.jetson_CPU > 85:
            self.jetson_cpu_stylesheet_change_ready__signal.emit(COLOR_ORANGE)
        elif data.jetson_CPU > 95:
            self.jetson_cpu_stylesheet_change_ready__signal.emit(COLOR_RED)
        else:
            self.jetson_cpu_stylesheet_change_ready__signal.emit(COLOR_GREEN)

        self.jetson_ram_update_ready__signal.emit("TX2 RAM\n" + str(data.jetson_RAM) + " %")

        if data.jetson_RAM > 79:
            self.jetson_ram_stylesheet_change_ready__signal.emit(COLOR_ORANGE)
        elif data.jetson_RAM > 89:
            self.jetson_ram_stylesheet_change_ready__signal.emit(COLOR_RED)
        else:
            self.jetson_ram_stylesheet_change_ready__signal.emit(COLOR_GREEN)

        self.jetson_gpu_temp_update_ready__signal.emit("TX2 TEMP\n" + str(data.jetson_GPU_temp) + " °C")

        if data.jetson_GPU_temp > 64:
            self.jetson_gpu_temp_stylesheet_change_ready__signal.emit(COLOR_ORANGE)
        elif data.jetson_GPU_temp > 79:
            self.jetson_gpu_temp_stylesheet_change_ready__signal.emit(COLOR_RED)
        else:
            self.jetson_gpu_temp_stylesheet_change_ready__signal.emit(COLOR_GREEN)

        self.jetson_emmc_update_ready__signal.emit("TX2 EMMC\n" + str(data.jetson_EMMC) + " %")

        if data.jetson_EMMC > 79:
            self.jetson_emmc_stylesheet_change_ready__signal.emit(COLOR_ORANGE)
        elif data.jetson_EMMC > 89:
            self.jetson_emmc_stylesheet_change_ready__signal.emit(COLOR_RED)
        else:
            self.jetson_emmc_stylesheet_change_ready__signal.emit(COLOR_GREEN)

    def __gps_callback(self, data):

        if data.gps_connected:
            self.gps_fix_update_ready__signal.emit("GPS Fix\nTrue")
            self.gps_stylesheet_change_ready__signal.emit(COLOR_GREEN)

        else:
            self.gps_stylesheet_change_ready__signal.emit(COLOR_RED)
            self.gps_fix_update_ready__signal.emit("GPS Fix\nFalse")

        if data.gps_heading != -1:
            self.gps_heading_valid_update_ready__signal.emit("GPS Heading Valid\nTrue")
        else:
            self.gps_heading_valid_update_ready__signal.emit("GPS Heading Valid\nFalse")

        self.gps_num_satellites_update_ready__signal.emit("GPS Satellites\n%s" % data.num_satellites)
        self.gps_accuracy_update_ready__signal.emit(
            "GPS Accuracy\n%2.2f m" % (data.horizontal_dilution * GPS_BEST_CASE_ACCURACY))

    def __misc_callback(self, data):
        self.misc_msg.arm_connection_status = data.arm_connection_status
        self.misc_msg.arm_end_effector_connection_statuses = data.arm_end_effector_connection_statuses
        self.misc_msg.sample_containment_connection_status = data.sample_containment_connection_status
        self.misc_msg.tower_connection_status = data.tower_connection_status
        self.misc_msg.chassis_pan_tilt_connection_status = data.chassis_pan_tilt_connection_status

    def __battery_callback(self, data):
        voltage = data.battery_voltage / 1000.0

        if voltage >= 21:
            self.battery_voltage_stylesheet_change_ready__signal.emit(COLOR_GREEN)
        elif voltage >= 19:
            self.battery_voltage_stylesheet_change_ready__signal.emit(COLOR_YELLOW)

            if (time.time() - self.low_battery_warning_last_shown) > LOW_BATTERY_DIALOG_TIMEOUT:
                self.show_battery_low__signal.emit()
                self.low_battery_warning_last_shown = time.time()

        else:
            self.battery_voltage_stylesheet_change_ready__signal.emit(COLOR_RED)

            if (time.time() - self.critical_battery_warning_last_shown) > CRITICAL_BATTERY_DIALOG_TIMEOUT:
                self.show_battery_critical__signal.emit()
                self.critical_battery_warning_last_shown = time.time()

        self.battery_voltage_update_ready__signal.emit("Battery Voltage\n" + str(voltage) + " V")

    def __co2_callback(self, data):
        if data.data != 9999:
            self.co2_levels_update_ready__signal.emit("CO2 Levels\n%d ppm" % data.data)
        else:
            self.co2_levels_update_ready__signal.emit("CO2 Levels\n--- ppm")

    def __display_time(self):
        time = QtCore.QTime.currentTime()
        temp = time.toString('hh:mm')
        self.clock.display(temp)

    def run(self):
        while self.run_thread_flag:
            # self.update_requester.publish(Empty())
            self.__display_time()
            self.msleep(1000)

    def connect_signals_and_slots(self):
        self.jetson_cpu_update_ready__signal.connect(self.cpu.setText)
        self.jetson_cpu_stylesheet_change_ready__signal.connect(self.cpu.setStyleSheet)
        self.jetson_ram_update_ready__signal.connect(self.ram.setText)
        self.jetson_ram_stylesheet_change_ready__signal.connect(self.ram.setStyleSheet)
        self.jetson_emmc_update_ready__signal.connect(self.emmc.setText)
        self.jetson_emmc_stylesheet_change_ready__signal.connect(self.emmc.setStyleSheet)
        self.jetson_gpu_temp_update_ready__signal.connect(self.gpu_temp.setText)
        self.jetson_gpu_temp_stylesheet_change_ready__signal.connect(self.gpu_temp.setStyleSheet)
        self.camera_zed_stylesheet_change_ready__signal.connect(self.zed.setStyleSheet)
        self.camera_under_stylesheet_change_ready__signal.connect(self.under_cam.setStyleSheet)
        self.camera_chassis_stylesheet_change_ready__signal.connect(self.chassis_cam.setStyleSheet)
        self.camera_main_stylesheet_change_ready__signal.connect(self.main_cam.setStyleSheet)
        self.gps_stylesheet_change_ready__signal.connect(self.gps_fix_label.setStyleSheet)
        self.frsky_stylesheet_change_ready__signal.connect(self.frsky.setStyleSheet)

        self.gps_fix_update_ready__signal.connect(self.gps_fix_label.setText)
        self.gps_heading_valid_update_ready__signal.connect(self.gps_heading_valid_label.setText)
        self.gps_num_satellites_update_ready__signal.connect(self.gps_num_satellites_label.setText)
        self.gps_accuracy_update_ready__signal.connect(self.gps_accuracy_label.setText)

        self.co2_levels_update_ready__signal.connect(self.co2_levels_label.setText)

        self.battery_voltage_update_ready__signal.connect(self.battery.setText)
        self.battery_voltage_stylesheet_change_ready__signal.connect(self.battery.setStyleSheet)

        self.show_battery_low__signal.connect(self.low_battery_warning_dialog.show)
        self.show_battery_critical__signal.connect(self.critical_battery_warning_dialog.show)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False


if __name__ == '__main__':
    rover_statuses = SensorCore()
    rover_statuses.run()
