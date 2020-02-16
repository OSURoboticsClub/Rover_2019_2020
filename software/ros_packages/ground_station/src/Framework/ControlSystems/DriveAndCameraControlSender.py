#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging
from inputs import devices, GamePad
from time import time

import rospy
from rover_control.msg import DriveCommandMessage, TowerPanTiltControlMessage

#####################################
# Global Variables
#####################################
#GAME_CONTROLLER_NAME = "Microsoft X-Box One S pad"  <-- This was the actual xbox controller that Dylan had to buy at CIRC 2018
GAME_CONTROLLER_NAME = "PowerA Xbox One wired controller"
DEFAULT_DRIVE_COMMAND_TOPIC = "/rover_control/command_control/ground_station_drive"
DEFAULT_TOWER_PAN_TILT_COMMAND_TOPIC = "/rover_control/tower/pan_tilt/control"
DEFAULT_CHASSIS_PAN_TILT_COMMAND_TOPIC = "/rover_control/chassis/pan_tilt/control"

DRIVE_COMMAND_HERTZ = 20

STICK_DEADBAND = 3500

STICK_MAX = 32768.0
STICK_OFFSET = 0

THROTTLE_MIN = 0.05

PAUSE_STATE_CHANGE_TIME = 0.5

CAMERA_CHANGE_TIME = 0.2
GUI_ELEMENT_CHANGE_TIME = 0.2
CAMERA_TOGGLE_CHANGE_TIME = 0.35

TOWER_PAN_TILT_X_AXIS_SCALAR = 2
TOWER_PAN_TILT_Y_AXIS_SCALAR = 15

CHASSIS_PAN_TILT_X_AXIS_SCALAR = 15
CHASSIS_PAN_TILT_Y_AXIS_SCALAR = 15

# ui controller
screenSelector = rospy.get_param("one_screen")
if screenSelector == True:
    left = "onescreen"
    right = "onescreen"
else:
    left = "left_screen"
    right = "right_screen"

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
        self.controller_acquired = False

        # ########## Class Variables ##########
        self.gamepad = None  # type: GamePad

        self.controller_states = {
            "left_x_axis": 0,
            "left_y_axis": 0,
            "right_x_axis": 0,
            "right_y_axis": 0,

            "left_trigger": 0,
            "right_trigger": 0,

            "left_stick": 0,
            "right_right": 0,

            "left_bumper": 0,
            "right_bumper": 0,

            "d_pad_x": 0,
            "d_pad_y": 0,

            "back": 0,
            "start": 0,

            "a": 0,
            "x": 0,
            "y": 0,
            "b": 0,
        }

        self.raw_mapping_to_class_mapping = {
            "ABS_X": "left_x_axis",
            "ABS_Y": "left_y_axis",
            "ABS_RX": "right_x_axis",
            "ABS_RY": "right_y_axis",

            "ABS_Z": "left_trigger",
            "ABS_RZ": "right_trigger",

            "BTN_THUMBL": "left_stick",
            "BTN_THUMBR": "right_right",

            "BTN_TL": "left_bumper",
            "BTN_TR": "right_bumper",

            "ABS_HAT0X": "d_pad_x",
            "ABS_HAT0Y": "d_pad_y",

            "BTN_SELECT": "back",
            "BTN_START": "start",

            "BTN_SOUTH": "a",
            "BTN_NORTH": "x",
            "BTN_WEST": "y",
            "BTN_EAST": "b",
        }

        self.ready = False

        self.start()

    def run(self):

        while self.run_thread_flag:
            if self.setup_controller_flag:
                self.controller_acquired = self.__setup_controller()
                self.setup_controller_flag = False
            if self.data_acquisition_and_broadcast_flag:
                self.__get_controller_data()

    def __setup_controller(self):
        for device in devices.gamepads:
            # print device
            if device.name == GAME_CONTROLLER_NAME:
                self.gamepad = device

                return True
        return False

    def __get_controller_data(self):
        if self.controller_acquired:
            events = self.gamepad.read()

            for event in events:
                # print event.code, event.state
                if event.code in self.raw_mapping_to_class_mapping:
                    self.controller_states[self.raw_mapping_to_class_mapping[event.code]] = event.state

            self.ready = True


#####################################
# Controller Class Definition
#####################################
class DriveAndCameraControlSender(QtCore.QThread):
    set_speed_limit__signal = QtCore.pyqtSignal(int)
    set_left_drive_output__signal = QtCore.pyqtSignal(int)
    set_right_drive_output__signal = QtCore.pyqtSignal(int)

    change_gui_element_selection__signal = QtCore.pyqtSignal(int)
    change_camera_selection__signal = QtCore.pyqtSignal(int)
    toggle_selected_gui_camera__signal = QtCore.pyqtSignal()

    def __init__(self, shared_objects):
        super(DriveAndCameraControlSender, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.video_coordinator = self.shared_objects["threaded_classes"]["Video Coordinator"]
        self.right_screen = self.shared_objects["screens"][right]
        self.rover_speed_limit_slider = self.right_screen.rover_speed_limit_slider  # type: QtWidgets.QSlider
        self.left_drive_progress_bar = self.right_screen.left_drive_progress_bar  # type: QtWidgets.QProgressBar
        self.right_drive_progress_bar = self.right_screen.right_drive_progress_bar  # type: QtWidgets.QProgressBar

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        self.joystick = LogitechJoystick()

        # ########## Class Variables ##########
        # Publishers
        self.drive_command_publisher = rospy.Publisher(DEFAULT_DRIVE_COMMAND_TOPIC, DriveCommandMessage, queue_size=1)
        self.tower_pan_tilt_command_publisher = rospy.Publisher(DEFAULT_TOWER_PAN_TILT_COMMAND_TOPIC,TowerPanTiltControlMessage, queue_size=1)
        self.chassis_pan_tilt_command_publisher = rospy.Publisher(DEFAULT_CHASSIS_PAN_TILT_COMMAND_TOPIC, TowerPanTiltControlMessage, queue_size=1)

        self.current_pan_tilt_selection = "no_pan_tilt"

        self.last_hat_x_was_movement = False
        self.last_hat_y_was_movement = False

        self.wait_time = 1.0 / DRIVE_COMMAND_HERTZ

        self.drive_paused = True

        self.last_pause_state_time = time()
        self.last_gui_element_change_time = time()
        self.last_camera_change_time = time()
        self.last_camera_toggle_time = time()

        self.speed_limit = 0.5

    def run(self):
        self.logger.debug("Starting Joystick Thread")

        while self.run_thread_flag:
            start_time = time()

            self.check_and_set_pause_state()
            self.__update_and_publish()

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

        self.logger.debug("Stopping Joystick Thread")

    def connect_signals_and_slots(self):
        self.set_left_drive_output__signal.connect(self.left_drive_progress_bar.setValue)
        self.set_right_drive_output__signal.connect(self.right_drive_progress_bar.setValue)

        self.video_coordinator.pan_tilt_selection_changed__signal.connect(self.on_pan_tilt_selection_changed__slot)
        self.rover_speed_limit_slider.valueChanged.connect(self.on_speed_limit_slider_value_changed__slot)

    def check_and_set_pause_state(self):
        thumb_pressed = self.joystick.controller_states["start"]
        if thumb_pressed and (time() - self.last_pause_state_time) > PAUSE_STATE_CHANGE_TIME:
            self.drive_paused = not self.drive_paused
            self.show_changed_pause_state()
            self.last_pause_state_time = time()

    def __update_and_publish(self):
        self.publish_drive_command()
        self.publish_camera_control_commands()
        self.publish_pan_tilt_control_commands()

    def publish_drive_command(self):
        if self.drive_paused:
            drive_message = DriveCommandMessage()
        else:
            drive_message = self.get_drive_message(self.speed_limit)

        left_output = abs(drive_message.drive_twist.linear.x - drive_message.drive_twist.angular.z)
        right_output = abs(drive_message.drive_twist.linear.x + drive_message.drive_twist.angular.z)

        self.set_speed_limit__signal.emit(self.speed_limit * 100)
        self.set_left_drive_output__signal.emit(left_output * 100)
        self.set_right_drive_output__signal.emit(right_output * 100)

        self.drive_command_publisher.publish(drive_message)

    def publish_camera_control_commands(self):
        trigger_pressed = self.joystick.controller_states["y"]
        three_pressed = self.joystick.controller_states["left_bumper"]
        four_pressed = self.joystick.controller_states["right_bumper"]
        five_pressed = self.joystick.controller_states["left_trigger"]
        six_pressed = self.joystick.controller_states["right_trigger"]

        if (five_pressed or six_pressed) and (time() - self.last_camera_change_time) > CAMERA_CHANGE_TIME:
            change = -1 if five_pressed else 1
            self.change_camera_selection__signal.emit(change)
            self.last_camera_change_time = time()

        if (three_pressed or four_pressed) and (time() - self.last_gui_element_change_time) > GUI_ELEMENT_CHANGE_TIME:
            change = -1 if three_pressed else 1
            self.change_gui_element_selection__signal.emit(change)
            self.last_gui_element_change_time = time()

        if trigger_pressed and (time() - self.last_camera_toggle_time) > CAMERA_TOGGLE_CHANGE_TIME:
            self.toggle_selected_gui_camera__signal.emit()
            self.last_camera_toggle_time = time()

    def publish_pan_tilt_control_commands(self):
        button_eight = self.joystick.controller_states["a"]
        hat_x = self.joystick.controller_states["d_pad_x"]
        hat_y = self.joystick.controller_states["d_pad_y"]

        if (hat_x == 0 and not self.last_hat_x_was_movement) and (
                hat_y == 0 and not self.last_hat_y_was_movement) and not button_eight:
            return

        self.last_hat_x_was_movement = True if hat_x != 0 else False
        self.last_hat_y_was_movement = True if hat_y != 0 else False

        pan_tilt_message = TowerPanTiltControlMessage()

        if button_eight:
            pan_tilt_message.should_center = 1

        if self.current_pan_tilt_selection == "tower_pan_tilt":
            pan_tilt_message.relative_pan_adjustment = hat_x * TOWER_PAN_TILT_X_AXIS_SCALAR
            pan_tilt_message.relative_tilt_adjustment = -(hat_y * TOWER_PAN_TILT_Y_AXIS_SCALAR)
            self.tower_pan_tilt_command_publisher.publish(pan_tilt_message)

        elif self.current_pan_tilt_selection == "chassis_pan_tilt":
            pan_tilt_message.relative_pan_adjustment = hat_x * CHASSIS_PAN_TILT_X_AXIS_SCALAR
            pan_tilt_message.relative_tilt_adjustment = -(hat_y * CHASSIS_PAN_TILT_Y_AXIS_SCALAR)
            self.chassis_pan_tilt_command_publisher.publish(pan_tilt_message)

    def get_drive_message(self, speed_limit):
        drive_message = DriveCommandMessage()

        left_y_axis = self.joystick.controller_states["left_y_axis"] if abs(self.joystick.controller_states["left_y_axis"]) > STICK_DEADBAND else 0
        right_y_axis = self.joystick.controller_states["right_y_axis"] if abs(self.joystick.controller_states["right_y_axis"]) > STICK_DEADBAND else 0

        left_y_axis = speed_limit * (-(left_y_axis - STICK_OFFSET) / STICK_MAX)
        right_y_axis = speed_limit * (-(right_y_axis - STICK_OFFSET) / STICK_MAX)

        drive_message.drive_twist.linear.x = (left_y_axis + right_y_axis) / 2.0
        drive_message.drive_twist.angular.z = (right_y_axis - left_y_axis) / 2.0

        return drive_message

    def on_pan_tilt_selection_changed__slot(self, selection):
        self.current_pan_tilt_selection = selection

    def on_speed_limit_slider_value_changed__slot(self, value):
        self.speed_limit = value / 100.0

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def show_changed_pause_state(self):
        if self.drive_paused:
            self.left_drive_progress_bar.setStyleSheet("background-color:darkred;")
            self.right_drive_progress_bar.setStyleSheet("background-color:darkred;")
        else:
            self.left_drive_progress_bar.setStyleSheet("background-color: transparent;")
            self.right_drive_progress_bar.setStyleSheet("background-color: transparent;")

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
