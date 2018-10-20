#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging
from inputs import devices, GamePad
from time import time

import rospy
from rover_arm.msg import ArmControlMessage
from rover_control.msg import MiningControlMessage, GripperControlMessage

#####################################
# Global Variables
#####################################
GAME_CONTROLLER_NAME = "Afterglow Gamepad for Xbox 360"

DRIVE_COMMAND_HERTZ = 20

GRIPPER_CONTROL_TOPIC = "/rover_control/gripper/control"
RELATIVE_ARM_CONTROL_TOPIC = "/rover_arm/control/relative"
MINING_CONTROL_TOPIC = "/rover_control/mining/control"

BASE_SCALAR = 0.003
SHOULDER_SCALAR = 0.002
ELBOW_SCALAR = 0.002
ROLL_SCALAR = 0.003
WRIST_PITCH_SCALAR = 0.003
WRIST_ROLL_SCALAR = 0.006

GRIPPER_MOVEMENT_SCALAR = 1500

LEFT_X_AXIS_DEADZONE = 1500
LEFT_Y_AXIS_DEADZONE = 1500

RIGHT_X_AXIS_DEADZONE = 1500
RIGHT_Y_AXIS_DEADZONE = 1500

THUMB_STICK_MAX = 32768.0

MINING_LIFT_SCALAR = 5
MINING_TILT_SCALAR = 5

COLOR_GREEN = "background-color:darkgreen; border: 1px solid black;"
COLOR_NONE = "border: 1px solid black;"


#####################################
# Controller Class Definition
#####################################
class XBOXController(QtCore.QThread):
    def __init__(self):
        super(XBOXController, self).__init__()

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
            "left_stick_button": 0,

            "right_x_axis": 0,
            "right_y_axis": 0,
            "right_stick_button": 0,

            "left_trigger": 0,
            "left_bumper": 0,

            "right_trigger": 0,
            "right_bumper": 0,

            "hat_x_axis": 0,
            "hat_y_axis": 0,

            "back_button": 0,
            "start_button": 0,
            "xbox_button": 0,

            "x_button": 0,
            "a_button": 0,
            "b_button": 0,
            "y_button": 0
        }

        self.raw_mapping_to_class_mapping = {
            "ABS_X": "left_x_axis",
            "ABS_Y": "left_y_axis",
            "BTN_THUMBL": "left_stick_button",

            "ABS_RX": "right_x_axis",
            "ABS_RY": "right_y_axis",
            "BTN_THUMBR": "right_stick_button",

            "ABS_Z": "left_trigger",
            "BTN_TL": "left_bumper",

            "ABS_RZ": "right_trigger",
            "BTN_TR": "right_bumper",

            "ABS_HAT0X": "hat_x_axis",
            "ABS_HAT0Y": "hat_y_axis",

            "BTN_SELECT": "back_button",
            "BTN_START": "start_button",
            "BTN_MODE": "xbox_button",

            "BTN_NORTH": "x_button",
            "BTN_SOUTH": "a_button",
            "BTN_EAST": "b_button",
            "BTN_WEST": "y_button"
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
                # For seeing codes you haven't added yet...
                # if event.code not in self.raw_mapping_to_class_mapping and event.code != "SYN_REPORT":
                #     print event.code, ":", event.state

                if event.code in self.raw_mapping_to_class_mapping:
                    # print event.code, ":", event.state
                    self.controller_states[self.raw_mapping_to_class_mapping[event.code]] = event.state

            self.ready = True


#####################################
# Controller Class Definition
#####################################
class EffectorsAndArmControlSender(QtCore.QThread):
    xbox_control_arm_stylesheet_update_ready__signal = QtCore.pyqtSignal(str)
    xbox_control_mining_stylesheet_update_ready__signal = QtCore.pyqtSignal(str)

    gripper_mode_normal_stylesheet_update_ready__signal = QtCore.pyqtSignal(str)
    gripper_mode_pinch_stylesheet_update_ready__signal = QtCore.pyqtSignal(str)
    gripper_mode_wide_stylesheet_update_ready__signal = QtCore.pyqtSignal(str)
    gripper_mode_scissor_stylesheet_update_ready__signal = QtCore.pyqtSignal(str)

    XBOX_CONTROL_STATES = [
        "ARM",
        "MINING"
    ]

    GRIPPER_CONTROL_MODES = {
        "NORMAL": 1,
        "TWO_FINGER_PINCH": 2,
        "WIDE": 3,
        "SCISSOR": 4
    }

    PINCH_MODE_ABSOLUTE_SET_POSITION = 57740

    def __init__(self, shared_objects):
        super(EffectorsAndArmControlSender, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]
        self.right_screen = self.shared_objects["screens"]["right_screen"]
        self.xbox_mode_arm_label = self.right_screen.xbox_mode_arm_label  # type: QtWidgets.QLabel
        self.xbox_mode_mining_label = self.right_screen.xbox_mode_mining_label  # type: QtWidgets.QLabel

        self.gripper_mode_normal_label = self.right_screen.gripper_mode_normal_label  # type: QtWidgets.QLabel
        self.gripper_mode_pinch_label = self.right_screen.gripper_mode_pinch_label  # type: QtWidgets.QLabel
        self.gripper_mode_wide_label = self.right_screen.gripper_mode_wide_label  # type: QtWidgets.QLabel
        self.gripper_mode_scissor_label = self.right_screen.gripper_mode_scissor_label  # type: QtWidgets.QLabel

        self.arm_speed_limit_slider = self.right_screen.arm_speed_limit_slider  # type: QtWidgets.QSlider

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        self.controller = XBOXController()

        # ########## Class Variables ##########

        self.wait_time = 1.0 / DRIVE_COMMAND_HERTZ

        self.gripper_control_publisher = rospy.Publisher(GRIPPER_CONTROL_TOPIC, GripperControlMessage, queue_size=1)

        self.relative_arm_control_publisher = rospy.Publisher(RELATIVE_ARM_CONTROL_TOPIC, ArmControlMessage,
                                                              queue_size=1)
        self.mining_control_publisher = rospy.Publisher(MINING_CONTROL_TOPIC, MiningControlMessage, queue_size=1)

        self.xbox_current_control_state = self.XBOX_CONTROL_STATES.index("ARM")
        self.xbox_control_state_just_changed = False

        self.last_xbox_button_state = 0
        self.last_left_bumper_state = 0
        self.last_right_bumper_state = 0
        self.last_back_button_state = 0

        self.gripper_control_mode = 1
        self.gripper_control_mode_just_changed = False
        self.send_new_gripper_mode = False

        self.GRIPPER_DISPLAY_SIGNAL_MAPPING = {
            self.GRIPPER_CONTROL_MODES["NORMAL"]: {
                "SET": [self.gripper_mode_normal_stylesheet_update_ready__signal],
                "UNSET": [self.gripper_mode_pinch_stylesheet_update_ready__signal,
                          self.gripper_mode_wide_stylesheet_update_ready__signal,
                          self.gripper_mode_scissor_stylesheet_update_ready__signal]
            },

            self.GRIPPER_CONTROL_MODES["TWO_FINGER_PINCH"]: {
                "SET": [self.gripper_mode_pinch_stylesheet_update_ready__signal],
                "UNSET": [self.gripper_mode_normal_stylesheet_update_ready__signal,
                          self.gripper_mode_wide_stylesheet_update_ready__signal,
                          self.gripper_mode_scissor_stylesheet_update_ready__signal]
            },

            self.GRIPPER_CONTROL_MODES["WIDE"]: {
                "SET": [self.gripper_mode_wide_stylesheet_update_ready__signal],
                "UNSET": [self.gripper_mode_pinch_stylesheet_update_ready__signal,
                          self.gripper_mode_normal_stylesheet_update_ready__signal,
                          self.gripper_mode_scissor_stylesheet_update_ready__signal]
            },

            self.GRIPPER_CONTROL_MODES["SCISSOR"]: {
                "SET": [self.gripper_mode_scissor_stylesheet_update_ready__signal],
                "UNSET": [self.gripper_mode_pinch_stylesheet_update_ready__signal,
                          self.gripper_mode_wide_stylesheet_update_ready__signal,
                          self.gripper_mode_normal_stylesheet_update_ready__signal],
                # "ABS_MOVE": self.PINCH_MODE_ABSOLUTE_SET_POSITION
            }
        }

    def run(self):
        self.logger.debug("Starting Joystick Thread")

        while self.run_thread_flag:
            start_time = time()

            self.change_control_state_if_needed()

            if self.xbox_current_control_state == self.XBOX_CONTROL_STATES.index("ARM"):
                self.send_gripper_home_on_back_press()
                self.process_and_send_arm_control()
            elif self.xbox_current_control_state == self.XBOX_CONTROL_STATES.index("MINING"):
                self.send_mining_commands()

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

        self.logger.debug("Stopping Joystick Thread")

    def connect_signals_and_slots(self):
        self.xbox_control_arm_stylesheet_update_ready__signal.connect(self.xbox_mode_arm_label.setStyleSheet)
        self.xbox_control_mining_stylesheet_update_ready__signal.connect(self.xbox_mode_mining_label.setStyleSheet)

        self.gripper_mode_normal_stylesheet_update_ready__signal.connect(self.gripper_mode_normal_label.setStyleSheet)
        self.gripper_mode_pinch_stylesheet_update_ready__signal.connect(self.gripper_mode_pinch_label.setStyleSheet)
        self.gripper_mode_wide_stylesheet_update_ready__signal.connect(self.gripper_mode_wide_label.setStyleSheet)
        self.gripper_mode_scissor_stylesheet_update_ready__signal.connect(self.gripper_mode_scissor_label.setStyleSheet)

    def change_control_state_if_needed(self):
        xbox_state = self.controller.controller_states["xbox_button"]
        left_bumper_state = self.controller.controller_states["left_bumper"]
        right_bumper_state = self.controller.controller_states["right_bumper"]

        if self.last_xbox_button_state == 0 and xbox_state == 1:
            self.xbox_current_control_state += 1
            self.xbox_current_control_state = self.xbox_current_control_state % len(self.XBOX_CONTROL_STATES)
            self.xbox_control_state_just_changed = True
            self.last_xbox_button_state = 1
        elif self.last_xbox_button_state == 1 and xbox_state == 0:
            self.last_xbox_button_state = 0

        if self.xbox_control_state_just_changed:
            if self.xbox_current_control_state == self.XBOX_CONTROL_STATES.index("ARM"):
                self.xbox_control_arm_stylesheet_update_ready__signal.emit(COLOR_GREEN)
                self.xbox_control_mining_stylesheet_update_ready__signal.emit(COLOR_NONE)
            elif self.xbox_current_control_state == self.XBOX_CONTROL_STATES.index("MINING"):
                self.xbox_control_arm_stylesheet_update_ready__signal.emit(COLOR_NONE)
                self.xbox_control_mining_stylesheet_update_ready__signal.emit(COLOR_GREEN)
            self.xbox_control_state_just_changed = False

        if self.xbox_current_control_state == self.XBOX_CONTROL_STATES.index("ARM"):
            if self.last_left_bumper_state == 0 and left_bumper_state == 1:
                # self.gripper_control_mode = ((self.gripper_control_mode - 1) % len(self.GRIPPER_CONTROL_MODES))
                # self.gripper_control_mode_just_changed = True
                self.last_left_bumper_state = 1
            elif self.last_left_bumper_state == 1 and left_bumper_state == 0:
                self.last_left_bumper_state = 0
            if self.last_right_bumper_state == 0 and right_bumper_state == 1:
                # self.gripper_control_mode = ((self.gripper_control_mode + 1) % len(self.GRIPPER_CONTROL_MODES))
                # self.gripper_control_mode_just_changed = True
                self.last_right_bumper_state = 1
            elif self.last_right_bumper_state == 1 and right_bumper_state == 0:
                self.last_right_bumper_state = 0

        if self.gripper_control_mode_just_changed:
            signal_map = self.GRIPPER_DISPLAY_SIGNAL_MAPPING[self.gripper_control_mode + 1]

            for signal in signal_map["SET"]:
                signal.emit(COLOR_GREEN)

            for signal in signal_map["UNSET"]:
                signal.emit(COLOR_NONE)

            if "ABS_MOVE" in signal_map:
                gripper_control_message = GripperControlMessage()
                gripper_control_message.gripper_mode = self.GRIPPER_CONTROL_MODES["SCISSOR"]
                gripper_control_message.gripper_position_absolute = signal_map["ABS_MOVE"]
                self.gripper_control_publisher.publish(gripper_control_message)
            else:
                self.send_new_gripper_mode = True

            self.gripper_control_mode_just_changed = False

    def process_and_send_arm_control(self):

        arm_control_message = ArmControlMessage()

        gripper_control_message = GripperControlMessage()

        should_publish_arm = False
        should_publish_gripper = True if self.send_new_gripper_mode else False

        if self.send_new_gripper_mode:
            gripper_control_message.gripper_position_absolute = 0
        else:
            gripper_control_message.gripper_position_absolute = -1

        gripper_control_message.gripper_mode = self.gripper_control_mode + 1

        left_trigger = self.controller.controller_states["left_trigger"]
        right_trigger = self.controller.controller_states["right_trigger"]

        left_x_axis = self.controller.controller_states["left_x_axis"] if abs(self.controller.controller_states[
                                                                                  "left_x_axis"]) > LEFT_X_AXIS_DEADZONE else 0
        left_y_axis = self.controller.controller_states["left_y_axis"] if abs(self.controller.controller_states[
                                                                                  "left_y_axis"]) > LEFT_Y_AXIS_DEADZONE else 0
        right_y_axis = self.controller.controller_states["right_y_axis"] if abs(self.controller.controller_states[
                                                                                    "right_y_axis"]) > RIGHT_Y_AXIS_DEADZONE else 0
        right_x_axis = self.controller.controller_states["right_x_axis"] if abs(self.controller.controller_states[
                                                                                    "right_x_axis"]) > RIGHT_X_AXIS_DEADZONE else 0

        speed_limit = self.arm_speed_limit_slider.value() / 100.0

        if left_trigger > 0:
            should_publish_arm = True
            arm_control_message.base = ((left_x_axis / THUMB_STICK_MAX) * BASE_SCALAR) * speed_limit
            arm_control_message.shoulder = ((left_y_axis / THUMB_STICK_MAX) * SHOULDER_SCALAR) * speed_limit
            arm_control_message.elbow = (-(right_y_axis / THUMB_STICK_MAX) * ELBOW_SCALAR) * speed_limit
            arm_control_message.roll = (-(right_x_axis / THUMB_STICK_MAX) * ROLL_SCALAR) * speed_limit

        elif right_trigger > 0:
            should_publish_arm = True
            should_publish_gripper = True

            arm_control_message.wrist_roll = ((left_x_axis / THUMB_STICK_MAX) * BASE_SCALAR) * speed_limit

            # ##### FIXME #####
            # Remove this once the arm is fixed
            # arm_control_message.wrist_pitch = (-(left_y_axis / THUMB_STICK_MAX) * WRIST_PITCH_SCALAR) * speed_limit
            # #################

            gripper_control_message.gripper_position_relative = (-(right_y_axis / THUMB_STICK_MAX) * GRIPPER_MOVEMENT_SCALAR)

        if should_publish_arm:
            self.relative_arm_control_publisher.publish(arm_control_message)

        if should_publish_gripper:
            self.gripper_control_publisher.publish(gripper_control_message)
            self.send_new_gripper_mode = False

    def send_gripper_home_on_back_press(self):
        gripper_control_message = GripperControlMessage()
        back_state = self.controller.controller_states["back_button"]

        if self.last_back_button_state == 0 and back_state == 1:
            gripper_control_message.should_home = True
            gripper_control_message.gripper_mode = self.gripper_control_mode + 1
            self.gripper_control_publisher.publish(gripper_control_message)
            self.last_back_button_state = 1
        elif self.last_back_button_state == 1 and back_state == 0:
            self.last_back_button_state = 0

    def send_mining_commands(self):

        left_y_axis = self.controller.controller_states["left_y_axis"] if abs(
            self.controller.controller_states["left_y_axis"]) > LEFT_Y_AXIS_DEADZONE else 0
        right_y_axis = self.controller.controller_states["right_y_axis"] if abs(self.controller.controller_states[
                                                                                    "right_y_axis"]) > RIGHT_Y_AXIS_DEADZONE else 0

        if left_y_axis or right_y_axis:
            message = MiningControlMessage()

            message.lift_set_absolute = 1024
            message.tilt_set_absolute = 1024

            message.lift_set_relative = (-(left_y_axis / THUMB_STICK_MAX) * MINING_LIFT_SCALAR)
            message.tilt_set_relative = ((right_y_axis / THUMB_STICK_MAX) * MINING_TILT_SCALAR)
            message.cal_factor = -1

            self.mining_control_publisher.publish(message)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
