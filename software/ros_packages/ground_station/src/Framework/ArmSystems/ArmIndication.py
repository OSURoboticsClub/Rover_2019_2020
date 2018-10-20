# coding=utf-8
#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets, QtGui
import logging
import rospy

from rover_arm.msg import ArmStatusMessage
from rover_control.msg import GripperStatusMessage

#####################################
# Global Variables
#####################################
ARM_STATUS_TOPIC = "/rover_arm/status"
GRIPPER_STATUS_TOPIC = "/rover_control/gripper/status"


COMMS_TO_STRING = {
    0: "NO STATUS",
    1: "COMMS OK",
    2: "NO DEVICE",
    4: "BUS ERROR",
    8: "GEN COMM ERROR",
    16: "PARAMETER ERROR",
    32: "LENGTH ERROR"
}

TARGET_REACHED_BIT_POSITION = 1

STATUS_TO_STRING = {
    1: "TARGET REACHED",
    2: "ERROR RECOVERY",
    3: "RUN",
    4: "ENABLED",
    5: "FAULT STOP",
    6: "WARNING",
    7: "STO ACTIVE",
    8: "SERVO READY",
    10: "BRAKING",
    11: "HOMING",
    12: "INITIALIZED",
    13: "VOLT OK",
    15: "PERMANENT STOP"
}

FAULT_TO_STRING = {
    1: "TRACKING ERROR",
    2: "OVER CURRENT",
    # 3: "COMMUNICATION ERROR",  # Was showing even though things were working???
    4: "ENCODER FAILURE",
    5: "OVER TEMP",
    6: "UNDER VOLTAGE",
    7: "OVER VOLTAGE",
    8: "PROG OR MEM ERROR",
    9: "HARDWARE ERROR",
    10: "OVER VELOCITY",
    11: "INIT ERROR",
    12: "MOTION ERROR",
    13: "RANGE ERROR",
    14: "POWER STAGE FORCED OFF",
    15: "HOST COMM ERROR"
}

GRIPPER_MODES = {
    0: "No Change",
    1: "Normal",
    2: "Pinch",
    3: "Wide ",
    4: "Scissor"
}


#####################################
# Controller Class Definition
#####################################
class ArmIndication(QtCore.QObject):
    base_position_updated__signal = QtCore.pyqtSignal(float)
    shoulder_position_updated__signal = QtCore.pyqtSignal(float)
    elbow_position_updated__signal = QtCore.pyqtSignal(float)
    roll_position_updated__signal = QtCore.pyqtSignal(float)
    wrist_pitch_position_updated__signal = QtCore.pyqtSignal(float)
    wrist_roll_position_updated__signal = QtCore.pyqtSignal(float)

    base_comms_state_update_ready__signal = QtCore.pyqtSignal(str)
    shoulder_comms_state_update_ready__signal = QtCore.pyqtSignal(str)
    elbow_comms_state_update_ready__signal = QtCore.pyqtSignal(str)
    roll_comms_state_update_ready__signal = QtCore.pyqtSignal(str)
    wrist_pitch_comms_state_update_ready__signal = QtCore.pyqtSignal(str)
    wrist_roll_comms_state_update_ready__signal = QtCore.pyqtSignal(str)

    base_status_update_ready__signal = QtCore.pyqtSignal(str)
    shoulder_status_update_ready__signal = QtCore.pyqtSignal(str)
    elbow_status_update_ready__signal = QtCore.pyqtSignal(str)
    roll_status_update_ready__signal = QtCore.pyqtSignal(str)
    wrist_pitch_status_update_ready__signal = QtCore.pyqtSignal(str)
    wrist_roll_status_update_ready__signal = QtCore.pyqtSignal(str)

    base_faults_update_ready__signal = QtCore.pyqtSignal(str)
    shoulder_faults_update_ready__signal = QtCore.pyqtSignal(str)
    elbow_faults_update_ready__signal = QtCore.pyqtSignal(str)
    roll_faults_update_ready__signal = QtCore.pyqtSignal(str)
    wrist_pitch_faults_update_ready__signal = QtCore.pyqtSignal(str)
    wrist_roll_faults_update_ready__signal = QtCore.pyqtSignal(str)

    pinch_position_updated__signal = QtCore.pyqtSignal(int)
    forefinger_position_updated__signal = QtCore.pyqtSignal(int)
    thumb_position_updated__signal = QtCore.pyqtSignal(int)
    middlefinger_position_updated__signal = QtCore.pyqtSignal(int)

    pinch_current_updated__signal = QtCore.pyqtSignal(int)
    forefinger_current_updated__signal = QtCore.pyqtSignal(int)
    thumb_current_updated__signal = QtCore.pyqtSignal(int)
    middlefinger_current_updated__signal = QtCore.pyqtSignal(int)

    gripper_reported_mode_updated__signal = QtCore.pyqtSignal(str)
    gripper_reported_setpoint_updated__signal = QtCore.pyqtSignal(int)

    def __init__(self, shared_objects):
        super(ArmIndication, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.right_screen = self.shared_objects["screens"]["right_screen"]

        self.base_position_lcd_number = self.right_screen.base_position_lcd_number  # type: QtWidgets.QLCDNumber
        self.shoulder_position_lcd_number = self.right_screen.shoulder_position_lcd_number  # type: QtWidgets.QLCDNumber
        self.elbow_position_lcd_number = self.right_screen.elbow_position_lcd_number  # type: QtWidgets.QLCDNumber
        self.roll_position_lcd_number = self.right_screen.roll_position_lcd_number  # type: QtWidgets.QLCDNumber
        self.wrist_pitch_position_lcd_number = self.right_screen.wrist_pitch_position_lcd_number  # type: QtWidgets.QLCDNumber
        self.wrist_roll_position_lcd_number = self.right_screen.wrist_roll_position_lcd_number  # type: QtWidgets.QLCDNumber

        self.pinch_position_lcd_number = self.right_screen.pinch_position_lcd_number  # type: QtWidgets.QLCDNumber
        self.forefinger_position_lcd_number = self.right_screen.forefinger_position_lcd_number  # type: QtWidgets.QLCDNumber
        self.thumb_position_lcd_number = self.right_screen.thumb_position_lcd_number  # type: QtWidgets.QLCDNumber
        self.middlefinger_position_lcd_number = self.right_screen.middlefinger_position_lcd_number  # type: QtWidgets.QLCDNumber

        self.pinch_current_lcd_number = self.right_screen.pinch_current_lcd_number  # type: QtWidgets.QLCDNumber
        self.forefinger_current_lcd_number = self.right_screen.forefinger_current_lcd_number  # type: QtWidgets.QLCDNumber
        self.thumb_current_lcd_number = self.right_screen.thumb_current_lcd_number  # type: QtWidgets.QLCDNumber
        self.middlefinger_current_lcd_number = self.right_screen.middlefinger_current_lcd_number  # type: QtWidgets.QLCDNumber

        self.arm_controls_base_comms_label = self.right_screen.arm_controls_base_comms_label  # type:QtWidgets.QLabel
        self.arm_controls_base_status_label = self.right_screen.arm_controls_base_status_label  # type:QtWidgets.QLabel
        self.arm_controls_base_faults_label = self.right_screen.arm_controls_base_faults_label  # type:QtWidgets.QLabel

        self.arm_controls_shoulder_comms_label = self.right_screen.arm_controls_shoulder_comms_label  # type:QtWidgets.QLabel
        self.arm_controls_shoulder_status_label = self.right_screen.arm_controls_shoulder_status_label  # type:QtWidgets.QLabel
        self.arm_controls_shoulder_faults_label = self.right_screen.arm_controls_shoulder_faults_label  # type:QtWidgets.QLabel
        self.arm_controls_elbow_comms_label = self.right_screen.arm_controls_elbow_comms_label  # type:QtWidgets.QLabel
        self.arm_controls_elbow_status_label = self.right_screen.arm_controls_elbow_status_label  # type:QtWidgets.QLabel
        self.arm_controls_elbow_faults_label = self.right_screen.arm_controls_elbow_faults_label  # type:QtWidgets.QLabel

        self.arm_controls_roll_comms_label = self.right_screen.arm_controls_roll_comms_label  # type:QtWidgets.QLabel
        self.arm_controls_roll_status_label = self.right_screen.arm_controls_roll_status_label  # type:QtWidgets.QLabel
        self.arm_controls_roll_faults_label = self.right_screen.arm_controls_roll_faults_label  # type:QtWidgets.QLabel

        self.arm_controls_wrist_pitch_comms_label = self.right_screen.arm_controls_wrist_pitch_comms_label  # type:QtWidgets.QLabel
        self.arm_controls_wrist_pitch_status_label = self.right_screen.arm_controls_wrist_pitch_status_label  # type:QtWidgets.QLabel
        self.arm_controls_wrist_pitch_faults_label = self.right_screen.arm_controls_wrist_pitch_faults_label  # type:QtWidgets.QLabel

        self.arm_controls_wrist_roll_comms_label = self.right_screen.arm_controls_wrist_roll_comms_label  # type:QtWidgets.QLabel
        self.arm_controls_wrist_roll_status_label = self.right_screen.arm_controls_wrist_roll_status_label  # type:QtWidgets.QLabel
        self.arm_controls_wrist_roll_faults_label = self.right_screen.arm_controls_wrist_roll_faults_label  # type:QtWidgets.QLabel

        self.gripper_reported_mode_label = self.right_screen.gripper_reported_mode_label  # type:QtWidgets.QLabel
        self.gripper_reported_setpoint_lcd_number = self.right_screen.gripper_reported_setpoint_lcd_number  # type: QtWidgets.QLCDNumber

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Class Variables ##########
        self.arm_status_subscriber = rospy.Subscriber(ARM_STATUS_TOPIC, ArmStatusMessage, self.on_arm_status_update_received__callback)
        self.gripper_status_subscriver = rospy.Subscriber(GRIPPER_STATUS_TOPIC, GripperStatusMessage, self.on_gripper_status_update_received__callback)

        # ########## Connect Signals and Slots ##########
        self.connect_signals_and_slots()

    def connect_signals_and_slots(self):
        self.base_position_updated__signal.connect(self.base_position_lcd_number.display)
        self.shoulder_position_updated__signal.connect(self.shoulder_position_lcd_number.display)
        self.elbow_position_updated__signal.connect(self.elbow_position_lcd_number.display)
        self.roll_position_updated__signal.connect(self.roll_position_lcd_number.display)
        self.wrist_pitch_position_updated__signal.connect(self.wrist_pitch_position_lcd_number.display)
        self.wrist_roll_position_updated__signal.connect(self.wrist_roll_position_lcd_number.display)

        self.base_comms_state_update_ready__signal.connect(self.arm_controls_base_comms_label.setText)
        self.shoulder_comms_state_update_ready__signal.connect(self.arm_controls_shoulder_comms_label.setText)
        self.elbow_comms_state_update_ready__signal.connect(self.arm_controls_elbow_comms_label.setText)
        self.roll_comms_state_update_ready__signal.connect(self.arm_controls_roll_comms_label.setText)
        self.wrist_pitch_comms_state_update_ready__signal.connect(self.arm_controls_wrist_pitch_comms_label.setText)
        self.wrist_roll_comms_state_update_ready__signal.connect(self.arm_controls_wrist_roll_comms_label.setText)

        self.base_status_update_ready__signal.connect(self.arm_controls_base_status_label.setText)
        self.shoulder_status_update_ready__signal.connect(self.arm_controls_shoulder_status_label.setText)
        self.elbow_status_update_ready__signal.connect(self.arm_controls_elbow_status_label.setText)
        self.roll_status_update_ready__signal.connect(self.arm_controls_roll_status_label.setText)
        self.wrist_pitch_status_update_ready__signal.connect(self.arm_controls_wrist_pitch_status_label.setText)
        self.wrist_roll_status_update_ready__signal.connect(self.arm_controls_wrist_roll_status_label.setText)

        self.base_faults_update_ready__signal.connect(self.arm_controls_base_faults_label.setText)
        self.shoulder_faults_update_ready__signal.connect(self.arm_controls_shoulder_faults_label.setText)
        self.elbow_faults_update_ready__signal.connect(self.arm_controls_elbow_faults_label.setText)
        self.roll_faults_update_ready__signal.connect(self.arm_controls_roll_faults_label.setText)
        self.wrist_pitch_faults_update_ready__signal.connect(self.arm_controls_wrist_pitch_faults_label.setText)
        self.wrist_roll_faults_update_ready__signal.connect(self.arm_controls_wrist_roll_faults_label.setText)

        self.pinch_position_updated__signal.connect(self.pinch_position_lcd_number.display)
        self.forefinger_position_updated__signal.connect(self.forefinger_position_lcd_number.display)
        self.thumb_position_updated__signal.connect(self.thumb_position_lcd_number.display)
        self.middlefinger_position_updated__signal.connect(self.middlefinger_position_lcd_number.display)

        self.pinch_current_updated__signal.connect(self.pinch_current_lcd_number.display)
        self.forefinger_current_updated__signal.connect(self.forefinger_current_lcd_number.display)
        self.thumb_current_updated__signal.connect(self.thumb_current_lcd_number.display)
        self.middlefinger_current_updated__signal.connect(self.middlefinger_current_lcd_number.display)

        self.gripper_reported_mode_updated__signal.connect(self.gripper_reported_mode_label.setText)
        self.gripper_reported_setpoint_updated__signal.connect(self.gripper_reported_setpoint_lcd_number.display)

    def on_arm_status_update_received__callback(self, data):
        self.base_comms_state_update_ready__signal.emit(self.process_comms_to_string(data.base_comm_status))
        self.shoulder_comms_state_update_ready__signal.emit(self.process_comms_to_string(data.shoulder_comm_status))
        self.elbow_comms_state_update_ready__signal.emit(self.process_comms_to_string(data.elbow_comm_status))
        self.roll_comms_state_update_ready__signal.emit(self.process_comms_to_string(data.roll_comm_status))
        self.wrist_pitch_comms_state_update_ready__signal.emit(
        self.process_comms_to_string(data.wrist_pitch_comm_status))
        self.wrist_roll_comms_state_update_ready__signal.emit(self.process_comms_to_string(data.wrist_roll_comm_status))

        self.base_status_update_ready__signal.emit(self.process_statuses_to_string(data.base_status))
        self.shoulder_status_update_ready__signal.emit(self.process_statuses_to_string(data.shoulder_status))
        self.elbow_status_update_ready__signal.emit(self.process_statuses_to_string(data.elbow_status))
        self.roll_status_update_ready__signal.emit(self.process_statuses_to_string(data.roll_status))
        self.wrist_pitch_status_update_ready__signal.emit(self.process_statuses_to_string(data.wrist_pitch_status))
        self.wrist_roll_status_update_ready__signal.emit(self.process_statuses_to_string(data.wrist_roll_status))

        self.base_faults_update_ready__signal.emit(self.process_faults_to_string(data.base_faults))
        self.shoulder_faults_update_ready__signal.emit(self.process_faults_to_string(data.shoulder_faults))
        self.elbow_faults_update_ready__signal.emit(self.process_faults_to_string(data.elbow_faults))
        self.roll_faults_update_ready__signal.emit(self.process_faults_to_string(data.roll_faults))
        self.wrist_pitch_faults_update_ready__signal.emit(self.process_faults_to_string(data.wrist_pitch_faults))
        self.wrist_roll_faults_update_ready__signal.emit(self.process_faults_to_string(data.wrist_roll_faults))

        self.base_position_updated__signal.emit(data.base)
        self.shoulder_position_updated__signal.emit(data.shoulder)
        self.elbow_position_updated__signal.emit(data.elbow)
        self.roll_position_updated__signal.emit(data.roll)
        self.wrist_pitch_position_updated__signal.emit(data.wrist_pitch)
        self.wrist_roll_position_updated__signal.emit(data.wrist_roll)

    def on_gripper_status_update_received__callback(self, data):
        data = data  # type: GripperStatusMessage

        self.pinch_position_updated__signal.emit(data.pinch_position_raw)
        self.forefinger_position_updated__signal.emit(data.forefinger_position_raw)
        self.thumb_position_updated__signal.emit(data.thumb_position_raw)
        self.middlefinger_position_updated__signal.emit(data.middlefinger_position_raw)

        self.pinch_current_updated__signal.emit(data.pinch_current)
        self.forefinger_current_updated__signal.emit(data.forefinger_current)
        self.thumb_current_updated__signal.emit(data.thumb_current)
        self.middlefinger_current_updated__signal.emit(data.middlefinger_current)

        self.gripper_reported_mode_updated__signal.emit(GRIPPER_MODES[data.current_mode])
        self.gripper_reported_setpoint_updated__signal.emit(data.current_finger_position)

    @staticmethod
    def process_faults_to_string(faults):
        fault_output = ""

        for bit_position in FAULT_TO_STRING:
            if (1 << bit_position) & faults:
                fault_output += FAULT_TO_STRING[bit_position] + "\n"

        return fault_output[:-1]

    @staticmethod
    def process_statuses_to_string(statuses):
        status_output = ""

        for bit_position in STATUS_TO_STRING:
            if (1 << bit_position) & statuses:
                status_output += STATUS_TO_STRING[bit_position] + "\n"

        return status_output[:-1]

    @staticmethod
    def process_comms_to_string(comms):
        return COMMS_TO_STRING[comms] if comms in COMMS_TO_STRING else "UNKNOWN"


