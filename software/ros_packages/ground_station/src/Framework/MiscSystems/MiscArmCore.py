# coding=utf-8
#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging
import rospy
from time import time

from rover_arm.msg import ArmControlMessage, ArmStatusMessage
from rover_control.msg import GripperControlMessage

#####################################
# Global Variables
#####################################
ARM_RELATIVE_CONTROL_TOPIC = "/rover_arm/control/relative"
ARM_ABSOLUTE_CONTROL_TOPIC = "/rover_arm/control/absolute"
ARM_STATUS_TOPIC = "/rover_arm/status"

GRIPPER_CONTROL_TOPIC = "/rover_control/gripper/control"

THREAD_HERTZ = 5

POSITIONAL_TOLERANCE = 0.02

# Order is [base, shoulder, elbow, roll, wrist_pitch, wrist_roll]
ARM_STOW_PROCEDURE = [
    [0.0, -0.035, -0.28, 0.0, 0.0, 0.0],        # Out in front of rover
    [0.0, -0.035, -0.28, -0.25, 0.25, 0.0],
    [0.0, -0.035, -0.5, -0.25, 0.25, 0.0],
    [0.0, -0.25, -0.5, -0.25, 0.25, -0.25]
]

ARM_UNSTOW_PROCEDURE = [
    [0.0, -0.25, -0.5, -0.25, 0.25, -0.25],
    [0.0, -0.035, -0.5, -0.25, 0.25, 0.0],
    [0.0, -0.035, -0.28, -0.25, 0.25, 0.0],
    [0.0, -0.035, -0.28, 0.0, 0.0, 0.0]
]

APPROACH_O2 = [
    [0.0, -0.035, -0.28, 0.0, 0.0, 0.0],                                                         # Out in front of rover
    [0.0, 0.0485472094896, -0.273685193022, -0.00102834607876, -0.17200773156, 0],  # Correct positioning out to left side of rover
    [-0.436250296246, 0.0485472094896, -0.273685193022, -0.00102834607876, -0.17200773156, 0]   # Directly positioned, ready to grip
]

APPROACH_BEACON = [
    [0.0, -0.035, -0.28, 0.0, 0.0, 0.0],                                                         # Out in front of rover
    [0.0, 0.0361371382336, -0.325145836608, -0.00731261537597, -0.129662333807, 0.0569339269566],  # Correct positioning out to right side of rover
    [0.458505849045, 0.0361371382336, -0.325145633259, -0.00731200471916, -0.131140162948, 0.0920117742056]
]


#####################################
# UbiquitiRadioSettings Class Definition
#####################################
class MiscArm(QtCore.QThread):
    def __init__(self, shared_objects):
        super(MiscArm, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]

        self.arm_control_upright_zeroed_button = self.left_screen.arm_control_upright_zeroed_button  # type:QtWidgets.QPushButton
        self.arm_controls_stow_arm_button = self.left_screen.arm_controls_stow_arm_button  # type:QtWidgets.QPushButton
        self.arm_controls_unstow_arm_button = self.left_screen.arm_controls_unstow_arm_button  # type:QtWidgets.QPushButton

        # ##### FIXME #####
        # Remove these once the arm is fixed
        self.arm_controls_stow_arm_button.setEnabled(False)
        self.arm_controls_unstow_arm_button.setEnabled(False)
        # #################

        self.arm_controls_calibration_button = self.left_screen.arm_controls_calibration_button  # type:QtWidgets.QPushButton
        self.arm_controls_clear_faults_button = self.left_screen.arm_controls_clear_faults_button  # type:QtWidgets.QPushButton
        self.arm_controls_reset_motor_drivers_button = self.left_screen.arm_controls_reset_motor_drivers_button  # type:QtWidgets.QPushButton

        self.arm_controls_approach_o2_button = self.left_screen.arm_controls_approach_o2_button  # type:QtWidgets.QPushButton
        self.arm_controls_depart_o2_button = self.left_screen.arm_controls_depart_o2_button  # type:QtWidgets.QPushButton
        self.arm_controls_approach_beacon_button = self.left_screen.arm_controls_approach_beacon_button  # type:QtWidgets.QPushButton
        self.arm_controls_depart_beacon_button = self.left_screen.arm_controls_depart_beacon_button  # type:QtWidgets.QPushButton

        self.gripper_home_button = self.left_screen.gripper_home_button  # type:QtWidgets.QPushButton
        self.gripper_toggle_light_button = self.left_screen.gripper_toggle_light_button  # type:QtWidgets.QPushButton

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.wait_time = 1.0 / THREAD_HERTZ

        self.arm_status_subscriber = rospy.Subscriber(ARM_STATUS_TOPIC, ArmStatusMessage,
                                                      self.new_arm_status_message_received__callback)

        self.arm_relative_control_publisher = rospy.Publisher(ARM_RELATIVE_CONTROL_TOPIC, ArmControlMessage,
                                                              queue_size=1)
        self.arm_absolute_control_publisher = rospy.Publisher(ARM_ABSOLUTE_CONTROL_TOPIC, ArmControlMessage,
                                                              queue_size=1)

        self.gripper_control_publisher = rospy.Publisher(GRIPPER_CONTROL_TOPIC, GripperControlMessage, queue_size=1)

        self.base_position = 0
        self.shoulder_position = 0
        self.elbow_position = 0
        self.roll_position = 0
        self.wrist_pitch_position = 0
        self.wrist_roll_position = 0

        self.should_stow_arm = False
        self.should_unstow_arm = False

        self.should_approach_o2 = False
        self.should_depart_o2 = False
        self.should_approach_beacon = False
        self.should_depart_beacon = False

    def run(self):
        self.logger.debug("Starting MiscArm Thread")

        while self.run_thread_flag:
            start_time = time()

            if self.should_stow_arm:
                self.stow_rover_arm()
                self.should_stow_arm = False
            elif self.should_unstow_arm:
                self.unstow_rover_arm()
                self.should_unstow_arm = False

            elif self.should_approach_o2:
                self.approach_o2()
                self.should_approach_o2 = False

            elif self.should_depart_o2:
                self.depart_o2()
                self.should_depart_o2 = False

            elif self.should_approach_beacon:
                self.approach_beacon()
                self.should_approach_beacon = False

            elif self.should_depart_beacon:
                self.depart_beacon()
                self.should_depart_beacon = False

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

        self.logger.debug("Stopping MiscArm Thread")

    def stow_rover_arm(self):
        for movement in ARM_STOW_PROCEDURE:
            self.process_absolute_move_command(movement)

    def unstow_rover_arm(self):
        for movement in ARM_UNSTOW_PROCEDURE:
            self.process_absolute_move_command(movement)

    def approach_o2(self):
        for movement in APPROACH_O2:
            self.process_absolute_move_command(movement)

    def depart_o2(self):
        for movement in reversed(APPROACH_O2):
            self.process_absolute_move_command(movement)

    def approach_beacon(self):
        for movement in APPROACH_BEACON:
            self.process_absolute_move_command(movement)

    def depart_beacon(self):
        for movement in reversed(APPROACH_BEACON):
            self.process_absolute_move_command(movement)

    def process_absolute_move_command(self, movement):
        message = ArmControlMessage()

        message.base = movement[0]
        message.shoulder = movement[1]
        message.elbow = movement[2]
        message.roll = movement[3]
        message.wrist_pitch = movement[4]
        message.wrist_roll = movement[5]

        self.arm_absolute_control_publisher.publish(message)

        self.wait_for_targets_reached(movement)
        self.msleep(250)

    def wait_for_targets_reached(self, movement):
        base_set = movement[0]
        shoulder_set = movement[1]
        elbow_set = movement[2]
        roll_set = movement[3]
        wrist_pitch_set = movement[4]
        wrist_roll_set = movement[5] - (wrist_pitch_set / 2.0)

        while abs(self.base_position - base_set) > POSITIONAL_TOLERANCE:
            # self.logger.debug("Waiting for base| %f\t%f" % (self.base_position, base_set))
            self.msleep(10)

        while abs(self.shoulder_position - shoulder_set) > POSITIONAL_TOLERANCE:
            # self.logger.debug("Waiting for shoulder| %f\t%f" % (self.shoulder_position, shoulder_set))
            self.msleep(10)

        while abs(self.elbow_position - elbow_set) > POSITIONAL_TOLERANCE:
            # self.logger.debug("Waiting for elbow| %f\t%f" % (self.elbow_position, elbow_set))
            self.msleep(10)

        while abs(self.roll_position - roll_set) > POSITIONAL_TOLERANCE:
            # self.logger.debug("Waiting for roll| %f\t%f" % (self.roll_position, roll_set))
            self.msleep(10)

        while abs(self.wrist_pitch_position - wrist_pitch_set) > POSITIONAL_TOLERANCE:
            # self.logger.debug("Waiting for wrist_pitch| %f\t%f" % (self.wrist_pitch_position, wrist_pitch_set))
            self.msleep(10)

        while abs(self.wrist_roll_position - wrist_roll_set) > POSITIONAL_TOLERANCE:
            # self.logger.debug("Waiting for wrist_roll| %f\t%f" % (self.wrist_roll_position, wrist_roll_set))
            self.msleep(10)

    def connect_signals_and_slots(self):
        self.arm_controls_stow_arm_button.clicked.connect(self.on_stow_arm_button_pressed__slot)
        self.arm_controls_unstow_arm_button.clicked.connect(self.on_unstow_arm_button_pressed__slot)
        self.arm_control_upright_zeroed_button.clicked.connect(self.on_upright_zeroed_button_pressed__slot)

        self.arm_controls_calibration_button.clicked.connect(self.on_set_calibration_button_pressed__slot)
        self.arm_controls_clear_faults_button.clicked.connect(self.on_clear_faults_button_pressed__slot)
        self.arm_controls_reset_motor_drivers_button.clicked.connect(self.on_reset_drivers_button_pressed__slot)

        self.arm_controls_approach_o2_button.clicked.connect(self.on_approach_o2_button_pressed__slot)
        self.arm_controls_approach_beacon_button.clicked.connect(self.on_approach_beacon_button_pressed__slot)

        self.gripper_home_button.clicked.connect(self.on_gripper_home_pressed)
        self.gripper_toggle_light_button.clicked.connect(self.on_gripper_toggle_light_pressed)

    def on_upright_zeroed_button_pressed__slot(self):
        self.process_absolute_move_command([0 for _ in range(6)])

    def on_set_calibration_button_pressed__slot(self):
        message = ArmControlMessage()
        message.calibrate = True
        self.arm_relative_control_publisher.publish(message)

    def on_clear_faults_button_pressed__slot(self):
        message = ArmControlMessage()
        message.clear_faults = True
        self.arm_relative_control_publisher.publish(message)

    def on_reset_drivers_button_pressed__slot(self):
        message = ArmControlMessage()
        message.reset_controllers = True
        self.arm_relative_control_publisher.publish(message)

    def on_stow_arm_button_pressed__slot(self):
        self.should_stow_arm = True

    def on_unstow_arm_button_pressed__slot(self):
        self.should_unstow_arm = True

    def on_approach_o2_button_pressed__slot(self):
        self.should_approach_o2 = True

    def on_depart_o2_button_pressed__slot(self):
        self.should_depart_o2 = True

    def on_approach_beacon_button_pressed__slot(self):
        self.should_approach_beacon = True

    def on_depart_beacon_button_pressed__slot(self):
        self.should_depart_beacon = True

    def on_gripper_home_pressed(self):
        message = GripperControlMessage()
        message.gripper_mode = 2
        message.gripper_position_absolute = -1
        message.should_home = True

        self.gripper_control_publisher.publish(message)

    def on_gripper_toggle_light_pressed(self):
        message = GripperControlMessage()
        message.gripper_mode = 2
        message.toggle_light = True
        message.gripper_position_absolute = -1

        self.gripper_control_publisher.publish(message)

    def new_arm_status_message_received__callback(self, data):
        self.base_position = data.base
        self.shoulder_position = data.shoulder
        self.elbow_position = data.elbow
        self.roll_position = data.roll
        self.wrist_pitch_position = data.wrist_pitch
        self.wrist_roll_position = data.wrist_roll

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
