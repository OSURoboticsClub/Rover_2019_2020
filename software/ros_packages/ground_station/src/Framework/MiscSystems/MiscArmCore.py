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
    [0.0, -0.035, -0.28, 0.0, 0.0, -0.25],        # Out in front of rover
    [0.0, -0.035, -0.28, 0.25, 0.0, 0.0],
    [0.0, -0.035, -0.45, 0.25, 0.0, 0.0],
    [0.0, -0.25, -0.5, 0.25, 0.0, 0.0]
]

ARM_UNSTOW_PROCEDURE = [
    [0.0, -0.25, -0.5, 0.25, 0.0, 0.0],
    [0.0, -0.035, -0.45, 0.25, 0.0, 0.0],
    [0.0, -0.035, -0.28, 0.25, 0.0, 0.0],
    [0.0, -0.035, -0.28, 0.0, 0.0, -0.25]
]

ARM_COBRA_POSE = [
    [0.0, -0.035, -0.28, 0.0, 0.0, -0.25]
]

ARM_PACKAGE_DROP = [
    #[0.0, -0.035, -0.28, 0.0, 0.0, -0.25]
    #[0.37, -0.02, -0.26, 0.01, 0.0, -0.25]
    [0.37, -0.02, -0.26, 0.01, -0.25, -0.12]    # Run after cobra position or unstow
]

# ui controller
screenSelector = rospy.get_param("one_screen")
if screenSelector == True:
    left = "onescreen"
    right = "onescreen"
else:
    left = "left_screen"
    right = "right_screen"


#####################################
# UbiquitiRadioSettings Class Definition
#####################################
class MiscArm(QtCore.QThread):
    def __init__(self, shared_objects):
        super(MiscArm, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"][left]

        self.arm_control_upright_zeroed_button = self.left_screen.arm_control_upright_zeroed_button  # type:QtWidgets.QPushButton
        self.arm_controls_stow_arm_button = self.left_screen.arm_controls_stow_arm_button  # type:QtWidgets.QPushButton
        self.arm_controls_unstow_arm_button = self.left_screen.arm_controls_unstow_arm_button  # type:QtWidgets.QPushButton
        self.arm_control_cobra_button = self.left_screen.arm_control_cobra_button  # type:QtWidgets.QPushButton
        self.arm_controls_package_drop_button = self.left_screen.arm_controls_package_drop_button # type:QtWidgets.QPushButton

        self.arm_controls_calibration_button = self.left_screen.arm_controls_calibration_button  # type:QtWidgets.QPushButton
        self.arm_controls_clear_faults_button = self.left_screen.arm_controls_clear_faults_button  # type:QtWidgets.QPushButton
        self.arm_controls_reset_motor_drivers_button = self.left_screen.arm_controls_reset_motor_drivers_button  # type:QtWidgets.QPushButton
        self.gripper_home_button = self.left_screen.gripper_home_button # type:QtWidgets.QPushButton
        self.gripper_toggle_light_button = self.left_screen.gripper_toggle_light_button # type:QtWidgets.QPushButton

        self.gripper_toggle_laser_button = self.left_screen.gripper_toggle_laser_button  # type:QtWidgets.QPushButton

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
        self.should_cobra_arm = False

        self.should_package_drop = False

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

            elif self.should_cobra_arm:
                self.cobra_pose()
                self.should_cobra_arm = False

            elif self.should_package_drop:
                self.package_drop()
                self.should_package_drop = False

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

        self.logger.debug("Stopping MiscArm Thread")

    def stow_rover_arm(self):
        for movement in ARM_STOW_PROCEDURE:
            self.process_absolute_move_command(movement)

    def unstow_rover_arm(self):
        for movement in ARM_UNSTOW_PROCEDURE:
            self.process_absolute_move_command(movement)

    def package_drop(self):
        for movement in ARM_PACKAGE_DROP:
            self.process_absolute_move_command(movement)
            self.process_absolute_move_command(movement)

    def cobra_pose(self):
        for movement in ARM_COBRA_POSE:
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
        wrist_roll_set = movement[5]

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
        self.arm_control_cobra_button.clicked.connect(self.on_cobra_arm_button_pressed__slot)

        self.arm_controls_calibration_button.clicked.connect(self.on_set_calibration_button_pressed__slot)
        self.arm_controls_clear_faults_button.clicked.connect(self.on_clear_faults_button_pressed__slot)
        self.arm_controls_reset_motor_drivers_button.clicked.connect(self.on_reset_drivers_button_pressed__slot)

        self.arm_controls_package_drop_button.clicked.connect(self.on_package_drop_button_pressed__slot)

        self.gripper_home_button.clicked.connect(self.on_gripper_home_pressed)
        self.gripper_toggle_light_button.clicked.connect(self.on_gripper_toggle_light_pressed)
        self.gripper_toggle_laser_button.clicked.connect(self.on_gripper_toggle_laser_pressed)

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

    def on_cobra_arm_button_pressed__slot(self):
        self.should_cobra_arm = True

    def on_package_drop_button_pressed__slot(self):
        self.should_package_drop = True

    def on_gripper_home_pressed(self):
        message = GripperControlMessage()
        message.should_home = True

        self.gripper_control_publisher.publish(message)

    def on_gripper_toggle_light_pressed(self):
        message = GripperControlMessage()
        message.toggle_light = True

        self.gripper_control_publisher.publish(message)

    def on_gripper_toggle_laser_pressed(self):
        message = GripperControlMessage()
        message.toggle_laser = True

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
