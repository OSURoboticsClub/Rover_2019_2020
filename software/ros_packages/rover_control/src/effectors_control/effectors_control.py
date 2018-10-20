#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rospy
from time import time, sleep

import serial.rs485
import minimalmodbus

# from std_msgs.msg import UInt8, UInt16

# Custom Imports
from rover_control.msg import MiningControlMessage, MiningStatusMessage, GripperControlMessage, GripperStatusMessage, CameraControlMessage

#####################################
# Global Variables
#####################################
NODE_NAME = "effectors_control"

# ##### Communication Defines #####
DEFAULT_PORT = "/dev/rover/ttyEffectors"
# DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200

GRIPPER_NODE_ID = 1
MINING_NODE_ID = 2
SCIENCE_NODE_ID = 3

GRIPPER_TIMEOUT = 0.5
MINING_TIMEOUT = 0.3

FAILED_GRIPPER_MODBUS_LIMIT = 20
FAILED_MINING_MODBUS_LIMIMT = 20

RX_DELAY = 0.01
TX_DELAY = 0.01

DEFAULT_HERTZ = 40

GRIPPER_CONTROL_SUBSCRIBER_TOPIC = "gripper/control"
GRIPPER_STATUS_PUBLISHER_TOPIC = "gripper/status"

MINING_CONTROL_SUBSCRIBER_TOPIC = "mining/control"
MINING_STATUS_PUBLISHER_TOPIC = "mining/status"

CAMERA_CONTROL_SUBSCRIBER_TOPIC = "camera/control"

# ##### Gripper Defines #####
GRIPPER_MODBUS_REGISTERS = {
    "MODE": 0,
    "FINGER_POSITION": 1,
    "HOME": 2,
    "LIGHT_STATE": 3,

    "CURRENT_PINCH": 4,
    "CURRENT_FOREFINGER": 5,
    "CURRENT_THUMB": 6,
    "CURRENT_MIDDLEFINGER": 7,
    "POSITION_PINCH": 8,
    "POSITION_FOREFINGER": 9,
    "POSITION_THUMB": 10,
    "POSITION_MIDDLEFINGER": 11,
    "LIGHT_STATE_OUPUT": 12,
    "MODE_OUTPUT": 13,
    "FINGER_POSITION_OUTPUT": 14
}

GRIPPER_MODES = {
    "NO_CHANGE": 0,
    "NORMAL": 1,
    "TWO_FINGER_PINCH": 2,
    "WIDE ": 3,
    "SCISSOR": 4
}

DEFAULT_GRIPPER_REGISTERS = [
    0,  # No change
    0,  # No positional update
    0,  # Do not home
    0,  # Light off
]

GRIPPER_UNIVERSAL_POSITION_MAX = 10000

# ##### Mining Defines #####
MINING_MODBUS_REGISTERS = {
    "LIFT_SET_POSITIVE": 0,
    "LIFT_SET_NEGATIVE": 1,
    "TILT_SET_POSITIVE": 2,
    "TILT_SET_NEGATIVE": 3,
    "TILT_SET_ABSOLUTE": 4,
    "LIFT_SET_ABSOLUTE": 5,
    "MEASURE": 6,
    "TARE": 7,
    "CAL_FACTOR": 8,

    "CHANGE_VIEW_MODE": 9,
    "ZOOM_IN": 10,
    "ZOOM_OUT": 11,
    "FULL_ZOOM_IN": 12,
    "FULL_ZOOM_OUT": 13,
    "SHOOT": 14,

    "CURRENT_POSITION_LIFT": 15,
    "CURRENT_POSITION_TILT": 16,
    "MEASURED_WEIGHT": 17
}


MINING_POSITIONAL_THRESHOLD = 20

# ##### Science Defines #####

# ##### Misc Defines #####
NODE_LAST_SEEN_TIMEOUT = 2  # seconds

UINT16_MAX = 65535


#####################################
# DriveControl Class Definition
#####################################
class EffectorsControl(object):
    EFFECTORS = [
        "GRIPPER",
        "MINING"
    ]

    def __init__(self):
        rospy.init_node(NODE_NAME)

        self.port = rospy.get_param("~port", DEFAULT_PORT)
        self.baud = rospy.get_param("~baud", DEFAULT_BAUD)

        self.gripper_node_id = rospy.get_param("~gripper_node_id", GRIPPER_NODE_ID)
        self.mining_node_id = rospy.get_param("~mining_node_id", MINING_NODE_ID)

        self.gripper_control_subscriber_topic = rospy.get_param("~gripper_control_subscriber_topic",
                                                                GRIPPER_CONTROL_SUBSCRIBER_TOPIC)
        self.gripper_status_publisher_topic = rospy.get_param("~gripper_status_publisher_topic",
                                                              GRIPPER_STATUS_PUBLISHER_TOPIC)

        self.mining_control_subscriber_topic = rospy.get_param("~mining_control_subscriber_topic",
                                                               MINING_CONTROL_SUBSCRIBER_TOPIC)

        self.mining_status_publisher_topic = rospy.get_param("~mining_status_publisher_topic",
                                                             MINING_STATUS_PUBLISHER_TOPIC)

        self.camera_control_subscriber_topic = rospy.get_param("~camera_control_subscriber_topic",
                                                               CAMERA_CONTROL_SUBSCRIBER_TOPIC)

        self.wait_time = 1.0 / rospy.get_param("~hertz", DEFAULT_HERTZ)

        self.gripper_node = None  # type:minimalmodbus.Instrument
        self.mining_node = None  # type:minimalmodbus.Instrument

        self.gripper_node_present = False
        self.mining_node_present = True

        self.connect_to_nodes()
        # self.check_which_nodes_present()

        # ##### Subscribers #####
        self.gripper_control_subscriber = rospy.Subscriber(self.gripper_control_subscriber_topic, GripperControlMessage, self.gripper_control_message_received__callback)

        self.mining_control_subscriber = rospy.Subscriber(self.mining_control_subscriber_topic, MiningControlMessage, self.mining_control_message_received__callback)

        self.camera_control_subscriber = rospy.Subscriber(self.camera_control_subscriber_topic, CameraControlMessage, self.camera_control_message_received__callback)

        # ##### Publishers #####
        self.gripper_status_publisher = rospy.Publisher(self.gripper_status_publisher_topic, GripperStatusMessage, queue_size=1)

        self.mining_status_publisher = rospy.Publisher(self.mining_status_publisher_topic, MiningStatusMessage, queue_size=1)

        # ##### Misc #####
        self.modbus_nodes_seen_time = time()

        # ##### Mining Variables #####
        self.mining_registers = [0 for _ in MINING_MODBUS_REGISTERS]
        self.gripper_registers = None

        self.mining_control_message = None  # type:MiningControlMessage
        self.new_mining_control_message = False

        self.gripper_control_message = None
        self.new_gripper_control_message = False

        self.camera_control_message = None  # type: CameraControlMessage
        self.new_camera_control_message = False

        self.failed_gripper_modbus_count = 0
        self.failed_mining_modbus_count = 0

        self.which_effector = self.EFFECTORS.index("GRIPPER")

        self.run()

    def __setup_minimalmodbus_for_485(self):
        self.gripper_node.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=GRIPPER_TIMEOUT)
        self.gripper_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0,
                                                                         delay_before_rx=RX_DELAY,
                                                                         delay_before_tx=TX_DELAY)

        self.mining_node.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=MINING_TIMEOUT)
        self.mining_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0,
                                                                        delay_before_rx=RX_DELAY,
                                                                        delay_before_tx=TX_DELAY)

    def run(self):
        while not rospy.is_shutdown():
            if self.which_effector == self.EFFECTORS.index("GRIPPER"):
                try:
                    self.run_arm()
                    self.failed_gripper_modbus_count = 0
                except:
                    self.failed_gripper_modbus_count += 1

                if self.failed_gripper_modbus_count == FAILED_GRIPPER_MODBUS_LIMIT:
                    print "Gripper not present. Trying mining."
                    self.which_effector = self.EFFECTORS.index("MINING")

            elif self.which_effector == self.EFFECTORS.index("MINING"):
                try:
                    self.run_mining()
                    self.failed_mining_modbus_count = 0
                except Exception, e:
                    print e
                    self.failed_mining_modbus_count += 1

                if self.failed_mining_modbus_count == FAILED_MINING_MODBUS_LIMIMT:
                    print "No effectors present. Exiting...."
                    return

    def run_arm(self):
        self.process_gripper_control_message()
        self.send_gripper_status_message()

    def run_mining(self):
        self.process_mining_control_message()
        self.send_mining_status_message()
        self.process_camera_control_message()

    def connect_to_nodes(self):
        self.gripper_node = minimalmodbus.Instrument(self.port, int(self.gripper_node_id))
        self.mining_node = minimalmodbus.Instrument(self.port, int(self.mining_node_id))

        self.__setup_minimalmodbus_for_485()

    def process_mining_control_message(self):
        if self.new_mining_control_message and self.mining_node_present:
            lift_set_relative = self.mining_control_message.lift_set_relative
            tilt_set_relative = self.mining_control_message.tilt_set_relative
            lift_set_absolute = self.mining_control_message.lift_set_absolute
            tilt_set_absolute = self.mining_control_message.tilt_set_absolute
            cal_factor = min(self.mining_control_message.cal_factor, UINT16_MAX)
            measure = self.mining_control_message.measure
            tare = self.mining_control_message.tare

            if lift_set_absolute < 1024:
                self.mining_registers[MINING_MODBUS_REGISTERS["LIFT_SET_ABSOLUTE"]] = lift_set_absolute
            else:
                if lift_set_relative >= 0:
                    self.mining_registers[MINING_MODBUS_REGISTERS["LIFT_SET_POSITIVE"]] = lift_set_relative
                else:
                    self.mining_registers[MINING_MODBUS_REGISTERS["LIFT_SET_NEGATIVE"]] = -lift_set_relative

            if tilt_set_absolute < 1024:
                self.mining_registers[MINING_MODBUS_REGISTERS["TILT_SET_ABSOLUTE"]] = tilt_set_absolute
            else:
                if tilt_set_relative >= 0:
                    self.mining_registers[MINING_MODBUS_REGISTERS["TILT_SET_POSITIVE"]] = tilt_set_relative
                else:
                    self.mining_registers[MINING_MODBUS_REGISTERS["TILT_SET_NEGATIVE"]] = -tilt_set_relative

            if cal_factor > -1:
                self.mining_registers[MINING_MODBUS_REGISTERS["CAL_FACTOR"]] = cal_factor

            self.mining_registers[MINING_MODBUS_REGISTERS["MEASURE"]] = int(measure)
            self.mining_registers[MINING_MODBUS_REGISTERS["TARE"]] = int(tare)

            self.mining_node.write_registers(0, self.mining_registers)

            self.modbus_nodes_seen_time = time()
            self.new_mining_control_message = False

    def process_camera_control_message(self):
        if self.new_camera_control_message:
            print self.camera_control_message
            self.mining_registers[MINING_MODBUS_REGISTERS["LIFT_SET_ABSOLUTE"]] = 1024
            self.mining_registers[MINING_MODBUS_REGISTERS["LIFT_SET_POSITIVE"]] = 0
            self.mining_registers[MINING_MODBUS_REGISTERS["LIFT_SET_NEGATIVE"]] = 0
            self.mining_registers[MINING_MODBUS_REGISTERS["TILT_SET_ABSOLUTE"]] = 1024
            self.mining_registers[MINING_MODBUS_REGISTERS["TILT_SET_POSITIVE"]] = 0
            self.mining_registers[MINING_MODBUS_REGISTERS["TILT_SET_NEGATIVE"]] = 0

            self.mining_registers[MINING_MODBUS_REGISTERS["MEASURE"]] = 0
            self.mining_registers[MINING_MODBUS_REGISTERS["TARE"]] = 0

            self.mining_registers[MINING_MODBUS_REGISTERS["CHANGE_VIEW_MODE"]] = self.camera_control_message.camera_mode
            self.mining_registers[MINING_MODBUS_REGISTERS["ZOOM_IN"]] = self.camera_control_message.zoom_in
            self.mining_registers[MINING_MODBUS_REGISTERS["ZOOM_OUT"]] = self.camera_control_message.zoom_out
            self.mining_registers[MINING_MODBUS_REGISTERS["FULL_ZOOM_IN"]] = self.camera_control_message.full_zoom_in
            self.mining_registers[MINING_MODBUS_REGISTERS["FULL_ZOOM_OUT"]] = self.camera_control_message.full_zoom_out
            self.mining_registers[MINING_MODBUS_REGISTERS["SHOOT"]] = self.camera_control_message.shoot

            self.mining_node.write_registers(0, self.mining_registers)
            self.modbus_nodes_seen_time = time()

            self.new_camera_control_message = False

    def send_mining_status_message(self):
        if self.mining_node_present:
            self.mining_registers = self.mining_node.read_registers(0, len(MINING_MODBUS_REGISTERS))

            message = MiningStatusMessage()
            message.lift_position = self.mining_registers[MINING_MODBUS_REGISTERS["CURRENT_POSITION_LIFT"]]
            message.tilt_position = self.mining_registers[MINING_MODBUS_REGISTERS["CURRENT_POSITION_TILT"]]
            message.measured_weight = self.mining_registers[MINING_MODBUS_REGISTERS["MEASURED_WEIGHT"]]

            self.mining_status_publisher.publish(message)

            self.modbus_nodes_seen_time = time()

    def process_gripper_control_message(self):
        if not self.gripper_registers:
            self.gripper_registers = self.gripper_node.read_registers(0, len(GRIPPER_MODBUS_REGISTERS))

        if self.new_gripper_control_message:
            self.gripper_registers[GRIPPER_MODBUS_REGISTERS["MODE"]] = self.gripper_control_message.gripper_mode

            if self.gripper_control_message.should_home:
                self.gripper_registers[GRIPPER_MODBUS_REGISTERS["HOME"]] = 1
                self.gripper_node.write_registers(0, self.gripper_registers)
                self.gripper_node.write_registers(0, DEFAULT_GRIPPER_REGISTERS)

                homing_complete = False

                while not homing_complete:
                    self.gripper_registers = self.gripper_node.read_registers(0, len(GRIPPER_MODBUS_REGISTERS))
                    self.send_gripper_status_message()

                    if self.gripper_registers[GRIPPER_MODBUS_REGISTERS["FINGER_POSITION_OUTPUT"]] == 0:
                        homing_complete = True
                        self.gripper_registers = None

            else:
                if self.gripper_control_message.toggle_light:
                    self.gripper_registers[GRIPPER_MODBUS_REGISTERS["LIGHT_STATE"]] = 0 if self.gripper_registers[GRIPPER_MODBUS_REGISTERS["LIGHT_STATE"]] else 1
                    self.gripper_control_message.toggle_light = False

                gripper_absolute = self.gripper_control_message.gripper_position_absolute
                gripper_relative = self.gripper_control_message.gripper_position_relative

                if -1 < gripper_absolute < UINT16_MAX:
                    self.gripper_registers[GRIPPER_MODBUS_REGISTERS["FINGER_POSITION"]] = min(max(gripper_absolute, 0), UINT16_MAX)
                else:
                    new_position = self.gripper_registers[GRIPPER_MODBUS_REGISTERS["FINGER_POSITION"]] + gripper_relative
                    self.gripper_registers[GRIPPER_MODBUS_REGISTERS["FINGER_POSITION"]] = min(max(new_position, 0), UINT16_MAX)

                self.gripper_node.write_registers(0, self.gripper_registers)

                self.gripper_node.write_registers(0, DEFAULT_GRIPPER_REGISTERS)

        self.gripper_control_message = None
        self.new_gripper_control_message = False

    def send_gripper_status_message(self):
        registers = self.gripper_node.read_registers(0, len(GRIPPER_MODBUS_REGISTERS))

        message = GripperStatusMessage()
        message.pinch_position_raw = registers[GRIPPER_MODBUS_REGISTERS["POSITION_PINCH"]]
        message.forefinger_position_raw = registers[GRIPPER_MODBUS_REGISTERS["POSITION_FOREFINGER"]]
        message.thumb_position_raw = registers[GRIPPER_MODBUS_REGISTERS["POSITION_THUMB"]]
        message.middlefinger_position_raw = registers[GRIPPER_MODBUS_REGISTERS["POSITION_MIDDLEFINGER"]]
        message.pinch_current = registers[GRIPPER_MODBUS_REGISTERS["CURRENT_PINCH"]]
        message.forefinger_current = registers[GRIPPER_MODBUS_REGISTERS["CURRENT_FOREFINGER"]]
        message.thumb_current = registers[GRIPPER_MODBUS_REGISTERS["CURRENT_THUMB"]]
        message.middlefinger_current = registers[GRIPPER_MODBUS_REGISTERS["CURRENT_MIDDLEFINGER"]]
        message.light_on = registers[GRIPPER_MODBUS_REGISTERS["LIGHT_STATE_OUPUT"]]
        message.current_mode = registers[GRIPPER_MODBUS_REGISTERS["MODE_OUTPUT"]]
        message.current_finger_position = registers[GRIPPER_MODBUS_REGISTERS["FINGER_POSITION_OUTPUT"]]

        self.gripper_status_publisher.publish(message)

    def gripper_control_message_received__callback(self, control_message):
        self.gripper_control_message = control_message
        self.new_gripper_control_message = True

    def mining_control_message_received__callback(self, control_message):
        self.mining_control_message = control_message
        self.new_mining_control_message = True

    def camera_control_message_received__callback(self, control_message):
        self.camera_control_message = control_message
        self.new_camera_control_message = True


if __name__ == "__main__":
    EffectorsControl()
