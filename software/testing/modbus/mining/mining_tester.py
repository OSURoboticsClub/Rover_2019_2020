#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports

from time import time, sleep

import serial.rs485
import minimalmodbus

# from std_msgs.msg import UInt8, UInt16

# Custom Imports
# from rover_control.msg import TowerPanTiltControlMessage

#####################################
# Global Variables
#####################################
NODE_NAME = "chassis_pan_tilt_control"

DEFAULT_PORT = "/dev/rover/ttyEffectors"
DEFAULT_BAUD = 115200

DEFAULT_INVERT = False

DEFAULT_PAN_TILT_CONTROL_TOPIC = "chassis/pan_tilt/control"

PAN_TILT_NODE_ID = 1

COMMUNICATIONS_TIMEOUT = 0.15
# Seconds

RX_DELAY = 0.01
TX_DELAY = 0.01

DEFAULT_HERTZ = 20

MINING_MODBUS_REGISTERS = {
    "LIFT_SET": 0,
    "TILT_SET": 1,
    "TARE": 2,
    "CAL_FACTOR": 3,

    "LIFT_POSITION": 4,
    "TILT_POSITION": 5,
    "MEASURED_WEIGHT": 6
}

POSITIONAL_THRESHOLD = 20

NODE_LAST_SEEN_TIMEOUT = 2  # seconds


#####################################
# DriveControl Class Definition
#####################################
class MiningControl(object):
    def __init__(self):
        self.port = DEFAULT_PORT
        self.baud = 115200

        self.mining_node = None
        self.tower_node = None

        self.connect_to_pan_tilt_and_tower()

        self.pan_tilt_control_message = None
        self.new_pan_tilt_control_message = False

        self.modbus_nodes_seen_time = time()

        self.mining_registers = [
            0,
            0,
            0,
            0,
            0,
            0,
            0
        ]

        self.run()

    def __setup_minimalmodbus_for_485(self):
        self.mining_node.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=COMMUNICATIONS_TIMEOUT)
        self.mining_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0,
                                                                        delay_before_rx=RX_DELAY,
                                                                        delay_before_tx=TX_DELAY)

    def run(self):
        self.initialize_mining_system()
        while True:
            try:
                print self.mining_node.read_registers(0, 7)
                self.mining_registers[MINING_MODBUS_REGISTERS["TILT_SET"]] = int(input("Enter new tilt value:"))
                self.mining_node.write_registers(0, self.mining_registers)

            except Exception, e:
                print e

    def connect_to_pan_tilt_and_tower(self):
        self.mining_node = minimalmodbus.Instrument(self.port, int(2))
        self.__setup_minimalmodbus_for_485()

    def initialize_mining_system(self):
        self.mining_registers[MINING_MODBUS_REGISTERS["LIFT_SET"]] = 1023
        self.mining_registers[MINING_MODBUS_REGISTERS["TILT_SET"]] = 350

        self.mining_registers[MINING_MODBUS_REGISTERS["CAL_FACTOR"]] = 114

        while abs(self.mining_registers[MINING_MODBUS_REGISTERS["LIFT_POSITION"]] - self.mining_registers[MINING_MODBUS_REGISTERS["LIFT_SET"]]) > POSITIONAL_THRESHOLD or \
                        abs(self.mining_registers[MINING_MODBUS_REGISTERS["TILT_POSITION"]] - self.mining_registers[MINING_MODBUS_REGISTERS["TILT_SET"]]) > POSITIONAL_THRESHOLD:
            try:
                self.mining_node.write_registers(0, self.mining_registers)
                self.mining_registers = self.mining_node.read_registers(0, 7)
            except Exception, e:
                print "Had trouble communicating:", e

        try:
            self.mining_registers[MINING_MODBUS_REGISTERS["TARE"]] = 1
            self.mining_node.write_registers(0, self.mining_registers)
            self.mining_registers[MINING_MODBUS_REGISTERS["TARE"]] = 0
        except:
            print "Had trouble communicating: no tare: ", e


if __name__ == "__main__":
    MiningControl()
