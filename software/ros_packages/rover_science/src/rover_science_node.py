#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rospy

from time import time, sleep

import serial.rs485
import minimalmodbus
import numpy

# Custom Imports
from rover_science.msg import SoilSensorStatusMessage
from std_msgs.msg import Float64MultiArray

#####################################
# Global Variables
#####################################
NODE_NAME = "science_node"

DEFAULT_PORT = "/dev/rover/ttyRDF_SoilProbe"
DEFAULT_RDF_BAUD = 115200
DEFAULT_SOIL_BAUD = 9600

DEFAULT_RDF_PUBLISHER_TOPIC = "rdf/data"
DEFAULT_SOIL_PROBE_PUBLISHER_TOPIC = "soil_probe/data"

RDF_NODE_ID = 1

COMMUNICATIONS_TIMEOUT = 0.1  # Seconds

RX_DELAY = 0.01
TX_DELAY = 0.01

DEFAULT_HERTZ = 20
FAILED_RDF_LIMIT = 20

SOIL_PROBE_READ_TIMEOUT = 8
SOIL_PROBE_EXIT_TIMEOUT = 20

SOIL_PROBE_ADDRESS = "mar"

PAN_TILT_MODBUS_REGISTERS = {
    "CENTER_ALL": 0,

    "PAN_ADJUST_POSITIVE": 1,
    "PAN_ADJUST_NEGATIVE": 2,
    "TILT_ADJUST_POSITIVE": 3,
    "TILT_ADJUST_NEGATIVE": 4
}

SOIL_PROBE_COMMANDS = {
    "GET_ADDRESS": "AD=",
    "DESCRIPTION": "DS=",
    "PROBE_ENABLED": "PE=",
    "TAKE_READING": "TR",
    "TRANSMIT_READING": "T3",

    "QUERY": "?"
}

TRANSMIT_SET_3_INDICES = {
    "TEMP C": 0,
    "Moisture": 2,
    "Loss Tangent": 3,
    "Soil Electrical Conductivity (tc)": 4,
    "Real Dielectric Permittivity (tc)": 6,
    "Imag Dielectric Permittivity (tc)": 8,
}

NODE_LAST_SEEN_TIMEOUT = 2  # seconds


#####################################
# DriveControl Class Definition
#####################################
class RoverScience(object):
    INSTRUMENTS = [
        "RDF",
        "SOIL"
    ]

    def __init__(self):
        rospy.init_node(NODE_NAME)

        self.port = rospy.get_param("~port", DEFAULT_PORT)
        self.baud = rospy.get_param("~baud", DEFAULT_RDF_BAUD)

        self.science_node_id = rospy.get_param("~pan_tilt_node_id", RDF_NODE_ID)

        self.wait_time = 1.0 / rospy.get_param("~hertz", DEFAULT_HERTZ)

        self.rdf_node = None
        self.soil_node = None

        self.connect_to_rdf()

        self.rdf_publisher = rospy.Publisher(DEFAULT_RDF_PUBLISHER_TOPIC, Float64MultiArray, queue_size=1)
        self.soil_probe_publisher = rospy.Publisher(DEFAULT_SOIL_PROBE_PUBLISHER_TOPIC, SoilSensorStatusMessage, queue_size=1)

        self.modbus_nodes_seen_time = time()

        self.failed_rdf_modbus_count = 0
        self.soil_probe_timeout_cumulative = 0

        self.which_instrument = self.INSTRUMENTS.index("RDF")

        self.probe_response_line = ""

        self.run()

    def __setup_minimalmodbus_for_485(self):
        self.rdf_node.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=COMMUNICATIONS_TIMEOUT)  # type: serial.Serial
        self.rdf_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0,
                                                                     delay_before_rx=RX_DELAY,
                                                                     delay_before_tx=TX_DELAY)

    def run(self):
        while not rospy.is_shutdown():
            if self.which_instrument == self.INSTRUMENTS.index("RDF"):
                try:
                    registers = self.rdf_node.read_registers(0, 1)
                    self.rdf_publisher.publish(Float64MultiArray(data=[registers[0], time()]))
                    self.failed_rdf_modbus_count = 0
                except Exception, e:
                    # print e
                    self.failed_rdf_modbus_count += 1

                if self.failed_rdf_modbus_count == FAILED_RDF_LIMIT:
                    print "RDF not present. Trying soil sensor"
                    self.which_instrument = self.INSTRUMENTS.index("SOIL")

            elif self.which_instrument == self.INSTRUMENTS.index("SOIL"):
                if not self.soil_node:
                    self.switch_node_to_soil_probe()

                self.broadcast_soil_sensor_data()

                if self.soil_probe_timeout_cumulative > SOIL_PROBE_EXIT_TIMEOUT:
                    print "No science devices present. Exiting..."
                    return

    def switch_node_to_soil_probe(self):
        del self.rdf_node

        self.soil_node = serial.rs485.RS485(self.port, baudrate=DEFAULT_SOIL_BAUD, timeout=COMMUNICATIONS_TIMEOUT)  # type: serial.Serial
        self.soil_node.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0, delay_before_rx=RX_DELAY, delay_before_tx=TX_DELAY)

        out = "%s%s\r\n" % (SOIL_PROBE_ADDRESS, SOIL_PROBE_COMMANDS["PROBE_ENABLED"] + "1")
        self.soil_node.write(out)
        sleep(0.1)

    def broadcast_soil_sensor_data(self):
        self.request_soil_reading()
        self.request_reading_results()
        self.get_probe_response_line()
        self.process_probe_response_and_send()

    def process_probe_response_and_send(self):
        if self.probe_response_line != "":

            try:
                self.probe_response_line = self.probe_response_line.replace(SOIL_PROBE_ADDRESS, "")

                split_results = self.probe_response_line.split(",")

                temp_c = split_results[TRANSMIT_SET_3_INDICES["TEMP C"]]
                moisture = split_results[TRANSMIT_SET_3_INDICES["Moisture"]]
                loss_tangent = split_results[TRANSMIT_SET_3_INDICES["Loss Tangent"]]
                sec_tc = split_results[TRANSMIT_SET_3_INDICES["Soil Electrical Conductivity (tc)"]]
                rdp_tc = split_results[TRANSMIT_SET_3_INDICES["Real Dielectric Permittivity (tc)"]]
                idp_tc = split_results[TRANSMIT_SET_3_INDICES["Imag Dielectric Permittivity (tc)"]]

                message = SoilSensorStatusMessage()
                message.temp_c = float(temp_c)
                message.moisture = float(moisture)
                message.loss_tangent = float(loss_tangent)
                message.soil_electrical_conductivity = float(sec_tc)
                message.real_dielectric_permittivity = float(rdp_tc)
                message.imaginary_dielectric_permittivity = float(idp_tc)

                self.soil_probe_publisher.publish(message)

            except:
                print "Soil probe line corrupted. Trying again..."

    def get_probe_response_line(self):
        start_time = time()
        char = ""
        self.probe_response_line = ""

        while char != '\n' and (time() - start_time) < 2:
            if self.soil_node.inWaiting():
                char = self.soil_node.read()
                self.probe_response_line += char

        if self.probe_response_line:
            self.soil_probe_timeout_cumulative = 0
            # print self.probe_response_line
        else:
            # print "timeout"
            self.soil_probe_timeout_cumulative += 2

    def request_soil_reading(self):
        out = "%s%s\r\n" % (SOIL_PROBE_ADDRESS, SOIL_PROBE_COMMANDS["TAKE_READING"])
        self.soil_node.write(out)
        sleep(0.1)

    def request_reading_results(self):
        out = "%s%s\r\n" % (SOIL_PROBE_ADDRESS, SOIL_PROBE_COMMANDS["TRANSMIT_READING"])
        self.soil_node.write(out)

    def smoothListTriangle(self, list, strippedXs=False, degree=5):

        weight = []

        window = degree * 2 - 1

        smoothed = [0.0] * (len(list) - window)

        for x in range(1, 2 * degree): weight.append(degree - abs(degree - x))

        w = numpy.array(weight)

        for i in range(len(smoothed)):
            smoothed[i] = sum(numpy.array(list[i:i + window]) * w) / float(sum(w))

        return smoothed

    def smoothListGaussian(self, list, strippedXs=False, degree=5):

        window = degree * 2 - 1

        weight = numpy.array([1.0] * window)

        weightGauss = []

        for i in range(window):
            i = i - degree + 1

            frac = i / float(window)

            gauss = 1 / (numpy.exp((4 * (frac)) ** 2))

            weightGauss.append(gauss)

        weight = numpy.array(weightGauss) * weight

        smoothed = [0.0] * (len(list) - window)

        for i in range(len(smoothed)):
            smoothed[i] = sum(numpy.array(list[i:i + window]) * weight) / sum(weight)

        return smoothed

    def connect_to_rdf(self):
        self.rdf_node = minimalmodbus.Instrument(self.port, int(self.science_node_id))
        self.__setup_minimalmodbus_for_485()


if __name__ == "__main__":
    RoverScience()
