#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64MultiArray

class RDFProceesor(object):
    def __init__(self):
        super(RDFProceesor, self).__init__()
        self.sub = rospy.Subscriber("/rover_science/rdf/data", Float64MultiArray, self.on_rdf_data_received)
        self.new_data = False
        self.current_data = None

        print self.sub

    def run(self):
        while True:
            if self.new_data:
                print self.current_data
                self.new_data = False
            time.sleep(0.01)

    def on_rdf_data_received(self, rdf_data):
        print rdf_data
        self.current_data = rdf_data
        self.new_data = True


if __name__ == '__main__':
    processor = RDFProceesor()
    processor.run()