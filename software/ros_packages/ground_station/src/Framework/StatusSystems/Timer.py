#!/usr/bin/env python2

import sys

from PyQt5 import Qt
from PyQt5 import QtCore
from PyQt5.uic import loadUi

# [ms]
TICK_TIME = 2**6


class Timer(Qt.QMainWindow):
    def __init__(self):
        super(Timer, self).__init__()

        # self.reset.clicked.connect(self.do_reset)
        # self.start.clicked.connect(self.do_start)

        self.timer = Qt.QTimer()
        self.timer.setInterval(TICK_TIME)
        self.timer.timeout.connect(self.tick)

        self.running = False;

        self.do_reset()

    def __start_stop_reset_event(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            if self.running is False:
                self.do_start
            elif self.running is True:
                self.do_pause
        elif event.button() == QtCore.Qt.RightButton:
            self.do_reset

    def display(self):
        self.timer.display("%d:%05.2f" % (self.time // 60, self.time % 60))

    @Qt.pyqtSlot()
    def tick(self):
        self.time += TICK_TIME/1000
        self.display()

    @Qt.pyqtSlot()
    def do_start(self):
        self.timer.start()
        # self.start.setText("Pause")
        # self.start.clicked.disconnect()
        # self.start.clicked.connect(self.do_pause)

    @Qt.pyqtSlot()
    def do_pause(self):
        self.timer.stop()
        # self.start.setText("Start")
        # self.start.clicked.disconnect()
        # self.start.clicked.connect(self.do_start)

    @Qt.pyqtSlot()
    def do_reset(self):
        self.time = 0
        self.display()


app = Qt.QApplication(sys.argv)

timer = Timer()
timer.show()

app.exec_()
