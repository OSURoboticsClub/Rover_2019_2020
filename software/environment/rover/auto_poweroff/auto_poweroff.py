#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
from time import time, sleep
from os.path import exists, dirname, realpath
from os import system, chdir
import sys

#####################################
# Global Variables
#####################################
UDEV_RULES_PATHS = ["../UDEV_rules/99-rover-usb-serial.rules"]

SHUTDOWN_TIMEOUT = 5


#####################################
# get_script_path Definition
#####################################
def get_script_path():
    return dirname(realpath(sys.argv[0]))


#####################################
# udev_parser Definition
#####################################
def udev_parser(rules_paths):
    device_paths = {}

    script_path = get_script_path()
    chdir(script_path)

    for current_file in rules_paths:
        lines = open(current_file).readlines()

        for line in lines:
            if line[0] != "#" and line[0] != "\n":
                current_path = line.split("SYMLINK+=")[1].strip("\"\n")
                device_paths["/dev/" + current_path] = time()

    return device_paths


#####################################
# AutoPoweroffWatchdog Class Definition
#####################################
class AutoPoweroffWatchdog(object):
    def __init__(self, devices, shutdown_timeout, do_poweroff=True):
        self.watched_devices = devices
        self.shutdown_timeout = shutdown_timeout
        self.do_poweroff = do_poweroff

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        self.run()

    def run(self):
        while self.run_thread_flag:
            self.check_and_update_devices()
            self.initiate_shutdown_if_needed()
            sleep(0.25)

    def check_and_update_devices(self):
        for device in self.watched_devices:
            if exists(device):
                self.watched_devices[device] = time()

    def initiate_shutdown_if_needed(self):
        for device in self.watched_devices:
            if (time() - self.watched_devices[device]) < self.shutdown_timeout:
                return

        if self.do_poweroff:
            system("sudo wall -n No devices seen for %s seconds. Powering down. Poweroff script exiting." %
                   self.shutdown_timeout)
            system("sudo poweroff")
            exit()
        else:
            system("sudo wall -n No devices seen for %s seconds, but not powering down. Poweroff script exiting.")
            exit()


#####################################
# Main
#####################################
if __name__ == "__main__":
    watched_devices = udev_parser(UDEV_RULES_PATHS)
    AutoPoweroffWatchdog(watched_devices, SHUTDOWN_TIMEOUT, True)
