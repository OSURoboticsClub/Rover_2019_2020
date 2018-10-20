# coding=utf-8
#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets, QtGui
import logging
from time import time
import paramiko


#####################################
# Global Variables
#####################################
THREAD_HERTZ = 5

IP = "192.168.1.10"
USER = "nvidia"
PASS = "nvidia"


#####################################
# BashConsole Class Definition
#####################################
class BashConsole(QtCore.QThread):

    text_update_ready__signal = QtCore.pyqtSignal(str)

    def __init__(self, shared_objects):
        super(BashConsole, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]

        self.console_text_edit = self.left_screen.console_line_edit  # type: QtWidgets.QTextEdit
        self.ssh_console_command_line_edit = self.left_screen.ssh_console_command_line_edit  # type:QtWidgets.QLineEdit

        self.ssh_scan_for_hosts_button = self.left_screen.ssh_scan_for_hosts_button  # type: QtWidgets.QPushButton
        self.ssh_host_line_edit = self.left_screen.ssh_host_line_edit  # type: QtWidgets.QLineEdit

        self.ssh_list_wifi_button = self.left_screen.ssh_list_wifi_button  # type: QtWidgets.QPushButton
        self.ssh_equipment_login_button = self.left_screen.ssh_equipment_login_button  # type: QtWidgets.QPushButton
        self.ssh_equipment_logout_button = self.left_screen.ssh_equipment_logout_button  # type: QtWidgets.QPushButton
        self.ssh_equipment_status_button = self.left_screen.ssh_equipment_status_button  # type: QtWidgets.QPushButton
        self.ssh_equipment_start_button = self.left_screen.ssh_equipment_start_button  # type: QtWidgets.QPushButton
        self.ssh_equipment_stop_button = self.left_screen.ssh_equipment_stop_button  # type: QtWidgets.QPushButton

        self.ssh_ssid_line_edit = self.left_screen.ssh_ssid_line_edit  # type:QtWidgets.QLineEdit
        self.ssh_connect_ssid_push_button = self.left_screen.ssh_ssid_push_button  # type: QtWidgets.QPushButton
        self.ssh_disconnect_wifi_button = self.left_screen.ssh_disconnect_wifi_button  # type: QtWidgets.QPushButton

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.bash_process = None
        self.new_widget = None
        self.window = None

        self.wait_time = 1.0 / THREAD_HERTZ

        self.ssh_client = None

        self.set_text_contents = ""

        self.new_command_text = ""
        self.new_command = False

    def run(self):
        while not self.ssh_client:
            try:
                self.ssh_client = paramiko.SSHClient()
                self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                self.ssh_client.connect(IP, username=USER, password=PASS, compress=True)
            except:
                print "No connection"
                if not self.run_thread_flag:
                    return
                self.ssh_client = None
                self.msleep(1000)

        while self.run_thread_flag:
            start_time = time()

            if self.new_command:
                _, ssh_stdout, ssh_stderr = self.ssh_client.exec_command(self.new_command_text)

                stdout_read = ssh_stdout.read()
                stderr_read = ssh_stderr.read()

                output = ""
                output += "\n%s@%s:$" % (USER, IP)
                output += self.new_command_text + "\n"
                output += stdout_read.decode("utf-8") if stdout_read else ""
                output += stderr_read.decode("utf-8") if stderr_read else ""

                self.set_text_contents += output
                self.text_update_ready__signal.emit(self.set_text_contents)
                self.new_command = False

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

        del self.bash_process

    def on_text_readout_updated__slot(self):
        self.console_text_edit.moveCursor(QtGui.QTextCursor.End)

    def on_text_editing_finished__slot(self):
        self.new_command_text = self.ssh_console_command_line_edit.text()
        self.new_command = True

    def on_list_wifi_button_pressed__slot(self):
        self.new_command_text = "nmcli dev wifi list"
        self.new_command = True

    def on_login_button_pressed__slot(self):
        current_ip = self.ssh_host_line_edit.text()
        self.new_command_text = "python equipment_servicing_interface.py '%s' 'LOGIN MTECH GITRDONE' HELP" % current_ip
        print self.new_command_text
        self.new_command = True

    def on_logout_button_pressed__slot(self):
        current_ip = self.ssh_host_line_edit.text()
        self.new_command_text = "python equipment_servicing_interface.py '%s' LOGOUT" % current_ip
        self.new_command = True

    def on_status_button_pressed__slot(self):
        current_ip = self.ssh_host_line_edit.text()
        self.new_command_text = "python equipment_servicing_interface.py '%s' STATUS" % current_ip
        self.new_command = True

    def on_start_button_pressed__slot(self):
        current_ip = self.ssh_host_line_edit.text()
        self.new_command_text = "python equipment_servicing_interface.py '%s' START" % current_ip
        self.new_command = True

    def on_stop_button_pressed__slot(self):
        current_ip = self.ssh_host_line_edit.text()
        self.new_command_text = "python equipment_servicing_interface.py '%s' STOP" % current_ip
        self.new_command = True

    def on_ssh_scan_for_hosts_pressed__slot(self):
        current_ip = self.ssh_host_line_edit.text()

        find_dot = current_ip.rfind(".")

        if find_dot > 0:
            current_ip = current_ip[:find_dot + 1] + "0"
            self.new_command_text = "nmap -sP %s/24 -oG - | awk '/Up$/{print $2}'" % current_ip
            self.new_command = True
        else:
            self.set_text_contents += "IP address for range search not valid. Try again."
            self.text_update_ready__signal.emit(self.set_text_contents)

    def on_connect_ssid_button_pressed__slot(self):
        ssid_text = self.ssh_ssid_line_edit.text()

        self.new_command_text = "sudo nmcli dev wifi connect %s" % ssid_text

        self.new_command = True

    def on_disconnect_ssid_button_pressed__slot(self):
        ssid_text = self.ssh_ssid_line_edit.text()

        self.new_command_text = "sudo nmcli con down id %s ; sudo nmcli connection delete %s" % (ssid_text, ssid_text)

        self.new_command = True

    def connect_signals_and_slots(self):
        self.text_update_ready__signal.connect(self.console_text_edit.setText)
        self.ssh_console_command_line_edit.editingFinished.connect(self.on_text_editing_finished__slot)
        self.console_text_edit.textChanged.connect(self.on_text_readout_updated__slot)

        self.ssh_scan_for_hosts_button.clicked.connect(self.on_ssh_scan_for_hosts_pressed__slot)

        self.ssh_equipment_login_button.clicked.connect(self.on_login_button_pressed__slot)
        self.ssh_equipment_logout_button.clicked.connect(self.on_logout_button_pressed__slot)
        self.ssh_equipment_status_button.clicked.connect(self.on_status_button_pressed__slot)
        self.ssh_equipment_start_button.clicked.connect(self.on_start_button_pressed__slot)
        self.ssh_equipment_stop_button.clicked.connect(self.on_stop_button_pressed__slot)

        self.ssh_list_wifi_button.clicked.connect(self.on_list_wifi_button_pressed__slot)
        self.ssh_connect_ssid_push_button.clicked.connect(self.on_connect_ssid_button_pressed__slot)
        self.ssh_disconnect_wifi_button.clicked.connect(self.on_disconnect_ssid_button_pressed__slot)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
