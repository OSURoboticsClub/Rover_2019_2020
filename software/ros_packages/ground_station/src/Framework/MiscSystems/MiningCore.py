# coding=utf-8
#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging
import rospy

from rover_control.msg import MiningStatusMessage, MiningControlMessage, CameraControlMessage, DrillControlMessage
from std_msgs.msg import Float64

#####################################
# Global Variables
#####################################
MINING_STATUS_TOPIC = "/rover_control/mining/status"
MINING_CONTROL_TOPIC = "/rover_control/mining/control"
DRILL_CONTROL_TOPIC = "/rover_control/mining/drill/control"

CAMERA_CONTROL_TOPIC = "/rover_control/camera/control"

MINING_COLLECTION_CUP_OPEN = 0
MINING_COLLECTION_CUP_CLOSED = 130

# servo 1
PROBE_DROP_POSITION = 43
SCOOP_DROP_POSITION = 132

# servo 2
CONTAINER_OPEN = 135
CONTAINER_CLOSED = 105

DRILL_SPEED = 150

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
class Mining(QtCore.QObject):
    mining_4bar_temp_update_ready__signal = QtCore.pyqtSignal(float)
    mining_4bar_current_update_ready__signal = QtCore.pyqtSignal(float)
    mining_linear_temp_update_ready__signal = QtCore.pyqtSignal(float)
    mining_linear_current_update_ready__signal = QtCore.pyqtSignal(float)

    temp_update_ready__signal = QtCore.pyqtSignal(float)
    moisture_update_ready__signal = QtCore.pyqtSignal(float)
    loss_tangent_update_ready__signal = QtCore.pyqtSignal(float)
    electrical_conductivity_update_ready__signal = QtCore.pyqtSignal(float)
    real_dielectric_update_ready__signal = QtCore.pyqtSignal(float)
    imaginary_dielectric_update_ready__signal = QtCore.pyqtSignal(float)

    def __init__(self, shared_objects):
        super(Mining, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"][left]

        self.mining_4bar_temp_lcd_number = self.left_screen.mining_4bar_temp_lcd_number  # type:QtWidgets.QLCDNumber
        self.mining_4bar_current_lcd_number = self.left_screen.mining_4bar_current_lcd_number  # type:QtWidgets.QLCDNumber
        self.mining_linear_temp_lcd_number = self.left_screen.mining_linear_temp_lcd_number  # type:QtWidgets.QLCDNumber
        self.mining_linear_current_lcd_number = self.left_screen.mining_linear_current_lcd_number  # type:QtWidgets.QLCDNumber
        self.mining_open_button = self.left_screen.mining_open_button  # type:QtWidgets.QPushButton
        self.mining_close_button = self.left_screen.mining_close_button  # type:QtWidgets.QPushButton
        self.mining_home_linear_button = self.left_screen.mining_home_linear_button  # type:QtWidgets.QPushButton
        self.mining_toggle_overtravel_button = self.left_screen.mining_toggle_overtravel_button  # type:QtWidgets.QPushButton
        self.drill_turn_clockwise_button = self.left_screen.drill_turn_clockwise_button  # type:QtWidgets.QPushButton
        self.drill_turn_counter_clockwise_button = self.left_screen.drill_turn_counter_clockwise_button  # type:QtWidgets.QPushButton
        self.drill_stop_button = self.left_screen.drill_stop_button  # type:QtWidgets.QPushButton
        self.science_probe_down_button = self.left_screen.science_probe_down_button  # type:QtWidgets.QPushButton
        self.science_scoop_down_button = self.left_screen.science_scoop_down_button  # type:QtWidgets.QPushButton
        self.science_container_open_button = self.left_screen.science_container_open_button  # type:QtWidgets.QPushButton
        self.science_container_close_button = self.left_screen.science_container_close_button  # type:QtWidgets.QPushButton
        self.science_probe_button = self.left_screen.science_probe_button  # type:QtWidgets.QPushButton

        # self.fourbar_position_slider = self.left_screen.fourbar_position_slider  # type:QtWidgets.QProgressBar
        # self.linear_position_slider = self.left_screen.linear_position_slider  # type:QtWidgets.QProgressBar

        self.science_temp_lcd_number = self.left_screen.science_temp_lcd_number  # type:QtWidgets.QLCDNumber
        self.science_moisture_lcd_number = self.left_screen.science_moisture_lcd_number  # type:QtWidgets.QLCDNumber
        self.science_loss_tangent_lcd_number = self.left_screen.science_loss_tangent_lcd_number  # type:QtWidgets.QLCDNumber
        self.science_electrical_conductivity_lcd_number = self.left_screen.science_electrical_conductivity_lcd_number  # type:QtWidgets.QLCDNumber
        self.science_real_dielectric_lcd_number = self.left_screen.science_real_dielectric_lcd_number  # type:QtWidgets.QLCDNumber
        self.science_imaginary_dielectric_lcd_number = self.left_screen.science_imaginary_dielectric_lcd_number  # type:QtWidgets.QLCDNumber

        self.cam_lcd_output_button = self.left_screen.cam_lcd_output_button  # type:QtWidgets.QPushButton
        self.cam_network_output_button = self.left_screen.cam_network_output_button  # type:QtWidgets.QPushButton

        self.cam_zoom_in_button = self.left_screen.cam_zoom_in_button  # type:QtWidgets.QPushButton
        self.cam_zoom_out_button = self.left_screen.cam_zoom_out_button  # type:QtWidgets.QPushButton
        self.cam_full_zoom_in_button = self.left_screen.cam_full_zoom_in_button  # type:QtWidgets.QPushButton
        self.cam_full_zoom_out_button = self.left_screen.cam_full_zoom_out_button  # type:QtWidgets.QPushButton

        self.cam_shoot_button = self.left_screen.cam_shoot_button  # type:QtWidgets.QPushButton

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.mining_status_subscriber = rospy.Subscriber(MINING_STATUS_TOPIC, MiningStatusMessage, self.mining_status_message_received__callback)

        self.mining_control_publisher = rospy.Publisher(MINING_CONTROL_TOPIC, MiningControlMessage, queue_size=1)
        self.drill_control_publisher = rospy.Publisher(DRILL_CONTROL_TOPIC, DrillControlMessage, queue_size=1)
        self.camera_control_publisher = rospy.Publisher(CAMERA_CONTROL_TOPIC, CameraControlMessage, queue_size=1)

        self.connect_signals_and_slots()

    def connect_signals_and_slots(self):
        self.mining_4bar_temp_update_ready__signal.connect(self.mining_4bar_temp_lcd_number.display)
        self.mining_4bar_current_update_ready__signal.connect(self.mining_4bar_current_lcd_number.display)
        self.mining_linear_temp_update_ready__signal.connect(self.mining_linear_temp_lcd_number.display)
        self.mining_linear_current_update_ready__signal.connect(self.mining_linear_current_lcd_number.display)

        self.mining_open_button.clicked.connect(self.on_mining_open_clicked__slot)
        self.mining_close_button.clicked.connect(self.on_mining_close_clicked__slot)
        self.mining_home_linear_button.clicked.connect(self.on_mining_home_linear_clicked__slot)
        self.mining_toggle_overtravel_button.clicked.connect(self.on_mining_toggle_overtravel_clicked__slot)
        self.drill_turn_clockwise_button.clicked.connect(self.on_drill_clockwise_clocked__slot)
        self.drill_turn_counter_clockwise_button.clicked.connect(self.on_drill_counter_clockwise_clicked__slot)
        self.drill_stop_button.clicked.connect(self.on_drill_stop_clicked__slot)
        self.science_probe_down_button.clicked.connect(self.on_science_probe_down_clicked__slot)
        self.science_scoop_down_button.clicked.connect(self.on_science_scoop_down_clicked__slot)
        self.science_container_open_button.clicked.connect(self.on_science_container_open_clicked__slot)
        self.science_container_close_button.clicked.connect(self.on_science_container_close_clicked__slot)
        self.science_probe_button.clicked.connect(self.on_science_probe_clicked__slot)

        #self.fourbar_position_slider.valueChanged.connect(self.fourbar_position_slider__slot)
        #self.linear_position_slider.valueChanged.connect(self.linear_position_slider__slot)

        self.temp_update_ready__signal.connect(self.science_temp_lcd_number.display)
        self.moisture_update_ready__signal.connect(self.science_moisture_lcd_number.display)
        self.loss_tangent_update_ready__signal.connect(self.science_loss_tangent_lcd_number.display)
        self.electrical_conductivity_update_ready__signal.connect(self.science_electrical_conductivity_lcd_number.display)
        self.real_dielectric_update_ready__signal.connect(self.science_real_dielectric_lcd_number.display)
        self.imaginary_dielectric_update_ready__signal.connect(self.science_imaginary_dielectric_lcd_number.display)

        self.cam_lcd_output_button.clicked.connect(self.on_cam_lcd_button_clicked__slot)
        self.cam_network_output_button.clicked.connect(self.on_cam_network_button_clicked__slot)

        self.cam_zoom_in_button.clicked.connect(self.on_cam_zoom_in_button_clicked__slot)
        self.cam_zoom_out_button.clicked.connect(self.on_cam_zoom_out_button_clicked__slot)
        self.cam_full_zoom_in_button.clicked.connect(self.on_cam_full_zoom_in_button_clicked__slot)
        self.cam_full_zoom_out_button.clicked.connect(self.on_cam_full_zoom_out_button_clicked__slot)
        self.cam_shoot_button.clicked.connect(self.on_cam_shoot_button_clicked__slot)

    def fourbar_position_slider__slot(self):
        message = MiningControlMessage()
        message.linear_p = self.fourbar_position_slider.value()
        self.mining_control_publisher.publish(message)

    def linear_position_slider__slot(self):
        #message = MiningControlMessage()
        #message.
        pass

    def on_mining_open_clicked__slot(self):
        message = MiningControlMessage()
        message.servo1_target = MINING_COLLECTION_CUP_OPEN
        self.mining_control_publisher.publish(message)

    def on_mining_close_clicked__slot(self):
        message = MiningControlMessage()
        message.servo1_target = MINING_COLLECTION_CUP_CLOSED
        self.mining_control_publisher.publish(message)

    def on_mining_home_linear_clicked__slot(self):
        message = MiningControlMessage()
        message.motor_go_home = True
        self.mining_control_publisher.publish(message)

    def on_mining_toggle_overtravel_clicked__slot(self):
        message = MiningControlMessage()
        message.overtravel = True
        self.mining_control_publisher.publish(message)

    def on_drill_clockwise_clocked__slot(self):
        message = DrillControlMessage()
        message.direction = True
        message.speed = DRILL_SPEED
        self.drill_control_publisher.publish(message)

    def on_drill_counter_clockwise_clicked__slot(self):
        message = DrillControlMessage()
        message.direction = False
        message.speed = DRILL_SPEED
        self.drill_control_publisher.publish(message)

    def on_drill_stop_clicked__slot(self):
        message = DrillControlMessage()
        message.speed = 0
        self.drill_control_publisher.publish(message)

    def on_science_probe_down_clicked__slot(self):
        message = MiningControlMessage()
        message.servo1_target = PROBE_DROP_POSITION
        self.mining_control_publisher.publish(message)

    def on_science_scoop_down_clicked__slot(self):
        message = MiningControlMessage()
        message.servo1_target = SCOOP_DROP_POSITION
        self.mining_control_publisher.publish(message)

    def on_science_container_open_clicked__slot(self):
        message = MiningControlMessage()
        message.servo2_target = CONTAINER_OPEN
        message.servo1_target = SCOOP_DROP_POSITION
        self.mining_control_publisher.publish(message)

    def on_science_container_close_clicked__slot(self):
        message = MiningControlMessage()
        message.servo2_target = CONTAINER_CLOSED
        message.servo1_target = SCOOP_DROP_POSITION
        self.mining_control_publisher.publish(message)

    def on_science_probe_clicked__slot(self):
        message = MiningControlMessage()
        message.probe_take_reading = True
        self.mining_control_publisher.publish(message)

    def on_cam_lcd_button_clicked__slot(self):
        message = CameraControlMessage()
        message.cam_change_view = 1
        self.camera_control_publisher.publish(message)

    def on_cam_network_button_clicked__slot(self):
        message = CameraControlMessage()
        message.cam_change_view = 2
        self.camera_control_publisher.publish(message)

    def on_cam_zoom_in_button_clicked__slot(self):
        message = CameraControlMessage()
        message.cam_zoom_in = 1
        self.camera_control_publisher.publish(message)

    def on_cam_zoom_out_button_clicked__slot(self):
        message = CameraControlMessage()
        message.cam_zoom_out = 1
        self.camera_control_publisher.publish(message)

    def on_cam_full_zoom_in_button_clicked__slot(self):
        message = CameraControlMessage()
        message.cam_zoom_in_full = 1
        self.camera_control_publisher.publish(message)

    def on_cam_full_zoom_out_button_clicked__slot(self):
        message = CameraControlMessage()
        message.cam_zoom_out_full= 1
        self.camera_control_publisher.publish(message)

    def on_cam_shoot_button_clicked__slot(self):
        message = CameraControlMessage()
        message.cam_shoot = 1
        self.camera_control_publisher.publish(message)

    def mining_status_message_received__callback(self, status):
        status = status  # type:MiningStatusMessage
        
        self.mining_4bar_temp_update_ready__signal.emit(status.temp2)
        self.mining_linear_temp_update_ready__signal.emit(status.temp1)

        self.mining_4bar_current_update_ready__signal.emit(status.linear_current)
        self.mining_linear_current_update_ready__signal.emit(status.motor_current)

        self.temp_update_ready__signal.emit(status.probe_temp_c)
        self.moisture_update_ready__signal.emit(status.probe_moisture)
        self.loss_tangent_update_ready__signal.emit(status.probe_loss_tangent)
        self.electrical_conductivity_update_ready__signal.emit(status.probe_soil_elec_cond)
        self.real_dielectric_update_ready__signal.emit(status.probe_real_dielec_perm)
        self.imaginary_dielectric_update_ready__signal.emit(status.probe_imag_dielec_perm)

