#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtGui, QtWidgets
import logging
import cv2
import numpy as np
import qimage2ndarray
from time import time

import rospy
import dynamic_reconfigure.client
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

# Custom Imports
from rover_camera.msg import CameraControlMessage

#####################################
# Global Variables
#####################################
CAMERA_TOPIC_PATH = "/cameras/"

QUALITY_MAX = 80
QUALITY_MIN = 15

FRAMERATE_AVERAGING_TIME = 10  # seconds

MIN_FRAMERATE_BEFORE_ADJUST = 23
MAX_FRAMERATE_BEFORE_ADJUST = 28


#####################################
# RoverVideoReceiver Class Definition
#####################################
class RoverVideoReceiver(QtCore.QThread):
    image_ready_signal = QtCore.pyqtSignal(str)

    RESOLUTION_OPTIONS = [(256, 144), (640, 360), (1280, 720)]

    RESOLUTION_MAPPINGS = {
        (1280, 720): None,
        (640, 360): None,
        (256, 144): None
    }

    def __init__(self, camera_name):
        super(RoverVideoReceiver, self).__init__()

        # ########## Reference to class init variables ##########
        self.camera_name = camera_name

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.camera_title_name = self.camera_name.replace("_", " ").title()

        self.topic_base_path = CAMERA_TOPIC_PATH + self.camera_name
        self.control_topic_path = self.topic_base_path + "/camera_control"

        # Subscription variables
        self.video_subscribers = []

        # Publisher variables
        self.camera_control_publisher = rospy.Publisher(self.control_topic_path, CameraControlMessage, queue_size=1)

        # Set up resolution mappings
        self.RESOLUTION_MAPPINGS[(1280, 720)] = CameraControlMessage()
        self.RESOLUTION_MAPPINGS[(640, 360)] = CameraControlMessage()
        self.RESOLUTION_MAPPINGS[(256, 144)] = CameraControlMessage()

        self.RESOLUTION_MAPPINGS[(1280, 720)].enable_large_broadcast = True
        self.RESOLUTION_MAPPINGS[(640, 360)].enable_medium_broadcast = True
        self.RESOLUTION_MAPPINGS[(256, 144)].enable_small_broadcast = True

        # Auto resolution adjustment variables
        self.current_resolution_index = 1
        self.last_resolution_index = self.current_resolution_index
        self.max_resolution_index = len(self.RESOLUTION_OPTIONS)

        self.frame_count = 0
        self.last_framerate_time = time()
        self.resolution_just_adjusted = False

        # Image variables
        self.raw_image = None
        self.opencv_1280x720_image = None
        self.opencv_640x360_image = None

        self.pixmap_1280x720_image = None
        self.pixmap_640x360_image = None

        # Processing variables
        self.bridge = CvBridge()  # OpenCV ROS Video Data Processor
        self.video_enabled = True
        self.new_frame = False

        # Text Drawing Variables
        self.font = cv2.FONT_HERSHEY_TRIPLEX
        self.font_thickness = 1
        self.font_baseline = 0

        self.camera_name_opencv_1280x720_image = None
        self.camera_name_opencv_640x360_image = None

        # Initial class setup to make text images and get camera resolutions
        self.__create_camera_name_opencv_images()

        # Attach subscribers now that everything is set up
        self.video_subscribers.append(rospy.Subscriber(self.topic_base_path + "/image_1280x720/compressed", CompressedImage, self.__image_data_received_callback))
        self.video_subscribers.append(rospy.Subscriber(self.topic_base_path + "/image_640x360/compressed", CompressedImage, self.__image_data_received_callback))
        self.video_subscribers.append(rospy.Subscriber(self.topic_base_path + "/image_256x144/compressed", CompressedImage, self.__image_data_received_callback))

    def run(self):
        self.logger.debug("Starting \"%s\" Camera Thread" % self.camera_title_name)

        self.__enable_camera_resolution(self.RESOLUTION_OPTIONS[self.current_resolution_index])

        while self.run_thread_flag:
            if self.video_enabled:
                self.__show_video_enabled()
            else:
                self.__show_video_disabled()

            self.msleep(10)

        self.logger.debug("Stopping \"%s\" Camera Thread" % self.camera_title_name)

    def __enable_camera_resolution(self, resolution):
        self.camera_control_publisher.publish(self.RESOLUTION_MAPPINGS[resolution])

    def __check_framerate_and_adjust_resolution(self):
        time_diff = time() - self.last_framerate_time
        if time_diff > FRAMERATE_AVERAGING_TIME:
            current_fps = self.frame_count / time_diff

            if current_fps >= MAX_FRAMERATE_BEFORE_ADJUST:
                self.current_resolution_index = min(self.current_resolution_index + 1, self.max_resolution_index)
            elif current_fps <= MIN_FRAMERATE_BEFORE_ADJUST:
                self.current_resolution_index = max(self.current_resolution_index - 1, 0)
            else:
                self.current_resolution_index = min(self.current_resolution_index, self.max_resolution_index)

            if self.last_resolution_index != self.current_resolution_index:
                self.camera_control_publisher.publish(
                    self.RESOLUTION_MAPPINGS[self.RESOLUTION_OPTIONS[self.current_resolution_index]])
                print "Setting %s to %s" % (self.camera_title_name, self.RESOLUTION_OPTIONS[self.current_resolution_index])
                self.last_resolution_index = self.current_resolution_index
                self.resolution_just_adjusted = True

            # print "%s: %s FPS" % (self.camera_title_name, current_fps)
            self.last_framerate_time = time()
            self.frame_count = 0

    def __show_video_enabled(self):
        if self.new_frame:
            self.__check_framerate_and_adjust_resolution()

            try:
                opencv_image = self.bridge.compressed_imgmsg_to_cv2(self.raw_image, "rgb8")

                self.__create_final_pixmaps(opencv_image)

                self.image_ready_signal.emit(self.camera_name)
            except Exception, error:
                print "Failed with error:" + str(error)

            self.new_frame = False

    def __show_video_disabled(self):
        blank_frame = np.zeros((720, 1280, 3), np.uint8)

        self.__create_final_pixmaps(blank_frame)

        self.image_ready_signal.emit(self.camera_name)

    def __create_final_pixmaps(self, opencv_image):
        height, width, _ = opencv_image.shape

        if width != 1280 and height != 720:
            self.opencv_1280x720_image = cv2.resize(opencv_image, (1280, 720))
        else:
            self.opencv_1280x720_image = opencv_image

        if width != 640 and height != 360:
            self.opencv_640x360_image = cv2.resize(opencv_image, (640, 360))
        else:
            self.opencv_640x360_image = opencv_image

        self.__apply_camera_name(self.opencv_1280x720_image, self.camera_name_opencv_1280x720_image)
        self.__apply_camera_name(self.opencv_640x360_image, self.camera_name_opencv_640x360_image)

        self.pixmap_1280x720_image = QtGui.QPixmap.fromImage(qimage2ndarray.array2qimage(
            self.opencv_1280x720_image))
        self.pixmap_640x360_image = QtGui.QPixmap.fromImage(qimage2ndarray.array2qimage(
            self.opencv_640x360_image))

    def __image_data_received_callback(self, raw_image):
        self.raw_image = raw_image
        self.frame_count += 1
        self.new_frame = True

        if self.resolution_just_adjusted:
            self.frame_count = 0
            self.last_framerate_time = time()
            self.resolution_just_adjusted = False

    def __create_camera_name_opencv_images(self):
        camera_name_text_width, camera_name_text_height = \
            cv2.getTextSize(self.camera_title_name, self.font, self.font_thickness, self.font_baseline)[0]

        camera_name_width_buffered = camera_name_text_width + 10
        camera_name_height_buffered = camera_name_text_height + 20

        camera_name_opencv_image = np.zeros(
            (camera_name_height_buffered, camera_name_width_buffered, 3), np.uint8)

        cv2.putText(
            camera_name_opencv_image,
            self.camera_title_name,
            ((camera_name_width_buffered - camera_name_text_width) / 2, int((camera_name_height_buffered * 2) / 3)),
            self.font,
            1,
            (255, 255, 255),
            1,
            cv2.LINE_AA)

        self.camera_name_opencv_1280x720_image = \
            cv2.resize(camera_name_opencv_image, (camera_name_width_buffered, camera_name_height_buffered))

        self.camera_name_opencv_640x360_image = \
            cv2.resize(camera_name_opencv_image, (camera_name_width_buffered / 2, camera_name_height_buffered / 2))

    def set_hard_max_resolution(self, resolution):
        self.max_resolution_index = self.RESOLUTION_OPTIONS.index(resolution)

    def toggle_video_display(self):
        if not self.video_enabled:
            self.camera_control_publisher.publish(self.RESOLUTION_MAPPINGS[self.RESOLUTION_OPTIONS[self.current_resolution_index]])
            self.video_enabled = True
            self.new_frame = True
        else:
            self.camera_control_publisher.publish(CameraControlMessage())
            self.video_enabled = False

    def connect_signals_and_slots(self):
        pass

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False

    @staticmethod
    def __apply_camera_name(opencv_image, font_opencv_image):
        opencv_image[0:0 + font_opencv_image.shape[0], 0:0 + font_opencv_image.shape[1]] = \
            font_opencv_image
