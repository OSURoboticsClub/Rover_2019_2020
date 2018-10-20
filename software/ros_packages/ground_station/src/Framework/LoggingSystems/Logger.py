#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore
from os import makedirs, rename, walk, unlink
from os.path import exists, getmtime
from os import environ
import logging
from datetime import datetime

#####################################
# Global Variables
#####################################
MAX_NUM_LOG_FILES = 30


#####################################
# Logger Definition
#####################################
class Logger(QtCore.QObject):
    def __init__(self, console_output=True):
        super(Logger, self).__init__()

        # ########## Local class variables ##########
        self.console_output = console_output

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Set variables with useful paths ##########
        self.appdata_base_directory = environ["HOME"] + "/.groundstation"
        self.log_directory = self.appdata_base_directory + "//logs"
        self.log_file_path = self.log_directory + "//log.txt"

        # ########## Cleanup old log files ##########
        self.__cleanup_log_files()

        # ########## Set up logger with desired settings ##########
        self.__setup_logger()

        # ########## Place divider in log file to see new program launch ##########
        self.__add_startup_log_buffer_text()

    def __setup_logger(self):
        # Get the appdata directory and make the log path if it doesn't exist
        if not exists(self.log_directory):
            makedirs(self.log_directory)

        # Set the debugging level
        self.logger.setLevel(logging.DEBUG)

        # Make a formatter with the log line format wanted
        formatter = logging.Formatter(fmt='%(levelname)s : %(asctime)s : %(message)s', datefmt='%m/%d/%y %H:%M:%S')

        # Set up a file handler so everything can be saved and attach it to the logger
        file_handler = logging.FileHandler(filename=self.log_file_path)
        file_handler.setFormatter(formatter)
        file_handler.setLevel(logging.DEBUG)
        self.logger.addHandler(file_handler)

        # Enable console output if requested
        if self.console_output:
            console_handler = logging.StreamHandler()
            console_handler.setFormatter(formatter)
            console_handler.setLevel(logging.DEBUG)
            self.logger.addHandler(console_handler)

    def __cleanup_log_files(self):
        # This copies the existing log.txt file to an old version with a datetime stamp
        # It then checks if there are too many log files, and if so, deletes the oldest
        if exists(self.log_directory):
            # Get the number of log files
            num_log_files = self.__get_num_files_in_directory(self.log_directory)

            # Check that we actually have log files
            if num_log_files > 0:
                date_time = datetime.now().strftime("%Y%m%d-%H%M%S")

                # If we do, move the current logfile to a backup in the format old_log_datetime
                if exists(self.log_file_path):
                    rename(self.log_file_path, self.log_directory + "\\old_log_" + date_time + ".txt")

                # If we have more than the max log files delete the oldest one
                if num_log_files >= MAX_NUM_LOG_FILES:
                    unlink(self.__get_name_of_oldest_file(self.log_directory))

    def __add_startup_log_buffer_text(self):
        # Prints a header saying when the program started
        self.logger.info("########## Application Starting ##########")

    @staticmethod
    def __get_name_of_oldest_file(input_path):
        oldest_file_path = None
        previous_oldest_time = 0

        # Walk the directory passed in to get all folders and files
        for dir_path, dir_names, file_names in walk(input_path):
            # Go through all of the filenames found
            for file in file_names:
                # Recreate the full path and get the modified time of the file
                current_path = dir_path + "\\" + file
                time = getmtime(current_path)

                # Default case for if the variables above have not been initially set
                if previous_oldest_time == 0:
                    previous_oldest_time = time
                    oldest_file_path = current_path

                # Saves the oldest time and file path of the current file if it's older (lower timestamp) than the
                # last file saved in the variables
                if time < previous_oldest_time:
                    previous_oldest_time = time
                    oldest_file_path = current_path

        # Returns the path to the oldest file after checking all the files
        return oldest_file_path

    @staticmethod
    def __get_num_files_in_directory(input_path):
        # Walk the directory passed in to get all the files
        for _, _, file_names in walk(input_path):
            # Return the number of files found in the directory
            return len(file_names)