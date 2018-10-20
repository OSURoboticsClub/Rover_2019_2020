from PyQt5 import QtCore, QtWidgets, QtGui
import logging
import rospy

from sensor_msgs.msg import NavSatFix

GPS_POSITION_TOPIC = "/rover_odometry/fix"


class WaypointsCoordinator(QtCore.QThread):
    new_manual_waypoint_entry = QtCore.pyqtSignal(str, float, float, int)
    update_waypoint_entry = QtCore.pyqtSignal(str, str, str, int)

    def __init__(self, shared_objects):
        super(WaypointsCoordinator, self).__init__()
        self.run_thread_flag = True
        self.navigation_table_cur_click = None
        self.landmark_table_cur_click = None

        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]

        self.navigation_label = (self.left_screen.
                                 navigation_waypoints_table_widget)
        self.landmark_label = self.left_screen.landmark_waypoints_table_widget

        self.name_edit_label = self.left_screen.manual_waypoint_landmark_name_line_edit
        self.latitude_label = self.left_screen.manual_waypoint_decimal_lattitude_spin_box
        self.longitude_label = self.left_screen.manual_waypoint_decimal_longitude_spin_box

        self.latitude_degree_label = self.left_screen.manual_waypoint_degrees_lattitude_spin_box

        self.longitude_degree_label = self.left_screen.manual_waypoint_degrees_longitude_spin_box

        self.latitude_minute_label = self.left_screen.manual_waypoint_minutes_lattitude_spin_box

        self.longitude_minute_label = self.left_screen.manual_waypoint_minutes_longitude_spin_box

        self.latitude_second_label = self.left_screen.manual_waypoint_seconds_lattitude_spin_box

        self.longitude_second_label = self.left_screen.manual_waypoint_seconds_longitude_spin_box

        self.latitude_card_label = self.left_screen.manual_waypoint_cardinal_lattitude_combo_box

        self.longitude_card_label = self.left_screen.manual_waypoint_cardinal_longitude_combo_box

        # Color Labels and Buttons
        self.nav_color_label = self.left_screen.manual_waypoint_navigation_color_label
        self.nav_color_set_button = self.left_screen.manual_waypoint_navigation_color_set_button

        self.landmark_color_label = self.left_screen.manual_waypoint_landmark_color_label
        self.landmark_color_set_button = self.left_screen.manual_waypoint_landmark_color_set_button

        # Nav Table Buttons
        self.nav_set_button_label = (self.left_screen.
                                     navigation_waypoints_set_button)
        self.nav_add_manual_button_label = (
            self.left_screen.navigation_waypoints_add_manual_button)
        self.nav_add_gps_button_label = (self.left_screen.
                                         navigation_waypoints_add_gps_button)
        self.nav_delete_button_label = (self.left_screen.
                                        navigation_waypoints_delete_button)

        # Land Table Buttons
        self.land_set_button_label = (self.left_screen.
                                      landmark_waypoints_set_button)

        self.land_add_manual_button_label = (
            self.left_screen.landmark_waypoints_add_manual_button)

        self.land_add_gps_button_label = (self.left_screen.
                                          landmark_waypoints_add_gps_button)

        self.land_delete_button_label = (self.left_screen.
                                         landmark_waypoints_delete_button)

        self.settings = QtCore.QSettings()

        self.logger = logging.getLogger("groundstation")

        self.gps_position_subscriber = rospy.Subscriber(GPS_POSITION_TOPIC, NavSatFix,
                                                        self.gps_position_updated_callback)

        self.longitude = None
        self.latitude = None

        self.nav_color = QtCore.Qt.yellow
        self.landmark_color = QtCore.Qt.blue

        self.nav_color_dialog = QtWidgets.QColorDialog()
        self.nav_color_dialog.setWindowFlags(
            QtCore.Qt.WindowStaysOnTopHint | QtCore.Qt.X11BypassWindowManagerHint | QtCore.Qt.WindowTitleHint | QtCore.Qt.WindowCloseButtonHint)

        self.landmark_color_dialog = QtWidgets.QColorDialog()
        self.landmark_color_dialog.setWindowFlags(
            QtCore.Qt.WindowStaysOnTopHint | QtCore.Qt.X11BypassWindowManagerHint | QtCore.Qt.WindowTitleHint | QtCore.Qt.WindowCloseButtonHint)

    def run(self):
        while self.run_thread_flag:
            self.msleep(3)

    def connect_signals_and_slots(self):
        self.new_manual_waypoint_entry.connect(self.update_manual_entry)

        # setting up signals for Navigation Table
        self.nav_add_gps_button_label.clicked.connect(self._nav_add_gps)
        self.nav_delete_button_label.clicked.connect(self._nav_del)
        self.nav_add_manual_button_label.clicked.connect(self._nav_add_manual)
        self.nav_set_button_label.clicked.connect(self._nav_save)

        # setting up signals for Landmark Table
        self.land_add_gps_button_label.clicked.connect(self._land_add_gps)
        self.land_delete_button_label.clicked.connect(self._land_del)
        self.land_add_manual_button_label.clicked.connect(self.
                                                          _land_add_manual)
        self.land_set_button_label.clicked.connect(self._land_save)

        self.navigation_label.cellClicked.connect(self._on_nav_clicked)
        self.landmark_label.cellClicked.connect(self._on_land_clicked)

        self.nav_color_set_button.clicked.connect(self.nav_color_dialog.show)
        self.landmark_color_set_button.clicked.connect(self.landmark_color_dialog.show)

        self.nav_color_dialog.currentColorChanged.connect(self.__on_new_nav_color_selected)
        self.landmark_color_dialog.currentColorChanged.connect(self.__on_new_landmark_color_selected)

    def _add_to_table(self, name, lat, lng, color, table):
        color_table_item = QtWidgets.QTableWidgetItem()
        color_table_item.setBackground(color)

        count = table.rowCount()
        table.insertRow(count)
        table.setItem(count, 0, QtWidgets.QTableWidgetItem(name))
        table.setItem(count, 1, QtWidgets.QTableWidgetItem(lat))
        table.setItem(count, 2, QtWidgets.QTableWidgetItem(lng))
        table.setItem(count, 3, color_table_item)

    def _clear_inputs(self):
        self.name_edit_label.clear()
        self.latitude_label.clear()
        self.longitude_label.clear()

    def _is_empty_inputs(self):
        if not self.name_edit_label.text():
            return True
        if not self.latitude_label.text():
            return True
        if not self.longitude_label.text():
            return True
        return False

    def _nav_add_gps(self):
        if self.longitude and self.latitude:
            name = self.navigation_label.rowCount()
            self._add_to_table(str(name + 1), str(self.latitude),
                               str(self.longitude), self.nav_color,
                               self.navigation_label)
            self._clear_inputs()

    def _nav_save(self):
        if not self._is_empty_inputs():
            lat = self.latitude_label.text()
            lng = self.longitude_label.text()
            self.navigation_label.setItem(
                self.navigation_table_cur_click,
                1,
                QtWidgets.QTableWidgetItem(lat))
            self.navigation_label.setItem(
                self.navigation_table_cur_click,
                2,
                QtWidgets.QTableWidgetItem(lng))

            color_table_item = QtWidgets.QTableWidgetItem()
            color_table_item.setBackground(self.nav_color)

            self.navigation_label.setItem(
                self.navigation_table_cur_click,
                3,
                color_table_item)
            self._clear_inputs()

    def _nav_add_manual(self):
        # request GPS data
        if not self._is_empty_inputs():
            name = self.navigation_label.rowCount()
            lat = self.latitude_label.text()
            lng = self.longitude_label.text()
            self._add_to_table(str(name + 1), lat,
                               lng, self.nav_color,
                               self.navigation_label)
            self._clear_inputs()

    def _nav_del(self):
        if self.navigation_table_cur_click is not None:
            self.navigation_label.removeRow(self.navigation_table_cur_click)
            count = self.navigation_label.rowCount()
            for x in range(self.navigation_table_cur_click, count):
                self.navigation_label.setItem(x,
                                              0,
                                              QtWidgets.
                                              QTableWidgetItem(str(x + 1)))
            self._clear_inputs()

    def _land_add_gps(self):
        if self.longitude and self.latitude:
            name = self.name_edit_label.text()
            self._add_to_table(name, str(self.latitude), str(self.longitude), self.landmark_color, self.landmark_label)
            self._clear_inputs()

    def _land_add_manual(self):
        if not self._is_empty_inputs():
            name = self.name_edit_label.text()
            lat = self.latitude_label.text()
            lng = self.longitude_label.text()
            self._add_to_table(name, lat,
                               lng, self.landmark_color,
                               self.landmark_label)
            self._clear_inputs()

    def _land_del(self):
        if self.landmark_table_cur_click is not None:
            self.landmark_label.removeRow(self.landmark_table_cur_click)
            count = self.landmark_label.rowCount()
            for x in range(self.landmark_table_cur_click, count):
                self.navigation_label.setItem(x,
                                              0,
                                              QtWidgets.
                                              QTableWidgetItem(str(x + 1)))
            self._clear_inputs()

    def _land_save(self):
        if not self._is_empty_inputs():
            name = self.name_edit_label.text()
            lat = self.latitude_label.text()
            lng = self.longitude_label.text()
            self.landmark_label.setItem(self.landmark_table_cur_click, 0,
                                        QtWidgets.QTableWidgetItem(name))

            self.landmark_label.setItem(self.landmark_table_cur_click, 1,
                                        QtWidgets.QTableWidgetItem(lat))

            self.landmark_label.setItem(self.landmark_table_cur_click, 2,
                                        QtWidgets.QTableWidgetItem(lng))

            color_table_item = QtWidgets.QTableWidgetItem()
            color_table_item.setBackground(self.landmark_color)

            self.landmark_label.setItem(self.landmark_table_cur_click, 3,
                                        color_table_item)

            self._clear_inputs()

    def setup_signals(self, start_signal,
                      signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested_slot)

    def on_kill_threads_requested_slot(self):
        self.run_thread_flag = False

    def update_manual_entry(self, name, lat, lng, table):
        self.name_edit_label.setEnabled(bool(table))
        self.name_edit_label.setText(name)
        self.latitude_label.setValue(lat)
        self.longitude_label.setValue(lng)

        lat_d = float(abs(int(lat)))
        lat_m = float(int((abs(lat) - lat_d) * 60))
        lat_s = ((abs(lat) - lat_d - (lat_m / 60.0)) * 3600.)
        if lat > 0.:
            self.latitude_card_label.setCurrentText("N")
        else:
            self.latitude_card_label.setCurrentText("S")
        self.latitude_degree_label.setValue(lat_d)
        self.latitude_minute_label.setValue(lat_m)
        self.latitude_second_label.setValue(lat_s)

        lng_d = float(abs(int(lng)))
        lng_m = float(int((abs(lng) - lng_d) * 60))
        lng_s = ((abs(lng) - lng_d - (lng_m / 60.0)) * 3600.)
        if lng > 0.:
            self.longitude_card_label.setCurrentText("E")
        else:
            self.longitude_card_label.setCurrentText("W")
        self.longitude_degree_label.setValue(lng_d)
        self.longitude_minute_label.setValue(lng_m)
        self.longitude_second_label.setValue(lng_s)

    def _on_nav_clicked(self, row, col):
        self.navigation_table_cur_click = row
        self.landmark_table_cur_click = None
        self.new_manual_waypoint_entry.emit(
            self.navigation_label.item(row, 0).text(),
            float(self.navigation_label.item(row, 1).text()),
            float(self.navigation_label.item(row, 2).text()),
            0
        )

    def _on_land_clicked(self, row, col):
        self.landmark_table_cur_click = row
        self.navigation_table_cur_click = None
        self.new_manual_waypoint_entry.emit(
            self.landmark_label.item(row, 0).text(),
            float(self.landmark_label.item(row, 1).text()),
            float(self.landmark_label.item(row, 2).text()),
            1
        )

    def __on_new_nav_color_selected(self, color):
        self.nav_color_label.setStyleSheet(
            "background-color: rgb(%s, %s, %s)" % (color.red(), color.green(), color.blue()))

        self.nav_color = color

    def __on_new_landmark_color_selected(self, color):
        self.landmark_color_label.setStyleSheet(
            "background-color: rgb(%s, %s, %s)" % (color.red(), color.green(), color.blue()))

        self.landmark_color = color

    def gps_position_updated_callback(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude
