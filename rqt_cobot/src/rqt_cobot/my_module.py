import os
import rospy
import rospkg
import html

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QTableWidgetItem, QTableWidget, QTextEdit
# import python_qt_binding.QtCore as QtCore
from python_qt_binding.QtCore import QObject, Signal, Slot, QTimer
from std_msgs.msg import String, Float64, Float64MultiArray
from geometry_msgs.msg import Vector3
from python_qt_binding.QtGui import QColor, QTextCursor
# from PyQt5.QtCore import Qt

class Signaller(QObject):
    mode_changed = Signal(str)
    time_changed = Signal(str)
    wrench_table_signal = Signal(int, int, float)
    torque_table_signal = Signal(int, int, float)
    error_signal = Signal(str)

class CobotPlugin(Plugin):

    def __init__(self, context):
        super(CobotPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('CobotPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        self.wrench_thresh = [0.0 for _ in range(6)]
        self.wrench_warn = [0.0 for _ in range(6)]

        self.torque_thresh = [0.0 for _ in range(7)]
        self.torque_warn = [0.0 for _ in range(7)]

        self.joint_collision = [0 for _ in range(7)]
        self.cart_collision = [0 for _ in range(6)]
        self.joint_flag = 0
        self.cart_flag = 0
        self.ignore_flag = 0
        self.tau = "\u03C4"
        self.joint_titles = [
            f'{self.tau}_1',
            f'{self.tau}_2',
            f'{self.tau}_3',
            f'{self.tau}_4',
            f'{self.tau}_5',
            f'{self.tau}_6',
            f'{self.tau}_7',
        ]
        self.cart_titles = [
            "F_x",
            "F_y",
            "F_z",
            f'{self.tau}_x',
            f'{self.tau}_y',
            f'{self.tau}_z',
        ]
        self.error_str = "Error Log:\n"

        self.latest_hybrid_mode = None
        self.latest_time_now = None
        self.latest_f_sense = None
        self.latest_tau_sense = None
        self.latest_wrench_thresh = None
        self.latest_tau_d = None
        self.latest_tau_thresh = None

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_gui)
        self.timer.start(100)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_cobot'), 'resource', 'CobotPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('CobotPluginUi')

        # Setup signaller for threading
        self.signaller = Signaller()
        self.signaller.mode_changed.connect(self.on_mode_changed)
        self.signaller.time_changed.connect(self.on_time_changed)
        self.signaller.wrench_table_signal.connect(self.on_wrench_changed)
        self.signaller.torque_table_signal.connect(self.on_torque_changed)
        self.signaller.error_signal.connect(self.on_error_changed)

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Subscribe to the ROS topics
        self.mode_sub = rospy.Subscriber('/hybrid_mode_string', String, self.hybrid_mode_callback, queue_size=10)
        self.time_sub = rospy.Subscriber('/time_now', Float64, self.time_now_callback, queue_size=10)
        self.f_sense_sub = rospy.Subscriber('/f_ext', Vector3, self.f_sense_callback, queue_size=10)
        self.tau_sense_sub = rospy.Subscriber('/tau_ext', Vector3, self.tau_sense_callback, queue_size=10)
        self.wrench_thresh_sub = rospy.Subscriber('/force_upper', Float64MultiArray, self.wrench_thresh_callback, queue_size=10)
        self.wrench_warn_sub = rospy.Subscriber('/force_lower', Float64MultiArray, self.wrench_warn_callback, queue_size=10)
        self.tau_d_sub = rospy.Subscriber('/tau_d', Float64MultiArray, self.tau_d_callback, queue_size=10)
        self.tau_thresh_sub = rospy.Subscriber('/torque_upper', Float64MultiArray, self.tau_thresh_callback, queue_size=10)
        self.tau_warn_sub = rospy.Subscriber('/torque_lower', Float64MultiArray, self.tau_warn_callback, queue_size=10)
        self.joint_collision_sub = rospy.Subscriber('/join_collision', Float64MultiArray, self.joint_collision_callback, queue_size=10)
        self.cart_collision_sub = rospy.Subscriber('/cart_collision', Float64MultiArray, self.cart_collision_callback, queue_size=10)

        wrench_table = self._widget.findChild(QTableWidget, 'wrenchTable')

        wrench_table.setHorizontalHeaderLabels([
            "F_x |N|",
            "F_y |N|",
            "F_z |N|",
            f'{self.tau}_x |Nm|',
            f'{self.tau}_y |Nm|',
            f'{self.tau}_z |Nm|',
        ])
        wrench_table.setVerticalHeaderLabels([
            "Threshold",
            "Sensor Value",
        ])

        torque_table = self._widget.findChild(QTableWidget, 'torqueTable')

        torque_table.setHorizontalHeaderLabels([
            f'{self.tau}_1 |Nm|',
            f'{self.tau}_2 |Nm|',
            f'{self.tau}_3 |Nm|',
            f'{self.tau}_4 |Nm|',
            f'{self.tau}_5 |Nm|',
            f'{self.tau}_6 |Nm|',
            f'{self.tau}_7 |Nm|',
        ])
        torque_table.setVerticalHeaderLabels([
            "Threshold",
            "Command",
        ])

        # self._widget.errorReport.append("Error Log:")


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        if self.mode_sub is not None:
            self.mode_sub.unregister()
        if self.time_sub is not None:
            self.time_sub.unregister()
        if self.f_sense_sub is not None:
            self.f_sense_sub.unregister()
        if self.tau_sense_sub is not None:
            self.tau_sense_sub.unregister()
        if self.wrench_thresh_sub is not None:
            self.wrench_thresh_sub.unregister()
        if self.wrench_warn_sub is not None:
            self.wrench_warn_sub.unregister()
        if self.tau_d_sub is not None:
            self.tau_d_sub.unregister()
        if self.tau_thresh_sub is not None:
            self.tau_thresh_sub.unregister()
        if self.tau_warn_sub is not None:
            self.tau_warn_sub.unregister()
        if self.joint_collision_sub is not None:
            self.joint_collision_sub.unregister()
        if self.cart_collision_sub is not None:
            self.cart_collision_sub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # Callback for hybrid mode
    def hybrid_mode_callback(self, msg):
        self.latest_hybrid_mode = msg
        # now = time.time()
        # if now - self.hybrid_mode_last_update > 1.0 / self.update_rate:
        #     self.hybrid_mode_last_update = now
        #     # This runs in ROS subscriber thread
        #     data_str = "Selected Hybrid Mode: " + msg.data
        #     # Emit via signal to GUI thread
        #     self.signaller.mode_changed.emit(data_str)

    # Callback for run time
    def time_now_callback(self, msg):
        self.latest_time_now = msg
        # now = time.time()
        # if now - self.time_now_last_update > 1.0 / self.update_rate:
        #     self.time_now_last_update = now
        #     # This runs in ROS subscriber thread
        #     time_str = f'{msg.data:.2f}'
        #     data_str = "Run Time: " + time_str + " sec"
        #     # Emit via signal to GUI thread
        #     self.signaller.time_changed.emit(data_str)

    # Callback for sensed external forces
    def f_sense_callback(self, msg):
        self.latest_f_sense = msg
        # now = time.time()
        # if now - self.f_sense_last_update > 1.0 / self.update_rate:
        #     self.f_sense_last_update = now            
        #     self.signaller.wrench_table_signal.emit(1, 0, msg.x)
        #     self.signaller.wrench_table_signal.emit(1, 1, msg.y)
        #     self.signaller.wrench_table_signal.emit(1, 2, msg.z)

    # Callback for sensed external torques
    def tau_sense_callback(self, msg):
        self.latest_tau_sense = msg
        # now = time.time()
        # if now - self.tau_sense_last_update > 1.0 / self.update_rate:
        #     self.tau_sense_last_update = now
        #     self.signaller.wrench_table_signal.emit(1, 3, msg.x)
        #     self.signaller.wrench_table_signal.emit(1, 4, msg.y)
        #     self.signaller.wrench_table_signal.emit(1, 5, msg.z)

    # Callback for upper wrench limits
    def wrench_thresh_callback(self, msg):
        self.latest_wrench_thresh = msg
        # now = time.time()
        # if now - self.wrench_thresh_last_update > 1.0 / self.update_rate:
        #     self.wrench_thresh_last_update = now
        #     row = 0
        #     for col in range(6):
        #         value = msg.data[col]
        #         self.wrench_thresh[col] = value
        #         self.signaller.wrench_table_signal.emit(row, col, value)

    # Callback for lower wrench limits
    def wrench_warn_callback(self, msg):
        for col in range(6):
            value = msg.data[col]
            self.wrench_warn[col] = value

    # Callback for desired joint torques
    def tau_d_callback(self, msg):
        self.latest_tau_d = msg
        # now = time.time()
        # if now - self.tau_d_last_update > 1.0 / self.update_rate:
        #     self.tau_d_last_update = now
        #     row = 1
        #     for col in range(7):
        #         value = msg.data[col]
        #         self.signaller.torque_table_signal.emit(row, col, value)

    # Callback for upper joint torque limits
    def tau_thresh_callback(self, msg):
        self.latest_tau_thresh = msg
        # now = time.time()
        # if now - self.tau_thresh_last_update > 1.0 / self.update_rate:
        #     self.tau_thresh_last_update = now
        #     row = 0
        #     for col in range(7):
        #         value = msg.data[col]
        #         self.torque_thresh[col] = value
        #         self.signaller.torque_table_signal.emit(row, col, value)

    # Callback for lower joint torque limits
    def tau_warn_callback(self, msg):
        for col in range(7):
            value = msg.data[col]
            self.torque_warn[col] = value

    # Callback for joint collision flags
    def joint_collision_callback(self, msg):
        for i in range(7):
            self.joint_collision[i] = msg.data[i]

    # Callback for cart collision flags
    def cart_collision_callback(self, msg):
        for i in range(6):
            self.cart_collision[i] = msg.data[i]

    # Helper to check threshold limits
    def check_limits(self):
        if any(self.joint_collision):
            self.joint_nonzero = [i for i, val in enumerate(self.joint_collision) if val != 0]
            self.joint_flag = 1
        else:
            self.joint_flag = 0
        if any(self.cart_collision):
            self.cart_nonzero = [i for i, val in enumerate(self.cart_collision) if val != 0]
            self.cart_flag = 1
        else:
            self.cart_flag = 0
        if self.ignore_flag:
            self.joint_flag = 0
            self.cart_flag = 0
        

    def refresh_gui(self):
        if self.latest_hybrid_mode is not None:
            # This runs in ROS subscriber thread
            data_str = "Selected Hybrid Mode: " + self.latest_hybrid_mode.data
            # Emit via signal to GUI thread
            self.signaller.mode_changed.emit(data_str)
        if self.latest_time_now is not None:
            # This runs in ROS subscriber thread
            time_str = f'{self.latest_time_now.data:.2f}'
            data_str = "Run Time: " + time_str + " sec"
            # Emit via signal to GUI thread
            self.signaller.time_changed.emit(data_str)
        if self.latest_f_sense is not None:
            self.signaller.wrench_table_signal.emit(1, 0, self.latest_f_sense.x)
            self.signaller.wrench_table_signal.emit(1, 1, self.latest_f_sense.y)
            self.signaller.wrench_table_signal.emit(1, 2, self.latest_f_sense.z)
        if self.latest_tau_sense is not None:
            self.signaller.wrench_table_signal.emit(1, 3, self.latest_tau_sense.x)
            self.signaller.wrench_table_signal.emit(1, 4, self.latest_tau_sense.y)
            self.signaller.wrench_table_signal.emit(1, 5, self.latest_tau_sense.z)
        if self.latest_wrench_thresh is not None:
            row = 0
            for col in range(6):
                value = self.latest_wrench_thresh.data[col]
                self.wrench_thresh[col] = value
                self.signaller.wrench_table_signal.emit(row, col, value)
        if self.latest_tau_d is not None:
            row = 1
            for col in range(7):
                value = self.latest_tau_d.data[col]
                self.signaller.torque_table_signal.emit(row, col, value)
        if self.latest_tau_thresh is not None:
            row = 0
            for col in range(7):
                value = self.latest_tau_thresh.data[col]
                self.torque_thresh[col] = value
                self.signaller.torque_table_signal.emit(row, col, value)
        self.check_limits()
        if self.joint_flag:
            self.ignore_flag = 1
            for i in self.joint_nonzero:
                joint = self.joint_titles[i]
                joint_val = self.latest_tau_d.data[i]
                joint_thresh = self.latest_tau_thresh.data[i]
                new_error = f'Joint Command Torque {joint} = {joint_val:.2f} exceeded the threshold of {joint_thresh:.2f}\n'
                self.error_str += new_error
                # self._widget.errorReport.append(new_error)
                # self._widget.errorReport.moveCursor(QTextCursor.End)
        if self.cart_flag:
            self.ignore_flag = 1
            for i in self.cart_nonzero:
                cart = self.cart_titles[i]
                cart_thresh = self.latest_wrench_thresh.data[i]
                if i == 0:
                    cart_val = self.latest_f_sense.x
                elif i == 1:
                    cart_val = self.latest_f_sense.y
                elif i == 2:
                    cart_val = self.latest_f_sense.z
                elif i == 3:
                    cart_val = self.latest_tau_sense.x
                elif i == 4:
                    cart_val = self.latest_tau_sense.y
                elif i == 5:
                    cart_val = self.latest_tau_sense.z
                else:
                    cart_val = 0.0
                new_error = f'External Wrench value {cart} = {cart_val:.2f} exceeded the threshold of {cart_thresh:.2f}\n'
                self.error_str += new_error
                # self._widget.errorReport.append(new_error)
                # self._widget.errorReport.moveCursor(QTextCursor.End)
        self.signaller.error_signal.emit(self.error_str)

    @Slot(str)
    def on_mode_changed(self, data_str):
        # This runs in Qt GUI thread
        self._widget.hybridModeLabel.setText(data_str)

    @Slot(str)
    def on_time_changed(self, data_str):
        # This runs in Qt GUI thread
        self._widget.timeNow.setText(data_str)

    @Slot(int, int, float)
    def on_wrench_changed(self, row, col, value):
        val = abs(value)
        item = QTableWidgetItem(f"{val:.2f}")
        if row == 1:
            if val < self.wrench_warn[col]:
                item.setBackground(QColor(255, 255, 255))
            elif val < self.wrench_thresh[col]:
                item.setBackground(QColor(255, 255, 0))
            else:
                item.setBackground(QColor(255, 0, 0))

        self._widget.wrenchTable.setItem(row, col, item)

    @Slot(int, int, float)
    def on_torque_changed(self, row, col, value):
        val = abs(value)
        item = QTableWidgetItem(f"{val:.2f}")
        if row == 1:
            if val < self.torque_warn[col]:
                item.setBackground(QColor(255, 255, 255))
            elif val < self.torque_thresh[col]:
                item.setBackground(QColor(255, 255, 0))
            else:
                item.setBackground(QColor(255, 0, 0))

        self._widget.torqueTable.setItem(row, col, item)

    @Slot(str)
    def on_error_changed(self, data_str):
        self._widget.errorReport.append(data_str)
        self._widget.errorReport.moveCursor(QTextCursor.End)
        self._widget.errorReport.setText(data_str)

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog