# gui/main_window.py

import os
import math
import cv2
import serial
import numpy as np
from serial.tools import list_ports

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QLineEdit, QPushButton, QComboBox, QSplitter, QStatusBar
)
from PyQt5.QtCore import QTimer, Qt, QDateTime
from PyQt5.QtGui import QPixmap, QImage

import pyqtgraph as pg
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from drivers import motor, imu, spectrometer, filterwheel, tc_36_25driver


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Spectrometer, Motor, IMU & Temperature Control")
        self.setMinimumSize(1000, 750)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

        # Central layout
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(8)

        # Temperature Controls
        temp_layout = QHBoxLayout()
        temp_layout.addWidget(QLabel("Set Temp (°C):"))
        self.set_temp_input = QLineEdit("20.0")
        self.set_temp_input.setFixedWidth(60)
        temp_layout.addWidget(self.set_temp_input)
        self.send_temp_button = QPushButton("Send Cmd")
        self.send_temp_button.clicked.connect(self.on_send_temp)
        temp_layout.addWidget(self.send_temp_button)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(QLabel("Current Temp:"))
        self.current_temp_label = QLabel("-- °C")
        temp_layout.addWidget(self.current_temp_label)
        temp_layout.addStretch()
        main_layout.addLayout(temp_layout)

        # Initialize temperature controller
        try:
            self.tc = TC36_25()
            self.tc.enable_computer_setpoint()
            self.tc.power(True)
            self.last_setpoint = None
            self.temp_timer = QTimer(self)
            self.temp_timer.timeout.connect(self.update_temperature)
            self.temp_timer.start(1000)
        except Exception as e:
            self.status_bar.showMessage(f"Temp ctrl init failed: {e}")

        # Spectrometer Controls
        spectro_layout = QHBoxLayout()
        self.toggle_save_button = QPushButton("Start Saving")
        self.toggle_save_button.setEnabled(False)
        self.toggle_save_button.clicked.connect(self.toggle_data_saving)
        spectro_layout.addWidget(self.toggle_save_button)
        self.connect_spec_button = QPushButton("Connect Spectrometer")
        self.connect_spec_button.clicked.connect(self.on_connect_spectrometer)
        spectro_layout.addWidget(self.connect_spec_button)
        self.start_meas_button = QPushButton("Start Measurement")
        self.start_meas_button.setEnabled(False)
        self.start_meas_button.clicked.connect(self.on_start_measurement)
        spectro_layout.addWidget(self.start_meas_button)
        self.stop_meas_button = QPushButton("Stop")
        self.stop_meas_button.setEnabled(False)
        self.stop_meas_button.clicked.connect(self.on_stop_measurement)
        spectro_layout.addWidget(self.stop_meas_button)

        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        self.spec_plot = pg.PlotWidget()
        self.spec_plot.setLabel('bottom', 'Wavelength', units='nm')
        self.spec_plot.setLabel('left', 'Intensity', units='counts')
        self.spec_plot.showGrid(x=True, y=True, alpha=0.3)
        self.spec_plot.setXRange(260, 600)
        self.spec_curve = self.spec_plot.plot([], [], pen=pg.mkPen('#2986cc', width=1))

        self.spectro_group = QGroupBox("Spectrometer")
        sp_layout = QVBoxLayout()
        sp_layout.addLayout(spectro_layout)
        sp_layout.addWidget(self.spec_plot, stretch=1)
        self.spectro_group.setLayout(sp_layout)

        # Motor Controls
        ports = list_ports.comports()
        motor_layout = QGridLayout()
        motor_layout.addWidget(QLabel("Motor COM:"), 0, 0)
        self.motor_port_combo = QComboBox(); self.motor_port_combo.setEditable(True)
        for p in ports: self.motor_port_combo.addItem(getattr(p, 'name', p.device))
        motor_layout.addWidget(self.motor_port_combo, 0, 1)
        self.motor_connect_button = QPushButton("Connect Motor")
        self.motor_connect_button.clicked.connect(self.on_connect_motor)
        motor_layout.addWidget(self.motor_connect_button, 0, 2)
        motor_layout.addWidget(QLabel("Angle (°):"), 1, 0)
        self.angle_input = QLineEdit(); self.angle_input.setFixedWidth(60)
        motor_layout.addWidget(self.angle_input, 1, 1)
        self.move_button = QPushButton("Move"); self.move_button.setEnabled(False)
        self.move_button.clicked.connect(self.on_move_motor)
        motor_layout.addWidget(self.move_button, 1, 2)
        self.motor_group = QGroupBox("Motor"); self.motor_group.setLayout(motor_layout)

        # Filter Wheel Controls
        filter_layout = QHBoxLayout()
        filter_layout.addWidget(QLabel("FilterWheel COM:"))
        self.filter_port_combo = QComboBox(); self.filter_port_combo.setEditable(True)
        for p in ports: self.filter_port_combo.addItem(getattr(p, 'name', p.device))
        self.filter_port_combo.setCurrentText("COM17")
        filter_layout.addWidget(self.filter_port_combo)
        filter_layout.addWidget(QLabel("Command:"))
        self.filter_cmd_input = QLineEdit(); self.filter_cmd_input.setPlaceholderText("F1r, F15, or ?")
        filter_layout.addWidget(self.filter_cmd_input)
        self.filter_send_button = QPushButton("Send")
        self.filter_send_button.clicked.connect(self.on_send_filter)
        filter_layout.addWidget(self.filter_send_button)
        filter_layout.addWidget(QLabel("Pos:"))
        self.filter_pos_label = QLabel("--"); filter_layout.addWidget(self.filter_pos_label)
        filter_layout.addStretch()
        self.filter_group = QGroupBox("Filter Wheel"); self.filter_group.setLayout(filter_layout)

        # IMU Controls & 3D Plot
        imu_layout = QHBoxLayout()
        imu_layout.addWidget(QLabel("IMU COM:"))
        self.imu_port_combo = QComboBox(); self.imu_port_combo.setEditable(True)
        for p in ports: self.imu_port_combo.addItem(getattr(p, 'name', p.device))
        imu_layout.addWidget(self.imu_port_combo)
        imu_layout.addWidget(QLabel("Baud:"))
        self.imu_baud_combo = QComboBox();
        for b in (9600, 57600, 115200): self.imu_baud_combo.addItem(str(b))
        imu_layout.addWidget(self.imu_baud_combo)
        self.imu_connect_button = QPushButton("Connect IMU")
        self.imu_connect_button.clicked.connect(self.on_connect_imu)
        imu_layout.addWidget(self.imu_connect_button)
        self.imu_data_label = QLabel("IMU: not connected")
        self.figure = plt.figure(figsize=(4,4))
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.canvas = FigureCanvas(self.figure)
        self.imu_group = QGroupBox("IMU")
        imu_group_layout = QVBoxLayout()
        imu_group_layout.addLayout(imu_layout)
        imu_group_layout.addWidget(self.imu_data_label)
        imu_group_layout.addWidget(self.canvas)
        self.imu_group.setLayout(imu_group_layout)

        # Assemble splitter
        right = QWidget(); right_layout = QVBoxLayout(right)
        right_layout.addWidget(self.motor_group)
        right_layout.addWidget(self.filter_group)
        right_layout.addWidget(self.imu_group)
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.spectro_group)
        splitter.addWidget(right)
        main_layout.addWidget(splitter)

        # Data saving
        self.csv_dir = "data"; self.log_dir = "data"
        os.makedirs(self.csv_dir, exist_ok=True)
        os.makedirs(self.log_dir, exist_ok=True)
        self.csv_file = None; self.log_file = None
        self.continuous_saving = False
        self.save_data_timer = QTimer(self)
        self.save_data_timer.timeout.connect(self.save_continuous_data)

        # State
        self.current_motor_angle = 0.0
        self.current_filter_position = None
        self.spec_handle = None
        self.measurement_active = False
        self.wavelengths = []
        self.intensities = []
        self.latest_data = {"rpy":(0,0,0),"accel":(0,0,0),"gyro":(0,0,0),"mag":(0,0,0),"pressure":0.0}

    # Motor handlers
    def on_connect_motor(self):
        port = self.motor_port_combo.currentText().strip()
        self.motor_thread = motor.MotorConnectThread(port)
        self.motor_thread.result_signal.connect(self.on_motor_connect_result)
        self.motor_thread.start()
    def on_motor_connect_result(self, ser, baud, msg):
        self.move_button.setEnabled(bool(ser and ser.is_open))
        self.status_bar.showMessage(msg)
    def on_move_motor(self):
        angle = int(self.angle_input.text())
        ok = motor.send_move_command(self.motor_serial, angle)
        if ok: self.current_motor_angle = angle

    # IMU handlers
    def on_connect_imu(self):
        port = self.imu_port_combo.currentText().strip()
        baud = int(self.imu_baud_combo.currentText())
        self.imu_serial = serial.Serial(port, baudrate=baud, timeout=1)
        self.imu_connected = True
        self.imu_stop_event = imu.start_imu_read_thread(self.imu_serial, self.latest_data)
        self.imu_timer = QTimer(self); self.imu_timer.timeout.connect(self.update_imu_visualization); self.imu_timer.start(100)
    def update_imu_visualization(self):
        r,p,y = self.latest_data["rpy"]
        self.ax.cla()
        # draw cube
        # ... (same as above)
        self.canvas.draw()

    # Spectrometer handlers
    def on_connect_spectrometer(self):
        h, wls, npix, s = spectrometer.connect_spectrometer()
        self.spec_handle, self.wavelengths, self.num_pixels = h, wls.tolist(), npix
        self.start_meas_button.setEnabled(True)
    def on_start_measurement(self):
        spectrometer.prepare_measurement(self.spec_handle, self.num_pixels, 50.0, 1)
        self.measurement_active = True
        cb = spectrometer.AVS_MeasureCallbackFunc(self._spectro_callback)
        spectrometer.AVS_MeasureCallback(self.spec_handle, cb, -1)
        self.spec_timer = QTimer(self); self.spec_timer.timeout.connect(self.update_spectrometer_plot); self.spec_timer.start(200)
    def _spectro_callback(self, h, arr, num, ctx):
        a = np.ctypeslib.as_array(arr, shape=(num,))
        self.intensities = [0.0]*self.num_pixels
        self.intensities[:len(a)] = a.tolist()
        return 0
    def update_spectrometer_plot(self):
        self.spec_curve.setData(self.wavelengths, self.intensities)
    def on_stop_measurement(self):
        spectrometer.stop_measurement(self.spec_handle)
        self.measurement_active = False
        self.spec_timer.stop()

    # Filter handlers
    def on_send_filter(self):
        cmd = self.filter_cmd_input.text().strip(); port = self.filter_port_combo.currentText().strip()
        if not getattr(self, 'filterwheel_connected', False): self.filterwheel_serial = filterwheel.open_port(port); self.filterwheel_connected = True
        t = filterwheel.FilterWheelCommandThread(self.filterwheel_serial, cmd)
        t.result_signal.connect(self.on_filter_command_result); t.start()
    def on_filter_command_result(self, pos, msg):
        self.current_filter_position = pos; self.status_bar.showMessage(msg)

    # Data saving
    def toggle_data_saving(self):
        if not self.continuous_saving:
            ts = QDateTime.currentDateTime().toString("yyyyMMdd_hhmmss")
            self.csv_file = open(f"{self.csv_dir}/log_{ts}.csv", 'w', encoding='utf-8')
            self.csv_file.write("Timestamp,MotorAngle,...\n")
            self.save_data_timer.start(1000); self.continuous_saving=True; self.toggle_save_button.setText("Pause Saving")
        else:
            self.save_data_timer.stop(); self.csv_file.close(); self.continuous_saving=False; self.toggle_save_button.setText("Start Saving")
    def save_continuous_data(self):
        t = QDateTime.currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz")
        row = [t, str(self.current_motor_angle)] + [str(x) for x in self.intensities]
        self.csv_file.write(",".join(row)+"\n"); self.csv_file.flush()

    # Temperature handlers
    def on_send_temp(self):
        sp = float(self.set_temp_input.text().strip()); self.tc.set_setpoint(sp); self.last_setpoint=sp
    def update_temperature(self):
        t = self.tc.get_temperature(); self.current_temp_label.setText(f"{t:.2f} °C")

    def closeEvent(self, event):
        try: spectrometer.stop_measurement(self.spec_handle); spectrometer.close_spectrometer()
        except: pass
        try: self.motor_serial.close()
        except: pass
        try: self.imu_stop_event.set(); self.imu_serial.close()
        except: pass
        event.accept()
