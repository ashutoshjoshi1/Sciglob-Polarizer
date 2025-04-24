from PyQt5.QtCore import QObject, QTimer, pyqtSignal, Qt
from PyQt5.QtWidgets import QGroupBox, QLabel, QComboBox, QPushButton, QVBoxLayout, QHBoxLayout
from PyQt5.QtGui import QImage, QPixmap
import cv2
import serial
from serial.tools import list_ports
import imu
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import utils

class IMUController(QObject):
    status_signal = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.groupbox = QGroupBox("IMU")
        self.groupbox.setObjectName("imuGroup")
        layout = QVBoxLayout()
        conn_layout = QHBoxLayout()
        conn_layout.addWidget(QLabel("IMU COM:"))
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        ports = [p.device for p in list_ports.comports()]
        self.port_combo.addItems(ports or [f"COM{i}" for i in range(1,10)])
        conn_layout.addWidget(self.port_combo)
        conn_layout.addWidget(QLabel("Baud:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600","57600","115200"])
        self.baud_combo.setCurrentText("9600")
        conn_layout.addWidget(self.baud_combo)
        self.connect_btn = QPushButton("Connect IMU")
        self.connect_btn.clicked.connect(self.connect)
        conn_layout.addWidget(self.connect_btn)
        layout.addLayout(conn_layout)

        self.data_label = QLabel("IMU data: not connected")
        layout.addWidget(self.data_label)

        # 3D orientation canvas
        self.figure = plt.figure(figsize=(4,4))
        self.ax = self.figure.add_subplot(111, projection='3d')
        utils.draw_device_orientation(self.ax, 0, 0, 0, 0.0, 0.0)
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)

        # Camera preview
        self.camera_label = QLabel()
        self.camera_label.setFixedHeight(200)
        self.camera_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.camera_label)
        self.camera = cv2.VideoCapture(0)
        self.cam_timer = QTimer(self)
        self.cam_timer.timeout.connect(self._update_camera)
        self.cam_timer.start(30)

        self.groupbox.setLayout(layout)

        self._connected = False
        self.serial = None
        self.latest_data = {'rpy':(0,0,0),'latitude':0.0,'longitude':0.0,
                            'temperature':0.0,'pressure':0.0}

    def connect(self):
        if self._connected:
            self.status_signal.emit("IMU is already connected.")
            return
        port = self.port_combo.currentText().strip()
        try:
            baud = int(self.baud_combo.currentText())
        except ValueError:
            baud = 9600
        try:
            self.serial = serial.Serial(port, baudrate=baud, timeout=1)
        except Exception as e:
            self.status_signal.emit(f"Failed to open IMU port: {e}")
            return
        self._connected = True
        self.status_signal.emit(f"IMU connected on {port} at {baud} baud.")
        # start IMU read thread
        self._stop_event = imu.start_imu_read_thread(self.serial, self.latest_data)
        # update visualization
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self._update_visual)
        self.update_timer.start(100)

    def _update_camera(self):
        if self.camera.isOpened():
            ret, frame = self.camera.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h,w,ch = frame.shape
                img = QImage(frame.data, w, h, ch*w, QImage.Format_RGB888)
                self.camera_label.setPixmap(QPixmap.fromImage(img))

    def _update_visual(self):
        if not self._connected:
            return
        roll,pitch,yaw = self.latest_data['rpy']
        temp = self.latest_data.get('temperature',0.0)
        pres = self.latest_data.get('pressure',0.0)
        lat = self.latest_data.get('latitude',0.0)
        lon = self.latest_data.get('longitude',0.0)
        self.data_label.setText(
            f"Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°\n"
            f"Temp={temp:.2f} °C, Pressure={pres:.2f} hPa\n"
            f"Lat={lat:.6f}, Lon={lon:.6f}"
        )
        utils.draw_device_orientation(self.ax, roll, pitch, yaw, lat, lon)
        self.canvas.draw()

    def is_connected(self):
        return self._connected