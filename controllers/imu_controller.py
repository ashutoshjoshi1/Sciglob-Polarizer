from PyQt5.QtCore import QObject, QTimer, pyqtSignal, Qt
from PyQt5.QtWidgets import QGroupBox, QLabel, QComboBox, QPushButton, QVBoxLayout, QHBoxLayout
from PyQt5.QtGui import QPixmap, QImage
import cv2, serial
from serial.tools import list_ports
from drivers.imu import start_imu_read_thread
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import utils

class IMUController(QObject):
    status_signal = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        # --- Group box and layout ---
        self.groupbox = QGroupBox("IMU")
        self.groupbox.setObjectName("imuGroup")
        layout = QVBoxLayout()

        # --- Connection row ---
        conn_layout = QHBoxLayout()
        conn_layout.addWidget(QLabel("COM:"))
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        ports = [p.device for p in list_ports.comports()]
        self.port_combo.addItems(ports or [f"COM{i}" for i in range(1, 10)])
        conn_layout.addWidget(self.port_combo)

        conn_layout.addWidget(QLabel("Baud:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "57600", "115200"])
        conn_layout.addWidget(self.baud_combo)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect)
        conn_layout.addWidget(self.connect_btn)
        layout.addLayout(conn_layout)

        # --- Camera preview (on top) ---
        self.cam_label = QLabel()
        self.cam_label.setAlignment(Qt.AlignCenter)
        self.cam_label.setFixedHeight(200)
        layout.addWidget(self.cam_label, stretch=2)
        self.camera = cv2.VideoCapture(0)
        self.cam_timer = QTimer(self)
        self.cam_timer.timeout.connect(self._update_cam)
        self.cam_timer.start(30)

        # --- IMU data text ---
        self.data_label = QLabel("Not connected")
        layout.addWidget(self.data_label, stretch=1)

        # --- 3D orientation plot ---
        self.fig = plt.figure(figsize=(4, 4))
        self.ax = self.fig.add_subplot(111, projection='3d')
        utils.draw_device_orientation(self.ax, 0, 0, 0, 0, 0)
        self.canvas = FigureCanvas(self.fig)
        layout.addWidget(self.canvas, stretch=3)

        self.groupbox.setLayout(layout)

        # --- Internal state ---
        self._connected = False
        self.serial = None
        self.latest = {
            'rpy': (0.0, 0.0, 0.0),
            'latitude': 0.0,
            'longitude': 0.0,
            'temperature': 0.0,
            'pressure': 0.0
        }

    def connect(self):
        if self._connected:
            self.status_signal.emit("Already connected")
            return

        port = self.port_combo.currentText().strip()
        try:
            baud = int(self.baud_combo.currentText())
        except ValueError:
            baud = 9600

        try:
            self.serial = serial.Serial(port, baudrate=baud, timeout=1)
        except Exception as e:
            self.status_signal.emit(f"Connection failed: {e}")
            return

        self._connected = True
        self.status_signal.emit(f"IMU connected on {port}@{baud}")
        self.stop_event = start_imu_read_thread(self.serial, self.latest)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._refresh)
        self.refresh_timer.start(100)

    def _update_cam(self):
        if not self.camera.isOpened():
            return
        ret, frame = self.camera.read()
        if not ret:
            return
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame.shape
        img = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
        self.cam_label.setPixmap(QPixmap.fromImage(img))

    def _refresh(self):
        roll, pitch, yaw = self.latest['rpy']
        lat = self.latest['latitude']
        lon = self.latest['longitude']
        temp = self.latest['temperature']
        pres = self.latest['pressure']

        self.data_label.setText(
            f"Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°\n"
            f"Temp={temp:.2f} °C, Pressure={pres:.2f} hPa\n"
            f"Lat={lat:.6f}, Lon={lon:.6f}"
        )

        utils.draw_device_orientation(self.ax, roll, pitch, yaw, lat, lon)
        self.canvas.draw()

    def is_connected(self):
        return self._connected
