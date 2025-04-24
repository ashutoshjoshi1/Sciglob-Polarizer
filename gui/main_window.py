from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QSplitter, QStatusBar
from PyQt5.QtCore import QTimer, Qt
from controllers.motor_controller import MotorController
from controllers.filterwheel_controller import FilterWheelController
from controllers.imu_controller import IMUController
from controllers.spectrometer_controller import SpectrometerController
from controllers.temp_controller import TempController

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Spectrometer, Motor, IMU & Temperature Control")
        self.setMinimumSize(1000, 750)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

        # Central widget & layout
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # Initialize controllers
        self.temp_ctrl = TempController(parent=self)
        self.spec_ctrl = SpectrometerController(parent=self)
        self.motor_ctrl = MotorController(parent=self)
        self.filter_ctrl = FilterWheelController(parent=self)
        self.imu_ctrl = IMUController(parent=self)

        # Connect status signals
        self.temp_ctrl.status_signal.connect(self.status_bar.showMessage)
        self.spec_ctrl.status_signal.connect(self.status_bar.showMessage)
        self.motor_ctrl.status_signal.connect(self.status_bar.showMessage)
        self.filter_ctrl.status_signal.connect(self.status_bar.showMessage)
        self.imu_ctrl.status_signal.connect(self.status_bar.showMessage)

        # Temperature control UI
        layout.addWidget(self.temp_ctrl.widget)

        # Splitter for main controls
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.spec_ctrl.groupbox)
        right_container = QWidget()
        right_layout = QVBoxLayout(right_container)
        right_layout.addWidget(self.motor_ctrl.groupbox)
        right_layout.addWidget(self.filter_ctrl.groupbox)
        right_layout.addWidget(self.imu_ctrl.groupbox)
        right_layout.addStretch()
        splitter.addWidget(right_container)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        layout.addWidget(splitter)

        # Status indicator timer
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.update_status_indicators)
        self.status_timer.start(1000)
        self.update_status_indicators()

    def update_status_indicators(self):
        # Motor
        color = "green" if self.motor_ctrl.is_connected() else "red"
        gb = self.motor_ctrl.groupbox
        gb.setTitle("● Motor")
        gb.setStyleSheet(f"QGroupBox#motorGroup::title {{ color: {color}; }}")

        # Filter Wheel
        color = "green" if self.filter_ctrl.is_connected() else "red"
        gb = self.filter_ctrl.groupbox
        gb.setTitle("● Filter Wheel")
        gb.setStyleSheet(f"QGroupBox#filterwheelGroup::title {{ color: {color}; }}")

        # IMU
        color = "green" if self.imu_ctrl.is_connected() else "red"
        gb = self.imu_ctrl.groupbox
        gb.setTitle("● IMU")
        gb.setStyleSheet(f"QGroupBox#imuGroup::title {{ color: {color}; }}")

        # Spectrometer
        color = "green" if self.spec_ctrl.is_ready() else "red"
        gb = self.spec_ctrl.groupbox
        gb.setTitle("● Spectrometer")
        gb.setStyleSheet(f"QGroupBox#specGroup::title {{ color: {color}; }}")
