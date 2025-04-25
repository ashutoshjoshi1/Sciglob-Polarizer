from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QGridLayout, QSplitter, QStatusBar
)
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

        # Central widget & main layout
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # Instantiate controllers
        self.temp_ctrl = TempController(parent=self)
        self.spec_ctrl = SpectrometerController(parent=self)
        self.motor_ctrl = MotorController(parent=self)
        self.filter_ctrl = FilterWheelController(parent=self)
        self.imu_ctrl = IMUController(parent=self)

        # Connect status signals to status bar
        for ctrl in (self.temp_ctrl, self.spec_ctrl, self.motor_ctrl, self.filter_ctrl, self.imu_ctrl):
            ctrl.status_signal.connect(self.status_bar.showMessage)

        # 1) Temperature control spans full width at top
        main_layout.addWidget(self.temp_ctrl.widget)

        # 2) Splitter: spectrometer on left, grid of other controllers on right
        splitter = QSplitter(Qt.Horizontal)

        # Left pane: Spectrometer
        splitter.addWidget(self.spec_ctrl.groupbox)

        # Right pane: grid layout for Motor, Filter, IMU
        right = QWidget()
        grid = QGridLayout(right)
        grid.addWidget(self.motor_ctrl.groupbox, 0, 0)
        grid.addWidget(self.filter_ctrl.groupbox, 0, 1)
        grid.addWidget(self.imu_ctrl.groupbox,    1, 0)
        # leave (1,1) empty (or you could put something else there)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.setRowStretch(0, 0)
        grid.setRowStretch(1, 1)

        splitter.addWidget(right)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        main_layout.addWidget(splitter)

        # Status indicator updater
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._update_indicators)
        self.status_timer.start(1000)
        self._update_indicators()

    def _update_indicators(self):
        # Motor
        color = "green" if self.motor_ctrl.is_connected() else "red"
        gb = self.motor_ctrl.groupbox
        gb.setTitle("● Motor")
        gb.setStyleSheet(f"QGroupBox#motorGroup::title {{ color: {color}; }}")

        # Filter wheel
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
