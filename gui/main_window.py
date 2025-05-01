# gui/main_window.py

import os
import numpy as np
from drivers import motor               # ← properly import your motor driver

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QGridLayout, QSplitter,
    QStatusBar, QPushButton
)
from PyQt5.QtCore import QTimer, Qt, QDateTime

from controllers.motor_controller        import MotorController
from controllers.filterwheel_controller import FilterWheelController
from controllers.imu_controller          import IMUController
from controllers.spectrometer_controller import SpectrometerController
from controllers.temp_controller         import TempController


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Spectrometer, Motor, IMU & Temperature Control")
        self.setMinimumSize(1920, 1000)

        # in-memory stores
        self.latest_data = {}
        self.wavelengths = []
        self.intensities = []

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

        # Layout
        central     = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # Controllers
        self.temp_ctrl   = TempController(parent=self)
        self.spec_ctrl   = SpectrometerController(parent=self)
        self.motor_ctrl  = MotorController(parent=self)
        self.filter_ctrl = FilterWheelController(parent=self)
        self.imu_ctrl    = IMUController(parent=self)

        # Route status messages into status bar
        for ctrl in (self.temp_ctrl, self.spec_ctrl,
                     self.motor_ctrl, self.filter_ctrl,
                     self.imu_ctrl):
            ctrl.status_signal.connect(self.status_bar.showMessage)

        # Build UI
        main_layout.addWidget(self.temp_ctrl.widget)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.spec_ctrl.groupbox)

        right = QWidget()
        grid  = QGridLayout(right)
        grid.addWidget(self.motor_ctrl.groupbox, 0, 0)
        grid.addWidget(self.filter_ctrl.groupbox, 0, 1)
        grid.addWidget(self.imu_ctrl.groupbox,    1, 0)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)

        splitter.addWidget(right)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        main_layout.addWidget(splitter)

        # Start/Pause button
        self.toggle_save_button = QPushButton("Start Saving")
        self.toggle_save_button.clicked.connect(self.toggle_data_saving)
        main_layout.addWidget(self.toggle_save_button)

        # Status-indicator refresher
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._update_indicators)
        self.status_timer.start(1000)
        self._update_indicators()

        # Prepare data folder & timers
        self.csv_dir    = "data"
        self.log_dir    = "data"
        os.makedirs(self.csv_dir, exist_ok=True)
        os.makedirs(self.log_dir, exist_ok=True)

        self.csv_file   = None
        self.log_file   = None
        self.continuous_saving = False

        self.save_data_timer = QTimer(self)
        self.save_data_timer.timeout.connect(self.save_continuous_data)


    def _spectro_callback(self, handle, read_array, num_read, p_context):
        arr = np.ctypeslib.as_array(read_array, shape=(num_read,))
        self.intensities = arr.tolist()
        self.wavelengths = np.linspace(
            self.spec_ctrl.start_nm,
            self.spec_ctrl.end_nm,
            num_read
        ).tolist()
        return 0


    def update_spectrometer_plot(self):
        if self.intensities and self.wavelengths:
            self.spec_ctrl.update_plot(self.wavelengths, self.intensities)


    def toggle_data_saving(self):
        if not self.continuous_saving:
            # --- START LOGGING ---
            if self.csv_file:
                self.csv_file.close()
            if self.log_file:
                self.log_file.close()

            ts = QDateTime.currentDateTime().toString("yyyyMMdd_hhmmss")
            self.csv_file_path = os.path.join(self.csv_dir, f"log_{ts}.csv")
            self.log_file_path = os.path.join(self.log_dir, f"log_{ts}.txt")

            try:
                self.csv_file = open(self.csv_file_path, "w", encoding="utf-8", newline="")
                self.log_file = open(self.log_file_path, "w", encoding="utf-8")
            except Exception as e:
                self.status_bar.showMessage(f"Failed to open files: {e}")
                return

            headers = [
                "Timestamp",
                "MotorPos_steps",
                "MotorSpeed_steps_s",
                "MotorCurrent_pct",
                "MotorAlarmCode",
                "MotorTemp_C",
                "MotorAngle_deg",
                "FilterWheelPos",
                "Roll_deg","Pitch_deg","Yaw_deg",
                "AccelX_g","AccelY_g","AccelZ_g",
                "GyroX_dps","GyroY_dps","GyroZ_dps",
                "MagX_uT","MagY_uT","MagZ_uT",
                "Pressure_hPa","Temperature_C",
                "Latitude_deg","Longitude_deg",
                "IntegrationTime_us",
                "TempController_curr","TempController_set"
            ]
            for wl in self.wavelengths:
                headers.append(f"I_{wl:.2f}nm")
            self.csv_file.write(",".join(headers) + "\n")
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())

            self.save_data_timer.start(1000)
            self.continuous_saving = True
            self.toggle_save_button.setText("Pause Saving")
            self.status_bar.showMessage(f"Started saving → {self.csv_file_path}")

        else:
            # --- STOP LOGGING ---
            self.continuous_saving = False
            self.save_data_timer.stop()
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
            if self.log_file:
                self.log_file.close()
                self.log_file = None
            self.toggle_save_button.setText("Start Saving")
            self.status_bar.showMessage(f"Finished saving to {self.csv_file_path}")


    def save_continuous_data(self):
        if not (self.csv_file and self.log_file):
            return

        try:
            now    = QDateTime.currentDateTime()
            ts_csv = now.toString("yyyy-MM-dd hh:mm:ss.zzz")
            ts_txt = now.toString("yyyy-MM-dd hh:mm:ss")

            # motor data from drivers.motor
            motor_pos         = int(getattr(self, "current_motor_angle", 0))
            motor_speed       = getattr(motor, "TrackerSpeed", 0)
            motor_current_pct = getattr(motor, "TrackerCurrent", 0) / 10.0

            # imu / gps / temp data
            r, p, y    = self.latest_data.get("rpy",         (0,0,0))
            ax, ay, az = self.latest_data.get("accel",       (0,0,0))
            gx, gy, gz = self.latest_data.get("gyro",        (0,0,0))
            mx, my, mz = self.latest_data.get("mag",         (0,0,0))
            pres       = self.latest_data.get("pressure",    0)
            temp       = self.latest_data.get("temperature", 0)
            lat        = self.latest_data.get("latitude",    0)
            lon        = self.latest_data.get("longitude",   0)
            integ      = getattr(self, "current_integration_time_us", 0)
            tc_curr    = self.latest_data.get("TempController_curr", 0)
            tc_set     = self.latest_data.get("TempController_set",  0)

            row = [
                ts_csv,
                str(motor_pos),
                str(motor_speed),
                f"{motor_current_pct:.1f}",
                "", "", "",
                getattr(self, "filter_position", ""),
                f"{r:.2f}", f"{p:.2f}", f"{y:.2f}",
                f"{ax:.2f}", f"{ay:.2f}", f"{az:.2f}",
                f"{gx:.2f}", f"{gy:.2f}", f"{gz:.2f}",
                f"{mx:.2f}", f"{my:.2f}", f"{mz:.2f}",
                f"{pres:.2f}", f"{temp:.2f}",
                f"{lat:.6f}", f"{lon:.6f}",
                str(int(integ)),
                f"{tc_curr:.2f}", f"{tc_set:.2f}"
            ]
            for inten in self.intensities:
                row.append(f"{inten:.4f}")

            line = ",".join(row) + "\n"
            self.csv_file.write(line)
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())

            peak = max(self.intensities) if self.intensities else 0
            txt_line = f"{ts_txt} | Peak {peak}\n"
            self.log_file.write(txt_line)
            self.log_file.flush()
            os.fsync(self.log_file.fileno())

            print("Saved row:", row)

        except Exception as e:
            print("Error in save_continuous_data:", e)
            self.status_bar.showMessage(f"Save error: {e}")


    def _update_indicators(self):
        for ctrl, name, prop in [
            (self.motor_ctrl,  "Motor",       self.motor_ctrl.is_connected),
            (self.filter_ctrl,"Filter Wheel", self.filter_ctrl.is_connected),
            (self.imu_ctrl,    "IMU",         self.imu_ctrl.is_connected),
            (self.spec_ctrl,   "Spectrometer",self.spec_ctrl.is_ready)
        ]:
            col = "green" if prop() else "red"
            gb  = ctrl.groupbox
            gb.setTitle(f"● {name}")
            gb.setStyleSheet(f"QGroupBox#{gb.objectName()}::title {{ color: {col}; }}")
