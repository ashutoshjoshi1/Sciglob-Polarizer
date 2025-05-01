# gui/main_window.py

import os
import numpy as np
import motor                                    # driver-level module for TrackerSpeed / TrackerCurrent
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QGridLayout, QSplitter,
    QStatusBar, QPushButton
)
from PyQt5.QtCore import QTimer, Qt, QDateTime

# your high-level controller widgets
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

        # stash for all incoming data
        self.latest_data   = {}
        self.wavelengths   = []
        self.intensities   = []

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

        # Central widget & main layout
        central     = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # Instantiate controllers
        self.temp_ctrl   = TempController(parent=self)
        self.spec_ctrl   = SpectrometerController(parent=self)
        self.motor_ctrl  = MotorController(parent=self)
        self.filter_ctrl = FilterWheelController(parent=self)
        self.imu_ctrl    = IMUController(parent=self)

        # route status messages
        for ctrl in (
            self.temp_ctrl,
            self.spec_ctrl,
            self.motor_ctrl,
            self.filter_ctrl,
            self.imu_ctrl
        ):
            ctrl.status_signal.connect(self.status_bar.showMessage)

        # add temperature widget full-width
        main_layout.addWidget(self.temp_ctrl.widget)

        # spectrometer on left, others on right
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

        # button to start/pause saving
        self.toggle_save_button = QPushButton("Start Saving")
        self.toggle_save_button.clicked.connect(self.toggle_data_saving)
        main_layout.addWidget(self.toggle_save_button)

        # periodic indicator update (red/green dots)
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._update_indicators)
        self.status_timer.start(1000)
        self._update_indicators()

        # prepare folders & timers for data logging
        self.csv_dir    = "data"
        self.log_dir    = "data"
        os.makedirs(self.csv_dir, exist_ok=True)
        os.makedirs(self.log_dir, exist_ok=True)

        self.csv_file   = None
        self.log_file   = None
        self.continuous_saving = False

        self.save_data_timer = QTimer(self)
        self.save_data_timer.timeout.connect(self.save_continuous_data)


    def on_start_measurement(self):
        """(unchanged) configure and kick off the spectrometer callback + plot timer."""
        import spectrometer  # driver-level API
        if getattr(self, "spec_handle", None) is None or \
           self.spec_handle == spectrometer.INVALID_AVS_HANDLE_VALUE:
            self.status_bar.showMessage("Spectrometer not connected.")
            return
        if getattr(self, "measurement_active", False):
            return

        integration_time_ms = 50.0
        res = spectrometer.prepare_measurement(
            self.spec_handle,
            self.num_pixels,
            integration_time_ms=integration_time_ms,
            averages=1
        )
        if res != 0:
            self.status_bar.showMessage(f"Error preparing measurement (code {res}).")
            return

        self.measurement_active = True
        self.current_integration_time_us = int(integration_time_ms * 1000)
        self.cb_func = spectrometer.AVS_MeasureCallbackFunc(self._spectro_callback)
        res = spectrometer.AVS_MeasureCallback(self.spec_handle, self.cb_func, -1)
        if res != 0:
            self.status_bar.showMessage(f"AVS_MeasureCallback failed (code {res}).")
            self.measurement_active = False
            return

        # disable start button, enable stop (if you have them)
        self.start_meas_button.setEnabled(False)
        self.stop_meas_button.setEnabled(True)

        # redraw plot at 5 Hz
        self.spec_timer = QTimer(self)
        self.spec_timer.timeout.connect(self.update_spectrometer_plot)
        self.spec_timer.start(200)


    def _spectro_callback(self, handle, read_array, num_read, p_context):
        """Driver callback: capture intensities & wavelengths."""
        arr = np.ctypeslib.as_array(read_array, shape=(num_read,))
        self.intensities = arr.tolist()
        # assume linear ramp from start_nm→end_nm stored in your controller
        self.wavelengths = np.linspace(
            self.spec_ctrl.start_nm,
            self.spec_ctrl.end_nm,
            num_read
        ).tolist()
        return 0


    def update_spectrometer_plot(self):
        """tell your spectrometer widget to redraw itself."""
        if self.intensities and self.wavelengths:
            self.spec_ctrl.update_plot(self.wavelengths, self.intensities)


    def toggle_data_saving(self):
        """Start or pause the 1 Hz data-logging timer and open/close files."""
        if not self.continuous_saving:
            # starting
            if self.csv_file:
                self.csv_file.close()
            if self.log_file:
                self.log_file.close()

            ts = QDateTime.currentDateTime().toString("yyyyMMdd_hhmmss")
            self.csv_file_path = os.path.join(self.csv_dir, f"log_{ts}.csv")
            self.log_file_path = os.path.join(self.log_dir, f"log_{ts}.txt")

            try:
                self.csv_file = open(self.csv_file_path, 'w', newline='', encoding='utf-8')
                self.log_file = open(self.log_file_path, 'w', encoding='utf-8')
            except Exception as e:
                self.status_bar.showMessage(f"Failed to open files: {e}")
                return

            # write CSV header
            headers = [
                "Timestamp",
                "MotorPos_steps",
                "MotorSpeed_steps_s",
                "MotorCurrent_pct",
                "MotorAlarmCode",
                "MotorTemp_C",
                "MotorAngle_deg",
                "FilterWheel1",
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

            self.save_data_timer.start(1000)
            self.continuous_saving = True
            self.toggle_save_button.setText("Pause Saving")
            self.status_bar.showMessage(f"Started saving to {self.csv_file_path}")

        else:
            # stopping
            self.continuous_saving = False
            self.save_data_timer.stop()
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
            if self.log_file:
                self.log_file.close()
                self.log_file = None
            self.toggle_save_button.setText("Start Saving")
            self.status_bar.showMessage(f"Data saved to {self.csv_file_path}")


    def save_continuous_data(self):
        """Called once per second—pulls in all latest data and appends to CSV & TXT."""
        if not (self.csv_file and self.log_file):
            return

        try:
            now    = QDateTime.currentDateTime()
            ts_csv = now.toString("yyyy-MM-dd hh:mm:ss.zzz")
            ts_txt = now.toString("yyyy-MM-dd hh:mm:ss")

            # motor driver globals
            motor_pos         = int(getattr(self, 'current_motor_angle', 0))
            motor_speed       = getattr(motor, 'TrackerSpeed', 0)
            motor_current_pct = getattr(motor, 'TrackerCurrent', 0) / 10.0

            # IMU / GPS / Temp-controller pulled from your latest_data dict
            r, p, y   = self.latest_data.get('rpy',    (0,0,0))
            ax, ay, az = self.latest_data.get('accel', (0,0,0))
            gx, gy, gz = self.latest_data.get('gyro',  (0,0,0))
            mx, my, mz = self.latest_data.get('mag',   (0,0,0))
            pres       = self.latest_data.get('pressure',    0)
            temp       = self.latest_data.get('temperature', 0)
            lat        = self.latest_data.get('latitude',    0)
            lon        = self.latest_data.get('longitude',   0)
            integ      = getattr(self, 'current_integration_time_us', 0)
            tc_curr    = self.latest_data.get('TempController_curr', 0)
            tc_set     = self.latest_data.get('TempController_set',  0)

            # build CSV row
            row = [
                ts_csv,
                str(motor_pos),
                str(motor_speed),
                f"{motor_current_pct:.1f}",
                "", "", "",         # (alarm / motor temp / motor angle) fill if you have
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

            # write
            self.csv_file.write(",".join(row) + "\n")

            # text log: just peak intensity
            peak = max(self.intensities) if self.intensities else 0
            self.log_file.write(f"{ts_txt} | Peak {peak}\n")

        except Exception as e:
            self.status_bar.showMessage(f"Error saving data: {e}")
            # keep the timer alive so you can fix and retry


    def _update_indicators(self):
        """Green/red dot in each groupbox title if hardware is up."""
        # Motor
        col = "green" if self.motor_ctrl.is_connected() else "red"
        gb  = self.motor_ctrl.groupbox
        gb.setTitle("● Motor")
        gb.setStyleSheet(f"QGroupBox#motorGroup::title {{ color: {col}; }}")

        # Filter wheel
        col = "green" if self.filter_ctrl.is_connected() else "red"
        gb  = self.filter_ctrl.groupbox
        gb.setTitle("● Filter Wheel")
        gb.setStyleSheet(f"QGroupBox#filterwheelGroup::title {{ color: {col}; }}")

        # IMU
        col = "green" if self.imu_ctrl.is_connected() else "red"
        gb  = self.imu_ctrl.groupbox
        gb.setTitle("● IMU")
        gb.setStyleSheet(f"QGroupBox#imuGroup::title {{ color: {col}; }}")

        # Spectrometer
        col = "green" if self.spec_ctrl.is_ready() else "red"
        gb  = self.spec_ctrl.groupbox
        gb.setTitle("● Spectrometer")
        gb.setStyleSheet(f"QGroupBox#specGroup::title {{ color: {col}; }}")
