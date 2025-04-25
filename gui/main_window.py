import os
import numpy as np
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QGridLayout, QSplitter, QStatusBar, QPushButton
)
from PyQt5.QtCore import QTimer, Qt, QDateTime
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
        splitter.addWidget(self.spec_ctrl.groupbox)

        right = QWidget()
        grid = QGridLayout(right)
        grid.addWidget(self.motor_ctrl.groupbox, 0, 0)
        grid.addWidget(self.filter_ctrl.groupbox, 0, 1)
        grid.addWidget(self.imu_ctrl.groupbox,    1, 0)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.setRowStretch(0, 0)
        grid.setRowStretch(1, 1)

        splitter.addWidget(right)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        main_layout.addWidget(splitter)

        # Toggle save button
        self.toggle_save_button = QPushButton("Start Saving")
        self.toggle_save_button.clicked.connect(self.toggle_data_saving)
        main_layout.addWidget(self.toggle_save_button)

        # Status indicator updater
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._update_indicators)
        self.status_timer.start(1000)
        self._update_indicators()

        # ── toggle_data_saving setup ─────────────────────────────────────────
        self.csv_file     = None
        self.log_file     = None
        self.csv_dir      = "data"
        self.log_dir      = "data"
        os.makedirs(self.csv_dir, exist_ok=True)
        os.makedirs(self.log_dir, exist_ok=True)

        self.continuous_saving = False
        self.save_data_timer   = QTimer(self)
        self.save_data_timer.timeout.connect(self.save_continuous_data)

    def on_start_measurement(self):
        """Configure and start continuous spectral measurement."""
        if self.spec_handle is None or self.spec_handle == spectrometer.INVALID_AVS_HANDLE_VALUE:
            self.status_bar.showMessage("Spectrometer not connected.")
            return
        if getattr(self, 'measurement_active', False):
            return  # already measuring

        # Prepare measurement (integration time, etc.)
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

        # Start measurement callback
        self.measurement_active = True
        try:
            self.cb_func = spectrometer.AVS_MeasureCallbackFunc(self._spectro_callback)
            res = spectrometer.AVS_MeasureCallback(self.spec_handle, self.cb_func, -1)
        except Exception as e:
            self.status_bar.showMessage(f"Failed to start measurement: {e}")
            self.measurement_active = False
            return
        if res != 0:
            self.status_bar.showMessage(f"AVS_MeasureCallback failed (code {res}).")
            self.measurement_active = False
            return

        # Record integration time
        self.current_integration_time_us = int(integration_time_ms * 1000)

        # Update UI state
        self.start_meas_button.setEnabled(False)
        self.stop_meas_button.setEnabled(True)

        # Plot update timer
        self.spec_timer = QTimer(self)
        self.spec_timer.timeout.connect(self.update_spectrometer_plot)
        self.spec_timer.start(200)

    def toggle_data_saving(self):
        """Start or stop continuous data logging to CSV and text log files."""
        if not self.continuous_saving:
            # Start logging to new files
            if self.csv_file or self.log_file:
                if self.csv_file:
                    self.csv_file.close()
                if self.log_file:
                    self.log_file.close()

            timestamp = QDateTime.currentDateTime().toString("yyyyMMdd_hhmmss")
            self.csv_file_path = os.path.join(self.csv_dir, f"log_{timestamp}.csv")
            self.log_file_path = os.path.join(self.log_dir, f"log_{timestamp}.txt")
            try:
                self.csv_file = open(self.csv_file_path, 'w', newline='', encoding='utf-8')
                self.log_file = open(self.log_file_path, 'w', encoding='utf-8')
                # Write CSV header
                header_fields = [
                    "Timestamp", "MotorPos_steps", "MotorSpeed_steps_s", "MotorCurrent_pct",
                    "MotorAlarmCode", "MotorTemp_C", "MotorAngle_deg", "FilterWheel 1",
                    "Roll_deg", "Pitch_deg", "Yaw_deg",
                    "AccelX_g", "AccelY_g", "AccelZ_g", "GyroX_dps", "GyroY_dps", "GyroZ_dps",
                    "MagX_uT", "MagY_uT", "MagZ_uT", "Pressure_hPa", "Temperature_C",
                    "Latitude_deg", "Longitude_deg", "IntegrationTime_us", "TempController_curr",
                    "TempController_set"
                ]
                for wl in (self.wavelengths if isinstance(self.wavelengths, (list, np.ndarray)) else []):
                    header_fields.append(f"I_{float(wl):.2f}nm")
                self.csv_file.write(",".join(header_fields) + "\n")

                # Start periodic logging
                self.save_data_timer.start(1000)
                self.continuous_saving = True
                self.toggle_save_button.setText("Pause Saving")
                self.status_bar.showMessage(f"Started saving data to {self.csv_file_path}")
            except Exception as e:
                self.status_bar.showMessage(f"Failed to start saving: {e}")
                if self.csv_file:
                    self.csv_file.close()
                    self.csv_file = None
                if self.log_file:
                    self.log_file.close()
                    self.log_file = None
                return
        else:
            # Stop logging
            self.continuous_saving = False
            self.save_data_timer.stop()
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
            if self.log_file:
                self.log_file.close()
                self.log_file = None
            self.toggle_save_button.setText("Start Saving")
            if hasattr(self, 'csv_file_path') and hasattr(self, 'log_file_path'):
                self.status_bar.showMessage(f"Data saved to {self.csv_file_path} and {self.log_file_path}")
            else:
                self.status_bar.showMessage("Data logging stopped.")

    def save_continuous_data(self):
        """Log one data sample (sensors + spectrum) to the CSV and text files."""
        if not self.csv_file or not self.log_file:
            return
        now = QDateTime.currentDateTime()
        ts_csv = now.toString("yyyy-MM-dd hh:mm:ss.zzz")
        ts_txt = now.toString("yyyy-MM-dd hh:mm:ss")

        # Motor data
        motor_pos = int(self.current_motor_angle)
        motor_angle = float(self.current_motor_angle)
        motor_speed = motor.TrackerSpeed
        motor_current_pct = motor.TrackerCurrent / 10.0
        motor_alarm = 0
        motor_temp = None

        # IMU data
        roll, pitch, yaw = self.latest_data.get("rpy", (0.0, 0.0, 0.0))
        ax_g, ay_g, az_g = self.latest_data.get("accel", (0.0, 0.0, 0.0))
        gx_dps, gy_dps, gz_dps = self.latest_data.get("gyro", (0.0, 0.0, 0.0))
        mx_uT, my_uT, mz_uT = self.latest_data.get("mag", (0.0, 0.0, 0.0))
        pres = self.latest_data.get("pressure", 0.0)
        temp = self.latest_data.get("temperature", 0.0)
        lat = self.latest_data.get("latitude", 0.0)
        lon = self.latest_data.get("longitude", 0.0)
        integration_us = getattr(self, "current_integration_time_us", 0)
        TempController_curr = self.latest_data.get("TempController_curr", 0.0)
        TempController_set = self.latest_data.get("TempController_set", 0.0)

        # Prepare CSV row
        row = [
            ts_csv,
            str(motor_pos),
            str(motor_speed),
            f"{motor_current_pct:.1f}",
            str(motor_alarm),
            "" if motor_temp is None else f"{motor_temp:.1f}",
            f"{motor_angle:.1f}",
            str(self.current_filter_position) if hasattr(self, 'current_filter_position') else "",
            f"{roll:.2f}",
            f"{pitch:.2f}",
            f"{yaw:.2f}",
            f"{ax_g:.2f}",
            f"{ay_g:.2f}",
            f"{az_g:.2f}",
            f"{gx_dps:.2f}",
            f"{gy_dps:.2f}",
            f"{gz_dps:.2f}",
            f"{mx_uT:.2f}",
            f"{my_uT:.2f}",
            f"{mz_uT:.2f}",
            f"{pres:.2f}",
            f"{temp:.2f}",
            f"{lat:.6f}",
            f"{lon:.6f}",
            str(int(integration_us)),
            f"{TempController_curr:.2f}",
            f"{TempController_set:.2f}"        
        ]
        # Append intensities
        if hasattr(self, 'intensities') and hasattr(self, 'wavelengths') and \
           isinstance(self.intensities, (list, np.ndarray)) and \
           len(self.intensities) == len(self.wavelengths):
            for inten in self.intensities:
                row.append(f"{inten:.4f}" if inten != 0 else "")
        else:
            for _ in (self.wavelengths if hasattr(self, 'wavelengths') else []):
                row.append("")

        # Write CSV
        try:
            self.csv_file.write(",".join(row) + "\n")
        except Exception as e:
            self.status_bar.showMessage(f"Error during data save: {e}")

        # Prepare text log line
        if hasattr(self, 'intensities') and len(self.intensities) > 0:
            max_int = max(self.intensities)
            if max_int != 0:
                idx = self.intensities.index(max_int) if isinstance(self.intensities, list) else int(np.argmax(self.intensities))
                peak_text = f"Peak {max_int:.1f} at {self.wavelengths[idx]:.1f} nm"
            else:
                peak_text = "Peak 0 at N/A nm"
        else:
            peak_text = "Peak 0 at N/A nm"
        motor_temp_text = f"{motor_temp:.1f}°C" if motor_temp is not None else "N/A"
        log_line = (
            f"Time {ts_txt}: Motor at {motor_pos} steps ({motor_angle:.0f}°), "
            f"Speed {motor_speed} steps/s, Current {motor_current_pct:.1f}%, "
            f"Temp {motor_temp_text}, AlarmCode {motor_alarm}; "
            f"Spectrometer {peak_text}; IMU Orientation: Roll {roll:.1f}°, "
            f"Pitch {pitch:.1f}°, Yaw {yaw:.1f}°; GPS: Lat {lat:.6f}°, Lon {lon:.6f}°."        
        )
        try:
            self.log_file.write(log_line + "\n")
        except Exception as e:
            self.status_bar.showMessage(f"Error writing log: {e}")

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
