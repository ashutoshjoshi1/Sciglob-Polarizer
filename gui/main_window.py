# gui/main_window.py

import sys
import os
import math
import cv2
import numpy as np
from datetime import datetime, timezone
from serial.tools import list_ports

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QLineEdit, QPushButton, QComboBox, QSplitter, QStatusBar
)
from PyQt5.QtCore import QTimer, Qt, QDateTime
from PyQt5.QtGui import QPixmap, QImage

import pyqtgraph as pg
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# — your drivers now live under drivers/
from drivers import motor, imu, spectrometer, filterwheel
from controllers.temp_controller import TC36_25
import utils


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Spectrometer, Motor, IMU & Temperature Control")
        self.setMinimumSize(1000, 750)

        # ── Status bar ──────────────────────────────────────────────────────────
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

        # ── Central widget & layout ────────────────────────────────────────────
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(5,5,5,5)
        main_layout.setSpacing(8)

        # ── Logo ────────────────────────────────────────────────────────────────
        logo_layout = QHBoxLayout()
        logo_layout.addStretch()
        logo = QLabel()
        pix = QPixmap("asset/sciglob_symbol.png")
        if not pix.isNull():
            logo.setPixmap(pix.scaled(50,50, Qt.KeepAspectRatio|Qt.SmoothTransformation))
        logo_layout.addWidget(logo)
        main_layout.addLayout(logo_layout)

        # ── Temperature Control ─────────────────────────────────────────────────
        temp_layout = QHBoxLayout(); temp_layout.setSpacing(10)
        temp_layout.addWidget(QLabel("Set Temp (°C):"))
        self.set_temp_input = QLineEdit("20.0"); self.set_temp_input.setFixedWidth(60)
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

        # init temp controller
        try:
            self.tc = TC36_25()          # from drivers/tc36_25_driver via controller
            self.tc.enable_computer_setpoint()
            self.tc.power(True)
            self.last_setpoint = None
            self.temp_timer = QTimer(self)
            self.temp_timer.timeout.connect(self.update_temperature)
            self.temp_timer.start(1000)
        except Exception as e:
            self.status_bar.showMessage(f"Temp init failed: {e}")

        # ── Spectrometer Controls & Plot ────────────────────────────────────────
        spectro_controls = QHBoxLayout()
        self.toggle_save_button = QPushButton("Start Saving")
        self.toggle_save_button.setEnabled(False)
        self.toggle_save_button.clicked.connect(self.toggle_data_saving)
        spectro_controls.addWidget(self.toggle_save_button)

        self.connect_spec_button = QPushButton("Connect Spectrometer")
        self.connect_spec_button.clicked.connect(self.on_connect_spectrometer)
        spectro_controls.addWidget(self.connect_spec_button)

        self.start_meas_button = QPushButton("Start Measurement")
        self.start_meas_button.setEnabled(False)
        self.start_meas_button.clicked.connect(self.on_start_measurement)
        spectro_controls.addWidget(self.start_meas_button)

        self.stop_meas_button = QPushButton("Stop")
        self.stop_meas_button.setEnabled(False)
        self.stop_meas_button.clicked.connect(self.on_stop_measurement)
        spectro_controls.addWidget(self.stop_meas_button)

        pg.setConfigOption('background','w')
        pg.setConfigOption('foreground','k')
        self.spec_plot = pg.PlotWidget()
        self.spec_plot.setLabel('bottom','Wavelength','nm')
        self.spec_plot.setLabel('left','Intensity','counts')
        self.spec_plot.showGrid(x=True,y=True,alpha=0.3)
        self.spec_plot.setXRange(260,600)
        self.spec_curve = self.spec_plot.plot([],[], pen=pg.mkPen('#2986cc',width=1))

        self.spectro_group = QGroupBox("Spectrometer")
        sp_layout = QVBoxLayout()
        sp_layout.addLayout(spectro_controls)
        sp_layout.addWidget(self.spec_plot, stretch=1)
        self.spectro_group.setLayout(sp_layout)

        # ── Motor Controls ─────────────────────────────────────────────────────
        ports = list_ports.comports()
        motor_layout = QGridLayout()
        motor_layout.addWidget(QLabel("Motor COM:"),0,0)
        self.motor_port_combo = QComboBox(); self.motor_port_combo.setEditable(True)
        for p in ports: self.motor_port_combo.addItem(getattr(p,'name',p.device))
        if self.motor_port_combo.count()==0:
            for i in range(1,10): self.motor_port_combo.addItem(f"COM{i}")
        motor_layout.addWidget(self.motor_port_combo,0,1)
        self.motor_connect_button=QPushButton("Connect Motor")
        self.motor_connect_button.clicked.connect(self.on_connect_motor)
        motor_layout.addWidget(self.motor_connect_button,0,2)

        motor_layout.addWidget(QLabel("Angle (°):"),1,0)
        self.angle_input=QLineEdit(); self.angle_input.setFixedWidth(60)
        motor_layout.addWidget(self.angle_input,1,1)
        self.move_button=QPushButton("Move"); self.move_button.setEnabled(False)
        self.move_button.clicked.connect(self.on_move_motor)
        motor_layout.addWidget(self.move_button,1,2)

        self.motor_group=QGroupBox("Motor"); self.motor_group.setLayout(motor_layout)

        # ── Filter Wheel Controls ───────────────────────────────────────────────
        filter_layout=QHBoxLayout()
        filter_layout.addWidget(QLabel("Filterwheel COM:"))
        self.filter_port_combo=QComboBox(); self.filter_port_combo.setEditable(True)
        for p in ports: self.filter_port_combo.addItem(getattr(p,'name',p.device))
        if self.filter_port_combo.count()==0:
            for i in range(1,10): self.filter_port_combo.addItem(f"COM{i}")
        self.filter_port_combo.setCurrentText("COM17")
        filter_layout.addWidget(self.filter_port_combo)

        filter_layout.addWidget(QLabel("Cmd:"))
        self.filter_cmd_input=QLineEdit(); self.filter_cmd_input.setPlaceholderText("F1r, F15, ?")
        filter_layout.addWidget(self.filter_cmd_input)
        self.filter_send_button=QPushButton("Send")
        self.filter_send_button.clicked.connect(self.on_send_filter)
        filter_layout.addWidget(self.filter_send_button)
        filter_layout.addWidget(QLabel("Pos:"))
        self.filter_pos_label=QLabel("--")
        filter_layout.addWidget(self.filter_pos_label)
        filter_layout.addStretch()
        self.filter_group=QGroupBox("Filter Wheel"); self.filter_group.setLayout(filter_layout)

        # ── IMU Controls & 3D Plot ─────────────────────────────────────────────
        imu_layout=QHBoxLayout()
        imu_layout.addWidget(QLabel("IMU COM:"))
        self.imu_port_combo=QComboBox(); self.imu_port_combo.setEditable(True)
        for p in ports: self.imu_port_combo.addItem(getattr(p,'name',p.device))
        imu_layout.addWidget(self.imu_port_combo)
        imu_layout.addWidget(QLabel("Baud:"))
        self.imu_baud_combo=QComboBox()
        for b in (9600,57600,115200): self.imu_baud_combo.addItem(str(b))
        self.imu_baud_combo.setCurrentText("9600")
        imu_layout.addWidget(self.imu_baud_combo)
        self.imu_connect_button=QPushButton("Connect IMU")
        self.imu_connect_button.clicked.connect(self.on_connect_imu)
        imu_layout.addWidget(self.imu_connect_button)

        self.imu_data_label=QLabel("IMU: not connected")
        self.figure=utils.create_3d_figure()     # you can factor out a helper
        self.canvas=FigureCanvas(self.figure)

        self.imu_group=QGroupBox("IMU")
        imu_group_layout=QVBoxLayout()
        imu_group_layout.addLayout(imu_layout)
        imu_group_layout.addWidget(self.imu_data_label)
        imu_group_layout.addWidget(self.canvas)
        self.imu_group.setLayout(imu_group_layout)

        # ── assemble right panel & splitter ────────────────────────────────────
        right_container=QWidget()
        right_layout=QVBoxLayout(right_container)
        right_layout.setContentsMargins(0,0,0,0)
        right_layout.setSpacing(10)
        right_layout.addWidget(self.motor_group)
        right_layout.addWidget(self.filter_group)
        right_layout.addWidget(self.imu_group)

        splitter=QSplitter(Qt.Horizontal)
        splitter.addWidget(self.spectro_group)
        splitter.addWidget(right_container)
        splitter.setStretchFactor(0,3); splitter.setStretchFactor(1,2)
        main_layout.addWidget(splitter, stretch=1)

        # ── data-logging state ─────────────────────────────────────────────────
        self.csv_dir="data"; self.log_dir="data"
        os.makedirs(self.csv_dir, exist_ok=True)
        os.makedirs(self.log_dir, exist_ok=True)
        self.csv_file=self.log_file=None
        self.continuous_saving=False
        self.save_data_timer=QTimer(self)
        self.save_data_timer.timeout.connect(self.save_continuous_data)

        # initial placeholders
        self.spec_handle=None
        self.measurement_active=False
        self.wavelengths=[]
        self.intensities=[]
        self.num_pixels=0
        self.current_motor_angle=0.0
        self.current_filter_position=None
        self.latest_data={"accel":(0,0,0),"gyro":(0,0,0),"mag":(0,0,0),
                          "rpy":(0,0,0),"pressure":0.0,"temperature":0.0,
                          "latitude":0.0,"longitude":0.0}

    # ── Temperature Handlers ────────────────────────────────────────────────
    def on_send_temp(self):
        try:
            sp = float(self.set_temp_input.text().strip())
            self.tc.set_setpoint(sp)
            self.last_setpoint=sp
            self.status_bar.showMessage(f"Setpoint→{sp:.2f}°C")
        except Exception as e:
            self.status_bar.showMessage(f"Temp set failed: {e}")

    def update_temperature(self):
        try:
            t=self.tc.get_temperature()
            self.current_temp_label.setText(f"{t:.2f}°C")
        except Exception as e:
            self.status_bar.showMessage(f"Temp read failed: {e}")

    # ── Spectrometer Handlers ───────────────────────────────────────────────
    def on_connect_spectrometer(self):
        self.status_bar.showMessage("Connecting…")
        try:
            h, wls, npix, serial_str = spectrometer.connect_spectrometer()
        except Exception as e:
            return self.status_bar.showMessage(str(e))
        self.spec_handle=h
        self.wavelengths = wls.tolist() if hasattr(wls,"tolist") else wls
        self.num_pixels  = npix
        self.start_meas_button.setEnabled(True)
        self.status_bar.showMessage(f"Spectrometer ready ({serial_str})")

    def on_start_measurement(self):
        if not self.spec_handle: return self.status_bar.showMessage("No spectrometer")
        if self.measurement_active: return
        it_ms=50.0
        if spectrometer.prepare_measurement(self.spec_handle,self.num_pixels,it_ms,averages=1)!=0:
            return self.status_bar.showMessage("Prep failed")
        self.measurement_active=True
        self.current_integration_time_us=int(it_ms*1000)
        cb=spectrometer.AVS_MeasureCallbackFunc(self._spectro_callback)
        if spectrometer.AVS_MeasureCallback(self.spec_handle,cb,-1)!=0:
            self.measurement_active=False
            return self.status_bar.showMessage("Callback failed")
        self.start_meas_button.setEnabled(False)
        self.stop_meas_button.setEnabled(True)
        self.spec_timer=QTimer(self); self.spec_timer.timeout.connect(self.update_spectrometer_plot)
        self.spec_timer.start(200)

    def _spectro_callback(self,hdl,arr,num,pctx):
        a=np.ctypeslib.as_array(arr,shape=(num,))
        self.intensities=a.tolist()
        # wavelengths already known from connect
        return 0

    def update_spectrometer_plot(self):
        if self.intensities and self.wavelengths:
            self.spec_curve.setData(self.wavelengths,self.intensities)

    def on_stop_measurement(self):
        if not self.measurement_active: return
        spectrometer.stop_measurement(self.spec_handle)
        self.measurement_active=False
        self.start_meas_button.setEnabled(True)
        self.stop_meas_button.setEnabled(False)
        if hasattr(self,'spec_timer'): self.spec_timer.stop()

    # ── Motor Handlers ─────────────────────────────────────────────────────
    def on_connect_motor(self):
        port=self.motor_port_combo.currentText().strip()
        if not port: return self.status_bar.showMessage("Select motor COM")
        self.motor_connect_button.setEnabled(False)
        self.motor_thread=motor.MotorConnectThread(port)
        self.motor_thread.result_signal.connect(self.on_motor_connect_result)
        self.motor_thread.start()

    def on_motor_connect_result(self,ser,baud,msg):
        self.motor_connect_button.setEnabled(True)
        if ser and ser.is_open:
            self.motor_serial=ser; self.motor_connected=True; self.move_button.setEnabled(True)
        else:
            self.motor_serial=None; self.motor_connected=False; self.move_button.setEnabled(False)
        self.status_bar.showMessage(msg)

    def on_move_motor(self):
        if not getattr(self,'motor_connected',False):
            return self.status_bar.showMessage("Motor not connected")
        try:
            angle=int(self.angle_input.text().strip())
        except:
            return self.status_bar.showMessage("Bad angle")
        ok=motor.send_move_command(self.motor_serial,angle)
        if ok:
            self.current_motor_angle=angle
            self.status_bar.showMessage(f"Motor→{angle}°")
        else:
            self.status_bar.showMessage("No ACK")

    # ── Filter Handlers ────────────────────────────────────────────────────
    def on_send_filter(self):
        cmd=self.filter_cmd_input.text().strip()
        if not cmd: return self.status_bar.showMessage("Enter cmd")
        port=self.filter_port_combo.currentText().strip()
        if not port: return self.status_bar.showMessage("Select filter COM")
        # (re)open if needed
        if not getattr(self,'filterwheel_connected',False):
            try:
                self.filterwheel_serial=filterwheel.open_port(port)
                self.filterwheel_connected=True
            except Exception as e:
                return self.status_bar.showMessage(f"Open fail: {e}")
        self.filter_cmd_thread=filterwheel.FilterWheelCommandThread(self.filterwheel_serial,cmd)
        self.filter_cmd_thread.result_signal.connect(self.on_filter_command_result)
        self.filter_cmd_thread.start()
        self.filter_send_button.setEnabled(False)

    def on_filter_command_result(self,pos,msg):
        self.filter_send_button.setEnabled(True)
        if pos is not None:
            self.current_filter_position=pos
            self.filter_pos_label.setText(str(pos))
        self.status_bar.showMessage(msg)

    # ── IMU Handlers ──────────────────────────────────────────────────────
    def on_connect_imu(self):
        port=self.imu_port_combo.currentText().strip()
        baud=int(self.imu_baud_combo.currentText().strip() or 9600)
        try:
            self.imu_serial=imu.open_port(port,baud)
        except Exception as e:
            return self.status_bar.showMessage(f"IMU open fail: {e}")
        self.imu_connected=True
        self.imu_stop_event=imu.start_imu_read_thread(self.imu_serial,self.latest_data)
        self.imu_timer=QTimer(self); self.imu_timer.timeout.connect(self.update_imu_visualization)
        self.imu_timer.start(100)

    def update_imu_visualization(self):
        r,p,y=self.latest_data.get("rpy",(0,0,0))
        self.imu_data_label.setText(f"Roll={r:.1f}° Pitch={p:.1f}° Yaw={y:.1f}°")
        utils.draw_orientation(self.figure.axes[0],r,p,y)
        self.canvas.draw()

    # ── Data Logging ───────────────────────────────────────────────────────
    def toggle_data_saving(self):
        if not self.continuous_saving:
            # open new files + write header
            ts=QDateTime.currentDateTime().toString("yyyyMMdd_hhmmss")
            self.csv_file=open(f"{self.csv_dir}/log_{ts}.csv","w",encoding="utf-8",newline="")
            self.log_file=open(f"{self.log_dir}/log_{ts}.txt","w",encoding="utf-8")
            # header
            fields=[
                "Timestamp","MotorAngle_deg","MotorSpeed_sps","MotorCurrent_pct",
                "FilterPos","Roll_deg","Pitch_deg","Yaw_deg",
                "AccelX_g","AccelY_g","AccelZ_g",
                "GyroX_dps","GyroY_dps","GyroZ_dps",
                "MagX_uT","MagY_uT","MagZ_uT",
                "Pressure_hPa","Temp_C","Set_C",
                "Latitude","Longitude","Integration_us"
            ]
            for wl in self.wavelengths:
                fields.append(f"I_{wl:.1f}nm")
            self.csv_file.write(",".join(fields)+"\n")
            self.save_data_timer.start(1000)
            self.continuous_saving=True
            self.toggle_save_button.setText("Pause Saving")
            self.status_bar.showMessage("Logging…")
        else:
            self.save_data_timer.stop()
            self.csv_file.close(); self.log_file.close()
            self.continuous_saving=False
            self.toggle_save_button.setText("Start Saving")
            self.status_bar.showMessage("Logging stopped.")

    def save_continuous_data(self):
        # gather all live values
        now=QDateTime.currentDateTime()
        t_csv=now.toString("yyyy-MM-dd hh:mm:ss.zzz")
        # motor
        angle=self.current_motor_angle
        speed=getattr(motor,"TrackerSpeed",0)
        curr=getattr(motor,"TrackerCurrent",0)/10.0
        # filter
        fpos=self.current_filter_position or 0
        # imu
        r,p,y=self.latest_data.get("rpy",(0,0,0))
        ax,ay,az=self.latest_data.get("accel",(0,0,0))
        gx,gy,gz=self.latest_data.get("gyro",(0,0,0))
        mx,my,mz=self.latest_data.get("mag",(0,0,0))
        pres=self.latest_data.get("pressure",0)
        # temp
        tc=self.tc.get_temperature() if hasattr(self.tc,"get_temperature") else 0
        sp=self.last_setpoint or ""
        lat=self.latest_data.get("latitude",0)
        lon=self.latest_data.get("longitude",0)
        integ=self.current_integration_time_us
        # build row
        row=[
            t_csv,
            f"{angle:.1f}",str(speed),f"{curr:.1f}",
            str(fpos),
            f"{r:.2f}",f"{p:.2f}",f"{y:.2f}",
            f"{ax:.2f}",f"{ay:.2f}",f"{az:.2f}",
            f"{gx:.2f}",f"{gy:.2f}",f"{gz:.2f}",
            f"{mx:.2f}",f"{my:.2f}",f"{mz:.2f}",
            f"{pres:.2f}",f"{tc:.2f}",str(sp),
            f"{lat:.6f}",f"{lon:.6f}",str(int(integ))
        ]
        # intensities
        for inten in self.intensities:
            row.append(f"{inten:.4f}" if inten else "")
        # write files
        self.csv_file.write(",".join(row)+"\n")
        self.csv_file.flush()
        # text log
        peak=max(self.intensities) if self.intensities else 0
        self.log_file.write(f"{t_csv} | Peak {peak:.1f}\n")
        self.log_file.flush()

if __name__=="__main__":
    app=QApplication(sys.argv)
    w=MainWindow(); w.show()
    sys.exit(app.exec_())
