from PyQt5.QtCore import QObject, pyqtSignal, QTimer, QDateTime
from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QPushButton
import pyqtgraph as pg
import os

from drivers.spectrometer import (
    connect_spectrometer,
    prepare_measurement,
    AVS_MeasureCallbackFunc,
    AVS_MeasureCallback,
    AVS_GetScopeData,
    StopMeasureThread
)
import numpy as np

class SpectrometerController(QObject):
    status_signal = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        # ——— Group box & layout ———
        self.groupbox = QGroupBox("Spectrometer")
        self.groupbox.setObjectName("specGroup")

        main_layout = QVBoxLayout()
        ctrl_layout = QHBoxLayout()

        # Continuous‐save toggle (not yet implemented)
        self.toggle_btn = QPushButton("Start Saving")
        self.toggle_btn.setEnabled(False)
        self.toggle_btn.clicked.connect(self.toggle)
        ctrl_layout.addWidget(self.toggle_btn)

        # Connect button
        self.conn_btn = QPushButton("Connect")
        self.conn_btn.clicked.connect(self.connect)
        ctrl_layout.addWidget(self.conn_btn)

        # Start / Stop / Save
        self.start_btn = QPushButton("Start")
        self.start_btn.setEnabled(False)
        self.start_btn.clicked.connect(self.start)
        ctrl_layout.addWidget(self.start_btn)

        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.stop)
        ctrl_layout.addWidget(self.stop_btn)

        self.save_btn = QPushButton("Save")
        self.save_btn.setEnabled(False)
        self.save_btn.clicked.connect(self.save)
        ctrl_layout.addWidget(self.save_btn)

        main_layout.addLayout(ctrl_layout)

        # ——— Spectral plot ———
        pg.setConfigOption('background','w')
        pg.setConfigOption('foreground','k')
        self.plot = pg.PlotWidget()
        self.plot.setLabel('bottom', 'Wavelength', 'nm')
        self.plot.setLabel('left', 'Intensity', 'counts')
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.curve = self.plot.plot([], [], pen=pg.mkPen('#2986cc', width=1))
        main_layout.addWidget(self.plot)

        self.groupbox.setLayout(main_layout)

        # ——— Internal state ———
        self._ready = False
        self.handle = None
        self.wls = []
        self.intens = []
        self.npix = 0

        self.csv_dir = "data"
        os.makedirs(self.csv_dir, exist_ok=True)

    def connect(self):
        # Emit status so you get feedback
        self.status_signal.emit("Connecting to spectrometer...")
        try:
            handle, wavelengths, num_pixels, serial_str = connect_spectrometer()
        except Exception as e:
            self.status_signal.emit(f"Connection failed: {e}")
            return

        self.handle = handle
        self.wls = wavelengths.tolist() if isinstance(wavelengths, np.ndarray) else wavelengths
        self.npix = num_pixels
        self._ready = True
        self.start_btn.setEnabled(True)
        self.status_signal.emit(f"Spectrometer ready (SN={serial_str})")

    def start(self):
        if not self._ready:
            self.status_signal.emit("Spectrometer not ready.")
            return

        code = prepare_measurement(self.handle, self.npix, integration_time_ms=50.0, averages=1)
        if code != 0:
            self.status_signal.emit(f"Prepare error: {code}")
            return

        self.measure_active = True
        self.cb = AVS_MeasureCallbackFunc(self._cb)
        err = AVS_MeasureCallback(self.handle, self.cb, -1)
        if err != 0:
            self.status_signal.emit(f"Callback error: {err}")
            self.measure_active = False
            return

        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.status_signal.emit("Measurement started")

        # Update plot every 200 ms
        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self._update_plot)
        self.plot_timer.start(200)

    def _cb(self, p_data, p_user):
        # Only update on successful scan (status_code == 0)
        status_code = p_user[0]
        if status_code == 0:
            _, data = AVS_GetScopeData(self.handle)
            full = [0.0] * self.npix
            full[:len(data)] = data
            self.intens = full
            self.save_btn.setEnabled(True)
            self.toggle_btn.setEnabled(True)
        else:
            self.status_signal.emit(f"Spectrometer error code {status_code}")

    def _update_plot(self):
        if not hasattr(self, 'intens'):
            return

        self.curve.setData(self.wls, self.intens)

    def stop(self):
        if not getattr(self, 'measure_active', False):
            return

        self.stop_btn.setEnabled(False)
        stopper = StopMeasureThread(self.handle, parent=self)
        stopper.finished_signal.connect(self._on_stopped)
        stopper.start()

    def _on_stopped(self):
        self.measure_active = False
        self.start_btn.setEnabled(True)
        self.status_signal.emit("Measurement stopped")

    def save(self):
        ts = QDateTime.currentDateTime().toString("yyyyMMdd_hhmmss")
        path = os.path.join(self.csv_dir, f"snapshot_{ts}.csv")
        try:
            with open(path, 'w') as f:
                f.write("Wavelength (nm),Intensity\n")
                for wl, inten in zip(self.wls, self.intens):
                    if inten != 0:
                        f.write(f"{wl:.4f},{inten:.4f}\n")
            self.status_signal.emit(f"Saved snapshot to {path}")
        except Exception as e:
            self.status_signal.emit(f"Save error: {e}")

    def toggle(self):
        # stub for continuous‐save implementation
        self.status_signal.emit("Continuous‐save not yet implemented")

    def is_ready(self):
        return self._ready
