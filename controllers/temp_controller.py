from PyQt5.QtCore import QObject, pyqtSignal, QTimer
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLabel, QLineEdit, QPushButton
from drivers.tc36_25_driver import TC36_25

class TempController(QObject):
    status_signal = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.widget = QWidget()
        h = QHBoxLayout()
        h.addWidget(QLabel("Set Temp (°C):"))
        self.input = QLineEdit("20.0"); self.input.setFixedWidth(60)
        h.addWidget(self.input)
        btn = QPushButton("Send"); btn.clicked.connect(self._send)
        h.addWidget(btn); h.addSpacing(20)
        h.addWidget(QLabel("Current:")); self.cur_lbl=QLabel("-- °C"); h.addWidget(self.cur_lbl)
        h.addStretch(); self.widget.setLayout(h)
        try:
            self.tc = TC36_25()
            self.tc.enable_computer_setpoint(); self.tc.power(True)
            self.timer = QTimer(self); self.timer.timeout.connect(self._upd); self.timer.start(1000)
        except Exception as e: self.status_signal.emit(f"TC init failed: {e}")

    def _send(self):
        try:
            t=float(self.input.text()); self.tc.set_setpoint(t)
            self.status_signal.emit(f"SP={t:.1f}°C")
        except Exception as e: self.status_signal.emit(f"Set fail: {e}")

    def _upd(self):
        try:
            v=self.tc.get_temperature(); self.cur_lbl.setText(f"{v:.2f} °C")
        except Exception as e:
            self.cur_lbl.setText("-- °C"); self.status_signal.emit(f"Read err: {e}")

    def is_connected(self): return True