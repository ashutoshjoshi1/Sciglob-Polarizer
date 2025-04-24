from PyQt5.QtCore import QObject, pyqtSignal, QTimer
from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLabel, QLineEdit, QPushButton
from temp_controller import TC36_25

class TempController(QObject):
    status_signal = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.widget = QWidget()
        layout = QHBoxLayout()
        layout.addWidget(QLabel("Set Temp (°C):"))
        self.set_temp_input = QLineEdit("20.0")
        self.set_temp_input.setFixedWidth(60)
        layout.addWidget(self.set_temp_input)
        self.send_temp_btn = QPushButton("Send Cmd")
        self.send_temp_btn.clicked.connect(self.send_setpoint)
        layout.addWidget(self.send_temp_btn)
        layout.addSpacing(20)
        layout.addWidget(QLabel("Current Temp:"))
        self.current_label = QLabel("-- °C")
        layout.addWidget(self.current_label)
        layout.addStretch()
        self.widget.setLayout(layout)

        try:
            self.tc = TC36_25()
            self.tc.enable_computer_setpoint()
            self.tc.power(True)
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.update_temperature)
            self.timer.start(1000)
        except Exception as e:
            self.status_signal.emit(f"Temp ctrl init failed: {e}")

    def send_setpoint(self):
        try:
            temp = float(self.set_temp_input.text().strip())
            self.tc.set_setpoint(temp)
            self.status_signal.emit(f"Setpoint updated to {temp:.2f} °C")
        except Exception as e:
            self.status_signal.emit(f"Failed to set temperature: {e}")

    def update_temperature(self):
        try:
            t = self.tc.get_temperature()
            self.current_label.setText(f"{t:.2f} °C")
        except Exception as e:
            self.current_label.setText("-- °C")
            self.status_signal.emit(f"Error reading temperature: {e}")

    def is_connected(self):
        return True  # always available