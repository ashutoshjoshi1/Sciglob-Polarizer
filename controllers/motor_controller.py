from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import QGroupBox, QLabel, QComboBox, QPushButton, QLineEdit, QGridLayout
from serial.tools import list_ports
import motor

class MotorController(QObject):
    status_signal = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.groupbox = QGroupBox("Motor")
        self.groupbox.setObjectName("motorGroup")
        layout = QGridLayout()
        layout.addWidget(QLabel("Motor COM:"), 0, 0)
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        ports = [p.device for p in list_ports.comports()]
        self.port_combo.addItems(ports or [f"COM{i}" for i in range(1,10)])
        layout.addWidget(self.port_combo, 0, 1)
        self.connect_btn = QPushButton("Connect Motor")
        self.connect_btn.clicked.connect(self.connect)
        layout.addWidget(self.connect_btn, 0, 2)
        layout.addWidget(QLabel("Angle (°):"), 1, 0)
        self.angle_input = QLineEdit()
        self.angle_input.setFixedWidth(60)
        layout.addWidget(self.angle_input, 1, 1)
        self.move_btn = QPushButton("Move")
        self.move_btn.setEnabled(False)
        self.move_btn.clicked.connect(self.move)
        layout.addWidget(self.move_btn, 1, 2)
        self.groupbox.setLayout(layout)

        self._connected = False
        self.serial = None

    def connect(self):
        port = self.port_combo.currentText().strip()
        self.connect_btn.setEnabled(False)
        thread = motor.MotorConnectThread(port, parent=self)
        thread.result_signal.connect(self._on_connect_result)
        thread.start()

    def _on_connect_result(self, ser, baud, message):
        self.connect_btn.setEnabled(True)
        self.status_signal.emit(message)
        if ser and ser.is_open:
            self.serial = ser
            self._connected = True
            self.move_btn.setEnabled(True)
        else:
            self._connected = False
            self.move_btn.setEnabled(False)

    def move(self):
        if not self._connected or not self.serial:
            self.status_signal.emit("Motor not connected.")
            return
        try:
            angle = int(self.angle_input.text().strip())
        except ValueError:
            self.status_signal.emit("Invalid angle input.")
            return
        ok = motor.send_move_command(self.serial, angle)
        msg = f"Motor moved to angle {angle}°." if ok else "No ACK from motor."
        self.status_signal.emit(msg)

    def is_connected(self):
        return self._connected