from PyQt5.QtCore import QObject, pyqtSignal, QTimer
from PyQt5.QtWidgets import QGroupBox, QLabel, QComboBox, QPushButton, QLineEdit, QGridLayout
from drivers.motor import MotorConnectThread, send_move_command
from serial.tools import list_ports

class MotorController(QObject):
    status_signal = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.groupbox = QGroupBox("Motor")
        self.groupbox.setObjectName("motorGroup")
        layout = QGridLayout()
        layout.addWidget(QLabel("COM:"), 0, 0)
        
        # Port selector
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        ports = [p.device for p in list_ports.comports()]
        self.port_combo.addItems(ports or [f"COM{i}" for i in range(1, 10)])
        self.port_combo.setCurrentText("COM11")
        layout.addWidget(self.port_combo, 0, 1)

        # Connect button
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect)
        layout.addWidget(self.connect_btn, 0, 2)

        # Angle input and Move button
        layout.addWidget(QLabel("Angle (°):"), 1, 0)
        self.angle_input = QLineEdit()
        self.angle_input.setFixedWidth(60)
        layout.addWidget(self.angle_input, 1, 1)
        self.move_btn = QPushButton("Move")
        self.move_btn.setEnabled(False)
        self.move_btn.clicked.connect(self.move)
        layout.addWidget(self.move_btn, 1, 2)

        self.groupbox.setLayout(layout)

        # Internal state
        self._connected = False
        self.serial = None
        self._connect_thread = None

        # Try initial connection after UI is ready
        QTimer.singleShot(0, self.connect)

    def connect(self):
        port = self.port_combo.currentText().strip()
        self.connect_btn.setEnabled(False)

        # Clean up any previous thread
        if self._connect_thread and self._connect_thread.isRunning():
            self._connect_thread.quit()
            self._connect_thread.wait()

        # Start new connection thread
        self._connect_thread = MotorConnectThread(port, parent=self)
        self._connect_thread.result_signal.connect(self._on_connect)
        self._connect_thread.start()

    def _on_connect(self, ser, baud, msg):
        self.connect_btn.setEnabled(True)
        self.status_signal.emit(msg)

        # Close old serial if replacing
        if ser and self.serial and self.serial is not ser:
            try:
                self.serial.close()
            except Exception:
                pass

        if ser:
            self.serial = ser
            self._connected = True
            self.move_btn.setEnabled(True)
        else:
            self._connected = False
            self.move_btn.setEnabled(False)

    def move(self):
        if not self._connected:
            self.status_signal.emit("Not connected")
            return
        try:
            angle = int(self.angle_input.text().strip())
        except ValueError:
            self.status_signal.emit("Invalid angle")
            return

        ok = send_move_command(self.serial, angle)
        self.status_signal.emit("Moved" if ok else "No ACK or error")

    def is_connected(self):
        return self._connected
