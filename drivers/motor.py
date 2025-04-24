import serial
from PyQt5.QtCore import QThread, pyqtSignal
import utils
from serial import SerialException

TrackerSpeed = 10000       # steps/s
TrackerCurrent = 1000      # in 0.1% units
SlaveID = 2                # Modbus ID
BaudRateList = [9600, 19200, 38400, 57600, 115200, 230400]
Parities = [serial.PARITY_EVEN, serial.PARITY_NONE, serial.PARITY_ODD]
StopBits = [serial.STOPBITS_ONE, serial.STOPBITS_TWO]

class MotorConnectThread(QThread):
    result_signal = pyqtSignal(object, int, str)

    def __init__(self, port_name, parent=None):
        super().__init__(parent)
        self.port_name = port_name

    def run(self):
        found_serial = None
        found_baud = 0
        message = ""
        for baud in BaudRateList:
            for parity in Parities:
                for sb in StopBits:
                    try:
                        ser = serial.Serial(
                            port=self.port_name,
                            baudrate=baud,
                            bytesize=serial.EIGHTBITS,
                            parity=parity,
                            stopbits=sb,
                            timeout=1.0
                        )
                        # Test minimal Modbus read at register 0x0000
                        test_cmd = bytes([SlaveID, 0x03, 0x00, 0x00, 0x00, 0x01])
                        crc = utils.modbus_crc16(test_cmd).to_bytes(2, 'little')
                        ser.reset_input_buffer()
                        ser.write(test_cmd + crc)
                        resp = ser.read(7)
                        if resp and len(resp) >= 5 and resp[1] == 0x03:
                            found_serial = ser
                            found_baud = baud
                            message = (
                                f"Motor connected on {self.port_name} at {baud} baud, "
                                f"parity={parity}, stopbits={sb}."
                            )
                            break
                        ser.close()
                    except SerialException:
                        continue
                if found_serial:
                    break
            if found_serial:
                break

        if not found_serial:
            message = f"No response from motor on {self.port_name}."
        self.result_signal.emit(found_serial, found_baud, message)


def send_move_command(serial_obj, angle: int) -> bool:
    base_cmd = bytes([SlaveID, 0x10, 0x00, 0x58, 0x00, 0x12, 0x24] + [0x00]*12)
    angle_bytes = angle.to_bytes(4, 'big', signed=True)
    speed_bytes = TrackerSpeed.to_bytes(4, 'big', signed=True)
    mid = bytes([0x00,0x00,0x1F,0x40,0x00,0x00,0x1F,0x40])
    current_bytes = TrackerCurrent.to_bytes(4, 'big', signed=True)
    end = bytes([0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01])
    full_cmd = base_cmd + angle_bytes + speed_bytes + mid + current_bytes + end
    crc = utils.modbus_crc16(full_cmd).to_bytes(2, 'little')
    try:
        serial_obj.reset_input_buffer()
        serial_obj.write(full_cmd + crc)
        resp = serial_obj.read(8)
        return bool(resp and len(resp) >= 6 and resp[1] == 0x10)
    except Exception:
        return False
