import serial
import struct
import time


# ================== Protocol Constants ==================

MESSAGE_HEAD = 0x55
ACTIVE_REPORT = 0x04

REPORT_RADAR = 0x03
REPORT_OTHER = 0x05
REPORT_FALL = 0x06

HEARTBEAT = 0x01
ABNORMAL = 0x02
ENVIRONMENT = 0x05
BODYSIGN = 0x06
CLOSE_AWAY = 0x07

CA_BE = 0x01
CA_CLOSE = 0x02
CA_AWAY = 0x03

SOMEBODY_BE = 0x01
SOMEBODY_MOVE = 0x01
SOMEBODY_STOP = 0x00
NOBODY = 0x00

ALARM = 0x01
ALARM_FALL = 0x01
ALARM_RESIDUAL = 0x02

SUSPECTED_FALL = 0x00
REAL_FALL = 0x01
NO_FALL = 0x02


# ================== CRC TABLES ==================

cuc_CRCHi = [  # paste your full 256 values here
    0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41, ...
]

cuc_CRCLo = [
    0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2, ...
]


# ================== Main Class ==================

class MR24FDB1Radar:
    """
    Python driver for Seeed Studio MR24FDB1 mmWave Radar Sensor (Fall Detection firmware profile)
    """

    def __init__(self, port: str, baudrate: int = 9600, timeout: float = 0.1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

        self.data_len = 12
        self.msg = bytearray(12)
        self.new_data = False

        self._recv_in_progress = False
        self._ndx = 0

    def read_frame(self):
        """Read bytes from UART and assemble a frame"""
        while self.ser.in_waiting and not self.new_data:
            rb = self.ser.read(1)[0]

            if self._recv_in_progress:
                if self.data_len > self._ndx:
                    self.msg[self._ndx] = rb
                    if self._ndx == 0:
                        self.data_len = self.msg[0]
                    self._ndx += 1
                else:
                    self._recv_in_progress = False
                    self._ndx = 0
                    self.new_data = True
            elif rb == MESSAGE_HEAD:
                self._recv_in_progress = True

    def get_message(self):
        """Return full message including header"""
        if not self.new_data:
            return None

        data = bytearray(self.data_len + 1)
        data[0] = MESSAGE_HEAD

        for i in range(self.data_len):
            data[i + 1] = self.msg[i]

        self.new_data = False
        return data

    def show_data(self, data: bytes):
        print(" ".join(f"{b:02X}" for b in data))

    # ================== Decoders ==================

    def parse_bodysign(self, data: bytes, move_min=1.0, move_max=15.0):
        if data[3] == ACTIVE_REPORT and data[5] == BODYSIGN:
            value = struct.unpack('<f', bytes(data[6:10]))[0]

            self.show_data(data)
            print(f"BodySign: {value:.2f}")

            if value < move_min:
                return "NOBODY"
            elif value < move_max:
                return "SOMEBODY_STOP"
            else:
                return "SOMEBODY_MOVE"

    def parse_fall_info(self, data: bytes):
        if data[3] == REPORT_FALL and data[4] == ALARM:
            self.show_data(data)

            if data[5] == ALARM_FALL:
                if data[6] == SUSPECTED_FALL:
                    return "SUSPECTED_FALL"
                elif data[6] == REAL_FALL:
                    return "REAL_FALL"
                elif data[6] == NO_FALL:
                    return "NO_FALL"

    def parse_situation(self, data: bytes):
        if data[3] != ACTIVE_REPORT:
            return None

        report_type = data[5]

        if report_type in (ENVIRONMENT, HEARTBEAT):
            if data[6] == NOBODY:
                return "Radar detects no one."
            elif data[6] == SOMEBODY_BE:
                if data[7] == SOMEBODY_MOVE:
                    return "Radar detects somebody moving."
                else:
                    return "Radar detects somebody stopping."

        elif report_type == CLOSE_AWAY:
            if data[8] == CA_CLOSE:
                return "Radar detects somebody approaching."
            elif data[8] == CA_AWAY:
                return "Radar detects somebody leaving."
            else:
                return "Radar detects no movement."

        elif report_type == ABNORMAL:
            return "Radar abnormal reset."

    # ================== CRC ==================

    def calculate_crc16(self, frame: bytes) -> int:
        crc_hi = 0xFF
        crc_lo = 0xFF

        for b in frame:
            idx = crc_lo ^ b
            crc_lo = crc_hi ^ cuc_CRCHi[idx]
            crc_hi = cuc_CRCLo[idx]

        return (crc_lo << 8) | crc_hi