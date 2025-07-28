# Replace "rokoko_motion.csv" with your actual CSV file path.
# using https://github.com/Sentdex/inspire_hands as reference

import pandas as pd
import serial
from typing import List

# --- Modbus Communication Class ---
class Modbus:
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200, slave_id: int = 1):
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.1)

    def connect(self):
        if not self.ser.is_open:
            self.ser.open()
        print(f"Connected to {self.port}")

    def disconnect(self):
        if self.ser.is_open:
            self.ser.close()
        print("Disconnected.")

    def crc16(self, data: bytes) -> bytes:
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if (crc & 0x0001):
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder='little')

    def write_multiple_registers(self, address: int, values: List[int]) -> bool:
        frame = bytearray()
        frame.append(self.slave_id)
        frame.append(0x10)  # Function code
        frame += address.to_bytes(2, byteorder='big')
        frame += len(values).to_bytes(2, byteorder='big')
        frame.append(len(values) * 2)
        for value in values:
            if value == -1:
                value = 0xFFFF  # RH56 "no action"
            frame += value.to_bytes(2, byteorder='big')
        frame += self.crc16(frame)

        self.ser.write(frame)
        response = self.ser.read(8)
        return len(response) >= 6 and response[1] == 0x10

# --- Register Map ---
class Register:
    ANGLE_SET = 1486  # 0x05CE

# --- Angle Normalization ---
ANGLE_LIMITS = {
    0: (19.0, 176.7),
    1: (19.0, 176.7),
    2: (19.0, 176.7),
    3: (19.0, 176.7),
    4: (-13.0, 53.6),
    5: (90.0, 165.0),
}

DOF_ANGLE_SOURCES = {
    0: ['LeftDigit5Metacarpophalangeal_flexion', 'LeftDigit5ProximalInterphalangeal_flexion'],
    1: ['LeftDigit4Metacarpophalangeal_flexion', 'LeftDigit4ProximalInterphalangeal_flexion'],
    2: ['LeftDigit3Metacarpophalangeal_flexion', 'LeftDigit3ProximalInterphalangeal_flexion'],
    3: ['LeftDigit2Metacarpophalangeal_flexion', 'LeftDigit2ProximalInterphalangeal_flexion'],
    4: ['LeftDigit1Metacarpophalangeal_flexion'],  # Thumb bend
    5: ['LeftDigit1Carpometacarpal_flexion'],      # Thumb rotate
}

def normalize_angle(angle_deg: float, min_deg: float, max_deg: float) -> int:
    angle_deg = max(min(angle_deg, max_deg), min_deg)
    return int(((angle_deg - min_deg) / (max_deg - min_deg)) * 1000)

def extract_dof_angles(row: pd.Series) -> List[int]:
    angles = []
    for dof, joints in DOF_ANGLE_SOURCES.items():
        values = [row[j] for j in joints if j in row and pd.notnull(row[j])]
        if not values:
            angles.append(-1)
        else:
            avg = sum(values) / len(values)
            norm = normalize_angle(avg, *ANGLE_LIMITS[dof])
            angles.append(norm)
    return angles

# --- Main Routine ---
def map_csv_to_inspire_hand(csv_path: str, port: str = '/dev/ttyUSB0', baudrate: int = 115200, hand_id: int = 1):
    df = pd.read_csv(csv_path)
    if df.empty:
        print("CSV file is empty.")
        return

    modbus = Modbus(port=port, baudrate=baudrate, slave_id=hand_id)
    modbus.connect()

    try:
        for _, row in df.iterrows():
            angles = extract_dof_angles(row)
            print(f"Sending angles: {angles}")
            modbus.write_multiple_registers(Register.ANGLE_SET, angles)
    finally:
        modbus.disconnect()

    print("Completed CSV streaming to Inspire hand.")

# --- Example Execution ---
if __name__ == "__main__":
    map_csv_to_inspire_hand("rokoko_motion.csv", port="/dev/ttyUSB0")
