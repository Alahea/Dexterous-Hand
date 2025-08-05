# this is an example code that demostrate a series of movements, mimiking a real hand movement

import json
import numpy as np
import serial
import time

# === Serial Communication Setup ===
hand_id = 1

def data2bytes(data):
    if data == -1:
        return [0xFF, 0xFF]
    return [data & 0xFF, (data >> 8) & 0xFF]

def num2str(num):
    return bytes.fromhex(f'{num:02x}')

def checknum(data, leng):
    return sum(data[2:leng]) & 0xFF

def send_pose(pos, ser):
    datanum = 0x0F
    b = [0] * (datanum + 5)

    b[0] = 0xEB
    b[1] = 0x90
    b[2] = hand_id
    b[3] = datanum
    b[4] = 0x12  # write
    b[5] = 0xC2
    b[6] = 0x05

    for i in range(6):
        dbytes = data2bytes(pos[i])
        b[7 + i*2] = dbytes[0]
        b[8 + i*2] = dbytes[1]

    b[19] = checknum(b, datanum + 4)

    putdata = b''
    for byte in b:
        putdata += num2str(byte)

    ser.write(putdata)

# === Shared Frame-Based Mapping ===
def get_shared_actuator_target(frame_idx):
    """Shared actuator targets for all fingers."""
    targets = [200, 1500, 400]  # open → closed → open again
    return targets[frame_idx]

def interpolate_poses(start_pose, end_pose, steps=40):
    return [list(map(int, start_pose + (end_pose - start_pose) * (i / (steps - 1)))) for i in range(steps)]

# === Load Gesture Data ===
with open("gestures.json", "r") as f:
    data = json.load(f)

grab = data["grab"]["data"]
fingers = ["thumb", "index", "middle", "ring", "little"]

# Construct 3 shared actuator poses: open → closed → open
frames = []
for frame_idx in range(3):
    pose = [get_shared_actuator_target(frame_idx) for _ in fingers]
    pose.append(0)  # wrist or placeholder
    frames.append(pose)

# Interpolate smooth transitions
poses_full = []
poses_full += interpolate_poses(np.array(frames[0]), np.array(frames[1]), steps=20)  # open → grasp
poses_full += interpolate_poses(np.array(frames[1]), np.array(frames[2]), steps=20)  # grasp → release

# === Send Poses to Inspire Hand ===
try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
    if not ser.is_open:
        print("Failed to open serial port.")
        exit()

    print("Serial port opened successfully.")
    
    for pose in poses_full:
        print(f"Sending pose: {pose}")
        send_pose(pose, ser)
        time.sleep(0.02)

    ser.close()
    print("Motion sequence complete.")

except serial.SerialException as e:
    print("Serial error:", e)
