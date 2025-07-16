import serial
import time

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

def generate_poses(start=1300, end=200, steps=50):
    step_size = (start - end) / (steps - 1)
    return [[int(start - step_size * i)] * 6 for i in range(steps)]

# --- MAIN ---
if __name__ == "__main__":
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
        if not ser.is_open:
            print("Failed to open serial port.")
            exit()

        print("Serial port opened successfully.")
        poses = generate_poses(1300, 200, 50)

        for pose in poses:
            print(f"Pose: {pose[0]}")
            send_pose(pose, ser)
            time.sleep(0.01)

        ser.close()
        print("Smooth motion sequence complete.")

    except serial.SerialException as e:
        print("Serial error:", e)
