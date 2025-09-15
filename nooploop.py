#!/usr/bin/env python3
import serial, time, sys

PORT = "/dev/serial0"
BAUDS = [921600, 460800, 230400, 115200]
HDR = bytes([0x57, 0x00, 0xFF, 0x00])
FRAME_LEN = 16

def find_frame(ser, timeout=3.0):
    ser.reset_input_buffer()
    t0 = time.time()
    # byte-wise sync to header
    while time.time() - t0 < timeout:
        b = ser.read(1)
        if not b:
            continue
        if b == HDR[:1]:
            rest = ser.read(3)
            if rest == HDR[1:]:
                payload = ser.read(FRAME_LEN - 4)
                if len(payload) == FRAME_LEN - 4:
                    frame = HDR + payload
                    if (sum(frame[:-1]) & 0xFF) == frame[-1]:
                        return frame
    return None

def parse_distance_cm(frame):
    # distance is 24-bit little-endian at bytes [10:13], in millimeters
    d0, d1, d2 = frame[10], frame[11], frame[12]
    dist_mm = d0 | (d1 << 8) | (d2 << 16)
    return dist_mm / 10.0  # mm → cm

for baud in BAUDS:
    try:
        print(f"Trying {PORT} @ {baud}…")
        with serial.Serial(PORT, baudrate=baud, timeout=0.2) as ser:
            f = find_frame(ser, timeout=4.0)
            if f:
                print(f"Found header at {baud}. Frame: {f.hex(' ')}")
                dist_cm = parse_distance_cm(f)
                status = f[13]
                print(f"Distance ≈ {dist_cm:.1f} cm | status={status}")
                # Now keep streaming distances:
                while True:
                    f = find_frame(ser, timeout=1.0)
                    if f:
                        d = parse_distance_cm(f)
                        s = f[13]
                        if s == 0:
                            print(f"{d:.1f} cm")
                    else:
                        print(".", end="", flush=True)
                sys.exit(0)
            else:
                print(f"No valid frames at {baud}.")
    except Exception as e:
        print(f"Error at {baud}: {e}")

print("No headers found. Check wiring, mode (UART Active), and that you're on /dev/serial0.")
