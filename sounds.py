#!/usr/bin/env python3
import serial, time, subprocess
from pathlib import Path

# --- SETTINGS ---
PORT = "/dev/serial0"        # Pi UART port
BAUD = 921600                # default for Nooploop
TRIGGER_CM = 120             # trigger distance
RELEASE_CM = 150             # re-arm distance
COOLDOWN = 5.0               # seconds between plays
AUDIO_DEVICE = None          # e.g., "hw:1,0" for USB, or None = default

# path to your witch laugh file
SOUND_FILE = Path("/home/upi/halloween/sounds/witch_laugh.mp3")

# Nooploop frame constants
HDR = bytes([0x57, 0x00, 0xFF, 0x00])
FRAME_LEN = 16

def play_sound():
    cmd = ["mpg123", "-q"]
    if AUDIO_DEVICE:
        cmd += ["-a", AUDIO_DEVICE]
    cmd.append(str(SOUND_FILE))
    subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def read_frame(ser):
    while True:
        b = ser.read(1)
        if not b: return None
        if b == HDR[:1]:
            rest = ser.read(3)
            if rest == HDR[1:]:
                payload = ser.read(FRAME_LEN - 4)
                if len(payload) == FRAME_LEN - 4:
                    frame = HDR + payload
                    if (sum(frame[:-1]) & 0xFF) == frame[-1]:
                        return frame

def parse_distance_cm(frame):
    d0, d1, d2 = frame[10], frame[11], frame[12]
    dist_u24 = d0 | (d1 << 8) | (d2 << 16)
    return dist_u24 / 10.0  # mm → cm

def main():
    if not SOUND_FILE.exists():
        raise SystemExit(f"Sound file missing: {SOUND_FILE}")

    ser = serial.Serial(PORT, BAUD, timeout=1)
    last = 0.0
    armed = True
    print("Witch laugh scare ready...")

    try:
        while True:
            f = read_frame(ser)
            if not f: continue
            dist_cm = parse_distance_cm(f)
            status = f[13]
            if status != 0:
                continue
            now = time.time()
            if armed and dist_cm <= TRIGGER_CM and (now - last) >= COOLDOWN:
                print(f"TRIGGER @ {dist_cm:.0f} cm → Witch Laugh!")
                play_sound()
                last = now
                armed = False
            if not armed and dist_cm >= RELEASE_CM:
                armed = True
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

if __name__ == "__main__":
    main()
