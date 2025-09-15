#!/usr/bin/env python3
import serial, time, os, random, subprocess, glob
from pathlib import Path

PORT = "/dev/serial0"        # Pi's UART
BAUD = 921600                # Use NAssistant to lower if needed (e.g., 230400)
TRIGGER_CM = 120             # fire when closer than this
RELEASE_CM = 150             # must move back past this to re-arm
COOLDOWN = 6.0               # seconds between scares
SOUNDS_DIR = Path("/home/pi/halloween/sounds")
AUDIO_DEVICE = None          # e.g., "hw:1,0" for USB DAC; None=default

HDR = bytes([0x57, 0x00, 0xFF, 0x00])  # Nooploop TOFSense UART frame header
FRAME_LEN = 16

def play_sound(path):
    cmd = ["mpg123", "-q"]
    if AUDIO_DEVICE: cmd += ["-a", AUDIO_DEVICE]
    cmd.append(str(path))
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def read_frame(ser):
    # sync to header
    while True:
        b = ser.read(1)
        if not b: return None
        if b == HDR[:1]:
            rest = ser.read(3)
            if rest == HDR[1:]:
                payload = ser.read(FRAME_LEN - 4)
                if len(payload) == FRAME_LEN - 4:
                    frame = HDR + payload
                    # checksum = low byte of sum of first 15 bytes
                    if (sum(frame[:-1]) & 0xFF) == frame[-1]:
                        return frame
                # if fail, continue searching

def parse_distance_cm(frame):
    # bytes layout from example doc:
    # 0-3: header, 4: reserved, 5: id, 6-9: system_time (u32 le)
    # 10-12: distance_u24 (little endian), 13: status, 14-15: signal (u16 le), 16: sum (but frame is 16 bytes total, index 15 is sum)
    # Our 16-byte frame indexes: 0..15, with checksum at 15
    # distance_u24 starts at index 10 in the example PDF
    d0, d1, d2 = frame[10], frame[11], frame[12]
    dist_u24 = d0 | (d1 << 8) | (d2 << 16)
    # Empirically this is distance in millimeters (see example decoding 0x00 0x08 0xAD -> 2221 mm ≈ 2.221 m)
    # Return centimeters:
    return dist_u24 / 10.0  # mm -> cm

def main():
    files = sorted([p for p in SOUNDS_DIR.glob("*") if p.suffix.lower() in (".mp3",".wav")])
    if not files: raise SystemExit(f"Put some MP3/WAV files in {SOUNDS_DIR}")
    ser = serial.Serial(PORT, BAUD, timeout=1)
    last = 0.0
    armed = True
    print("Listening for Nooploop TOF frames…")
    try:
        while True:
            f = read_frame(ser)
            if not f: continue
            dist_cm = parse_distance_cm(f)
            status = f[13]
            if status != 0:
                # ignore invalid/weak signal frames
                continue
            now = time.time()
            if armed and dist_cm <= TRIGGER_CM and (now - last) >= COOLDOWN:
                snd = random.choice(files)
                print(f"TRIGGER @ {dist_cm:.0f} cm → {snd.name}")
                play_sound(snd)
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
