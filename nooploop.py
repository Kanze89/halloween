#!/usr/bin/env python3
import serial, time, subprocess
from pathlib import Path

# === CONFIG ===
PORT = "/dev/serial0"       # or "/dev/ttyUSB0" if using a USB-TTL adapter
BAUDS = [921600, 230400, 115200]  # try these in order
TRIGGER_CM = 120
RELEASE_CM = 150
COOLDOWN = 5.0
AUDIO_DEVICE = "hw:1,0"     # set to your USB audio card from `aplay -l`, or None
SOUND_FILE = Path("/home/pi/halloween/sounds/witch_laugh.mp3")

# Frame layout (TOFSense S/F/FP/F2 series "Frame0"): 16 bytes total
# [0] 0x57 header
# [1] Function Mark: 0x00 (S/F...) or 0x01 (M series)  ← accept both
# [2] 0xFF (fixed)
# [3] ID (0x00..0xFE)                                   ← accept any
# [4..7] system_time (u32 LE)
# [8..10] distance (int24 LE, value = meters * 1000)
# [11] dis_status
# [12..13] signal_strength (u16 LE)
# [14] reserved (varies)
# [15] checksum (low byte of sum of bytes [0..14])
# Source/format refs incl. example frames. :contentReference[oaicite:2]{index=2}

def play_sound():
    cmd = ["mpg123", "-q"]
    if AUDIO_DEVICE:
        cmd += ["-a", AUDIO_DEVICE]
    cmd.append(str(SOUND_FILE))
    subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def sync_and_read_frame(ser, timeout=2.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        b = ser.read(1)
        if not b:
            continue
        if b[0] != 0x57:
            continue
        hdr_tail = ser.read(3)
        if len(hdr_tail) != 3:
            continue
        func_mark, fixed_ff, module_id = hdr_tail[0], hdr_tail[1], hdr_tail[2]
        if func_mark not in (0x00, 0x01) or fixed_ff != 0xFF:
            # not a TOFSense frame start
            continue
        rest = ser.read(16 - 4)
        if len(rest) != 12:
            continue
        frame = bytes([0x57, func_mark, fixed_ff, module_id]) + rest
        if (sum(frame[:-1]) & 0xFF) == frame[-1]:
            return frame
        # else keep looking
    return None

def parse_distance_cm(frame):
    # distance is int24 little-endian representing meters*1000
    d0, d1, d2 = frame[8], frame[9], frame[10]
    raw = d0 | (d1 << 8) | (d2 << 16)
    # treat as unsigned => meters = raw / 1000.0; cm = meters*100
    return (raw / 1000.0) * 100.0  # → centimeters

def run():
    if not SOUND_FILE.exists():
        raise SystemExit(f"Sound file not found: {SOUND_FILE}")

    ser = None
    # try bauds until we see a valid frame
    for b in BAUDS:
        try:
            ser = serial.Serial(PORT, baudrate=b, timeout=0.2)
            f = sync_and_read_frame(ser, timeout=2.0)
            if f:
                print(f"Locked: baud={b}, id=0x{f[3]:02x}, func_mark=0x{f[1]:02x}")
                baud = b
                break
            ser.close()
            ser = None
        except Exception as e:
            if ser:
                ser.close()
            ser = None
    if not ser:
        raise SystemExit("No valid TOFSense frames found. Check wiring/mode/baud.")

    last = 0.0
    armed = True
    try:
        while True:
            f = sync_and_read_frame(ser, timeout=1.0)
            if not f:
                continue
            dist_cm = parse_distance_cm(f)
            status = f[11]
            if status != 0:
                # ignore invalid/weak readings
                continue
            now = time.time()
            if armed and dist_cm <= TRIGGER_CM and (now - last) >= COOLDOWN:
                print(f"TRIGGER @ {dist_cm:.0f} cm (ID {f[3]}) → Witch Laugh")
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
    run()
