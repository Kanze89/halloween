#!/usr/bin/env python3
import serial, time, subprocess, statistics
from pathlib import Path
from collections import deque

# ====== USER SETTINGS ======
PORTS   = ["/dev/serial0", "/dev/ttyUSB0"]   # try GPIO UART first, then USB-TTL
BAUDS   = [921600, 230400, 115200]           # try in this order
AUDIO_DEVICE = "hw:1,0"                       # set from `aplay -l` (None = default)
SOUND_FILE   = Path("/home/pi/halloween/sounds/witch_laugh.mp3")

TRIGGER_CM   = 120      # fire when smoothed distance <= this
RELEASE_CM   = 160      # must move back >= this to re-arm
MIN_SIGNAL   = 20       # ignore weak returns (< this). Tune: 10–80 typical
VALID_FRAMES = 3        # need N consecutive valid frames to trigger
REARM_FRAMES = 3        # need N consecutive far frames to re-arm
EMA_ALPHA    = 0.35     # smoothing for exponential moving average
MED_WIN      = 5        # median window length (odd number best)
COOLDOWN     = 5.0      # seconds between scares
MIN_CM       = 20       # ignore absurdly close readings (sensor face hits)
MAX_CM       = 500      # ignore far junk beyond this gate (5 m)
DEBUG_EVERY  = 10       # print a debug line every N valid frames; 0 = off
# ===========================

HDR = 0x57
FRAME_LEN = 16

def play_sound():
    cmd = ["mpg123", "-q"]
    if AUDIO_DEVICE:
        cmd += ["-a", AUDIO_DEVICE]
    cmd.append(str(SOUND_FILE))
    subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def try_open():
    for port in PORTS:
        for baud in BAUDS:
            try:
                ser = serial.Serial(port, baudrate=baud, timeout=0.2)
                frame = sync_and_read_frame(ser, timeout=2.0)
                if frame:
                    func_mark = frame[1]
                    module_id = frame[3]
                    print(f"[LOCK] port={port} baud={baud} id=0x{module_id:02X} mark=0x{func_mark:02X}")
                    return ser
                ser.close()
            except Exception:
                try:
                    ser.close()
                except Exception:
                    pass
    return None

def sync_and_read_frame(ser, timeout=1.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        b = ser.read(1)
        if not b:
            continue
        if b[0] != HDR:
            continue
        tail = ser.read(3)
        if len(tail) != 3:
            continue
        func_mark, fixed_ff, module_id = tail
        if fixed_ff != 0xFF or func_mark not in (0x00, 0x01):
            continue
        rest = ser.read(FRAME_LEN - 4)
        if len(rest) != (FRAME_LEN - 4):
            continue
        frame = bytes([HDR, func_mark, fixed_ff, module_id]) + rest
        if (sum(frame[:-1]) & 0xFF) == frame[-1]:
            return frame
    return None

def parse_fields(frame):
    # Layout:
    # [0]=0x57, [1]=func_mark, [2]=0xFF, [3]=id
    # [4..7]=system_time(u32 LE)
    # [8..10]=distance(u24 LE) meters*1000
    # [11]=status (0=OK)
    # [12..13]=signal(u16 LE)
    # [14]=reserved, [15]=checksum
    d0, d1, d2 = frame[8], frame[9], frame[10]
    dist_cm = (d0 | (d1 << 8) | (d2 << 16)) / 10.0  # mm->cm (meters*1000 → cm)
    status  = frame[11]
    sig     = frame[12] | (frame[13] << 8)
    return dist_cm, status, sig

def main():
    if not SOUND_FILE.exists():
        raise SystemExit(f"Sound file not found: {SOUND_FILE}")

    ser = try_open()
    if not ser:
        raise SystemExit("No valid TOFSense frames found. Check wiring/mode/baud/port.")

    medbuf = deque(maxlen=MED_WIN)
    ema = None
    consec_near = 0
    consec_far = 0
    last_play = 0.0
    armed = True
    debug_count = 0

    try:
        while True:
            frame = sync_and_read_frame(ser, timeout=1.0)
            if not frame:
                continue

            dist_cm, status, signal = parse_fields(frame)

            # gate obvious junk
            if status != 0:
                consec_near = 0
                continue
            if not (MIN_CM <= dist_cm <= MAX_CM):
                consec_near = 0
                continue
            if signal < MIN_SIGNAL:
                consec_near = 0
                continue

            # smoothing: median then EMA
            medbuf.append(dist_cm)
            med = statistics.median(medbuf) if medbuf else dist_cm
            ema = med if ema is None else (EMA_ALPHA * med + (1 - EMA_ALPHA) * ema)
            smoothed = ema

            if DEBUG_EVERY and (debug_count % DEBUG_EVERY == 0):
                print(f"[DBG] raw={dist_cm:.1f}cm med={med:.1f} ema={smoothed:.1f} sig={signal} st={status}")
            debug_count += 1

            now = time.time()
            if armed:
                if smoothed <= TRIGGER_CM:
                    consec_near += 1
                    if consec_near >= VALID_FRAMES and (now - last_play) >= COOLDOWN:
                        print(f"[TRIGGER] {smoothed:.0f} cm (sig={signal}) → Witch Laugh")
                        play_sound()
                        last_play = now
                        armed = False
                        consec_near = 0
                else:
                    consec_near = 0
            else:
                if smoothed >= RELEASE_CM:
                    consec_far += 1
                    if consec_far >= REARM_FRAMES:
                        armed = True
                        consec_far = 0
                else:
                    consec_far = 0

    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
