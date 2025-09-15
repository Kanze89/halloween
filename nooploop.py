#!/usr/bin/env python3
# Nooploop TOFSense -> Witch Laugh (1m zone) with robust lock + auto offset

import serial, time, subprocess, statistics
from pathlib import Path
from collections import deque

# ---------- USER CONFIG ----------
PORTS        = ["/dev/serial0", "/dev/ttyUSB0"]   # GPIO UART first, then USB-TTL
BAUDS        = [921600, 460800, 230400, 115200]   # include 460800 (some firmwares)
AUDIO_DEVICE = "hw:1,0"                            # from `aplay -l` (or None for default)
SOUND_FILE   = Path("/home/upi/halloween/sounds/witch_laugh.mp3")

# 1-meter zone
ZONE_MIN_CM  = 35
ZONE_MAX_CM  = 100
ENTER_FRAMES = 3
EXIT_FRAMES  = 3
COOLDOWN_SEC = 5.0

# Filters (start forgiving; tighten later)
MIN_SIGNAL   = 0        # start at 0 while testing; set 15–30 once stable
MIN_CM       = 20
MAX_CM       = 160      # ignore anything farther than ~1.6 m
EMA_ALPHA    = 0.35
MED_WIN      = 5
DEBUG_EVERY  = 1        # print every frame; set 0 later
# ---------------------------------

HDR        = 0x57
FRAME_LEN  = 16
OFFSETS    = [8, 10]    # try both distance byte positions

def play_sound():
    if not SOUND_FILE.exists():
        print(f"[ERR] Missing sound: {SOUND_FILE}")
        return
    cmd = ["mpg123", "-q"]
    if AUDIO_DEVICE:
        cmd += ["-a", AUDIO_DEVICE]
    cmd.append(str(SOUND_FILE))
    subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def sync_and_read_frame(ser, timeout=1.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        b = ser.read(1)
        if not b or b[0] != HDR:
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

def parse_dist_cm(frame, offset):
    d0, d1, d2 = frame[offset], frame[offset+1], frame[offset+2]
    return (d0 | (d1 << 8) | (d2 << 16)) / 10.0  # mm -> cm (meters*1000 → cm)

def fields(frame, offset):
    dist_cm = parse_dist_cm(frame, offset)
    status  = frame[11]
    signal  = frame[12] | (frame[13] << 8)
    return dist_cm, status, signal

def try_open_and_autoconfig():
    """
    Lock only after >=5 valid frames; then auto-choose distance offset (8 vs 10)
    by counting how many frames land in a plausible human range (20..500 cm)
    with status==0. Tie-breaker = lower variance.
    """
    for port in PORTS:
        for baud in BAUDS:
            try:
                ser = serial.Serial(port, baudrate=baud, timeout=0.2)
                frames = []
                t0 = time.time()
                while time.time() - t0 < 3.0 and len(frames) < 12:
                    f = sync_and_read_frame(ser, timeout=0.5)
                    if f:
                        frames.append(f)
                if len(frames) < 5:
                    ser.close()
                    continue

                # choose offset
                def score(off):
                    vals = []
                    for fr in frames:
                        d, st, _ = fields(fr, off)
                        if 20 <= d <= 500 and st == 0:
                            vals.append(d)
                    if not vals:
                        return (0, float("inf"))
                    var = statistics.pvariance(vals) if len(vals) > 1 else 0.0
                    return (len(vals), var)

                s8 = score(8)
                s10 = score(10)
                if s8 > s10:
                    offset = 8
                elif s10 > s8:
                    offset = 10
                else:
                    # if equal counts, pick the one with smaller variance
                    offset = 8 if s8[1] <= s10[1] else 10

                fid = frames[0][3]
                mark = frames[0][1]
                print(f"[LOCK] port={port} baud={baud} id=0x{fid:02X} mark=0x{mark:02X} dist_offset={offset}")
                return ser, offset
            except Exception:
                try: ser.close()
                except Exception: pass
    return None, None

def main():
    if not SOUND_FILE.exists():
        raise SystemExit(f"Sound file missing: {SOUND_FILE}")

    ser, dist_offset = try_open_and_autoconfig()
    if not ser:
        raise SystemExit("No valid TOFSense frames. Check wiring, UART Active mode, and baud.")

    medbuf = deque(maxlen=MED_WIN)
    ema = None
    in_count = 0
    out_count = 0
    armed = True
    last_play = 0.0
    dbg = 0

    try:
        while True:
            fr = sync_and_read_frame(ser, timeout=1.0)
            if not fr:
                continue
            dist_cm, status, signal = fields(fr, dist_offset)

            # gate junk lightly while testing
            if status != 0 or dist_cm < MIN_CM or dist_cm > MAX_CM:
                in_count = 0
                continue
            if MIN_SIGNAL and signal < MIN_SIGNAL:
                in_count = 0
                continue

            # smoothing
            medbuf.append(dist_cm)
            med = statistics.median(medbuf) if medbuf else dist_cm
            ema = med if ema is None else (EMA_ALPHA * med + (1 - EMA_ALPHA) * ema)
            smoothed = ema

            if DEBUG_EVERY and (dbg % DEBUG_EVERY == 0):
                print(f"[DBG] raw={dist_cm:6.1f} med={med:6.1f} ema={smoothed:6.1f} sig={signal:4d} st={status}")
            dbg += 1

            now = time.time()
            inside = (ZONE_MIN_CM <= smoothed <= ZONE_MAX_CM)

            if armed:
                if inside:
                    in_count += 1
                    if in_count >= ENTER_FRAMES and (now - last_play) >= COOLDOWN_SEC:
                        print(f"[TRIGGER] {smoothed:.0f} cm in zone ({ZONE_MIN_CM}-{ZONE_MAX_CM}) → Witch Laugh")
                        play_sound()
                        last_play = now
                        armed = False
                        in_count = 0
                        out_count = 0
                else:
                    in_count = 0
            else:
                if not inside:
                    out_count += 1
                    if out_count >= EXIT_FRAMES:
                        armed = True
                        out_count = 0
                else:
                    out_count = 0

    except KeyboardInterrupt:
        pass
    finally:
        try: ser.close()
        except Exception: pass

if __name__ == "__main__":
    main()
