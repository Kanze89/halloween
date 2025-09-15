#!/usr/bin/env python3
# Nooploop TOFSense -> Witch Laugh (1 mm .. 1 m), robust lock + auto offset
# Adds "retrigger while staying inside zone" option.

import serial, time, subprocess, statistics
from pathlib import Path
from collections import deque

# ---------- USER CONFIG ----------
PORTS        = ["/dev/serial0", "/dev/ttyUSB0"]     # GPIO UART first, then USB-TTL
BAUDS        = [921600, 460800, 230400, 115200]     # include 460800 (some firmwares)
AUDIO_DEVICE = "hw:1,0"                              # from `aplay -l` (or None for default)
SOUND_FILE   = Path("/home/pi/halloween/sounds/witch_laugh.mp3")

# ZONE: ONLY trigger when inside 1 mm .. 1 m
ZONE_MIN_CM  = 0.1    # 1 mm
ZONE_MAX_CM  = 100.0  # 1 m

# Trigger behavior
ENTER_FRAMES = 2       # lower = more responsive, higher = more stable
EXIT_FRAMES  = 2
COOLDOWN_SEC = 4.0     # gap between scares
RETRIGGER_ON_STAY = True   # <-- if True, it will re-trigger after cooldown even if you don't leave the zone

# Filters (start forgiving; tighten later if noisy)
MIN_SIGNAL   = 0       # set to ~15–30 after you confirm it's working
MIN_CM       = 0.1
MAX_CM       = 105.0
EMA_ALPHA    = 0.6     # higher = snappier response
MED_WIN      = 3       # smaller window = less lag
DEBUG_EVERY  = 1       # print every frame; set 0 later
# ---------------------------------

HDR        = 0x57
FRAME_LEN  = 16

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
    # distance is int24 little-endian of meters*1000 -> convert to cm
    return (d0 | (d1 << 8) | (d2 << 16)) / 10.0  # mm -> cm

def fields(frame, offset):
    dist_cm = parse_dist_cm(frame, offset)
    status  = frame[11]
    signal  = frame[12] | (frame[13] << 8)
    return dist_cm, status, signal

def try_open_and_autoconfig():
    """
    1) Scan ports/bauds. Only accept lock after >=5 valid frames in ~3 s.
    2) Auto-choose distance offset: 8 vs 10 (firmware variants).
       Score = count of plausible frames (0.1..500 cm, st==0), tiebreaker = lower variance.
    """
    for port in PORTS:
        for baud in BAUDS:
            try:
                ser = serial.Serial(port, baudrate=baud, timeout=0.2)
                frames = []
                t0 = time.time()
                while time.time() - t0 < 3.0 and len(frames) < 14:
                    f = sync_and_read_frame(ser, timeout=0.5)
                    if f:
                        frames.append(f)
                if len(frames) < 5:
                    ser.close()
                    continue

                def score(off):
                    vals = []
                    for fr in frames:
                        d, st, _ = fields(fr, off)
                        if 0.1 <= d <= 500 and st == 0:
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

            # gates
            if status != 0 or dist_cm < MIN_CM or dist_cm > MAX_CM:
                in_count = 0
                continue
            if MIN_SIGNAL and signal < MIN_SIGNAL:
                in_count = 0
                continue

            # smoothing: median -> EMA
            medbuf.append(dist_cm)
            med = statistics.median(medbuf) if medbuf else dist_cm
            ema = med if ema is None else (EMA_ALPHA * med + (1 - EMA_ALPHA) * ema)
            smoothed = ema

            if DEBUG_EVERY and (dbg % DEBUG_EVERY == 0):
                print(f"[DBG] raw={dist_cm:6.2f}cm  med={med:6.2f}  ema={smoothed:6.2f}  sig={signal:4d}  st={status}")
            dbg += 1

            now = time.time()
            inside = (ZONE_MIN_CM <= smoothed <= ZONE_MAX_CM)

            if RETRIGGER_ON_STAY:
                # simpler logic: retrigger on cooldown while staying inside
                if inside:
                    in_count += 1
                    if in_count >= ENTER_FRAMES and (now - last_play) >= COOLDOWN_SEC:
                        print(f"[TRIGGER] {smoothed:.1f} cm in zone → Witch Laugh")
                        play_sound()
                        last_play = now
                        in_count = 0      # require a few more frames before next retrigger
                else:
                    in_count = 0
            else:
                # classic logic: trigger once, then require exit to re-arm
                if armed:
                    if inside:
                        in_count += 1
                        if in_count >= ENTER_FRAMES and (now - last_play) >= COOLDOWN_SEC:
                            print(f"[TRIGGER] {smoothed:.1f} cm in zone → Witch Laugh")
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
