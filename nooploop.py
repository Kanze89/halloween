#!/usr/bin/env python3
# Halloween: Nooploop TOFSense -> Witch Laugh (only within 1 meter)

import serial, time, subprocess, statistics
from pathlib import Path
from collections import deque

# -------------------- USER CONFIG --------------------
# Serial: try GPIO UART first, then USB-TTL
PORTS         = ["/dev/serial0", "/dev/ttyUSB0"]
BAUDS         = [921600, 230400, 115200]   # sensor's UART Active baud (try in this order)

# Audio: set your USB audio device from `aplay -l` (None = default ALSA)
AUDIO_DEVICE  = "hw:1,0"                   # change to match your dongle, or set to None
SOUND_FILE    = Path("/home/pi/halloween/sounds/witch_laugh.mp3")

# Zone: ONLY trigger when someone is inside 1 meter
ZONE_MIN_CM   = 35       # ignore too-close sensor face hits
ZONE_MAX_CM   = 100      # <-- 1 meter max
ENTER_FRAMES  = 3        # need N consecutive in-zone frames to fire
EXIT_FRAMES   = 3        # need N consecutive out-of-zone frames to re-arm
COOLDOWN_SEC  = 5.0      # minimum seconds between scares

# Filtering & gates
MIN_SIGNAL    = 20       # ignore weak returns; tune by watching [DBG]
EMA_ALPHA     = 0.35     # exponential moving average strength (higher = snappier)
MED_WIN       = 5        # median filter window length (odd numbers best)
MIN_CM        = 20       # drop absurdly-close junk
MAX_CM        = 110      # hard cap so far wall (~>1.1m) is ignored entirely
DEBUG_EVERY   = 1        # print every N valid frames; 0 = silence debug

# Frame format
HDR           = 0x57
FRAME_LEN     = 16

# Some firmware variants place distance at [8..10] (typical) vs [10..12].
# If readings look stuck/odd, flip this to 10 and rerun.
DIST_OFFSET   = 8
# ------------------ END USER CONFIG ------------------


def play_sound():
    """Fire the witch laugh via mpg123."""
    if not SOUND_FILE.exists():
        print(f"[ERR] Sound file not found: {SOUND_FILE}")
        return
    cmd = ["mpg123", "-q"]
    if AUDIO_DEVICE:
        cmd += ["-a", AUDIO_DEVICE]
    cmd.append(str(SOUND_FILE))
    subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def sync_and_read_frame(ser, timeout=1.0):
    """
    Sync to a valid 16-byte TOFSense frame:
      [0]=0x57, [1]=func_mark (0x00/0x01), [2]=0xFF, [3]=id,
      [4..7]=time, [8..10]=distance(int24LE m*1000), [11]=status,
      [12..13]=signal(u16LE), [14]=reserved, [15]=checksum(low-byte sum[0..14])
    """
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


def parse_fields(frame):
    """
    Return (dist_cm, status, signal).
    Distance is int24 LE of meters*1000. Convert to cm.
    """
    d0, d1, d2 = frame[DIST_OFFSET], frame[DIST_OFFSET+1], frame[DIST_OFFSET+2]
    dist_cm = (d0 | (d1 << 8) | (d2 << 16)) / 10.0  # mm -> cm (m*1000 → cm)
    status  = frame[11]                              # 0 = OK
    signal  = frame[12] | (frame[13] << 8)
    return dist_cm, status, signal


def try_open():
    """
    Only lock the port/baud after 5 consecutive, checksummed frames within ~3s.
    Prevents false locks on random 0x57 bytes.
    """
    for port in PORTS:
        for baud in BAUDS:
            try:
                ser = serial.Serial(port, baudrate=baud, timeout=0.2)
                ok = 0
                first = None
                t0 = time.time()
                while time.time() - t0 < 3.0 and ok < 5:
                    f = sync_and_read_frame(ser, timeout=0.5)
                    if f:
                        ok += 1
                        if first is None:
                            first = f
                if ok >= 5:
                    fid = first[3]
                    mark = first[1]
                    print(f"[LOCK] port={port} baud={baud} id=0x{fid:02X} mark=0x{mark:02X}")
                    return ser
                ser.close()
            except Exception:
                try:
                    ser.close()
                except Exception:
                    pass
    return None


def main():
    # Pre-flight audio note
    if not SOUND_FILE.exists():
        raise SystemExit(f"Sound file missing: {SOUND_FILE}")

    ser = try_open()
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
            f = sync_and_read_frame(ser, timeout=1.0)
            if not f:
                continue

            dist_cm, status, signal = parse_fields(f)

            # basic gating
            if status != 0 or signal < MIN_SIGNAL or dist_cm < MIN_CM or dist_cm > MAX_CM:
                in_count = 0
                # we don't increment out_count on junk; it shouldn't re-arm us
                continue

            # smoothing: median -> EMA
            medbuf.append(dist_cm)
            med = statistics.median(medbuf) if medbuf else dist_cm
            ema = med if ema is None else (EMA_ALPHA * med + (1 - EMA_ALPHA) * ema)
            smoothed = ema

            if DEBUG_EVERY and (dbg % DEBUG_EVERY == 0):
                print(f"[DBG] raw={dist_cm:6.1f} med={med:6.1f} ema={smoothed:6.1f} sig={signal:4d} st={status}")
            dbg += 1

            now = time.time()
            inside_zone = (ZONE_MIN_CM <= smoothed <= ZONE_MAX_CM)

            if armed:
                if inside_zone:
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
                # Re-arm only after consistently outside the zone
                if not inside_zone:
                    out_count += 1
                    if out_count >= EXIT_FRAMES:
                        armed = True
                        out_count = 0
                else:
                    out_count = 0

    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
