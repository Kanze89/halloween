#!/usr/bin/env python3
# Nooploop TOFSense -> MP4 scare player (blocks until video ends, then resumes sensing)
# User: upi | Media: /home/upi/halloween/sounds/video.mp4
# Zone: 1 mm .. 1 m, robust port/baud lock, auto distance offset

import serial, time, subprocess, statistics, shutil, os
from pathlib import Path
from collections import deque

# ---------- USER CONFIG ----------
# Serial: try GPIO UART first, then USB-TTL
PORTS        = ["/dev/serial0", "/dev/ttyUSB0"]
BAUDS        = [921600, 460800, 230400, 115200]   # includes 460800 (some firmwares)

# Media (MP4 recommended; mpv also plays MP3/WAV)
MEDIA_FILE   = Path("/home/upi/halloween/sounds/video.mp4")
SHOW_VIDEO   = True                  # True = show video; False = audio-only
FULLSCREEN   = True                  # fullscreen when SHOW_VIDEO=True
MPV_AUDIO_DEVICE = "alsa/plughw:1,0" # set from `mpv --audio-device=help`, or None for default
MPV_VOLUME   = 100                   # 0..130 (mpv allows boost)
FORCE_DRM    = False                 # True if running headless (no desktop), renders straight to HDMI

# Zone: ONLY trigger when inside 1 mm .. 1 m (0.1..100 cm)
ZONE_MIN_CM  = 0.1
ZONE_MAX_CM  = 100.0

# Trigger behavior
ENTER_FRAMES = 2           # consecutive in-zone frames required to fire
COOLDOWN_AFTER_END = 0.4   # brief pause after video ends (seconds)

# Filters (start forgiving; tighten later when stable)
MIN_SIGNAL   = 0           # set to ~15–30 once confirmed working
MIN_CM       = 0.1         # hard lower clamp (1 mm)
MAX_CM       = 105.0       # hard upper clamp slightly above 1 m
EMA_ALPHA    = 0.5         # higher = snappier response
MED_WIN      = 3           # smaller window = less lag
DEBUG_EVERY  = 1           # print every frame; set 0 when done tuning
# ---------------------------------

HDR        = 0x57
FRAME_LEN  = 16

# ---------- MEDIA PLAYBACK ----------
def play_media_and_wait():
    """Launch mpv, BLOCK until it finishes (no re-triggers while playing)."""
    if not MEDIA_FILE.exists():
        print(f"[ERR] Missing media: {MEDIA_FILE}")
        return
    if not shutil.which("mpv"):
        print("[ERR] mpv not found. Install with: sudo apt update && sudo apt install -y mpv")
        return

    cmd = ["mpv", "--no-config", "--no-terminal", "--really-quiet", f"--volume={MPV_VOLUME}"]
    if MPV_AUDIO_DEVICE:
        cmd += [f"--audio-device={MPV_AUDIO_DEVICE}"]
    if SHOW_VIDEO:
        if FULLSCREEN:
            cmd += ["--fs"]
        if FORCE_DRM:
            # render directly to HDMI when no desktop is running
            cmd += ["--vo=gpu", "--gpu-context=drm"]
    else:
        cmd += ["--no-video"]

    cmd.append(str(MEDIA_FILE))

    # Try to target the desktop display if SHOW_VIDEO and not forcing DRM
    env = os.environ.copy()
    if SHOW_VIDEO and not FORCE_DRM and "DISPLAY" not in env:
        env["DISPLAY"] = ":0"

    try:
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env, check=False)
    except Exception as e:
        print(f"[ERR] mpv run failed: {e}")

# ---------- TOFSense FRAME UTILS ----------
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

def parse_dist_cm(frame, offset):
    # distance is int24 little-endian of meters*1000 -> convert to cm
    d0, d1, d2 = frame[offset], frame[offset+1], frame[offset+2]
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
       Score = count of plausible frames (0.1..500 cm, st==0); tiebreaker = lower variance.
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

# ---------- MAIN LOOP ----------
def main():
    if not MEDIA_FILE.exists():
        raise SystemExit(f"Media file missing: {MEDIA_FILE}")

    ser, dist_offset = try_open_and_autoconfig()
    if not ser:
        raise SystemExit("No valid TOFSense frames. Check wiring, UART Active mode, and baud.")

    medbuf = deque(maxlen=MED_WIN)
    ema = None
    in_count = 0
    dbg = 0

    try:
        while True:
            fr = sync_and_read_frame(ser, timeout=1.0)
            if not fr:
                continue

            dist_cm, status, signal = fields(fr, dist_offset)

            # hard gates
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

            # in-zone counting
            if ZONE_MIN_CM <= smoothed <= ZONE_MAX_CM:
                in_count += 1
                if in_count >= ENTER_FRAMES:
                    print(f"[TRIGGER] {smoothed:.1f} cm → Play MP4 (blocking)")
                    # Block here until the video completes. While playing, do NOTHING.
                    play_media_and_wait()

                    # After mpv exits: flush serial input, brief cooldown, then resume sensing
                    try:
                        ser.reset_input_buffer()
                    except Exception:
                        pass
                    time.sleep(COOLDOWN_AFTER_END)
                    in_count = 0
                    ema = None
            else:
                in_count = 0

    except KeyboardInterrupt:
        pass
    finally:
        try: ser.close()
        except Exception: pass


if __name__ == "__main__":
    main()
