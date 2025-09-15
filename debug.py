#!/usr/bin/env python3
import serial, time, statistics

PORTS = ["/dev/serial0", "/dev/ttyUSB0"]
BAUDS = [921600, 230400, 115200, 460800]
HDR = 0x57
FRAME_LEN = 16

def read_frame(ser, timeout=1.0):
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

def parse(frame):
    # distance: int24 LE, meters*1000 -> treat as mm
    d0, d1, d2 = frame[8], frame[9], frame[10]
    dist_cm = (d0 | (d1 << 8) | (d2 << 16)) / 10.0
    status = frame[11]
    signal = frame[12] | (frame[13] << 8)
    return dist_cm, status, signal

def try_lock():
    for port in PORTS:
        for baud in BAUDS:
            try:
                ser = serial.Serial(port, baudrate=baud, timeout=0.2)
                ok = 0
                first = None
                t0 = time.time()
                while time.time() - t0 < 3.0 and ok < 5:
                    f = read_frame(ser, timeout=0.5)
                    if not f:
                        continue
                    if first is None:
                        first = f
                    ok += 1
                if ok >= 5:
                    fid = first[3]
                    mark = first[1]
                    print(f"[LOCK] port={port} baud={baud} id=0x{fid:02X} mark=0x{mark:02X}")
                    return ser, baud
                ser.close()
            except Exception:
                try: ser.close()
                except Exception: pass
    return None, None

def main():
    ser, baud = try_lock()
    if not ser:
        print("No valid frames. Check wiring, mode (UART Active), and baud.")
        return
    try:
        print("Printing distance/status/signal… Ctrl+C to stop")
        while True:
            f = read_frame(ser, timeout=1.0)
            if not f:
                print(".", end="", flush=True)
                continue
            dist_cm, status, signal = parse(f)
            print(f"{dist_cm:7.1f} cm  status={status}  signal={signal}")
    except KeyboardInterrupt:
        pass
    finally:
        try: ser.close()
        except Exception: pass

if __name__ == "__main__":
    main()
