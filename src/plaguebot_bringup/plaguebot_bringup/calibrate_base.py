#!/usr/bin/env python3
"""Interactive base-calibration helper for the Plague-Bot ESP32 Maestro.

Standalone (pyserial only) — the esp32_bridge must NOT be running, since both
would fight for /dev/ttyUSB1. Talks the same protocol as the bridge:
  telemetry IN : E1,E2,E3,E4,AccX,AccY,AccZ,GyrX,GyrY,GyrZ\\n  (50 Hz)
  drive OUT    : D,M1_R,M1_L,M2_R,M2_L,M3_R,M3_L,M4_R,M4_L\\n  (PWM 0-255)

Motor forward = the '_R' channel per Adan (D,127,0,...  drives M1 forward).

SAFETY: 'pulse' spins ONE motor for a bounded time then brakes. Keep the
wheels OFF THE GROUND for mapping. Ctrl-C brakes and exits.

Usage:
  python3 calibrate_base.py watch                 # live encoder deltas (hand-spin wheels)
  python3 calibrate_base.py pulse <motor 0-3> [pwm] [ms]   # power ONE motor forward briefly
  python3 calibrate_base.py verify [pwm] [ms]     # drive each motor FORWARD via motor_signs, check enc sign
  python3 calibrate_base.py revs <enc 1-4> <n_turns>  # hand-spin N full turns -> ticks_per_rev
  python3 calibrate_base.py brake                 # emergency stop
"""
import sys
import time

import serial

PORT = "/dev/ttyUSB1"
BAUD = 115200

# Keep in sync with esp32_bridge 'motor_signs'. M4 (rear-right) is wired
# reversed, so its forward direction lives on the _L channel.
MOTOR_SIGNS = [1.0, 1.0, 1.0, -1.0]


def open_port():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    ser.reset_input_buffer()
    return ser


def read_enc(ser):
    """Return [E1,E2,E3,E4] ints from the next valid telemetry line, or None."""
    raw = ser.readline().decode("ascii", errors="replace").strip()
    parts = raw.split(",")
    if len(parts) != 10:
        return None
    try:
        return [int(float(p)) for p in parts[:4]]
    except ValueError:
        return None


def brake(ser):
    ser.write(b"D,0,0,0,0,0,0,0,0\n")
    ser.flush()


def watch(ser):
    base = None
    print("Hand-spin each wheel FORWARD. Watch which E# changes and its sign.")
    print("Columns: E1 E2 E3 E4  (delta from start)   [Ctrl-C to stop]\n")
    try:
        while True:
            enc = read_enc(ser)
            if enc is None:
                continue
            if base is None:
                base = enc
            d = [enc[i] - base[i] for i in range(4)]
            print(f"\rE1={enc[0]:>7} E2={enc[1]:>7} E3={enc[2]:>7} E4={enc[3]:>7}   "
                  f"dE=[{d[0]:>6} {d[1]:>6} {d[2]:>6} {d[3]:>6}]", end="", flush=True)
    except KeyboardInterrupt:
        print("\nstopped.")


def pulse(ser, motor, pwm, ms):
    """Drive one motor forward (its _R channel) for ms, print encoder delta."""
    fields = [0] * 8
    fields[2 * motor] = pwm  # forward channel of motor `motor`
    cmd = "D," + ",".join(str(v) for v in fields) + "\n"

    # baseline
    e0 = None
    t_end = time.time() + 0.3
    while time.time() < t_end and e0 is None:
        e0 = read_enc(ser)
    if e0 is None:
        print("no telemetry; is the ESP32 on and the bridge stopped?")
        return

    print(f"Pulsing motor M{motor+1} FORWARD at pwm={pwm} for {ms} ms ...")
    t_end = time.time() + ms / 1000.0
    last = e0
    while time.time() < t_end:
        ser.write(cmd.encode("ascii"))
        ser.flush()
        e = read_enc(ser)
        if e is not None:
            last = e
        time.sleep(0.01)
    brake(ser)
    time.sleep(0.1)
    # settle read
    for _ in range(3):
        e = read_enc(ser)
        if e is not None:
            last = e
    d = [last[i] - e0[i] for i in range(4)]
    print(f"encoder delta during pulse: dE=[{d[0]} {d[1]} {d[2]} {d[3]}]")
    print("-> the encoder(s) that moved belong to this motor; sign = drive direction.")


def _forward_cmd(motor, pwm):
    """Encode a FORWARD command for one motor the way the bridge does:
    apply motor_signs, then split into the M#_R / M#_L PWM channels."""
    signed = pwm * MOTOR_SIGNS[motor]
    fields = [0] * 8
    if signed >= 0:
        fields[2 * motor] = int(round(signed))       # M#_R
    else:
        fields[2 * motor + 1] = int(round(-signed))  # M#_L
    return "D," + ",".join(str(v) for v in fields) + "\n"


def verify(ser, pwm, ms):
    """Drive each motor FORWARD (via motor_signs) and confirm its encoder goes +."""
    ok = True
    for motor in range(4):
        e0 = None
        t_end = time.time() + 0.3
        while time.time() < t_end and e0 is None:
            e0 = read_enc(ser)
        if e0 is None:
            print("no telemetry; is the ESP32 on and the bridge stopped?")
            return
        cmd = _forward_cmd(motor, pwm).encode("ascii")
        t_end = time.time() + ms / 1000.0
        last = e0
        while time.time() < t_end:
            ser.write(cmd)
            ser.flush()
            e = read_enc(ser)
            if e is not None:
                last = e
            time.sleep(0.01)
        brake(ser)
        time.sleep(0.15)
        for _ in range(3):
            e = read_enc(ser)
            if e is not None:
                last = e
        d = [last[i] - e0[i] for i in range(4)]
        idx = max(range(4), key=lambda i: abs(d[i]))
        sign = "+" if d[idx] > 0 else "-"
        good = d[idx] > 0
        ok = ok and good
        mark = "OK" if good else "MAL (deberia ser +)"
        print(f"M{motor+1} adelante -> E{idx+1} dE={d[idx]:>6}  signo {sign}  [{mark}]")
        time.sleep(0.3)
    print("\nRESULTADO:", "las 4 llantas giran ADELANTE (+)" if ok
          else "hay un motor con signo incorrecto; revisar mapeo")


def revs(ser, enc_idx, n_turns):
    """Hand-spin encoder `enc_idx` (0-3) exactly n_turns FORWARD, then stop.
    Auto-detects the stop and prints ticks_per_rev = |delta| / n_turns."""
    base = None
    t0 = time.time()
    while base is None and time.time() - t0 < 3:
        base = read_enc(ser)
    if base is None:
        print("no telemetry; is the ESP32 on and the bridge stopped?")
        return
    print(f"Gira la rueda del encoder E{enc_idx+1} EXACTAMENTE {n_turns} "
          f"vueltas completas hacia ADELANTE, despacio, y para.")
    print("(me auto-corto ~2.5 s despues de que dejes de girar)\n")
    last = base
    moved = False
    last_change_t = time.time()
    deadline = time.time() + 120
    prev = base[enc_idx]
    last_print = 0.0
    while time.time() < deadline:
        e = read_enc(ser)
        if e is None:
            continue
        last = e
        d = e[enc_idx] - base[enc_idx]
        if abs(e[enc_idx] - prev) > 2:
            last_change_t = time.time()
            if abs(d) > 20:
                moved = True
            prev = e[enc_idx]
        now = time.time()
        if now - last_print > 0.3:  # throttle: pipe-friendly (no live \r overwrite)
            print(f"E{enc_idx+1} delta = {d:>8}", flush=True)
            last_print = now
        if moved and now - last_change_t > 2.5:
            break
    delta = abs(last[enc_idx] - base[enc_idx])
    print(f"\n\ndelta total E{enc_idx+1} = {delta} ticks en {n_turns} vueltas")
    if n_turns > 0 and delta > 0:
        tpr = delta / n_turns
        print(f"-> ticks_per_rev = {tpr:.1f}")
        print(f"   (pon ticks_per_rev: {round(tpr)} en el bridge)")
    else:
        print("-> no detecte giro suficiente; reintenta.")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    mode = sys.argv[1]
    ser = open_port()
    try:
        if mode == "watch":
            watch(ser)
        elif mode == "verify":
            pwm = int(sys.argv[2]) if len(sys.argv) > 2 else 60
            ms = int(sys.argv[3]) if len(sys.argv) > 3 else 700
            verify(ser, max(0, min(255, pwm)), ms)
        elif mode == "revs":
            enc_idx = int(sys.argv[2]) - 1
            n_turns = float(sys.argv[3])
            if not 0 <= enc_idx <= 3:
                print("encoder debe ser 1..4")
                return
            revs(ser, enc_idx, n_turns)
        elif mode == "brake":
            brake(ser)
            print("braked.")
        elif mode == "pulse":
            motor = int(sys.argv[2])
            pwm = int(sys.argv[3]) if len(sys.argv) > 3 else 60
            ms = int(sys.argv[4]) if len(sys.argv) > 4 else 600
            if not 0 <= motor <= 3:
                print("motor must be 0..3 (M1..M4)")
                return
            pwm = max(0, min(255, pwm))
            pulse(ser, motor, pwm, ms)
        else:
            print(__doc__)
    finally:
        try:
            brake(ser)
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
