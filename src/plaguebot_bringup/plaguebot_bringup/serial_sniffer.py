#!/usr/bin/env python3
"""Read-only sniffer for the Plague-Bot ESP32 (Adan's protocol).

Listens on the serial port and prints the 10-value telemetry:
    E1,E2,E3,E4,AccX,AccY,AccZ,GyrX,GyrY,GyrZ

Does NOT send anything. Does NOT move the robot. Spin each wheel BY HAND
and watch which encoder (E1..E4) changes to map encoder -> corner.

Usage:
    python3 serial_sniffer.py [port] [baud]
    (defaults: /dev/ttyUSB0 115200)
Ctrl-C to stop. Prints a per-encoder min/max/delta summary on exit.
"""
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial no instalado. Instala con: pip install pyserial "
             "(o: sudo apt install python3-serial)")

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

LABELS = ["E1", "E2", "E3", "E4",
          "AccX", "AccY", "AccZ", "GyrX", "GyrY", "GyrZ"]


def main():
    print(f"Abriendo {PORT} @ {BAUD} (8N1, solo lectura)...")
    with serial.Serial(PORT, BAUD, timeout=1.0) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        print("Escuchando. Gira cada rueda A MANO y observa que E# cambia.")
        print("Ctrl-C para terminar.\n")
        enc_min = [None] * 4
        enc_max = [None] * 4
        n = 0
        last_print = 0.0
        while True:
            raw = ser.readline().decode("ascii", errors="replace").strip()
            if not raw:
                continue
            parts = raw.split(",")
            if len(parts) != 10:
                # Show malformed lines occasionally so we notice framing issues.
                if n % 50 == 0:
                    print(f"  [linea con {len(parts)} campos, ignorada]: {raw!r}")
                n += 1
                continue
            try:
                vals = [float(p) for p in parts]
            except ValueError:
                continue
            encs = [int(round(v)) for v in vals[:4]]
            for i, e in enumerate(encs):
                enc_min[i] = e if enc_min[i] is None else min(enc_min[i], e)
                enc_max[i] = e if enc_max[i] is None else max(enc_max[i], e)
            n += 1
            now = time.time()
            if now - last_print >= 0.2:  # ~5 Hz screen refresh
                enc_str = "  ".join(f"{LABELS[i]}={encs[i]:>7d}" for i in range(4))
                imu_str = "  ".join(f"{LABELS[i]}={vals[i]:+7.3f}"
                                    for i in range(4, 10))
                print(f"\r{enc_str}   |   {imu_str}", end="", flush=True)
                last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n--- resumen encoders (min / max / rango) ---")
