#!/usr/bin/env python3
"""
serial_bridge.py — Real-time robot telemetry bridge

Reads $S and $C CSV lines from the HC-12 wireless serial receiver (YP-05 USB adapter)
and forwards each field as a UDP datagram to Teleplot (VS Code extension, port 47269).
Also logs every raw line to a timestamped CSV file for post-run analysis.

Usage:
    python tools/serial_bridge.py --port /dev/tty.usbserial-XXXX
    python tools/serial_bridge.py          # lists available serial ports and exits

Install dependency once:
    pip install pyserial
"""

import argparse
import csv
import datetime
import socket
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial not installed. Run:  pip install pyserial")
    sys.exit(1)

TELEPLOT_HOST = "127.0.0.1"
TELEPLOT_PORT = 47269
BAUD = 115200

# Sensor line: $S,irF,irL,irR,irRR,sonar,gyroZ,battV
SENSOR_FIELDS = ["ir_f_mm", "ir_l_mm", "ir_r_mm", "ir_rr_mm",
                 "sonar_cm", "gyro_z", "batt_v"]

# Control line: $C,hdgSp,hdgActual,hdgErr,wfSp,wfActual,wfErr,vx,vy,wz,state
CONTROL_FIELDS = ["hdg_sp", "hdg_actual", "hdg_err",
                  "wf_sp", "wf_actual", "wf_err",
                  "vx", "vy", "wz", "state"]


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
    else:
        print("Available serial ports:")
        for p in ports:
            print(f"  {p.device:30s}  {p.description}")


def send_to_teleplot(sock, name, value):
    msg = f"{name}:{value}\n".encode()
    sock.sendto(msg, (TELEPLOT_HOST, TELEPLOT_PORT))


def parse_and_forward(line, sock, fields):
    parts = line.split(",")
    # parts[0] is the prefix ($S or $C), data starts at parts[1]
    values = parts[1:]
    if len(values) != len(fields):
        return  # malformed line, skip silently
    for name, raw in zip(fields, values):
        try:
            send_to_teleplot(sock, name, float(raw))
        except ValueError:
            pass  # non-numeric field (shouldn't happen), skip


def open_log_file():
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"telemetry_log_{ts}.csv"
    f = open(filename, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(["wall_time"] + SENSOR_FIELDS + CONTROL_FIELDS)
    print(f"Logging to {filename}")
    return f, writer


def main():
    parser = argparse.ArgumentParser(description="Robot telemetry → Teleplot bridge")
    parser.add_argument("--port", help="Serial port for HC-12 USB receiver")
    args = parser.parse_args()

    if not args.port:
        list_ports()
        print("\nRe-run with --port <port> to start streaming.")
        sys.exit(0)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    log_file, log_writer = open_log_file()

    # Live terminal summary state
    summary = {k: "—" for k in SENSOR_FIELDS + CONTROL_FIELDS}
    last_summary_time = time.time()
    line_count = 0

    print(f"Opening {args.port} at {BAUD} baud...")
    try:
        ser = serial.Serial(args.port, BAUD, timeout=1.0)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    print("Connected. Open Teleplot panel in VS Code to see live graphs.\n"
          "Press Ctrl+C to stop.\n")

    try:
        # Partial-line buffer for cross-chunk line assembly
        buf = ""
        while True:
            chunk = ser.read(64).decode("ascii", errors="replace")
            buf += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                line_count += 1

                if line.startswith("$S,"):
                    parse_and_forward(line, sock, SENSOR_FIELDS)
                    # Update summary values
                    parts = line.split(",")[1:]
                    for k, v in zip(SENSOR_FIELDS, parts):
                        summary[k] = v
                elif line.startswith("$C,"):
                    parse_and_forward(line, sock, CONTROL_FIELDS)
                    parts = line.split(",")[1:]
                    for k, v in zip(CONTROL_FIELDS, parts):
                        summary[k] = v

                # Log every raw line with wall clock
                log_writer.writerow(
                    [datetime.datetime.now().isoformat()] +
                    [summary.get(k, "") for k in SENSOR_FIELDS + CONTROL_FIELDS]
                )
                log_file.flush()

            # Print live terminal summary once per second
            now = time.time()
            if now - last_summary_time >= 1.0:
                last_summary_time = now
                ir = (f"ir F:{summary['ir_f_mm']:>6} "
                      f"L:{summary['ir_l_mm']:>6} "
                      f"R:{summary['ir_r_mm']:>6} "
                      f"RR:{summary['ir_rr_mm']:>6} mm")
                ctrl = (f"hdg err:{summary['hdg_err']:>7}° "
                        f"wf err:{summary['wf_err']:>7} mm "
                        f"batt:{summary['batt_v']:>4} V "
                        f"state:{summary['state']}")
                print(f"\r{ir}  |  {ctrl}  [{line_count} lines]", end="", flush=True)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()
        log_file.close()
        sock.close()


if __name__ == "__main__":
    main()
