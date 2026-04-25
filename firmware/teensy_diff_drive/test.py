#!/usr/bin/env python3
"""Simple bench-test tool for the Teensy diff-drive bridge.

Connects to /dev/ttyACM0 @ 115200, sends one command, samples encoder
feedback for a short window, prints a compact summary, then stops.

Quick usage:
  ./test.py u 0.05 0           # left 5% duty, right 0, for 2s (default)
  ./test.py u 0.05 -0.05 3     # both, opposite directions, 3s
  ./test.py v 100 0            # left 100 RPM velocity mode (requires PID tuned)
  ./test.py s                  # stop
  ./test.py d                  # one DIAG line
  ./test.py live               # stream E lines until Ctrl-C
  ./test.py raw "UL0.03"       # send any raw command string

Safety:
  - Default run time = 2 s; max 10 s via --max-time
  - Always sends S on exit (including Ctrl-C)
  - Duty clamped to |d| <= 0.6 (matches firmware MAX_DUTY)
  - RPM clamped to |v| <= 3000 (matches firmware MAX_RPM)
"""

import argparse
import re
import signal
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial missing — install with: pip install pyserial")
    sys.exit(1)

PORT = "/dev/ttyACM0"
BAUD = 115200
E_LINE_RE = re.compile(r"E L(-?\d+) (-?[\d.]+) R(-?\d+) (-?[\d.]+)")


class Teensy:
    def __init__(self, port=PORT, baud=BAUD):
        self.s = serial.Serial(port, baud, timeout=0.3)
        time.sleep(0.3)
        self.s.reset_input_buffer()

    def send(self, line):
        self.s.write((line.rstrip() + "\n").encode())

    def stop(self):
        self.send("S")
        time.sleep(0.1)

    def drain_for(self, duration):
        """Read E lines for `duration` seconds. Returns list of (lrpm, lpos, rrpm, rpos)."""
        samples = []
        non_e_lines = []
        t0 = time.time()
        buf = ""
        while time.time() - t0 < duration:
            chunk = self.s.read(self.s.in_waiting or 256).decode(errors="replace")
            if not chunk:
                time.sleep(0.01)
                continue
            buf += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                m = E_LINE_RE.match(line)
                if m:
                    samples.append((int(m.group(1)), float(m.group(2)),
                                    int(m.group(3)), float(m.group(4))))
                elif line:
                    non_e_lines.append(line)
        return samples, non_e_lines

    def close(self):
        try:
            self.stop()
            self.s.close()
        except Exception:
            pass


def summarize(samples, label):
    if not samples:
        print(f"  [{label}] no encoder samples")
        return
    lrpms = [s[0] for s in samples]
    rrpms = [s[2] for s in samples]
    lposΔ = samples[-1][1] - samples[0][1]
    rposΔ = samples[-1][3] - samples[0][3]
    print(f"  [{label}] n={len(samples)}  "
          f"L rpm [min {min(lrpms):>5} max {max(lrpms):>5} last {lrpms[-1]:>5}]  "
          f"R rpm [min {min(rrpms):>5} max {max(rrpms):>5} last {rrpms[-1]:>5}]  "
          f"Δpos L={lposΔ:+.3f} R={rposΔ:+.3f}")


def cmd_duty(t, left, right, duration):
    left  = max(-0.60, min(0.60, left))
    right = max(-0.60, min(0.60, right))
    print(f">>> DUTY  L={left:+.3f}  R={right:+.3f}  for {duration:.1f}s")
    # Prime: send once, capture short window (0.2s) to confirm ack
    t.send(f"UL{left} UR{right}")
    t.drain_for(0.2)
    # Main window — refresh command every 100ms to keep watchdog happy
    t0 = time.time()
    all_samples = []
    while time.time() - t0 < duration:
        t.send(f"UL{left} UR{right}")
        ss, _ = t.drain_for(min(0.1, duration - (time.time() - t0)))
        all_samples.extend(ss)
    t.stop()
    decel, _ = t.drain_for(0.8)
    summarize(all_samples, "running")
    summarize(decel, "decel after S")


def cmd_velocity(t, left, right, duration):
    left  = max(-3000, min(3000, int(left)))
    right = max(-3000, min(3000, int(right)))
    print(f">>> VEL   L={left} RPM  R={right} RPM  for {duration:.1f}s")
    print("    (requires SparkMAX PID tuned via REV Hardware Client; "
          "if motors don't move, use duty mode instead)")
    t.send(f"L{left} R{right}")
    t.drain_for(0.2)
    t0 = time.time()
    all_samples = []
    while time.time() - t0 < duration:
        t.send(f"L{left} R{right}")
        ss, _ = t.drain_for(min(0.1, duration - (time.time() - t0)))
        all_samples.extend(ss)
    t.stop()
    decel, _ = t.drain_for(0.8)
    summarize(all_samples, "running")
    summarize(decel, "decel after S")


def cmd_diag(t):
    t.send("D")
    _, non_e = t.drain_for(0.3)
    for ln in non_e:
        if ln.startswith("DIAG") or ln.startswith("#"):
            print(ln)


def cmd_live(t):
    print("Streaming E lines. Ctrl-C to stop (sends S first).")
    t.send("D")
    time.sleep(0.2)
    # just dump everything
    try:
        while True:
            raw = t.s.read(t.s.in_waiting or 256).decode(errors="replace")
            if raw:
                sys.stdout.write(raw)
                sys.stdout.flush()
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n^C")


def cmd_raw(t, payload, hold):
    print(f">>> RAW   {payload!r}  hold={hold:.1f}s")
    t.send(payload)
    t0 = time.time()
    all_samples = []
    while time.time() - t0 < hold:
        ss, non_e = t.drain_for(min(0.1, hold - (time.time() - t0)))
        for ln in non_e:
            if ln.startswith("OK") or ln.startswith("ERR") or ln.startswith("#"):
                print(f"    {ln}")
        all_samples.extend(ss)
    summarize(all_samples, "raw")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("u", help="duty cycle")
    p.add_argument("left", type=float)
    p.add_argument("right", type=float)
    p.add_argument("duration", type=float, nargs="?", default=2.0)

    p = sub.add_parser("v", help="velocity (RPM)")
    p.add_argument("left", type=float)
    p.add_argument("right", type=float)
    p.add_argument("duration", type=float, nargs="?", default=2.0)

    sub.add_parser("s", help="stop")
    sub.add_parser("d", help="diag")
    sub.add_parser("live", help="stream E lines")

    p = sub.add_parser("raw", help="send raw serial command")
    p.add_argument("payload")
    p.add_argument("hold", type=float, nargs="?", default=0.3)

    ap.add_argument("--max-time", type=float, default=10.0,
                    help="safety cap on duration (default 10s)")
    args = ap.parse_args()

    # Clamp duration to max-time
    if hasattr(args, "duration"):
        if args.duration > args.max_time:
            print(f"duration {args.duration}s > --max-time {args.max_time}s; clamping.")
            args.duration = args.max_time

    t = Teensy()
    # always stop on exit, including on signals
    def on_exit(*_):
        try: t.stop()
        finally: sys.exit(0)
    signal.signal(signal.SIGINT, on_exit)
    signal.signal(signal.SIGTERM, on_exit)

    try:
        if args.cmd == "u":
            cmd_duty(t, args.left, args.right, args.duration)
        elif args.cmd == "v":
            cmd_velocity(t, args.left, args.right, args.duration)
        elif args.cmd == "s":
            t.stop()
            print("OK stopped")
        elif args.cmd == "d":
            cmd_diag(t)
        elif args.cmd == "live":
            cmd_live(t)
        elif args.cmd == "raw":
            cmd_raw(t, args.payload, args.hold)
    finally:
        t.close()


if __name__ == "__main__":
    main()
