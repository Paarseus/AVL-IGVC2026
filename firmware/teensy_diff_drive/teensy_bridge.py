"""Shared helper module for phase bench-test scripts.

Talks to teensy_diff_drive.ino over USB-serial. Every phase script imports
this module; nothing else should touch /dev/ttyACM* directly.

Design notes:
  - One class (Teensy) for the serial connection and E-line parsing.
  - Plain functions for common command patterns (set_gains, run_duty,
    run_velocity). Each returns a dict of stats — no dataclasses.
  - CSV logger writes data/phase<n>_YYYYMMDD_HHMMSS.csv with timestamped
    encoder samples. Re-analysable in a spreadsheet.
  - install_stop_handler() ensures any script sends S and closes the port
    on Ctrl-C or kill.
"""

from __future__ import annotations

import csv
import os
import re
import signal
import sys
import time
from contextlib import contextmanager

import serial

# ---------------------------------------------------------------------------
PORT_DEFAULT = "/dev/ttyACM0"
BAUD = 115200

E_LINE_RE = re.compile(r"E L(-?\d+) (-?[\d.]+) R(-?\d+) (-?[\d.]+)")
DIAG_RE   = re.compile(
    r"DIAG tx=(\d+) rx=(\d+) wdt=(\d+) mode=(\w+) "
    r"L=(-?[\d.]+)/(-?[\d.]+) R=(-?[\d.]+)/(-?[\d.]+) "
    r"duty L=(-?[\d.]+) R=(-?[\d.]+)(?: V=(-?[\d.]+))?"
)


# ---------------------------------------------------------------------------
class Teensy:
    """USB-serial connection to teensy_diff_drive.ino firmware."""

    def __init__(self, port: str = PORT_DEFAULT, baud: int = BAUD):
        self.port = port
        self.s = serial.Serial(port, baud, timeout=0.3)
        time.sleep(0.3)
        self.s.reset_input_buffer()

    def send(self, line: str) -> None:
        self.s.write((line.rstrip() + "\n").encode())

    def stop(self) -> None:
        try:
            self.send("S")
            time.sleep(0.1)
        except Exception:
            pass

    def close(self) -> None:
        try:
            self.stop()
        finally:
            try: self.s.close()
            except Exception: pass

    def drain_for(self, duration: float, refresh: str | None = None,
                  refresh_period: float = 0.1):
        """Read for `duration` seconds. Optionally re-send `refresh` every
        `refresh_period` seconds to keep the firmware watchdog alive.

        Returns (e_samples, other_lines) where e_samples is a list of
        (t, lrpm, lpos, rrpm, rpos) tuples and other_lines are non-E
        lines (OK/ERR/#/DIAG)."""
        samples = []
        other = []
        t0 = time.time()
        last_refresh = 0.0
        buf = ""
        while time.time() - t0 < duration:
            if refresh and time.time() - last_refresh > refresh_period:
                self.send(refresh)
                last_refresh = time.time()
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
                    samples.append((time.time() - t0, int(m.group(1)),
                                    float(m.group(2)), int(m.group(3)),
                                    float(m.group(4))))
                elif line:
                    other.append(line)
        return samples, other

    def diag(self) -> dict | None:
        """Send D, parse one DIAG line, return fields as dict. Returns None
        if no DIAG line arrives within 300 ms."""
        self.s.reset_input_buffer()
        self.send("D")
        t0 = time.time()
        buf = ""
        while time.time() - t0 < 0.3:
            buf += self.s.read(self.s.in_waiting or 256).decode(errors="replace")
            for line in buf.splitlines():
                m = DIAG_RE.match(line.strip())
                if m:
                    return {
                        "tx": int(m.group(1)),
                        "rx": int(m.group(2)),
                        "wdt": int(m.group(3)),
                        "mode": m.group(4),
                        "l_meas_rpm": float(m.group(5)),
                        "l_cmd_rpm":  float(m.group(6)),
                        "r_meas_rpm": float(m.group(7)),
                        "r_cmd_rpm":  float(m.group(8)),
                        "l_duty": float(m.group(9)),
                        "r_duty": float(m.group(10)),
                        "v_bus": float(m.group(11)) if m.group(11) else None,
                    }
        return None


# ---------------------------------------------------------------------------
def set_gains(t: Teensy, kp=None, ki=None, kd=None, kf=None,
              gap: float = 0.20) -> list[str]:
    """Write any subset of kP/kI/kD/kF. Returns list of OK/ERR strings.
    `gap` is the sleep between consecutive writes (prevents CAN flooding
    and the USB-reset we saw with 50 ms gaps)."""
    acks = []
    for name, val in [("KF", kf), ("KP", kp), ("KI", ki), ("KD", kd)]:
        if val is None:
            continue
        t.send(f"{name}{val}")
        time.sleep(gap)
        raw = t.s.read(t.s.in_waiting or 256).decode(errors="replace")
        for line in raw.splitlines():
            if line.startswith(("OK ", "ERR ")):
                acks.append(line.strip())
    return acks


def stats(values: list[float]) -> dict:
    """Return mean/std/min/max of a list. Empty list returns zeros."""
    if not values:
        return {"n": 0, "mean": 0.0, "std": 0.0, "min": 0.0, "max": 0.0}
    n = len(values)
    mean = sum(values) / n
    var = sum((v - mean) ** 2 for v in values) / n
    return {"n": n, "mean": mean, "std": var ** 0.5,
            "min": min(values), "max": max(values)}


def run_duty(t: Teensy, left: float, right: float, duration: float,
             settle_frac: float = 0.4) -> dict:
    """Send UL/UR duty setpoints for `duration`, collect encoder samples.
    Returns {"samples": [...], "l_ss": stats, "r_ss": stats, "posΔ_l":,
    "posΔ_r":, "duration":}. Steady-state stats use the last (1 - settle_frac)
    fraction of samples."""
    refresh = f"UL{left} UR{right}"
    t.send(refresh)
    samples, _ = t.drain_for(duration, refresh=refresh)
    t.stop()
    # brief drain for decel
    _, _ = t.drain_for(0.3)
    if not samples:
        return {"samples": [], "l_ss": stats([]), "r_ss": stats([]),
                "posΔ_l": 0.0, "posΔ_r": 0.0, "duration": 0.0}
    cut = int(len(samples) * settle_frac)
    ss = samples[cut:]
    return {
        "samples": samples,
        "l_ss": stats([s[1] for s in ss]),
        "r_ss": stats([s[3] for s in ss]),
        "posΔ_l": samples[-1][2] - samples[0][2],
        "posΔ_r": samples[-1][4] - samples[0][4],
        "duration": samples[-1][0] - samples[0][0],
    }


def run_velocity(t: Teensy, left: float, right: float, duration: float,
                 settle_frac: float = 0.4) -> dict:
    """Same as run_duty but commands velocity (L/R RPM)."""
    refresh = f"L{left} R{right}"
    t.send(refresh)
    samples, _ = t.drain_for(duration, refresh=refresh)
    t.stop()
    _, _ = t.drain_for(0.3)
    if not samples:
        return {"samples": [], "l_ss": stats([]), "r_ss": stats([]),
                "posΔ_l": 0.0, "posΔ_r": 0.0, "duration": 0.0}
    cut = int(len(samples) * settle_frac)
    ss = samples[cut:]
    return {
        "samples": samples,
        "l_ss": stats([s[1] for s in ss]),
        "r_ss": stats([s[3] for s in ss]),
        "posΔ_l": samples[-1][2] - samples[0][2],
        "posΔ_r": samples[-1][4] - samples[0][4],
        "duration": samples[-1][0] - samples[0][0],
    }


# ---------------------------------------------------------------------------
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


@contextmanager
def open_log(phase: str, columns: list[str]):
    """Open a CSV file at data/<phase>_YYYYMMDD_HHMMSS.csv. Yields a
    writer with the given header row already written. File auto-closes."""
    os.makedirs(DATA_DIR, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(DATA_DIR, f"{phase}_{ts}.csv")
    f = open(path, "w", newline="")
    try:
        w = csv.writer(f)
        w.writerow(columns)
        print(f"[log] writing {path}")
        yield w, path
    finally:
        f.close()


def install_stop_handler(t: Teensy) -> None:
    """SIGINT/SIGTERM → t.stop() + sys.exit(0). Call once at top of each
    phase script after Teensy() construction."""
    def handler(_signum, _frame):
        print("\n[aborted — sending S]", file=sys.stderr)
        t.close()
        sys.exit(0)
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
