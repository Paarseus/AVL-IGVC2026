#!/usr/bin/env python3
"""Phase 2 — Baseline DIAG + voltage + frame-rate check.

Zero motor motion. Just polls the firmware for 5 seconds, verifies:
  - DIAG parses correctly
  - bus voltage is within 11.5-13.0 V (12V pack healthy)
  - CAN rx rate is much higher than tx rate (both SparkMAXes emitting
    STATUS_0/STATUS_2 frames)
  - Encoders at rest read 0 RPM on both wheels

Writes data/phase2_baseline_<ts>.csv with per-second DIAG snapshots so
later phases can diff counters against this baseline.
"""

import os
import sys
import time

# Import shared helper
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teensy_bridge import Teensy, install_stop_handler, open_log


def main():
    print("Phase 2 — baseline health check (no motor motion)")
    t = Teensy()
    install_stop_handler(t)

    # Five DIAG snapshots, one per second
    snapshots = []
    with open_log("phase2_baseline",
                  ["t_s", "tx", "rx", "wdt", "mode",
                   "l_meas_rpm", "l_cmd_rpm", "r_meas_rpm", "r_cmd_rpm",
                   "l_duty", "r_duty", "v_bus"]) as (w, path):
        t0 = time.time()
        for i in range(5):
            time.sleep(1.0)
            d = t.diag()
            if d is None:
                print(f"  [{i}] no DIAG reply!")
                continue
            elapsed = time.time() - t0
            print(f"  [{i}] t={elapsed:4.1f}s tx={d['tx']:>6} rx={d['rx']:>6} "
                  f"V={d['v_bus']:.2f}  L={d['l_meas_rpm']:>5.0f} R={d['r_meas_rpm']:>5.0f}")
            snapshots.append((elapsed, d))
            w.writerow([f"{elapsed:.3f}", d["tx"], d["rx"], d["wdt"], d["mode"],
                        d["l_meas_rpm"], d["l_cmd_rpm"], d["r_meas_rpm"], d["r_cmd_rpm"],
                        d["l_duty"], d["r_duty"],
                        f"{d['v_bus']:.3f}" if d["v_bus"] is not None else ""])

    if len(snapshots) < 2:
        print("\n!! insufficient DIAG replies — firmware may not be running")
        t.close()
        return 1

    # Compute tx/rx rates over the window
    t_start, d_start = snapshots[0]
    t_end,   d_end   = snapshots[-1]
    dt = t_end - t_start
    tx_rate = (d_end["tx"] - d_start["tx"]) / dt
    rx_rate = (d_end["rx"] - d_start["rx"]) / dt

    print("\n--- ANALYSIS ---")
    print(f"  TX rate: {tx_rate:6.1f} frames/s   (expect ~200/s: heartbeats + setpoints)")
    print(f"  RX rate: {rx_rate:6.1f} frames/s   (expect ~500/s: STATUS_0 at 200Hz x2 + STATUS_2 + STATUS_1)")
    print(f"  rx/tx:   {rx_rate/max(tx_rate,1):.1f}x  (expect ~2.5x)")

    v = d_end["v_bus"]
    if v is None:
        print("  !! voltage not reported — firmware may predate voltage decode")
    elif v < 11.5:
        print(f"  !! bus voltage {v:.2f} V is LOW (<11.5 V) — charge/replace battery")
    elif v > 13.0:
        print(f"  !! bus voltage {v:.2f} V is HIGH (>13.0 V) — check supply")
    else:
        print(f"  ✓ bus voltage {v:.2f} V is in healthy 11.5-13.0 V range")

    if abs(d_end["l_meas_rpm"]) > 10 or abs(d_end["r_meas_rpm"]) > 10:
        print(f"  !! motors not at rest: L={d_end['l_meas_rpm']:.0f} R={d_end['r_meas_rpm']:.0f}")

    if rx_rate < 200:
        print(f"  !! rx rate low — SparkMAXes may not both be responding")
    elif rx_rate < 400:
        print(f"  !! rx rate below expected (only one SparkMAX responding?)")
    else:
        print("  ✓ CAN traffic healthy")

    print(f"\n[log] {path}")
    t.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
