#!/usr/bin/env python3
"""Phase 3 — Stiction sweep (minimum duty to start motion).

Finds the lowest duty that produces meas_rpm > 20 RPM on each wheel.
Tests each wheel independently (the other held at 0). This isolates
per-side stiction differences.

Risk: low. Duties capped at 0.15. Motors may briefly spin at low RPMs.
Total motor-on time: ~16 seconds (15 steps x 0.8s).

Pass criteria:
  - Both wheels start within ±0.02 of each other
  - Stiction duty typically 0.03 - 0.08 on a healthy Raptor/NEO drivetrain
  - Anything above 0.10 is a yellow flag for excessive drag
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teensy_bridge import Teensy, install_stop_handler, open_log, run_duty

STEP      = 0.01
DUTY_MAX  = 0.15
RPM_THRESHOLD = 20     # meas_rpm at which we consider motion started
DURATION  = 0.8        # hold each duty for this long


def sweep_side(t: Teensy, side: str, csv_w) -> float | None:
    """Sweep duty on `side` (L or R) until motion is detected.
    Returns the first duty at which |meas_rpm| > RPM_THRESHOLD, or None."""
    duty = 0.0
    while duty <= DUTY_MAX + 1e-9:
        duty = round(duty, 3)
        l = duty if side == "L" else 0.0
        r = duty if side == "R" else 0.0
        result = run_duty(t, l, r, DURATION)
        # Use the relevant side's steady-state mean
        key = "l_ss" if side == "L" else "r_ss"
        ss = result[key]
        pos_delta = result["posΔ_l"] if side == "L" else result["posΔ_r"]
        print(f"  {side} duty={duty:.3f}  mean_rpm={ss['mean']:>6.1f}  "
              f"|max|={max(abs(ss['min']), abs(ss['max'])):>6.1f}  Δpos={pos_delta:+.3f}")
        csv_w.writerow([side, f"{duty:.3f}", ss["n"], f"{ss['mean']:.2f}",
                        f"{ss['std']:.2f}", f"{ss['min']:.1f}",
                        f"{ss['max']:.1f}", f"{pos_delta:.4f}"])
        if abs(ss["mean"]) > RPM_THRESHOLD:
            print(f"  >> {side} started moving at duty={duty:.3f}")
            return duty
        duty += STEP
    return None


def main():
    print("Phase 3 — Stiction sweep (per-wheel minimum duty to start)")
    t = Teensy()
    install_stop_handler(t)

    # Make sure we start clean
    t.stop()
    time.sleep(0.3)

    results = {}
    with open_log("phase3_stiction",
                  ["side", "duty", "n", "mean_rpm", "std", "min", "max", "pos_delta"]) as (w, path):
        for side in ("L", "R"):
            print(f"\n--- {side} side ---")
            results[side] = sweep_side(t, side, w)
            time.sleep(0.5)

    t.close()

    print("\n--- ANALYSIS ---")
    l = results["L"]; r = results["R"]
    if l is None:
        print(f"  !! LEFT never started within duty ≤ {DUTY_MAX} — drivetrain locked or cable issue")
    if r is None:
        print(f"  !! RIGHT never started within duty ≤ {DUTY_MAX} — drivetrain locked or cable issue")
    if l is not None and r is not None:
        print(f"  L stiction: {l:.3f} duty")
        print(f"  R stiction: {r:.3f} duty")
        diff = abs(l - r)
        print(f"  |L - R|   : {diff:.3f} duty")
        if diff <= 0.02:
            print("  ✓ symmetric stiction — electrical or mechanical asymmetry cannot be attributed to starting drag")
        else:
            harder = "R" if l < r else "L"
            print(f"  !! asymmetric stiction — {harder} side needs {diff:.3f} more duty to start")
        if max(l, r) > 0.10:
            print(f"  !! high stiction — >0.10 duty to start suggests excess drag (bearings/tension)")
        elif max(l, r) < 0.05:
            print("  ✓ stiction in healthy range (<0.05)")

    print(f"\n[log] {path}")


if __name__ == "__main__":
    main()
