#!/usr/bin/env python3
"""Phase 4 — Duty vs steady-state RPM curve (per wheel).

Sweeps duty from 0.05 to 0.30 in 0.05 steps for each wheel independently.
Records steady-state RPM + variance. Produces the "load line" — a plot of
duty vs achievable RPM. Extrapolating to 100% duty estimates max achievable
RPM under current drivetrain load.

Firmware MAX_DUTY=0.30 is our ceiling here — we don't raise it because
the firmware is the reference. To go higher, use Phase 6 (PID) which
commands RPM and the SparkMAX internally drives up to 100% duty if needed.

Risk: low-medium. Motors spin at up to 30% duty. Active brake stops them
in ~0.3s between steps. Total motor-on time: ~24 seconds (6 steps x 2s x 2 wheels).
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teensy_bridge import Teensy, install_stop_handler, open_log, run_duty

DUTIES   = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30]
DURATION = 2.0
M_PER_REV = 0.01994  # AndyMark Raptor (from CLAUDE.md + research)


def sweep_side(t: Teensy, side: str, csv_w) -> list[tuple[float, float, float]]:
    """Returns list of (duty, mean_rpm, std_rpm) for the sweep."""
    rows = []
    print(f"\n--- {side} side ---")
    print(f"  {'duty':>5}  {'n':>4}  {'mean':>7}  {'std':>6}  {'min':>5}  {'max':>5}  {'m/s':>6}")
    for duty in DUTIES:
        l = duty if side == "L" else 0.0
        r = duty if side == "R" else 0.0
        result = run_duty(t, l, r, DURATION)
        ss = result["l_ss" if side == "L" else "r_ss"]
        mps = ss["mean"] * M_PER_REV / 60
        print(f"  {duty:>5.2f}  {ss['n']:>4}  {ss['mean']:>7.1f}  {ss['std']:>6.2f}  "
              f"{ss['min']:>5.0f}  {ss['max']:>5.0f}  {mps:>6.3f}")
        csv_w.writerow([side, f"{duty:.3f}", ss["n"], f"{ss['mean']:.2f}",
                        f"{ss['std']:.2f}", f"{ss['min']:.1f}",
                        f"{ss['max']:.1f}", f"{mps:.4f}"])
        rows.append((duty, ss["mean"], ss["std"]))
        time.sleep(0.4)
    return rows


def linear_fit(rows):
    """Simple y = mx + b least-squares fit. Returns (slope, intercept, r2)."""
    if len(rows) < 2:
        return (0.0, 0.0, 0.0)
    xs = [r[0] for r in rows]
    ys = [r[1] for r in rows]
    n = len(xs)
    mx = sum(xs) / n; my = sum(ys) / n
    num = sum((xs[i] - mx) * (ys[i] - my) for i in range(n))
    den = sum((xs[i] - mx) ** 2 for i in range(n))
    if den == 0:
        return (0.0, my, 0.0)
    slope = num / den
    intercept = my - slope * mx
    # r^2
    ss_res = sum((ys[i] - (slope * xs[i] + intercept)) ** 2 for i in range(n))
    ss_tot = sum((ys[i] - my) ** 2 for i in range(n))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    return (slope, intercept, r2)


def main():
    print("Phase 4 — Duty-vs-RPM curve per wheel")
    t = Teensy()
    install_stop_handler(t)

    t.stop()
    time.sleep(0.3)

    with open_log("phase4_duty_sweep",
                  ["side", "duty", "n", "mean_rpm", "std", "min", "max", "m_per_s"]) as (w, path):
        l_rows = sweep_side(t, "L", w)
        r_rows = sweep_side(t, "R", w)

    t.close()

    # Fit + extrapolate
    lm, lb, lr2 = linear_fit(l_rows)
    rm, rb, rr2 = linear_fit(r_rows)
    l_max = lm * 1.0 + lb  # extrapolate to 100% duty
    r_max = rm * 1.0 + rb
    l_max_mps = l_max * M_PER_REV / 60
    r_max_mps = r_max * M_PER_REV / 60

    print("\n--- LINEAR FIT (RPM = slope × duty + intercept) ---")
    print(f"  L: slope={lm:>9.1f} RPM/duty   intercept={lb:>7.1f}   r²={lr2:.3f}")
    print(f"  R: slope={rm:>9.1f} RPM/duty   intercept={rb:>7.1f}   r²={rr2:.3f}")
    print("\n--- EXTRAPOLATED MAX (@ 100% duty under current load) ---")
    print(f"  L: {l_max:>6.0f} RPM = {l_max_mps:.3f} m/s")
    print(f"  R: {r_max:>6.0f} RPM = {r_max_mps:.3f} m/s")
    print(f"  ratio (R/L): {r_max/l_max*100:.1f}% — R is the slower wheel")

    print("\n--- INTERPRETATION ---")
    free = 5676  # NEO free speed RPM at 12V
    print(f"  NEO free speed = {free} RPM")
    print(f"  L fraction of free: {l_max/free*100:.1f}%")
    print(f"  R fraction of free: {r_max/free*100:.1f}%")
    if r_max / free < 0.70:
        print("  !! Right drivetrain is <70% of free speed — significant mechanical drag")
    if (l_max - r_max) / l_max > 0.05:
        print(f"  !! L/R asymmetry >5% — right side loses {(l_max-r_max)/l_max*100:.1f}% more speed to friction")

    print(f"\n[log] {path}")
    print("\nRecommendation for Phase 6 PID: use kFF = 1/min(L_max, R_max) to avoid saturation on right wheel")


if __name__ == "__main__":
    main()
