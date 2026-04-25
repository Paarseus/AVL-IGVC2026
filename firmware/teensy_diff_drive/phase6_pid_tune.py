#!/usr/bin/env python3
"""Phase 6 — Velocity PID tuning using Phase 4 empirical data.

Methodology (per REV / Road Runner / Chief Delphi consensus):
  1. kFF = 1 / max_achievable_RPM_under_load (from Phase 4)
  2. Start kP small (5e-5), iterate × 2 until tracking within ±5% without oscillation
  3. kI only if steady-state error persists (usually not needed for drivetrain)
  4. kD skip for velocity control

Uses the slower wheel's max RPM so kFF doesn't saturate that side. Tunes
globally (firmware writes same gains to both) — per-wheel compensation
requires firmware changes, out of scope.

Safety: Same as Phase 4 + 5 — motors on blocks, brake idle mode, test
at midrange setpoint (50% of max) not at limit.

Writes data/phase6_pid_tune_<ts>.csv with one row per tuning iteration
plus validation sweep.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teensy_bridge import Teensy, install_stop_handler, open_log, set_gains, run_velocity

# ---------- Inputs from Phase 4 ----------
R_MAX_RPM = 5072     # extrapolated max on slower (right) wheel
L_MAX_RPM = 5532
M_PER_REV = 0.01994

# Start gains
kFF0  = 1.0 / R_MAX_RPM     # 0.000197
kP0   = 5e-5                # REV's recommended starting point

# Tuning targets
TUNE_TARGET  = int(R_MAX_RPM * 0.50)   # 50% of slower wheel max = safe midrange
VALIDATION_RPMS = [1000, 2000, 3000, 4000, 4514]   # 4514 = 1.5 m/s

# Convergence criteria
RATIO_OK_LOW   = 0.95
RATIO_OK_HIGH  = 1.05
OSCILLATION_STD_MAX = 150   # RPM std during steady state — above = oscillation
MAX_KP_ITERS = 6


def tune_kp(t, csv_w, kFF):
    """Iterate kP (doubling) until both L and R track within ±5% of target.
    Stops on oscillation or max iterations."""
    kP = kP0
    best_kP = kP
    best_error = float("inf")
    for i in range(MAX_KP_ITERS):
        acks = set_gains(t, kp=kP, kf=kFF, ki=0, kd=0)
        result = run_velocity(t, TUNE_TARGET, TUNE_TARGET, 2.0)
        l_ss = result["l_ss"]; r_ss = result["r_ss"]
        l_ratio = l_ss["mean"] / TUNE_TARGET if TUNE_TARGET else 0
        r_ratio = r_ss["mean"] / TUNE_TARGET if TUNE_TARGET else 0
        oscillating = l_ss["std"] > OSCILLATION_STD_MAX or r_ss["std"] > OSCILLATION_STD_MAX
        avg_err = (abs(1 - l_ratio) + abs(1 - r_ratio)) / 2
        print(f"  iter {i}  kP={kP:.5f}  "
              f"L={l_ss['mean']:>5.0f}({l_ratio:.2f}±{l_ss['std']:.0f})  "
              f"R={r_ss['mean']:>5.0f}({r_ratio:.2f}±{r_ss['std']:.0f}) "
              f"{'OSC!' if oscillating else ''}")
        csv_w.writerow(["kp_sweep", f"{kFF:.6f}", f"{kP:.6f}",
                        TUNE_TARGET, l_ss["mean"], r_ss["mean"],
                        l_ss["std"], r_ss["std"]])
        if avg_err < best_error and not oscillating:
            best_error = avg_err
            best_kP = kP
        if RATIO_OK_LOW <= l_ratio <= RATIO_OK_HIGH and RATIO_OK_LOW <= r_ratio <= RATIO_OK_HIGH:
            print(f"  >> converged at kP={kP:.5f}")
            return kP
        if oscillating:
            print(f"  !! oscillation detected — backing off to {best_kP:.5f}")
            return best_kP
        kP *= 2.0
        time.sleep(0.3)
    print(f"  (max iters reached; best kP={best_kP:.5f} with avg error {best_error:.2%})")
    return best_kP


def validate(t, csv_w, kFF, kP):
    """Run a sweep of validation RPMs and report tracking at each."""
    print(f"\n--- Validation with kFF={kFF:.6f} kP={kP:.6f} ---")
    print(f"{'target':>7}  {'L mean':>7}  {'R mean':>7}  {'L ratio':>7}  {'R ratio':>7}  {'L m/s':>6}  {'R m/s':>6}  {'osc?':>5}")
    for tgt in VALIDATION_RPMS:
        result = run_velocity(t, tgt, tgt, 2.0)
        l_ss = result["l_ss"]; r_ss = result["r_ss"]
        l_r = l_ss["mean"] / tgt if tgt else 0
        r_r = r_ss["mean"] / tgt if tgt else 0
        l_mps = l_ss["mean"] * M_PER_REV / 60
        r_mps = r_ss["mean"] * M_PER_REV / 60
        osc = "yes" if (l_ss["std"] > OSCILLATION_STD_MAX or r_ss["std"] > OSCILLATION_STD_MAX) else "no"
        print(f"{tgt:>7}  {l_ss['mean']:>7.0f}  {r_ss['mean']:>7.0f}  "
              f"{l_r:>7.2f}  {r_r:>7.2f}  {l_mps:>6.3f}  {r_mps:>6.3f}  {osc:>5}")
        csv_w.writerow(["validation", f"{kFF:.6f}", f"{kP:.6f}",
                        tgt, l_ss["mean"], r_ss["mean"],
                        l_ss["std"], r_ss["std"]])
        time.sleep(0.3)


def main():
    print("Phase 6 — PID tune using empirical Phase 4 max RPM")
    print(f"  slower wheel R_max = {R_MAX_RPM} RPM → kFF = 1/R_max = {kFF0:.6f}")
    print(f"  tuning target = {TUNE_TARGET} RPM (50% of R_max)")

    t = Teensy()
    install_stop_handler(t)

    # Zero any stale kI/kD, set initial kFF
    print("\n--- setting initial gains ---")
    acks = set_gains(t, kf=kFF0, kp=0, ki=0, kd=0)
    for a in acks: print(f"  {a}")
    time.sleep(0.3)

    # Smoke test: with pure kFF no kP, what do we get at the tuning target?
    print(f"\n--- kFF-only smoke test at {TUNE_TARGET} RPM ---")
    result = run_velocity(t, TUNE_TARGET, TUNE_TARGET, 2.0)
    l_ss = result["l_ss"]; r_ss = result["r_ss"]
    print(f"  L={l_ss['mean']:.0f}({l_ss['mean']/TUNE_TARGET:.2f})  "
          f"R={r_ss['mean']:.0f}({r_ss['mean']/TUNE_TARGET:.2f})")
    time.sleep(0.3)

    with open_log("phase6_pid_tune",
                  ["phase", "kFF", "kP", "target_rpm", "l_mean", "r_mean", "l_std", "r_std"]) as (w, path):
        print(f"\n--- tuning kP at target {TUNE_TARGET} RPM ---")
        kP_final = tune_kp(t, w, kFF0)
        validate(t, w, kFF0, kP_final)

    print(f"\n[FINAL GAINS]  kFF={kFF0:.6f}  kP={kP_final:.6f}")
    print("(values are in SparkMAX RAM, not flashed — run Phase 7 to BURN if good)")
    print(f"[log] {path}")
    t.close()


if __name__ == "__main__":
    main()
