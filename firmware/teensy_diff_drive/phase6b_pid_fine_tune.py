#!/usr/bin/env python3
"""Phase 6b — Closed-loop PID fine-tuner (kP + kI, keeps kFF from Phase 4).

Extends phase6 with kI tuning for steady-state error elimination.

Algorithm:
  Step 1: kFF fixed at Phase 4 empirical value (1 / slower-wheel max RPM)
  Step 2: kP sweep — double until oscillation OR tracking ratio > 0.95
  Step 3: Take last non-oscillating kP; add kI from 1e-7 tripling until
          tracking > 0.99 or oscillation
  Step 4: Validation sweep at 500 / 1000 / 1500 / 2000 / 3000 RPM
  Step 5: Offer to BURN

Oscillation detection: steady-state std dev > OSC_STD_RPM or peak-to-peak
> OSC_PP_RPM in the last half of the hold window.

Safety: motors on blocks, Brake idle mode set, slew enforced in each
run via the firmware's 300 ms watchdog (refresh every 100 ms). Each
test holds for 2-3 s then sends S. Full tuning takes ~90 s of motor on-time.

CSV output: data/phase6b_pid_fine_<ts>.csv with every iteration row for
post-hoc analysis.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teensy_bridge import Teensy, install_stop_handler, open_log, set_gains, run_velocity

# ---------- Tuning targets & thresholds ----------
KFF_FIXED      = 0.000197       # Phase 4 empirical (1 / R_max_rpm)
TUNE_TARGET    = 1500           # RPM target for sweeps
VALIDATION_RPMS = [500, 1000, 1500, 2000, 3000]

RATIO_KP_TARGET = 0.95          # exit kP sweep when tracking reaches this
RATIO_KI_TARGET = 0.99          # exit kI sweep when tracking reaches this
OSC_STD_RPM     = 120           # steady-state std above this = oscillation
OSC_PP_RPM      = 400           # peak-to-peak above this = oscillation

KP_MAX = 5e-3
KI_MAX = 5e-4


def is_oscillating(result) -> bool:
    """Look at both wheels for oscillation signatures."""
    for key in ('l_ss', 'r_ss'):
        ss = result[key]
        if ss['n'] == 0:
            continue
        if ss['std'] > OSC_STD_RPM:
            return True
        pp = ss['max'] - ss['min']
        if pp > OSC_PP_RPM:
            return True
    return False


def ratios(result, target):
    l = result['l_ss']['mean'] / target if target else 0
    r = result['r_ss']['mean'] / target if target else 0
    return l, r


def measure(t, target, dur=2.5):
    return run_velocity(t, target, target, dur)


def main():
    print("=" * 72)
    print("Phase 6b — Closed-loop PID fine-tuner (kP + kI)")
    print(f"  kFF = {KFF_FIXED:.6f}  (fixed, from Phase 4)")
    print(f"  tune target = {TUNE_TARGET} RPM (both wheels commanded equally)")
    print(f"  oscillation: std > {OSC_STD_RPM} RPM OR peak-to-peak > {OSC_PP_RPM} RPM")
    print("=" * 72)

    t = Teensy()
    install_stop_handler(t)
    t.stop()
    time.sleep(0.3)

    with open_log("phase6b_pid_fine",
                  ["phase", "kFF", "kP", "kI", "target", "l_mean", "r_mean",
                   "l_std", "r_std", "l_ratio", "r_ratio", "oscillating"]) as (w, path):

        # ============== Step 1: Baseline (kP=0, kI=0) ==============
        print("\n--- Step 1: baseline with kFF alone (kP=0, kI=0) ---")
        set_gains(t, kf=KFF_FIXED, kp=0, ki=0, kd=0)
        r0 = measure(t, TUNE_TARGET)
        lr, rr = ratios(r0, TUNE_TARGET)
        print(f"  baseline: L={r0['l_ss']['mean']:.0f}({lr:.2f}±{r0['l_ss']['std']:.0f})  "
              f"R={r0['r_ss']['mean']:.0f}({rr:.2f}±{r0['r_ss']['std']:.0f})")
        w.writerow(["baseline", f"{KFF_FIXED:.6f}", 0, 0, TUNE_TARGET,
                    r0['l_ss']['mean'], r0['r_ss']['mean'],
                    r0['l_ss']['std'], r0['r_ss']['std'],
                    f"{lr:.3f}", f"{rr:.3f}", is_oscillating(r0)])

        # ============== Step 2: kP sweep ==============
        print(f"\n--- Step 2: sweep kP (target ratio >= {RATIO_KP_TARGET}) ---")
        kP_list = []
        kP = 5e-5
        best_kP = 0
        best_ratio = 0
        while kP <= KP_MAX:
            set_gains(t, kf=KFF_FIXED, kp=kP, ki=0, kd=0)
            r = measure(t, TUNE_TARGET)
            lr, rr = ratios(r, TUNE_TARGET)
            osc = is_oscillating(r)
            avg_ratio = (lr + rr) / 2
            print(f"  kP={kP:.6f}  L={r['l_ss']['mean']:>5.0f}({lr:.2f}±{r['l_ss']['std']:>4.0f})  "
                  f"R={r['r_ss']['mean']:>5.0f}({rr:.2f}±{r['r_ss']['std']:>4.0f})  "
                  f"{'OSC!' if osc else ''}")
            w.writerow(["kp_sweep", f"{KFF_FIXED:.6f}", f"{kP:.6f}", 0, TUNE_TARGET,
                        r['l_ss']['mean'], r['r_ss']['mean'],
                        r['l_ss']['std'], r['r_ss']['std'],
                        f"{lr:.3f}", f"{rr:.3f}", osc])
            if osc:
                print(f"  >> oscillation at kP={kP:.6f}, backing off to {best_kP:.6f}")
                kP = best_kP
                break
            if avg_ratio > best_ratio:
                best_ratio = avg_ratio
                best_kP = kP
            if avg_ratio >= RATIO_KP_TARGET:
                print(f"  >> kP converged at {kP:.6f} (ratio {avg_ratio:.2f})")
                best_kP = kP
                break
            kP *= 2.0

        if best_kP == 0:
            print("  !! kP sweep found no stable value — aborting")
            t.close()
            return 1

        # ============== Step 3: kI sweep (with fixed best kP) ==============
        print(f"\n--- Step 3: sweep kI (kP fixed at {best_kP:.6f}) ---")
        best_kI = 0
        best_err = float('inf')
        kI = 1e-7
        # kI needs longer dwell because integrator accumulates
        while kI <= KI_MAX:
            set_gains(t, kf=KFF_FIXED, kp=best_kP, ki=kI, kd=0)
            r = run_velocity(t, TUNE_TARGET, TUNE_TARGET, 4.0,
                             settle_frac=0.5)  # last 50% for SS with integrator
            lr, rr = ratios(r, TUNE_TARGET)
            osc = is_oscillating(r)
            avg_ratio = (lr + rr) / 2
            err = abs(1 - avg_ratio)
            print(f"  kI={kI:.7f}  L={r['l_ss']['mean']:>5.0f}({lr:.2f}±{r['l_ss']['std']:>4.0f})  "
                  f"R={r['r_ss']['mean']:>5.0f}({rr:.2f}±{r['r_ss']['std']:>4.0f})  "
                  f"{'OSC!' if osc else ''}")
            w.writerow(["ki_sweep", f"{KFF_FIXED:.6f}", f"{best_kP:.6f}", f"{kI:.7f}",
                        TUNE_TARGET, r['l_ss']['mean'], r['r_ss']['mean'],
                        r['l_ss']['std'], r['r_ss']['std'],
                        f"{lr:.3f}", f"{rr:.3f}", osc])
            if osc:
                print(f"  >> oscillation at kI={kI:.7f}, backing off to {best_kI:.7f}")
                break
            if err < best_err:
                best_err = err
                best_kI = kI
            if avg_ratio >= RATIO_KI_TARGET:
                print(f"  >> kI converged at {kI:.7f} (ratio {avg_ratio:.2f})")
                best_kI = kI
                break
            kI *= 3.0

        # ============== Step 4: validation sweep ==============
        print(f"\n--- Step 4: validation at final gains ---")
        print(f"   kFF={KFF_FIXED:.6f}  kP={best_kP:.6f}  kI={best_kI:.7f}")
        set_gains(t, kf=KFF_FIXED, kp=best_kP, ki=best_kI, kd=0)
        time.sleep(0.3)
        print(f"\n{'target':>7}  {'L mean':>7}  {'R mean':>7}  {'L ratio':>7}  {'R ratio':>7}  {'L std':>5}  {'R std':>5}")
        for tgt in VALIDATION_RPMS:
            r = run_velocity(t, tgt, tgt, 3.0, settle_frac=0.5)
            lr, rr = ratios(r, tgt)
            osc = is_oscillating(r)
            print(f"{tgt:>7}  {r['l_ss']['mean']:>7.0f}  {r['r_ss']['mean']:>7.0f}  "
                  f"{lr:>7.2f}  {rr:>7.2f}  {r['l_ss']['std']:>5.0f}  {r['r_ss']['std']:>5.0f}  "
                  f"{'OSC!' if osc else ''}")
            w.writerow(["validation", f"{KFF_FIXED:.6f}", f"{best_kP:.6f}", f"{best_kI:.7f}",
                        tgt, r['l_ss']['mean'], r['r_ss']['mean'],
                        r['l_ss']['std'], r['r_ss']['std'],
                        f"{lr:.3f}", f"{rr:.3f}", osc])
            time.sleep(0.3)

    t.stop()
    t.close()

    print(f"\n{'=' * 72}")
    print(f"  FINAL GAINS")
    print(f"    kFF = {KFF_FIXED:.6f}")
    print(f"    kP  = {best_kP:.6f}")
    print(f"    kI  = {best_kI:.7f}")
    print(f"    kD  = 0")
    print(f"\n  Gains are in SparkMAX RAM only. Run phase7_burn_verify.py to persist.")
    print(f"  Log: {path}")
    print(f"\n  If you want to use these in actuator_node, update")
    print(f"  src/avros_bringup/config/actuator_params.yaml:")
    print(f"    kFF: {KFF_FIXED}")
    print(f"    kP:  {best_kP}")
    print(f"    kI:  {best_kI}")
    print(f"{'=' * 72}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
