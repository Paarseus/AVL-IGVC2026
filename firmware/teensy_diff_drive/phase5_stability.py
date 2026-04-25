#!/usr/bin/env python3
"""Phase 5 — Long-duration stability hold.

Commands constant velocity (below the kOutputMax_0 clamp discovered in
Phase 6) for 10 seconds, logs every E line, computes:
  - RPM std dev (should be < 3% of mean for a stable PID)
  - Position delta (should match mean_rpm × duration × M_PER_REV/60 / track_ratio)
  - Per-revolution RPM variation (detects periodic dips = bad tooth/bearing)

Tests BOTH velocity mode (closed-loop) and duty-cycle mode side by side.

Safety: low. Tracks spin at ~500-1000 RPM motor for 10 seconds each.
Active brake between phases. Total motor-on time: ~22 s.

Writes data/phase5_stability_<ts>.csv with full time series so a dip
analysis can be done offline.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teensy_bridge import Teensy, install_stop_handler, open_log, set_gains

# Hold below the 2450 RPM kOutputMax clamp found in Phase 6
VEL_TARGET    = 1500    # RPM (≈ 0.50 m/s ground speed)
DUTY_TARGET   = 0.15    # 15% duty ≈ 700 RPM on L, ~580 RPM on R (Phase 4)
HOLD_DURATION = 10.0
E_LINE_RE = None  # imported via bridge

# PID gains from Phase 6 tuning
kFF = 0.000197
kP  = 0.0004


def hold_and_log(t: Teensy, cmd_str: str, duration: float, csv_w, label: str):
    """Send cmd_str at 10 Hz for `duration` seconds. Log every E line."""
    from teensy_bridge import E_LINE_RE as RE
    import re
    samples = []
    t0 = time.time()
    last_refresh = 0
    buf = ""
    t.s.reset_input_buffer()
    while time.time() - t0 < duration:
        if time.time() - last_refresh > 0.1:
            t.send(cmd_str)
            last_refresh = time.time()
        chunk = t.s.read(t.s.in_waiting or 256).decode(errors='replace')
        if not chunk:
            time.sleep(0.01)
            continue
        buf += chunk
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            m = RE.match(line.strip())
            if m:
                now_t = time.time() - t0
                lrpm = int(m.group(1)); lpos = float(m.group(2))
                rrpm = int(m.group(3)); rpos = float(m.group(4))
                samples.append((now_t, lrpm, lpos, rrpm, rpos))
                csv_w.writerow([label, f"{now_t:.4f}", lrpm, f"{lpos:.4f}",
                                rrpm, f"{rpos:.4f}"])
    t.stop()
    time.sleep(0.5)
    return samples


def stats(samples, col_idx):
    vals = [s[col_idx] for s in samples]
    if not vals: return (0, 0, 0, 0)
    n = len(vals); mean = sum(vals) / n
    var = sum((v - mean) ** 2 for v in vals) / n
    return (n, mean, var ** 0.5, vals[-1] - vals[0])


def analyze(samples, label):
    """Trim first 1s (startup), report steady-state stability."""
    if not samples:
        print(f"  [{label}] no samples captured"); return
    ss = [s for s in samples if s[0] >= 1.0]
    n_l, lm, ls, _ = stats(ss, 1)   # L RPM
    n_r, rm, rs, _ = stats(ss, 3)   # R RPM
    l_pos_delta = ss[-1][2] - ss[0][2]
    r_pos_delta = ss[-1][4] - ss[0][4]
    dur = ss[-1][0] - ss[0][0] if len(ss) > 1 else 1.0

    l_cov = (ls / abs(lm) * 100) if abs(lm) > 1 else 0
    r_cov = (rs / abs(rm) * 100) if abs(rm) > 1 else 0
    l_mps = lm * 0.01994 / 60
    r_mps = rm * 0.01994 / 60
    print(f"  [{label}]  n={n_l}  dur={dur:.2f}s")
    print(f"    L: mean={lm:>6.1f}  std={ls:>5.2f}  CoV={l_cov:>4.1f}%  Δpos={l_pos_delta:+6.2f} rot  {l_mps:.3f} m/s")
    print(f"    R: mean={rm:>6.1f}  std={rs:>5.2f}  CoV={r_cov:>4.1f}%  Δpos={r_pos_delta:+6.2f} rot  {r_mps:.3f} m/s")
    verdict = []
    if l_cov > 5 or r_cov > 5:
        verdict.append("!! high jitter (CoV > 5%) — PID may be chattering")
    else:
        verdict.append("✓ stable within ±5% CoV")
    if len(ss) > 10:
        # Detect periodic dips: look for samples where rpm < 50% of mean
        l_dips = [s for s in ss if abs(s[1]) < 0.5 * abs(lm)]
        r_dips = [s for s in ss if abs(s[3]) < 0.5 * abs(rm)]
        if len(l_dips) > n_l * 0.02:
            verdict.append(f"!! L has {len(l_dips)} dips below 50% mean — periodic mechanical catch?")
        if len(r_dips) > n_r * 0.02:
            verdict.append(f"!! R has {len(r_dips)} dips below 50% mean — periodic mechanical catch?")
        if len(l_dips) <= 1 and len(r_dips) <= 1:
            verdict.append("✓ no RPM dips detected (no periodic mechanical catch)")
    for v in verdict: print(f"    {v}")


def main():
    print("Phase 5 — Long-duration stability (velocity + duty mode comparison)")
    t = Teensy()
    install_stop_handler(t)

    # Set PID gains for velocity test
    print(f"\n--- setting PID gains: kFF={kFF}  kP={kP} ---")
    for a in set_gains(t, kf=kFF, kp=kP, ki=0, kd=0):
        print(f"  {a}")
    time.sleep(0.3)

    with open_log("phase5_stability",
                  ["mode", "t_s", "l_rpm", "l_pos", "r_rpm", "r_pos"]) as (w, path):

        print(f"\n=== Velocity mode: L{VEL_TARGET} R{VEL_TARGET} for {HOLD_DURATION}s ===")
        vel_samples = hold_and_log(t, f"L{VEL_TARGET} R{VEL_TARGET}",
                                    HOLD_DURATION, w, "velocity")
        analyze(vel_samples, "velocity")
        time.sleep(1.0)

        print(f"\n=== Duty mode: UL{DUTY_TARGET} UR{DUTY_TARGET} for {HOLD_DURATION}s ===")
        duty_samples = hold_and_log(t, f"UL{DUTY_TARGET} UR{DUTY_TARGET}",
                                     HOLD_DURATION, w, "duty")
        analyze(duty_samples, "duty")

    print(f"\n[log] {path}")
    t.close()


if __name__ == "__main__":
    main()
