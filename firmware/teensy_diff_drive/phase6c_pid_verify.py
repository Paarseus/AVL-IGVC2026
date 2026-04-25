#!/usr/bin/env python3
"""Phase 6c — Tuned-PID rigorous verification.

Four independent tests to validate the tuned gains are actually
accurate and precise across the operating envelope:

  Test 1 — REPEATABILITY: same setpoint 5 times, variance across trials
  Test 2 — STEP RESPONSE: 0 → 1500 → 0, measure overshoot + rise time + settle time
  Test 3 — MAX-RPM TRACKING: command 4514 RPM (1.5 m/s target), no saturation
  Test 4 — SYNC PRECISION: L vs R delta every sample during a 1500-RPM hold

Expects the kFF/kP/kI from phase6b to already be in SparkMAX RAM. Does
not change gains (unless you uncomment the SET_GAINS block).

Safety: motors on blocks. Total motor-on time: ~45 s.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teensy_bridge import Teensy, install_stop_handler, open_log, set_gains, run_velocity, stats

# Re-apply gains at start so we know exactly what's in RAM
KFF = 0.000197
KP  = 0.0004
KI  = 9e-7


def run_and_capture(t, target, dur):
    """Same as run_velocity but returns full time series too."""
    from teensy_bridge import E_LINE_RE
    t.send(f'L{target} R{target}')
    samples = []
    t0 = time.time()
    last = time.time()
    buf = ''
    while time.time() - t0 < dur:
        if time.time() - last > 0.1:
            t.send(f'L{target} R{target}')
            last = time.time()
        chunk = t.s.read(t.s.in_waiting or 256).decode(errors='replace')
        if not chunk:
            time.sleep(0.005); continue
        buf += chunk
        while '\n' in buf:
            line, buf = buf.split('\n', 1)
            m = E_LINE_RE.match(line.strip())
            if m:
                samples.append((time.time() - t0, int(m.group(1)),
                                float(m.group(2)), int(m.group(3)),
                                float(m.group(4))))
    t.stop()
    time.sleep(0.3)
    return samples


def ss_stats(samples, start_frac=0.5):
    """Steady-state stats from last (1-start_frac) of samples."""
    if not samples: return {}
    cut = int(len(samples) * start_frac)
    ss = samples[cut:]
    return {
        'l_mean': sum(s[1] for s in ss) / len(ss),
        'r_mean': sum(s[3] for s in ss) / len(ss),
        'l_std':  stats([s[1] for s in ss])['std'],
        'r_std':  stats([s[3] for s in ss])['std'],
        'l_pp':   max(s[1] for s in ss) - min(s[1] for s in ss),
        'r_pp':   max(s[3] for s in ss) - min(s[3] for s in ss),
    }


def main():
    print("=" * 72)
    print("Phase 6c — Tuned PID rigorous verification")
    print(f"  gains: kFF={KFF}  kP={KP}  kI={KI}")
    print("=" * 72)

    t = Teensy()
    install_stop_handler(t)
    t.stop()
    time.sleep(0.3)

    # Ensure gains are set
    print("\n[setup] applying gains from phase6b")
    set_gains(t, kf=KFF, kp=KP, ki=KI, kd=0)
    time.sleep(0.3)

    with open_log("phase6c_pid_verify",
                  ["test", "trial", "target", "t_s", "l_rpm", "r_rpm", "l_pos", "r_pos"]) as (w, path):

        # ============ Test 1: Repeatability ============
        print("\n--- Test 1: REPEATABILITY — 5 trials at L1500 R1500 ---")
        print(f"{'trial':>6}  {'L mean':>7}  {'R mean':>7}  {'L std':>5}  {'R std':>5}  {'L-R diff':>9}")
        repeat_l_means = []; repeat_r_means = []
        for trial in range(5):
            samples = run_and_capture(t, 1500, 3.0)
            for s in samples:
                w.writerow(["repeat", trial, 1500, f"{s[0]:.3f}", s[1], s[3],
                            f"{s[2]:.4f}", f"{s[4]:.4f}"])
            st = ss_stats(samples)
            if not st: continue
            repeat_l_means.append(st['l_mean'])
            repeat_r_means.append(st['r_mean'])
            diff = st['l_mean'] - st['r_mean']
            print(f"{trial:>6}  {st['l_mean']:>7.1f}  {st['r_mean']:>7.1f}  "
                  f"{st['l_std']:>5.1f}  {st['r_std']:>5.1f}  {diff:>+9.1f}")
            time.sleep(0.5)

        if len(repeat_l_means) >= 3:
            l_across = stats(repeat_l_means)
            r_across = stats(repeat_r_means)
            print(f"\n  across-trial variability:")
            print(f"    L mean-of-means = {l_across['mean']:.2f}, std-of-means = {l_across['std']:.2f} ({l_across['std']/l_across['mean']*100:.2f}%)")
            print(f"    R mean-of-means = {r_across['mean']:.2f}, std-of-means = {r_across['std']:.2f} ({r_across['std']/r_across['mean']*100:.2f}%)")
            print(f"    accuracy (vs 1500 cmd): L {l_across['mean']-1500:+.1f} RPM, R {r_across['mean']-1500:+.1f} RPM")

        # ============ Test 2: Step Response ============
        print("\n--- Test 2: STEP RESPONSE — 0 → 1500 → 0 ---")
        # Capture with fine timing so we can extract rise time
        t.stop(); time.sleep(0.5)
        samples = run_and_capture(t, 1500, 3.0)
        for s in samples:
            w.writerow(["step", 0, 1500, f"{s[0]:.3f}", s[1], s[3],
                        f"{s[2]:.4f}", f"{s[4]:.4f}"])

        # Find rise time (10% → 90% of setpoint)
        target = 1500
        t10 = t90 = settle = None
        for ti, lr, lp, rr, rp in samples:
            avg = (lr + rr) / 2
            if t10 is None and avg >= 0.10 * target:
                t10 = ti
            if t90 is None and avg >= 0.90 * target:
                t90 = ti
            if t90 is not None and settle is None and ti > t90 + 0.3 and abs(avg - target) < 0.05 * target:
                settle = ti
        peak = max((s[1] + s[3]) / 2 for s in samples)
        overshoot = (peak - target) / target * 100 if peak > target else 0
        print(f"  peak RPM:        {peak:.0f}")
        print(f"  overshoot:       {overshoot:+.1f}%")
        print(f"  10% rise:        {t10*1000:.0f} ms" if t10 is not None else "  10% rise:        n/a")
        print(f"  90% rise:        {t90*1000:.0f} ms" if t90 is not None else "  90% rise:        n/a")
        print(f"  rise 10-90%:     {(t90-t10)*1000:.0f} ms" if t10 and t90 else "  rise 10-90%:     n/a")
        print(f"  settle to ±5%:   {settle*1000:.0f} ms" if settle is not None else "  settle to ±5%:   not settled in window")

        # ============ Test 3: Max-RPM tracking ============
        print("\n--- Test 3: MAX-RPM TRACKING — L4514 R4514 (1.5 m/s target) ---")
        t.stop(); time.sleep(0.5)
        samples = run_and_capture(t, 4514, 3.5)
        for s in samples:
            w.writerow(["max_rpm", 0, 4514, f"{s[0]:.3f}", s[1], s[3],
                        f"{s[2]:.4f}", f"{s[4]:.4f}"])
        st = ss_stats(samples, start_frac=0.6)
        if st:
            l_ratio = st['l_mean'] / 4514
            r_ratio = st['r_mean'] / 4514
            print(f"  L: {st['l_mean']:.0f} RPM ({l_ratio*100:.1f}% of cmd, σ={st['l_std']:.0f})")
            print(f"  R: {st['r_mean']:.0f} RPM ({r_ratio*100:.1f}% of cmd, σ={st['r_std']:.0f})")
            saturated = l_ratio < 0.95 or r_ratio < 0.95
            if saturated:
                print(f"  !! Saturating — one wheel can't reach 4514 RPM mechanically")
            else:
                print(f"  ✓ 1.5 m/s (4514 RPM) is achievable cleanly")

        # ============ Test 4: Sync precision ============
        print("\n--- Test 4: SYNC PRECISION — L/R delta during 1500 RPM hold ---")
        t.stop(); time.sleep(0.5)
        samples = run_and_capture(t, 1500, 4.0)
        for s in samples:
            w.writerow(["sync", 0, 1500, f"{s[0]:.3f}", s[1], s[3],
                        f"{s[2]:.4f}", f"{s[4]:.4f}"])
        # look at steady-state window only
        cut = int(len(samples) * 0.4)
        ss = samples[cut:]
        deltas = [abs(s[1] - s[3]) for s in ss]
        if deltas:
            d_stats = stats(deltas)
            l_mean = sum(s[1] for s in ss) / len(ss)
            r_mean = sum(s[3] for s in ss) / len(ss)
            print(f"  L-R |delta| stats over {len(deltas)} samples:")
            print(f"    mean: {d_stats['mean']:.1f} RPM ({d_stats['mean']/1500*100:.2f}% of setpoint)")
            print(f"    max:  {d_stats['max']:.0f} RPM ({d_stats['max']/1500*100:.2f}%)")
            print(f"    std:  {d_stats['std']:.1f}")
            print(f"  L steady = {l_mean:.1f}, R steady = {r_mean:.1f}")
            print(f"  Per-wheel PID → {'SYNCED' if d_stats['mean'] < 30 else 'DRIFTING'}")

    t.stop()
    t.close()
    print(f"\n[log] {path}")
    print("=" * 72)


if __name__ == "__main__":
    main()
