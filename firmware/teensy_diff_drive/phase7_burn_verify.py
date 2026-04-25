#!/usr/bin/env python3
"""Phase 7 — BURN persistence verification.

Proves cls=63 idx=15 PERSIST_PARAMETERS frame works end-to-end by:
  1. Writing a distinctive kP sentinel (0.00013579 — unusual number)
  2. Sending BURN
  3. Prompting user to power-cycle the SparkMAXes (main 12V off, wait 3s, on)
  4. After power-up, prompting user to read kP in REV Hardware Client
  5. User reports whether the sentinel persisted

The firmware cannot read parameters back over CAN (no PARAMETER_READ
support), so the check must be visual via Hardware Client. This is the
only manual step in the test suite after Phase 1.

Risk: zero. No motor motion. Just writes to SparkMAX flash.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teensy_bridge import Teensy, install_stop_handler, set_gains

SENTINEL = 0.00013579     # distinctive value unlikely to already be set


def prompt(q: str, options: list[str] | None = None) -> str:
    while True:
        resp = input(f"{q} ").strip().lower()
        if options is None and resp:
            return resp
        if options and resp in options:
            return resp
        if options:
            print(f"  (expected one of: {', '.join(options)})")


def main():
    print("Phase 7 — BURN persistence verification (requires manual power-cycle)")
    print()
    print(f"Writing sentinel kP = {SENTINEL}")
    print("Then BURN, then you power-cycle the main 12V to the SparkMAXes,")
    print("then check via REV Hardware Client that kP still reads as the sentinel.")
    print()
    input("Press Enter when ready... ")

    t = Teensy()
    install_stop_handler(t)

    print(f"\n[1/3] writing kP = {SENTINEL}")
    for a in set_gains(t, kp=SENTINEL):
        print(f"      {a}")
    time.sleep(0.3)

    print("\n[2/3] sending BURN")
    t.send("BURN")
    time.sleep(0.3)
    raw = t.s.read(t.s.in_waiting or 512).decode(errors="replace")
    for line in raw.splitlines():
        if line.startswith(("OK ", "ERR ", "#")):
            print(f"      {line}")

    t.close()
    print("\n[3/3] MANUAL STEPS:")
    print("  a) Cut main 12V power to BOTH SparkMAXes")
    print("  b) Wait 3 seconds")
    print("  c) Restore 12V power")
    print("  d) Wait 5 seconds for SparkMAXes to boot")
    print("  e) Plug USB-C from laptop to EITHER SparkMAX (disconnect CAN first)")
    print("  f) Open REV Hardware Client, select the device")
    print("  g) Read PID Controllers → Slot 0 → P value")
    print()
    result = prompt(f"Does kP read back as {SENTINEL}? (yes/no)", ["yes", "no", "y", "n"])

    if result in ("yes", "y"):
        print("\n  ✓ PASS — BURN (cls=63 idx=15) persists gains across power cycle.")
        print("  The full parameter-write → BURN → persist chain works end-to-end.")
    else:
        actual = prompt("What value does kP show instead? ")
        print(f"\n  ✗ FAIL — BURN did not persist. kP shows {actual} instead of {SENTINEL}")
        print("  Possible causes:")
        print("    - cls=63 idx=15 is wrong for FW 26.1.4 (re-verify REV-Specs)")
        print("    - Magic bytes 0x3AA3 incorrect")
        print("    - SparkMAX rejected BURN silently (check for fault LEDs)")
        print("    - DLC wrong (should be 2 for uint16 magic, we send 8)")


if __name__ == "__main__":
    main()
