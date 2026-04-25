#!/usr/bin/env python3
"""Phase 1 — Hand-spin resistance check (MANUAL, zero electrical).

No motor commands. No risk. You physically spin each track by hand;
this script just logs your observations to data/phase1_handspin_<ts>.md
so later phases have a written record of what you felt.

Procedure (follow the prompts):
  1. Ensure Teensy USB is connected and main power can be SHUT OFF
     (but isn't yet — we want Coast idle mode, so SparkMAX should be
     unpowered for this test to eliminate electrical braking).
  2. Cut main 12V power to the SparkMAXes.
  3. Spin each track by hand through 3 full track revolutions.
  4. Report: smooth? catchy? symmetric L vs R?

What each result means:
  - Smooth, symmetric  → drivetrain healthy mechanically
  - Notchy at fixed angle → bad tooth / bad bearing
  - L vs R asymmetric → one side has excess friction
  - Gritty texture → bearing contamination
"""

import os
import sys
import time

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(THIS_DIR, "data")


def prompt(q: str, options: list[str] | None = None) -> str:
    while True:
        if options:
            resp = input(f"{q} [{'/'.join(options)}] ").strip().lower()
            if resp in options:
                return resp
            print(f"  (expected one of: {', '.join(options)})")
        else:
            resp = input(f"{q} ").strip()
            if resp:
                return resp


def main():
    print(__doc__)
    print()
    input("Press Enter when main 12V to the SparkMAXes is OFF. ")

    os.makedirs(DATA_DIR, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(DATA_DIR, f"phase1_handspin_{ts}.md")

    notes = {}
    for side in ("left", "right"):
        print(f"\n--- {side.upper()} TRACK ---")
        input(f"Spin the {side} track by hand, 3 full track revolutions. Press Enter when done. ")
        notes[f"{side}_smooth"]   = prompt(f"{side}: smooth and uniform?", ["y", "n"])
        notes[f"{side}_notch"]    = prompt(f"{side}: any notchy/catchy spots at a fixed position?", ["y", "n"])
        notes[f"{side}_gritty"]   = prompt(f"{side}: gritty feel (bearing contamination)?", ["y", "n"])
        notes[f"{side}_effort"]   = prompt(f"{side}: subjective effort 1-10 (1=free, 10=stuck)?")

    print("\n--- COMPARISON ---")
    notes["lr_compare"] = prompt("L vs R — same / left harder / right harder?",
                                 ["same", "left", "right"])
    notes["free_notes"] = prompt("Any other observations (belt tension, alignment, rubbing)? ")

    with open(path, "w") as f:
        f.write(f"# Phase 1 — Hand-spin observations\n\n")
        f.write(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        for side in ("left", "right"):
            f.write(f"## {side.title()} track\n")
            f.write(f"- Smooth: {notes[f'{side}_smooth']}\n")
            f.write(f"- Notchy: {notes[f'{side}_notch']}\n")
            f.write(f"- Gritty: {notes[f'{side}_gritty']}\n")
            f.write(f"- Effort (1-10): {notes[f'{side}_effort']}\n\n")
        f.write(f"## L vs R comparison\n")
        f.write(f"- {notes['lr_compare']}\n\n")
        f.write(f"## Free-form notes\n")
        f.write(f"{notes['free_notes']}\n")

    print(f"\n[saved] {path}")

    # Immediate verdict
    left_e  = int(notes.get("left_effort",  "0")) if notes.get("left_effort",  "0").isdigit() else 0
    right_e = int(notes.get("right_effort", "0")) if notes.get("right_effort", "0").isdigit() else 0
    print("\n--- VERDICT ---")
    if notes["left_notch"] == "y" or notes["right_notch"] == "y":
        print("  !! Notchy feel detected — likely a bad tooth or bearing flat spot.")
        print("     Inspect the affected side before running any motor tests.")
    if abs(left_e - right_e) >= 3:
        harder = "left" if left_e > right_e else "right"
        print(f"  !! L/R effort differs significantly ({left_e} vs {right_e}) — {harder} track has excess friction.")
    if notes["left_gritty"] == "y" or notes["right_gritty"] == "y":
        print("  !! Gritty bearings reported — repack or replace before continuing.")
    if (notes["left_smooth"] == "y" and notes["right_smooth"] == "y"
            and notes["left_notch"] == "n" and notes["right_notch"] == "n"
            and abs(left_e - right_e) < 3):
        print("  ✓ Mechanics look healthy. Move to Phase 2 (baseline).")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[aborted]")
        sys.exit(1)
