# Teensy Diff-Drive Firmware

Thin USB-Serial ↔ CAN bridge for the IGVC differential drive. The Jetson's ROS2 `actuator_node` owns the diff-drive kinematics and streams per-wheel RPM setpoints; this firmware forwards them to each SparkMAX's built-in velocity PID and echoes wheel feedback.

Derived from `Paarseus/AVL-IGVC2026/firmware/teensy_sparkmax_arduino/teensy_sparkmax_fw26_synced/` with several fixes and simplifications (see "Delta from upstream" below). This folder is local staging — the canonical upstream is the IGVC firmware repo. Nothing here is wired into the ROS2 build; it's flashed independently.

## Hardware

- Teensy 4.1 — CAN1 on pins CTX1=22, CRX1=23
- SN65HVD230 or TJA1051T/3 CAN transceiver, 120 Ω termination at each bus end
- 2× REV SparkMAX **FW 26.1.4** — CAN ID 1 = left wheel, CAN ID 2 = right wheel
- 2× REV NEO brushless motors (free speed 5676 RPM)
- CAN bus: 1 Mbit/s, extended (29-bit) frames only

## Build & flash

```bash
# From the Jetson (preferred — enables remote iteration):
arduino-cli compile --fqbn teensy:avr:teensy41 firmware/teensy_diff_drive
arduino-cli upload  --fqbn teensy:avr:teensy41 -p /dev/ttyACM0 firmware/teensy_diff_drive

# Interactive serial monitor:
screen /dev/ttyACM0 115200
# or:
python3 -m serial.tools.miniterm /dev/ttyACM0 115200
```

## Serial protocol (115200 baud, newline-terminated, case-insensitive)

**Host → Teensy**

| Command | Effect |
|---|---|
| `L<rpm> R<rpm>` | Set both wheel setpoints (signed RPM) |
| `L<rpm>` / `R<rpm>` | Set one wheel, other unchanged |
| `S` | Stop both wheels (zero RPM, resets watchdog) |
| `D` | Print one `DIAG` line |
| `KP<v>` / `KI<v>` / `KD<v>` / `KF<v>` | Tune SparkMAX PID slot 0, both motors |
| `BURN` | Persist current PID gains to SparkMAX flash |

**Teensy → Host**

| Line prefix | Meaning |
|---|---|
| `E L<rpm> <pos> R<rpm> <pos>` | 50 Hz wheel feedback (motor RPM + cumulative rotations) |
| `OK …` | Command ack |
| `ERR …` | Parse failure |
| `DIAG …` | Response to `D` |
| `# …` | Log / info line (watchdog trips, boot banner, etc.) |

**Safety**: 300 ms host watchdog — no `L/R/S` in that window → both wheels forced to 0 RPM and one `# WDT host-timeout stop` line emitted. `MAX_RPM = 3000` clamp in `setVelocity()`.

## SparkMAX prerequisites (configure once via REV Hardware Client)

These cannot be reliably configured over CAN and must be set via USB-C from REV Hardware Client on a Windows host before first bring-up:

- **CAN ID** = 1 (left) or 2 (right)
- **Motor Type** = Brushless
- **Feedback Sensor** = Hall Sensor (NEO internal encoder)
- **Velocity Conversion Factor** = 1.0 — otherwise `MAX_RPM = 3000` is in the wrong units
- **Position Conversion Factor** = 1.0 — otherwise the `E` line position scale is wrong

## Research status — CAN protocol verification (2026-04-19)

Four parallel research agents cross-checked the protocol against REVLib-2024, REVLib-2026 driver headers, REV SPARK-MAX-Server proto, and WPILib DriverStation source. Summary:

**Firmly confirmed for FW 26.1.4**
- Duty cycle setpoint: `cls=0 idx=2`, float LE in bytes [0:3]
- Universal Heartbeat CAN ID `0x01011840`, byte 3 = `0x12` (enabled=bit1, systemWatchdog=bit4)
- Secondary Heartbeat `cls=11 idx=2 dev=0`, 0xFF×8 (ignored after first Universal HB, harmless)
- STATUS_2 at `cls=46 idx=2`, velocity@[0:3] + position@[4:7] as `float32 LE` (FW 25 reworked status frames from class 6 to class 46)
- SparkMAX heartbeat timeout: 100 ms
- PID slot 0 IDs: `kP_0=13, kI_0=14, kD_0=15` (from REV `SPARK-MAX-Types.proto`)
- `kF_0 = 16`, `kIZone_0 = 17` (slot stride = 8, so slot 1: kP=21, kI=22, kD=23, kF=24)
- `PTYPE_FLOAT = 2`
- BURN magic bytes `0xA3 0x3A`
- Native units: motor RPM (pre-gearbox), cumulative motor rotations, when both conversion factors are 1.0
- STATUS_2 is **on by default at 20 ms** on a factory SparkMAX — our `SET_STATUSES_ENABLED` call is redundant

**Real bug fixed from upstream `_synced.ino`**
- **kFF parameter ID was 17 in `_synced.ino`. Correct value is 16.** Parameter 17 is `kIZone_0` (integrator dead zone), not feedforward. The upstream has been silently disabling the integrator on every `KF<val>` command instead of tuning feedforward for who knows how long. Fixed here in `teensy_diff_drive.ino` around line 66.

**Needs bench verification — see `BRING_UP.md`**
- **Velocity setpoint CAN frame**: upstream `_synced.ino` uses `0x2050480` (cls=1 idx=2). REVLib-2026's `SPARK_VELOCITY_SETPOINT_FRAME_ID` is `0x2050000` (cls=0 idx=0). Upstream is "currently running" on FW 26.1.4 so firmware appears to accept the legacy path, but REVLib-2026 is the authoritative current format. Phase 4 of bring-up settles which actually works (or both).
- **Parameter SET byte layout**: upstream uses `[value@0:3, id@4, ptype@5]`. One research source (REV's USB-side server tool, probably not authoritative for CAN) claims `[id@0, _, value@2:5, ptype@6]`. Phase 3 of bring-up resolves this via read-back.

## Delta from upstream `_synced.ino`

| Area | `_synced.ino` | This firmware |
|---|---|---|
| kFF parameter ID | 17 (writes to kIZone, wrong) | 16 (correct, per REV proto) |
| `configurePID()` in `setup()` | Yes — clobbers BURNed values every boot | Removed; flash is authoritative |
| `V<rpm> W<rpm>` serial mode | Yes (confusing RPM sum/diff hack) | Removed |
| `directMode` flag / branching | Yes | Removed (everything is direct) |
| Host watchdog | None | 300 ms → zero both wheels |
| Feedback print | 500 ms human-readable | 50 Hz machine-readable `E` line |
| Serial grammar | Ad-hoc | Structured `OK / ERR / DIAG / # / E` |
| Non-blocking TX guard | No | `Serial.availableForWrite() >= 64` |
| Single-motor variants | Leftover from single-wheel sketches | Removed (dual-only) |

CAN wire format is otherwise **byte-for-byte identical** to `_synced.ino` except for the kFF parameter ID.

## TODOs

- [ ] Run all 7 phases of `BRING_UP.md` (bench verification)
- [ ] Tune PID on the robot on the actual surface (grass / pavement)
- [ ] Upstream the kFF=16 fix to `Paarseus/AVL-IGVC2026` as a PR against `_synced.ino`
- [ ] If Phase 1 confirms STATUS_2 is on by default, drop `enableStatus2()` and the Secondary Heartbeat as dead code
- [ ] Consider adding `GET <id>` command to the firmware for in-field parameter readback (currently only write path exists)
