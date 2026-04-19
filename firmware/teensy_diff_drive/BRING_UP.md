# Bring-Up Test Plan — Teensy Diff-Drive Firmware

Bench verification for `teensy_diff_drive.ino`. Run the phases in order; each builds on the previous. Stop and debug if a phase fails before moving on.

Time budget: ~2 hours for a successful first run. Add 1–2 hours if any phase fails and needs investigation.

---

## Prerequisites

- Jetson reachable via `ssh jetson` (Tailscale up)
- `arduino-cli` installed on the Jetson, or a separate machine with Teensyduino ready to flash
- Teensy 4.1 plugged into the Jetson USB, visible as `/dev/ttyACM*`
- 2× SparkMAX powered (24–48 V main battery), CAN bus terminated (120 Ω each end), CAN IDs set to 1/2 via REV Hardware Client (Phase 0 verifies)
- 2× NEO motors physically connected to the SparkMAX outputs
- **A physical power kill** reachable in < 1 second — battery disconnect, main contactor, or wall switch feeding the 24 V supply

## Safety policy

**Phases 4 and later spin the motors.** Before starting Phase 4, **one** of the following must be true:

- Robot up on blocks, tracks/wheels free to spin, nothing contacting the ground
- Motors physically disconnected from the belts / transmission
- Motors in a standalone test rig, not on the robot

**Rules while motors are live**:

1. Start at **100 RPM**, not 3000. Raise in 100 RPM increments.
2. Hands away from belts, tracks, and motor shafts whenever a command is live.
3. If a motor behaves unexpectedly — runaway, oscillation, wrong direction — **kill main power first, debug second**. Do not try to stop it in software first.
4. Do not leave the bench unattended with a live command.

---

## Phase 0 — Physical setup verification

**Risk**: zero. Nothing moves, nothing is flashed.

**Goal**: confirm the SparkMAX config is what we assume before touching CAN.

**Procedure**: Plug each SparkMAX into a Windows laptop over USB-C. Open REV Hardware Client. Verify:

| Setting | Required value |
|---|---|
| Firmware version | 26.1.4 |
| CAN ID | 1 (left), 2 (right) |
| Motor Type | Brushless |
| Feedback Sensor | Hall Sensor |
| Velocity Conversion Factor | 1.0 |
| Position Conversion Factor | 1.0 |
| Idle Mode | Coast or Brake (user choice — Brake is safer on blocks) |

**Pass**: all settings match.
**Fail → stop**: correct in REV Hardware Client before any CAN testing. None of this can be reliably set from the Teensy side.

---

## Phase 1 — Passive CAN sniff

**Risk**: zero. Nothing moves. Teensy only listens.

**Goal**: observe what CAN frames SparkMAX FW 26.1.4 actually emits with no commands being sent. This settles the class=6 vs class=46 status-frame dispute and confirms whether `SET_STATUSES_ENABLED` is needed at all.

**Setup**: Flash a "sniffer" sketch (to be written — see Notes below) that sets `ACCEPT_ALL` filter and prints every received extended frame with decoded `[class, idx, dev]` and raw bytes. No commands sent out.

**Procedure**:
```bash
ssh jetson
arduino-cli upload --fqbn teensy:avr:teensy41 -p /dev/ttyACM0 firmware/teensy_sniff
screen /dev/ttyACM0 115200
# Watch for ~10 seconds with SparkMAXes powered on, motors at rest
```

**Expected**: periodic frames from `dev=1` and `dev=2`. Decode the API class field. For FW 25+, STATUS_2 should show up as `class=46 idx=2` with 8-byte payload. If interpreted as float32 LE, `vel@[0:3]` and `pos@[4:7]` should both read very near 0 with motors at rest.

**Pass criteria**:
- Frames received at all (confirms CAN bus wiring + termination)
- At least one periodic frame from each SparkMAX (ID 1 and ID 2)
- `class=46 idx=2` frames present with the expected 8-byte layout

**Fail modes**:
- No frames at all → CAN termination missing, transceiver wiring swapped, wrong baud rate, or SparkMAX not powered. Fix the physical layer.
- Frames present but different API class → REV changed status frames for the specific FW revision. Note the observed class and update `CLS_STATUS` in `teensy_diff_drive.ino`.
- Frames present but vel/pos decode as garbage → payload might be int32 fixed-point instead of float32 (REVLib-2024 vs REVLib-2026 layout). Update the decode accordingly.

**Settles**:
- Status frame API class for FW 26.1.4 (46 vs 6)
- STATUS_2 default period
- STATUS_2 wire encoding (float vs fixed-point)
- Whether `SET_STATUSES_ENABLED` is required (if STATUS_2 streams without being enabled, the call is confirmed redundant)

---

## Phase 2 — Parameter readback

**Risk**: zero. Nothing moves.

**Goal**: disambiguate the kFF parameter ID (16 vs 17). Confirm the parameter read path works.

**Setup**: Extend the sniffer sketch with a serial command `GET <id>` that sends a SparkMAX parameter GET request (`cls=48 idx=1` traditionally — verify from REVLib headers) and prints the response frame.

**Procedure**:
```
GET 13      # kP_0  — expect current flash value (usually a small float like 0.0001)
GET 14      # kI_0
GET 15      # kD_0
GET 16      # kF_0 — this is what REV proto says is feedforward
GET 17      # kIZone_0 — this is what _synced.ino has been writing as "kFF"
```

Write a distinctive value to ID 16 via the firmware's `KF` command and read it back:
```
KF0.00042069
GET 16      # should report 0.00042069
GET 17      # should NOT report 0.00042069 (distinct register)
```

Clean up:
```
KF0
```

**Pass**: `GET 16` reads back exactly what was written, and `GET 17` is unchanged. This confirms:
- ID 16 is the real feedforward parameter
- The kFF=16 fix in our firmware is correct
- Parameter SET byte layout is compatible with parameter GET byte layout

**Fail modes**:
- `GET 16` reads back garbage → parameter SET byte layout is wrong. Jump to Phase 3 to test the alternate layout.
- `GET 17` also changes when we write to 16 → parameter IDs 16 and 17 are aliased (unlikely but would explain the upstream confusion).
- `GET` response never arrives → parameter GET API class/index is wrong. Look up in REVLib-2026 headers.

**Settles**: kFF parameter ID (16 vs 17), parameter read path working.

---

## Phase 3 — Parameter SET byte layout verification

**Risk**: zero. Nothing moves. Only touches SparkMAX RAM (not flash).

**Goal**: resolve the parameter SET byte layout dispute between `_synced.ino` (`[value@0:3, id@4, ptype@5]`) and the SPARK-MAX-Server source (`[id@0, _, value@2:5, ptype@6]`).

**Setup**: Extend the sniffer sketch with a `RAW_SET <can_id_hex> <byte0> … <byte7>` command that sends a raw 8-byte CAN payload to a given extended ID. (This is a power user escape hatch.)

**Procedure**:

**Layout A** — `_synced.ino` / current firmware (`[value@0:3, id@4, ptype@5]`):

Pack `kP = 0.0001` (IEEE 754 LE = `CD CC CC 3D` as 4 bytes via memcpy — verify with a Python one-liner), parameter ID 13, ptype 2:

```
RAW_SET 0x2050C01 CD CC CC 3D 0D 02 00 00
GET 13
# Expect: 0.0001 (or very close)
```

If layout A reads back correctly → done, upstream was right. Skip layout B.

**Layout B** — SPARK-MAX-Server claim (`[id@0, _, value@2:5, ptype@6]`):

```
RAW_SET 0x2050C01 0D 00 CD CC CC 3D 02 00
GET 13
# Expect: 0.0001
```

**Pass**: one of the two layouts reads back the value. That's the correct one.

**Fail**: neither layout works →
- The parameter SET API class or index is wrong (we've assumed `cls=48 idx=0`). Cross-check REVLib-2026 headers for the correct frame ID.
- Or the CAN ID bit structure is different than we think. Extremely unlikely but would explain cascading failures.

**Settles**: parameter SET byte layout AND the cls=48/idx=0 frame ID.

---

## Phase 4 — Velocity setpoint + low-RPM motion test

**Risk**: MEDIUM. Motor spins.

### Pre-flight safety check

Tick every box before proceeding:
- [ ] Robot on blocks OR motors disconnected from belts OR standalone test rig
- [ ] Nothing within 1 m of any moving part
- [ ] Physical power kill reachable
- [ ] Start RPM = 100, not 500, not 1000
- [ ] You (the operator) are within arm's reach of the kill switch

**Goal**: confirm the correct velocity setpoint CAN frame. Settles class=1/idx=2 (upstream) vs class=0/idx=0 (REVLib-2026).

**Setup**: Flash `teensy_diff_drive.ino` (the actual firmware, not the sniffer). Open serial monitor.

**Procedure A — test upstream class=1/idx=2 path** (current default in this firmware):

```
KF0.000176          # roughly-right feedforward for a NEO
L100 R0
```

Watch the `E` line. Left wheel velocity should climb to ~100 RPM within ~1 s. If it does:

```
S                   # stop
L0 R100
```

Right wheel should spin. Then:
```
S
L100 R100
```

Both wheels.

```
S
L-100 R-100
```

Both reverse.

**If Procedure A works**: upstream's class=1/idx=2 path is correct. Skip Procedure B.

**Procedure B — test REVLib-2026 class=0/idx=0 path** (fallback if Procedure A fails):

Only reach this if Procedure A's wheels did not spin. Edit `teensy_diff_drive.ino`:
```cpp
static constexpr uint8_t  CLS_VELOCITY   = 0;   // was 1
static constexpr uint8_t  IDX_VELOCITY   = 0;   // was 2
```
Reflash, rerun Procedure A's commands.

**Pass criteria**:
- Motor RPM (from the `E` line) tracks commanded RPM to within ±20 RPM at steady state
- No oscillation, no runaway, no reverse direction when commanded forward

**Fail modes**:
- Procedure A and B both fail → feedback sensor misconfigured on the SparkMAX, or PID slot not 0, or a deeper firmware issue. Drop back to REV Hardware Client to check SparkMAX internal config. You cannot fix these from the Teensy.
- Motor spins but wildly off target (e.g., 100 RPM command → 500 RPM actual) → kFF way too high. Reduce `KF` and retry. If kFF=0 still overshoots, there's a stuck PID gain in flash from a previous session; read back with `GET 13/14/15/16` and reset.
- Motor spins in the wrong direction → hall sensor polarity is inverted. Fix via REV Hardware Client (`Inverted` setting on the motor controller), not in firmware.

**Settles**: the velocity setpoint CAN frame format for FW 26.1.4.

---

## Phase 5 — PID tune + BURN persistence

**Risk**: MEDIUM. Motors spin at tuning setpoints (up to 1000 RPM on blocks).

**Goal**: tune a working PID on the bench, burn to flash, verify persistence across power cycle. Proves the full parameter write → persist → restore chain.

**Setup**: same as Phase 4. Robot on blocks. kFF=16 fix applied. Velocity setpoint frame confirmed working.

**Procedure**: Follow the belt-drive PID tuning procedure:

1. Zero all gains: `KP0`, `KI0`, `KD0`, `KF0`
2. Start `KF0.000176` (NEO nominal), command `L200 R200`, read actual RPM from `E` line, scale kFF to match
3. Add `KP0.00005`, step-command `L0 R0` → `L500 R500`, watch rise time / overshoot, iterate kP
4. Add `KI0.0000005` only if there's steady-state error
5. Leave `KD` at 0

Record final gains via `D`. Then:

```
D                    # note kP/kI/kD/kFF from DIAG output
BURN
# Wait for "OK BURN"
```

**Power-cycle the SparkMAXes** (main battery off, wait 5 seconds, on). The Teensy can stay powered (it pulls from USB) or be reset — doesn't matter.

Once SparkMAXes boot (~2 s after power-on), reconnect serial monitor and:
```
GET 13              # kP — should match what was burned, not factory default
GET 14              # kI
GET 15              # kD
GET 16              # kF (the real kFF)
```

**Pass**: all four parameters match what was burned. This proves:
- `BURN` command actually persists to flash
- Our `setup()` does not clobber flash values at boot (confirms the `configurePID()` removal was correct)
- The parameter read path works across power cycles

**Fail modes**:
- Values reverted to factory defaults → BURN magic bytes wrong or BURN frame ID wrong
- Values reverted to `_synced.ino` `DEFAULT_KP/KI/KD/KFF` (0.0001, 0.0000005, 0, 0.000176) → our rewrite is accidentally still writing defaults somewhere. Grep the source for `DEFAULT_`. Should not happen.

**Settles**: BURN flash persistence, `configurePID()` removal correctness.

---

## Phase 6 — Host watchdog timeout

**Risk**: LOW (once Phase 4 passes). Motors briefly spin, then the test intentionally kills serial to see if they stop.

**Goal**: verify the 300 ms host watchdog actually zeroes the motors when serial goes quiet.

**Setup**: robot still on blocks. Motors verified spinning from Phase 4/5.

**Procedure**:
```
L100 R100           # wheels should spin up to ~100 RPM
```

Confirm `E` line shows ~100 RPM on both wheels.

Close the serial terminal **without sending `S` first**:
```
# In screen: Ctrl+A, then k, then y
```

Wait 1 second. Open a fresh serial terminal:
```
screen /dev/ttyACM0 115200
D
```

**Pass**:
- `DIAG` line shows `wdt=1` (watchdog tripped)
- `E` lines show wheels at rest (0 RPM)
- Recent `#` lines include `# WDT host-timeout stop`

**Fail modes**:
- Motors still spinning when serial reopens → watchdog logic broken. Check `loop()`'s `if (now - t_last_host > WATCHDOG_MS)` branch.
- Motors stopped but `wdt` still shows 0 → the flag isn't being set. Check `handleLine` / `loop` ordering.
- Motors stopped immediately on terminal close (not after 300 ms) → might be USB disconnect resetting the Teensy, not the firmware watchdog. Distinguish by checking uptime in the DIAG line (reset → small `millis()`).

**Settles**: the host watchdog safety layer works as documented.

---

## Phase 7 — ROS end-to-end integration

**Risk**: MEDIUM. Full ROS stack drives the wheels.

**Goal**: prove `/cmd_vel` → `actuator_node` → Teensy → SparkMAX → motors works end-to-end under ROS control, including the diff-drive inverse kinematics and wheel odometry publish-back.

**Prereqs**:
- `actuator_node.py` updated to speak the new serial protocol (L/R/S/E lines, 115200 baud on `/dev/ttyACM0`)
- `actuator_params.yaml` has correct `wheel_diameter_m`, `gear_ratio`, `track_width_m`
- Robot still on blocks

**Procedure**:
```bash
ssh jetson
source ~/AVROS/install/setup.bash
ros2 launch avros_bringup actuator.launch.py

# In another terminal:
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
ros2 topic echo /wheel_odom        # or whatever topic the feedback publishes to
```

Watch:
- Serial port: `E` lines should show wheel velocities
- `/wheel_odom` topic: published velocities should match
- Actual wheels: spinning forward at a rate consistent with 0.1 m/s

Sanity check the math:
```
expected_rpm = (0.1 / (π × wheel_diameter)) × gear_ratio × 60
```

Then test rotation:
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

For positive `angular.z` (counterclockwise from above):
- Left wheel should reverse
- Right wheel should spin forward
- Wheel RPM magnitude = `0.5 × (track_width / 2) / (π × wheel_diameter) × gear_ratio × 60`

**Pass criteria**:
- Wheel RPM from `E` lines matches expected value within 10%
- Wheel direction correct for both linear and angular commands
- Stopping `ros2 topic pub` causes wheels to stop (cmd_vel stops → `actuator_node` stops publishing → Teensy watchdog trips)

**Fail modes**:
- Wheels spin wrong direction → sign flip in the diff-drive inverse inside `actuator_node.py`
- RPM off by a constant factor → wheel_diameter or gear_ratio wrong in `actuator_params.yaml`
- Wheels don't stop on topic unpublish → `actuator_node` is still publishing stale cmd_vel; check the timeout path

**Settles**: the full ROS → firmware → motors chain.

---

## After all phases pass

- Commit `firmware/teensy_diff_drive/` to the AVROS `main` branch with verified PID gains burned to the SparkMAXes
- Consider upstreaming the kFF=16 fix to `Paarseus/AVL-IGVC2026` as a PR against `_synced.ino` — the upstream bug is biting anyone using that sketch
- Drop the robot off blocks, do a slow teleop drive on a flat, obstacle-free area before attempting autonomy
- If Phase 1 confirmed `SET_STATUSES_ENABLED` is redundant, delete `enableStatus2()` + its loop hook to reduce bus traffic
- If Phase 1 showed Secondary HB is ignored, delete that from `sendHeartbeats()` too

## Notes on the sniffer / param-tool sketches

Phases 1–3 need a diagnostic sketch separate from `teensy_diff_drive.ino`. Suggested name: `firmware/teensy_diag/`. It should:

- Accept `ACCEPT_ALL` filter so every frame is visible
- Print every RX frame as `RX id=0x…. dlc=.. cls=.. idx=.. dev=.. data=..`
- Support `TX <id_hex> <b0>..<b7>` for raw CAN transmit
- Support `GET <param_id>` that sends a parameter GET frame and prints the response
- Send Universal HB + Secondary HB at 20 Hz to keep the SparkMAXes enabled (else they'll refuse responses)
- Do NOT send any motor setpoints unless explicitly asked via `TX`

This sketch doesn't exist yet — write it before running Phase 1.
