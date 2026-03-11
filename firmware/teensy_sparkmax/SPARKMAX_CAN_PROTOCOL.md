# REV SparkMAX CAN Protocol Reference

This document describes the CAN bus protocol used by REV Robotics SparkMAX motor controllers, as reverse-engineered from working firmware and REV's public documentation.

---

## 1. Physical Layer

| Parameter        | Value                         |
|-----------------|-------------------------------|
| Bus standard    | CAN 2.0B (extended frames)    |
| Baud rate       | **1 Mbit/s**                  |
| Frame type      | Extended (29-bit identifier)  |
| Termination     | 120 Ω at each end of the bus  |
| Connector       | JST-PH 4-pin on the SparkMAX |

SparkMAX uses **only extended (29-bit) CAN frames**. Standard 11-bit frames are ignored.

### Wiring

```
Teensy 4.1          CAN Transceiver         SparkMAX
─────────           (SN65HVD230 / MCP2562)
 CTX (pin 22) ───── TXD                     CAN_H ─────── CAN_H
 CRX (pin 23) ───── RXD                     CAN_L ─────── CAN_L
 3.3V ──────────── VCC                      GND ────────── GND
 GND ───────────── GND

                    120Ω between CAN_H and CAN_L at each bus endpoint
```

You can daisy-chain multiple SparkMAXes on the same CAN bus. Each must have a unique Device ID (set via the REV Hardware Client or USB-C on the SparkMAX).

---

## 2. CAN ID Structure (29-bit Extended)

Every SparkMAX CAN frame uses a 29-bit extended identifier packed as follows:

```
 28  27  26  25  24  23  22  21  20  19  18  17  16  15  14  13  12  11  10   9   8   7   6   5   4   3   2   1   0
├──────────────┤├──────────────────────────────────┤├──────────────────────────────────┤├────────────────┤├────────────────────┤
   Device Type              Manufacturer                      API Class                   API Index          Device ID
   (5 bits)                   (8 bits)                         (6 bits)                   (4 bits)           (6 bits)
```

| Field             | Bits    | Width | SparkMAX Value | Description                              |
|-------------------|---------|-------|----------------|------------------------------------------|
| **Device Type**   | [28:24] | 5     | `2`            | Motor Controller                         |
| **Manufacturer**  | [23:16] | 8     | `5`            | REV Robotics                             |
| **API Class**     | [15:10] | 6     | varies         | Command/status group (see table below)   |
| **API Index**     | [9:6]   | 4     | varies         | Specific function within the API class   |
| **Device ID**     | [5:0]   | 6     | 1–62           | Unique per controller (0 = broadcast)    |

### Building a CAN ID in code

```cpp
uint32_t can_id = (device_type  << 24)   // 2
                | (manufacturer << 16)   // 5
                | (api_class    << 10)
                | (api_index    <<  6)
                | (device_id);
```

### Example

For a **percent output command** to **device 1**:
- Device Type = 2, Manufacturer = 5, API Class = 0, API Index = 2, Device ID = 1
- `can_id = (2 << 24) | (5 << 16) | (0 << 10) | (2 << 6) | 1 = 0x02050081`

---

## 3. API Classes and Indices

### 3.1 Command APIs (Controller → SparkMAX)

| Function           | API Class | API Index | DLC | Data Payload                     |
|--------------------|-----------|-----------|-----|----------------------------------|
| Percent Output     | 0         | 2         | 8   | `float` (bytes 0–3, LE), rest 0 |
| Velocity Setpoint  | 1         | 2         | 8   | `float` RPM (bytes 0–3, LE)     |
| Position Setpoint  | 3         | 2         | 8   | `float` rotations (bytes 0–3, LE)|

All setpoint values are encoded as an **IEEE 754 single-precision float** in **little-endian** byte order in the first 4 bytes of the data payload. Bytes 4–7 are zero-padded.

#### Percent Output (-1.0 to +1.0)
```
Data[0..3] = float value, little-endian
Data[4..7] = 0x00
```
- `+1.0` = full forward
- `-1.0` = full reverse
- `0.0` = stop

#### Velocity (RPM)
```
Data[0..3] = float RPM, little-endian
Data[4..7] = 0x00
```

#### Position (rotations)
```
Data[0..3] = float rotations, little-endian
Data[4..7] = 0x00
```

### 3.2 Non-RIO Heartbeat (CRITICAL)

| Function           | API Class | API Index | DLC | Data Payload                     |
|--------------------|-----------|-----------|-----|----------------------------------|
| Non-RIO Heartbeat  | 11        | 2         | 8   | `FF FF FF FF 00 00 00 00`       |

**The heartbeat is the most important frame.** If the SparkMAX does not receive a heartbeat within its timeout (~100 ms), it will **disable all motor output** as a safety measure.

Key points:
- Must be sent at ≤ 100 ms intervals (recommended: every 10–20 ms)
- The Device ID field in the CAN ID is set to **0** (broadcast) — one heartbeat enables all controllers on the bus
- Data is always the fixed 8 bytes: `0xFF 0xFF 0xFF 0xFF 0x00 0x00 0x00 0x00`

```
CAN ID for heartbeat = (2 << 24) | (5 << 16) | (11 << 10) | (2 << 6) | 0
                      = 0x02052C80
```

### 3.3 Feedback Configuration (Request Periodic Status)

To receive encoder/position feedback, you must first tell the SparkMAX *how often* to send it. This is done by writing a period (in milliseconds) to the appropriate status frame config:

| Feedback Type             | API Class | API Index | DLC | Data                                         |
|---------------------------|-----------|-----------|-----|----------------------------------------------|
| Velocity/Encoder config   | 6         | 2         | 2   | `uint16_t` period_ms, little-endian          |
| Position feedback config  | 6         | 3         | 2   | `uint16_t` period_ms, little-endian          |
| Absolute encoder config   | 6         | 5         | 2   | `uint16_t` period_ms, little-endian          |

Example: to request encoder data every 50 ms:
```
Data[0] = 0x32   (50 & 0xFF)
Data[1] = 0x00   (50 >> 8)
```

### 3.4 Feedback Frames (SparkMAX → Controller)

Once configured, the SparkMAX sends periodic status frames:

| Feedback Type         | API Class | API Index | Data (bytes 0–3)              |
|-----------------------|-----------|-----------|-------------------------------|
| Drive encoder position| 6         | 2         | `float` rotations, LE         |
| Absolute encoder pos  | 6         | 5         | `float` position (0.0–1.0), LE|

The **drive encoder position** is cumulative rotations of the motor shaft. The **absolute encoder position** is a value from 0.0 to 1.0 representing one full revolution.

### 3.5 Parameters

| Function           | API Class | API Index | DLC | Description              |
|--------------------|-----------|-----------|-----|--------------------------|
| Set Parameter      | 48        | 0         | 8   | Parameter ID + value     |

Parameters control things like PID gains, current limits, idle mode, etc. These are typically configured once via the REV Hardware Client rather than over CAN at runtime.

---

## 4. Data Encoding

### Float Encoding (Little-Endian)

All floating-point values on the SparkMAX CAN bus use **IEEE 754 single-precision (32-bit) float** in **little-endian** byte order.

```cpp
// Encode
void floatToBytes(float f, uint8_t *out) {
    memcpy(out, &f, sizeof(float));  // ARM is little-endian natively
}

// Decode
float bytesToFloat(const uint8_t *data) {
    float v;
    memcpy(&v, data, sizeof(float));
    return v;
}
```

On ARM (Teensy 4.1 / STM32), the native byte order is little-endian, so `memcpy` works directly with no byte-swapping.

### Integer Encoding

Integers (like the feedback period) are also little-endian:
```cpp
uint16_t period = 50;
data[0] = period & 0xFF;       // low byte
data[1] = (period >> 8) & 0xFF; // high byte
```

---

## 5. Communication Flow

### Startup Sequence

```
1. Initialize CAN bus at 1 Mbit/s
2. For each SparkMAX:
   a. Send feedback config frames to request periodic encoder data
3. Enter main loop
```

### Main Loop (every 10–20 ms)

```
1. Send Non-RIO Heartbeat (broadcast, device_id=0)
2. Send motor commands (percent output / velocity / position)
3. Process incoming feedback frames from SparkMAX
```

### Timing Diagram

```
Time ──────────────────────────────────────────────────────────►

Controller:  [HB] [CMD1] [CMD2]        [HB] [CMD1] [CMD2]       [HB] ...
               │     │      │             │     │      │           │
SparkMAX 1:    │     └──►(drive)         │     └──►(drive)       │
SparkMAX 2:    │            └──►(drive)  │            └──►(drive)│
               │                          │                       │
SparkMAX 1:  [ENC]─────────────────────[ENC]──────────────────[ENC]──►
SparkMAX 2:  [ENC]─────────────────────[ENC]──────────────────[ENC]──►
               ◄───── 20 ms ──────────►
```

- **HB** = Heartbeat (broadcast)
- **CMD** = Motor command (per-device)
- **ENC** = Encoder feedback (per-device, at configured rate)

---

## 6. CAN ID Quick Reference Table

| Purpose                    | API Class | API Index | Device ID | Full CAN ID (device 1)  |
|----------------------------|-----------|-----------|-----------|--------------------------|
| Percent output             | 0         | 2         | 1         | `0x02050081`             |
| Velocity setpoint          | 1         | 2         | 1         | `0x02050481`             |
| Position setpoint          | 3         | 2         | 1         | `0x02050C81`             |
| Encoder config (50 ms)     | 6         | 2         | 1         | `0x02051881`             |
| Position config (50 ms)    | 6         | 3         | 1         | `0x020518C1`             |
| Abs encoder config (50 ms) | 6         | 5         | 1         | `0x02051941`             |
| Encoder feedback           | 6         | 2         | 1         | `0x02051881`             |
| Abs encoder feedback       | 6         | 5         | 1         | `0x02051941`             |
| Non-RIO heartbeat          | 11        | 2         | 0         | `0x02052C80`             |
| Set parameter              | 48        | 0         | 1         | `0x0205C001`             |

---

## 7. Troubleshooting

### Motors not spinning
1. **No heartbeat** — The most common issue. SparkMAX disables output if it doesn't receive a heartbeat within ~100 ms. Verify heartbeat frames are being sent at ≤ 20 ms intervals.
2. **Wrong baud rate** — SparkMAX defaults to 1 Mbit/s. Double-check your CAN init.
3. **Missing termination** — The bus needs 120 Ω resistors at both endpoints. Without them, signals reflect and corrupt.
4. **Wrong device ID** — Verify via the REV Hardware Client (connect SparkMAX over USB-C).

### No encoder feedback
1. **Feedback not configured** — You must send the feedback config frames (API Class 6, Index 2/3/5) with a period before the SparkMAX will start sending data.
2. **CAN filter too strict** — Make sure your receive filters accept the SparkMAX's feedback CAN IDs.

### CAN bus errors
1. **No transceiver** — Teensy 4.1 CAN pins output logic-level signals. You need a transceiver chip (SN65HVD230 or MCP2562) to convert to differential CAN_H/CAN_L.
2. **Wiring swapped** — CAN_H and CAN_L swapped will prevent communication.
3. **Bus contention** — If another device is sending at a different baud rate, the bus will produce errors.

---

## 8. Hardware Setup for Teensy 4.1

### Teensy 4.1 CAN Pins

| Teensy Pin | Function |
|------------|----------|
| 22         | CTX1 (CAN1 Transmit) |
| 23         | CRX1 (CAN1 Receive)  |

### Required Components

1. **CAN Transceiver** — SN65HVD230 (3.3V) or MCP2562 (with 3.3V VIO). The transceiver converts the Teensy's logic-level TX/RX to differential CAN_H/CAN_L.
2. **120 Ω termination resistors** — One at the Teensy end, one at the far end of the bus. If you only have 2 devices close together, one resistor may suffice.
3. **Power** — SparkMAXes are powered from the main battery through their power input. The CAN bus only carries signal, not power.

### Wiring Diagram

```
                     ┌──────────────────┐
    Teensy 4.1       │  CAN Transceiver │         CAN Bus
   ┌──────────┐      │  (SN65HVD230)    │    ┌─────────────────┐
   │  Pin 22 ─┼──────┤ TXD         CANH ├────┤ CAN_H           │
   │  Pin 23 ─┼──────┤ RXD         CANL ├────┤ CAN_L           │
   │   3.3V  ─┼──────┤ VCC              │    │   ┌─[120Ω]─┐   │
   │   GND   ─┼──────┤ GND              │    │   │         │   │
   └──────────┘      └──────────────────┘    │  CANH     CANL  │
                                              │                 │
                      ┌───────────────┐       │                 │
                      │  SparkMAX #1  │       │                 │
                      │  (ID = 1)     ├───────┤                 │
                      └───────────────┘       │                 │
                      ┌───────────────┐       │                 │
                      │  SparkMAX #2  │       │                 │
                      │  (ID = 2)     ├───────┤                 │
                      └───────────────┘       │   ┌─[120Ω]─┐   │
                                              │  CANH     CANL  │
                                              └─────────────────┘
```
