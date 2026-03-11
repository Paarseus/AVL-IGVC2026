// ============================================================================
// Teensy 4.1 — SparkMAX Dual Motor Driver over CAN
// ============================================================================
// Hardware:
//   - Teensy 4.1 (CAN1 on pins CRX1=23, CTX1=22)
//   - CAN transceiver (e.g. SN65HVD230 or MCP2562) between Teensy and bus
//   - 2x REV SparkMAX motor controllers (CAN IDs 1 and 2)
//   - Bus termination: 120 Ω at each end of the CAN bus
//
// This sketch:
//   1. Initialises CAN at 1 Mbit/s (SparkMAX default)
//   2. Sends a non‑RIO heartbeat every 20 ms so the controllers stay enabled
//   3. Reads percent‑output commands from USB Serial ("L0.5 R-0.3\n")
//   4. Forwards encoder feedback back over Serial for debugging
// ============================================================================

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "SparkMaxCAN.h"

// ─── CAN bus instance (Teensy 4.1 CAN1, pins 22/23) ────────────────────────
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> canBus;

// ─── Two SparkMAX motor controllers ────────────────────────────────────────
static constexpr uint8_t LEFT_MOTOR_ID  = 1;
static constexpr uint8_t RIGHT_MOTOR_ID = 2;

SparkMaxCAN leftMotor;
SparkMaxCAN rightMotor;

// Lookup table: device_id → SparkMaxCAN*  (IDs 0‑63)
SparkMaxCAN *sparkRegistry[64] = { nullptr };

// ─── Timing ─────────────────────────────────────────────────────────────────
static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 20;
static constexpr uint32_t PRINT_INTERVAL_MS     = 200;

uint32_t lastHeartbeat = 0;
uint32_t lastPrint     = 0;

// ─── Motor setpoints (percent output: -1.0 … +1.0) ────────────────────────
float leftSetpoint  = 0.0f;
float rightSetpoint = 0.0f;

// ─── CAN RX callback ───────────────────────────────────────────────────────
void onCanRx(const CAN_message_t &msg) {
    if (!msg.flags.extended) return;          // SparkMAX uses 29‑bit IDs only

    uint8_t api_class, api_index, device_id;
    parseSparkCanId(msg.id, api_class, api_index, device_id);

    SparkMaxCAN *spark = sparkRegistry[device_id];
    if (spark) {
        spark->handleFeedback(api_class, api_index, msg.buf);
    }
}

// ─── Parse serial commands ─────────────────────────────────────────────────
// Format: "L<float> R<float>\n"   e.g.  "L0.5 R-0.3\n"
// Or:     "S\n"                   → stop (both to 0)
void processSerial() {
    static char buf[64];
    static uint8_t idx = 0;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            buf[idx] = '\0';

            if (buf[0] == 'S' || buf[0] == 's') {
                leftSetpoint  = 0.0f;
                rightSetpoint = 0.0f;
            } else {
                // Look for L and R tokens
                char *lp = strchr(buf, 'L');
                if (!lp) lp = strchr(buf, 'l');
                char *rp = strchr(buf, 'R');
                if (!rp) rp = strchr(buf, 'r');

                if (lp) leftSetpoint  = atof(lp + 1);
                if (rp) rightSetpoint = atof(rp + 1);
            }
            idx = 0;
        } else if (idx < sizeof(buf) - 1) {
            buf[idx++] = c;
        }
    }
}

// ============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}   // wait up to 3 s for USB serial
    Serial.println("Teensy 4.1 SparkMAX CAN controller");

    // ── Initialise CAN at 1 Mbit/s ────────────────────────────────────
    canBus.begin();
    canBus.setBaudRate(1000000);
    canBus.enableFIFO();
    canBus.enableFIFOInterrupt();
    canBus.onReceive(onCanRx);

    // Accept only extended frames whose upper 10 bits match SparkMAX
    // (device_type=2, manufacturer=5 → 0x0205xxxx)
    // For simplicity we accept all extended frames and filter in software.
    canBus.setFIFOFilter(REJECT_ALL);
    canBus.setFIFOFilterRange(0, 0x02050000, 0x0205FFFF, EXT);
    canBus.setFIFOFilterRange(1, 0x02050000, 0x0205FFFF, EXT);

    // ── Initialise motor objects ───────────────────────────────────────
    leftMotor.init(LEFT_MOTOR_ID,  &canBus, false);
    rightMotor.init(RIGHT_MOTOR_ID, &canBus, false);

    sparkRegistry[LEFT_MOTOR_ID]  = &leftMotor;
    sparkRegistry[RIGHT_MOTOR_ID] = &rightMotor;

    Serial.println("CAN bus started — send commands: L<pct> R<pct>  or  S (stop)");
}

// ============================================================================
void loop() {
    // Let FlexCAN process incoming messages
    canBus.events();

    // ── Read serial commands ───────────────────────────────────────────
    processSerial();

    uint32_t now = millis();

    // ── Send heartbeat + motor commands at fixed interval ──────────────
    if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeat = now;

        // Heartbeat keeps the SparkMAXes enabled (only need to send once,
        // it's broadcast to device_id 0)
        leftMotor.sendHeartbeat();

        // Send percent‑output commands
        leftMotor.setPercentOutput(leftSetpoint);
        rightMotor.setPercentOutput(rightSetpoint);
    }

    // ── Print encoder feedback for debugging ───────────────────────────
    if (now - lastPrint >= PRINT_INTERVAL_MS) {
        lastPrint = now;
        Serial.print("L_enc=");
        Serial.print(leftMotor.drivePosition(), 4);
        Serial.print("  R_enc=");
        Serial.print(rightMotor.drivePosition(), 4);
        Serial.print("  L_abs=");
        Serial.print(leftMotor.absolutePosition(), 4);
        Serial.print("  R_abs=");
        Serial.println(rightMotor.absolutePosition(), 4);
    }
}
