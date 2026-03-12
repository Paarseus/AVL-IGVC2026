// ============================================================================
// Teensy 4.1 -- SparkMAX Single Motor CAN Controller (FW 26 Compatible)
// ============================================================================
// Hardware:
//   - Teensy 4.1 (CAN1: CTX1=pin22, CRX1=pin23)
//   - CAN transceiver (TJA1051T/3 or SN65HVD230)
//   - 1x REV SparkMAX motor controller (CAN ID 1)
//   - 120 ohm termination at each end of CAN bus
//
// Serial commands (115200 baud):
//   "M0.5"  -> motor 50% forward
//   "M-0.3" -> motor 30% reverse
//   "S"     -> stop motor
//   "D"     -> print CAN diagnostics
// ============================================================================

#include <FlexCAN_T4.h>
#include <string.h>

#define MOTOR_ID  1

// =====================================================================
// CAN ID structure (29-bit extended)
//   [28:24] Device Type   (5 bits)
//   [23:16] Manufacturer  (8 bits)
//   [15:10] API Class     (6 bits)
//    [9:6]  API Index     (4 bits)
//    [5:0]  Device ID     (6 bits)
// =====================================================================

// SparkMAX constants
#define SPARK_DEV_TYPE   2   // Motor Controller
#define SPARK_MFG        5   // REV Robotics

// Duty cycle setpoint
#define API_DUTYCYCLE_CLASS   0
#define API_DUTYCYCLE_INDEX   2

// Universal Heartbeat CAN ID (roboRIO format, required by fw 25+)
// Device Type=1 (Robot Controller), Manufacturer=1 (NI), API Class=6, Index=1, ID=0
#define UNIVERSAL_HEARTBEAT_ID  0x01011840

// =====================================================================
// Helpers
// =====================================================================
uint32_t sparkCanId(uint8_t api_class, uint8_t api_index, uint8_t device_id) {
    return ((uint32_t)SPARK_DEV_TYPE << 24) |
           ((uint32_t)SPARK_MFG     << 16) |
           ((uint32_t)(api_class & 0x3F) << 10) |
           ((uint32_t)(api_index & 0x0F) <<  6) |
           ((uint32_t)(device_id & 0x3F));
}

void floatToBytes(float f, uint8_t *out) {
    memcpy(out, &f, sizeof(float));
}

float bytesToFloat(const uint8_t *data) {
    float v;
    memcpy(&v, data, sizeof(float));
    return v;
}

// =====================================================================
// CAN bus
// =====================================================================
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> canBus;

// =====================================================================
// Global state
// =====================================================================
uint32_t lastHeartbeat = 0;
uint32_t lastPrint     = 0;
uint32_t frameRxCount  = 0;
uint32_t frameTxCount  = 0;
float    motorSetpoint = 0.0f;

#define HEARTBEAT_INTERVAL_MS  20
#define PRINT_INTERVAL_MS      500

// =====================================================================
// Send Universal Heartbeat (roboRIO format) - CAN ID 0x01011840
//
// Exact bytes from CanControl library (proven working with SparkMAX):
//   Byte 0: 0x78 (matchTimeSeconds=120)
//   Byte 1: 0x01 (matchNumber=1)
//   Byte 2: 0x00
//   Byte 3: 0x12 (bit 25: enabled=1, bit 28: systemWatchdog=1)
//   Byte 4: 0x59 (timeOfDay_yr=25)
//   Byte 5: 0x04 (timeOfDay_month/day)
//   Byte 6: 0x00
//   Byte 7: 0x60 (timeOfDay_hr=12)
// =====================================================================
void sendUniversalHeartbeat() {
    CAN_message_t msg;
    msg.flags.extended = 1;
    msg.id  = UNIVERSAL_HEARTBEAT_ID;
    msg.len = 8;
    msg.buf[0] = 0x78;
    msg.buf[1] = 0x01;
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x12;  // enabled + systemWatchdog
    msg.buf[4] = 0x59;
    msg.buf[5] = 0x04;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x60;
    int r = canBus.write(msg);
    if (r > 0) frameTxCount++;
}

// =====================================================================
// Send Secondary Heartbeat (non-RIO, API Class 11, Index 2)
// Data = 64-bit bitfield, bit N enables device N
// =====================================================================
void sendSecondaryHeartbeat() {
    CAN_message_t msg;
    msg.flags.extended = 1;
    // Secondary heartbeat uses device_id=0 in CAN ID (broadcast)
    msg.id  = sparkCanId(11, 2, 0);
    msg.len = 8;
    memset(msg.buf, 0xFF, 8);  // enable all devices
    int r = canBus.write(msg);
    if (r > 0) frameTxCount++;
}

// =====================================================================
// Send duty cycle command to motor
// =====================================================================
void sendDutyCycle(float pct) {
    if (pct >  1.0f) pct =  1.0f;
    if (pct < -1.0f) pct = -1.0f;
    CAN_message_t msg;
    msg.flags.extended = 1;
    msg.id  = sparkCanId(API_DUTYCYCLE_CLASS, API_DUTYCYCLE_INDEX, MOTOR_ID);
    msg.len = 8;
    memset(msg.buf, 0, 8);
    floatToBytes(pct, msg.buf);
    int r = canBus.write(msg);
    if (r > 0) frameTxCount++;
}

// =====================================================================
// CAN RX callback -- dump ALL received frames for diagnostics
// =====================================================================
void onCanRx(const CAN_message_t &msg) {
    frameRxCount++;

    // Print raw frame for debugging
    Serial.print("[RX] ID=0x");
    Serial.print(msg.id, HEX);
    Serial.print(msg.flags.extended ? " EXT" : " STD");
    Serial.print(" DLC=");
    Serial.print(msg.len);
    Serial.print(" DATA=");
    for (int i = 0; i < msg.len; i++) {
        if (msg.buf[i] < 0x10) Serial.print("0");
        Serial.print(msg.buf[i], HEX);
        if (i < msg.len - 1) Serial.print(" ");
    }

    // Decode SparkMAX fields if extended
    if (msg.flags.extended) {
        uint8_t dev_id    = msg.id & 0x3F;
        uint8_t api_idx   = (msg.id >> 6)  & 0x0F;
        uint8_t api_cls   = (msg.id >> 10) & 0x3F;
        uint8_t mfg       = (msg.id >> 16) & 0xFF;
        uint8_t dev_type  = (msg.id >> 24) & 0x1F;
        Serial.print(" | DT=");
        Serial.print(dev_type);
        Serial.print(" MFG=");
        Serial.print(mfg);
        Serial.print(" CLS=");
        Serial.print(api_cls);
        Serial.print(" IDX=");
        Serial.print(api_idx);
        Serial.print(" DEV=");
        Serial.print(dev_id);
    }
    Serial.println();
}

// =====================================================================
// Parse serial commands
// =====================================================================
void processSerial() {
    static char buf[64];
    static uint8_t idx = 0;

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            buf[idx] = '\0';
            if (idx == 0) { idx = 0; continue; }

            if (buf[0] == 'S' || buf[0] == 's') {
                motorSetpoint = 0.0f;
                Serial.println(">> STOP");
            } else if (buf[0] == 'D' || buf[0] == 'd') {
                Serial.println("\n=== DIAGNOSTICS ===");
                canBus.mailboxStatus();
                Serial.print("TX: "); Serial.println(frameTxCount);
                Serial.print("RX: "); Serial.println(frameRxCount);
                Serial.println("===================\n");
            } else if (buf[0] == 'M' || buf[0] == 'm') {
                motorSetpoint = atof(buf + 1);
                Serial.print(">> Motor = ");
                Serial.println(motorSetpoint, 2);
            }
            idx = 0;
        } else if (idx < sizeof(buf) - 1) {
            buf[idx++] = c;
        }
    }
}

// =====================================================================
// setup()
// =====================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println();
    Serial.println("========================================");
    Serial.println(" SparkMAX CAN Diagnostic (FW 26)");
    Serial.println(" Single Motor - CAN ID 1");
    Serial.println("========================================");
    Serial.println();

    // --- CAN bus init ---
    canBus.begin();
    canBus.setBaudRate(1000000);
    canBus.setMaxMB(16);
    canBus.enableFIFO();
    canBus.enableFIFOInterrupt();
    canBus.onReceive(onCanRx);

    // Accept ALL extended frames (no filtering -- we want to see everything)
    canBus.setFIFOFilter(ACCEPT_ALL);

    Serial.println("[CAN] CAN1 initialized at 1 Mbit/s");
    Serial.println("[CAN] Filters: ACCEPT ALL (diagnostic mode)");
    canBus.mailboxStatus();
    Serial.println();

    Serial.println("Heartbeat: Universal (0x01011840) for fw 26.1.4");
    Serial.println();
    Serial.println("Commands: M<pct> (motor), S (stop), D (diag)");
    Serial.println("Example: M0.3");
    Serial.println("----------------------------------------");
    Serial.println();
}

// =====================================================================
// loop()
// =====================================================================
void loop() {
    canBus.events();
    processSerial();

    uint32_t now = millis();

    // --- Send heartbeats every 20ms ---
    if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeat = now;

        // Send BOTH heartbeat formats
        sendUniversalHeartbeat();   // roboRIO format (fw 25/26)
        sendSecondaryHeartbeat();   // REV non-RIO format (fallback)

        // Always send duty cycle so SparkMAX receives 0.0 on stop
        sendDutyCycle(motorSetpoint);
    }

    // --- Status print every 500ms ---
    if (now - lastPrint >= PRINT_INTERVAL_MS) {
        lastPrint = now;

        Serial.print("TX=");
        Serial.print(frameTxCount);
        Serial.print(" RX=");
        Serial.print(frameRxCount);

        if (frameRxCount == 0 && now > 5000) {
            Serial.println(" | NO RESPONSE - check wiring & power");
        } else {
            Serial.println();
        }
    }
}
