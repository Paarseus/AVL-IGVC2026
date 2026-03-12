// ============================================================================
// Teensy 4.1 -- SparkMAX Single Motor CAN Controller (FW 26.1.4)
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

// Status frames (fw 25/26: API Class 46)
#define API_STATUS_CLASS     46

// Enable status frames (fw 25/26: API Class 1, Index 0)
#define API_SET_STATUS_CLASS  1
#define API_SET_STATUS_INDEX  0

// Universal Heartbeat CAN ID (roboRIO format, required by fw 25+)
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
uint32_t lastHeartbeat    = 0;
uint32_t lastPrint        = 0;
uint32_t lastEncoderCfg   = 0;
uint32_t frameRxCount     = 0;
uint32_t frameTxCount     = 0;
float    motorSetpoint    = 0.0f;

// Encoder feedback (from STATUS_2, API Class 46, Index 2)
volatile float encoderVelocity = 0.0f;
volatile float encoderPosition = 0.0f;
volatile bool  gotEncoderData  = false;

#define HEARTBEAT_INTERVAL_MS  20
#define PRINT_INTERVAL_MS      500

// =====================================================================
// Send Universal Heartbeat (roboRIO format) - CAN ID 0x01011840
//
// Exact bytes from CanControl library (proven working with SparkMAX):
//   Byte 3: 0x12 = bit 25 (enabled) + bit 28 (systemWatchdog)
// =====================================================================
void sendUniversalHeartbeat() {
    CAN_message_t msg;
    msg.flags.extended = 1;
    msg.id  = UNIVERSAL_HEARTBEAT_ID;
    msg.len = 8;
    msg.buf[0] = 0x78;  // matchTimeSeconds=120
    msg.buf[1] = 0x01;  // matchNumber=1
    msg.buf[2] = 0x00;
    msg.buf[3] = 0x12;  // enabled + systemWatchdog
    msg.buf[4] = 0x59;  // timeOfDay_yr=25
    msg.buf[5] = 0x04;  // timeOfDay_month/day
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x60;  // timeOfDay_hr=12
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
    msg.id  = sparkCanId(11, 2, 0);  // broadcast
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
// Enable STATUS_2 (encoder feedback) on fw 25/26
// SET_STATUSES_ENABLED: API Class 1, Index 0
//   Bytes [0:1] = mask (which status frames to affect)
//   Bytes [2:3] = enable (which to turn on)
//   Bit 2 = STATUS_2 (primary encoder)
// =====================================================================
void enableEncoderFeedback() {
    CAN_message_t msg;
    msg.flags.extended = 1;
    msg.id  = sparkCanId(API_SET_STATUS_CLASS, API_SET_STATUS_INDEX, MOTOR_ID);
    msg.len = 4;
    // mask = 0x0004 (affect STATUS_2), enable = 0x0004 (turn it on)
    msg.buf[0] = 0x04; msg.buf[1] = 0x00;  // mask
    msg.buf[2] = 0x04; msg.buf[3] = 0x00;  // enable
    canBus.write(msg);
}

// =====================================================================
// CAN RX callback
// =====================================================================
void onCanRx(const CAN_message_t &msg) {
    if (!msg.flags.extended) return;
    frameRxCount++;

    uint8_t dev_id  = msg.id & 0x3F;
    uint8_t api_idx = (msg.id >> 6)  & 0x0F;
    uint8_t api_cls = (msg.id >> 10) & 0x3F;

    // Parse STATUS_2 (encoder) from our motor
    if (dev_id == MOTOR_ID && api_cls == API_STATUS_CLASS && api_idx == 2) {
        if (msg.len >= 8) {
            encoderVelocity = bytesToFloat(msg.buf);      // bytes 0-3
            encoderPosition = bytesToFloat(msg.buf + 4);  // bytes 4-7
            gotEncoderData = true;
        }
    }
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
                Serial.print("Encoder data: ");
                Serial.println(gotEncoderData ? "YES" : "NO");
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
    Serial.println(" SparkMAX CAN Controller (FW 26.1.4)");
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
    canBus.setFIFOFilter(ACCEPT_ALL);

    Serial.println("[CAN] CAN1 initialized at 1 Mbit/s");
    canBus.mailboxStatus();
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

    // --- Send heartbeats + motor command every 20ms ---
    if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeat = now;

        sendUniversalHeartbeat();
        sendSecondaryHeartbeat();

        // Always send duty cycle (so 0.0 is sent on stop)
        sendDutyCycle(motorSetpoint);
    }

    // --- Request encoder feedback until we get it ---
    if (!gotEncoderData && (now - lastEncoderCfg >= 1000)) {
        lastEncoderCfg = now;
        Serial.println("[CAN] Enabling encoder feedback (STATUS_2)...");
        enableEncoderFeedback();
    }

    // --- Print status every 500ms ---
    if (now - lastPrint >= PRINT_INTERVAL_MS) {
        lastPrint = now;

        Serial.print("TX=");
        Serial.print(frameTxCount);
        Serial.print(" RX=");
        Serial.print(frameRxCount);
        Serial.print(" | vel=");
        Serial.print(encoderVelocity, 2);
        Serial.print(" pos=");
        Serial.print(encoderPosition, 4);
        Serial.println();
    }
}
