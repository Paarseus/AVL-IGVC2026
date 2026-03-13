// ============================================================================
// Teensy 4.1 -- SparkMAX Dual Motor CAN Controller (FW 26.1.4)
// ============================================================================
// Hardware:
//   - Teensy 4.1 (CAN1: CTX1=pin22, CRX1=pin23)
//   - CAN transceiver (TJA1051T/3 or SN65HVD230)
//   - 2x REV SparkMAX motor controllers (CAN ID 1 = left, CAN ID 2 = right)
//   - 120 ohm termination at each end of CAN bus
//
// Serial commands (115200 baud):
//   "L0.5 R-0.3" -> left 50% fwd, right 30% rev
//   "L0.5"       -> set left only
//   "R-0.3"      -> set right only
//   "S"          -> stop both motors
//   "D"          -> print CAN diagnostics
// ============================================================================

#include <FlexCAN_T4.h>
#include <string.h>

#define LEFT_MOTOR_ID   1
#define RIGHT_MOTOR_ID  2

// =====================================================================
// CAN ID structure (29-bit extended)
//   [28:24] Device Type   (5 bits)
//   [23:16] Manufacturer  (8 bits)
//   [15:10] API Class     (6 bits)
//    [9:6]  API Index     (4 bits)
//    [5:0]  Device ID     (6 bits)
// =====================================================================

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

// Secondary Heartbeat (non-RIO, API Class 11, Index 2)
#define API_HEARTBEAT_CLASS  11
#define API_HEARTBEAT_INDEX   2

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

float leftSetpoint  = 0.0f;
float rightSetpoint = 0.0f;

// Encoder feedback per motor
volatile float leftVelocity  = 0.0f;
volatile float leftPosition  = 0.0f;
volatile float rightVelocity = 0.0f;
volatile float rightPosition = 0.0f;
volatile bool  gotLeftEncoder  = false;
volatile bool  gotRightEncoder = false;

#define HEARTBEAT_INTERVAL_MS  20
#define PRINT_INTERVAL_MS      500

// =====================================================================
// Send Universal Heartbeat (roboRIO format) - CAN ID 0x01011840
// Byte 3: 0x12 = bit 25 (enabled) + bit 28 (systemWatchdog)
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
// =====================================================================
void sendSecondaryHeartbeat() {
    CAN_message_t msg;
    msg.flags.extended = 1;
    msg.id  = sparkCanId(API_HEARTBEAT_CLASS, API_HEARTBEAT_INDEX, 0);
    msg.len = 8;
    memset(msg.buf, 0xFF, 8);
    int r = canBus.write(msg);
    if (r > 0) frameTxCount++;
}

// =====================================================================
// Send duty cycle command to a motor
// =====================================================================
void sendDutyCycle(uint8_t motor_id, float pct) {
    if (pct >  1.0f) pct =  1.0f;
    if (pct < -1.0f) pct = -1.0f;
    CAN_message_t msg;
    msg.flags.extended = 1;
    msg.id  = sparkCanId(API_DUTYCYCLE_CLASS, API_DUTYCYCLE_INDEX, motor_id);
    msg.len = 8;
    memset(msg.buf, 0, 8);
    floatToBytes(pct, msg.buf);
    int r = canBus.write(msg);
    if (r > 0) frameTxCount++;
}

// =====================================================================
// Enable STATUS_2 (encoder feedback) for a motor
// =====================================================================
void enableEncoderFeedback(uint8_t motor_id) {
    CAN_message_t msg;
    msg.flags.extended = 1;
    msg.id  = sparkCanId(API_SET_STATUS_CLASS, API_SET_STATUS_INDEX, motor_id);
    msg.len = 8;
    memset(msg.buf, 0, 8);
    msg.buf[0] = 0x04; msg.buf[1] = 0x00;  // mask: STATUS_2
    msg.buf[2] = 0x04; msg.buf[3] = 0x00;  // enable: STATUS_2
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

    // Parse STATUS_2 (encoder): API Class 46, Index 2
    if (api_cls == API_STATUS_CLASS && api_idx == 2 && msg.len >= 8) {
        float vel = bytesToFloat(msg.buf);
        float pos = bytesToFloat(msg.buf + 4);

        if (dev_id == LEFT_MOTOR_ID) {
            leftVelocity = vel;
            leftPosition = pos;
            gotLeftEncoder = true;
        } else if (dev_id == RIGHT_MOTOR_ID) {
            rightVelocity = vel;
            rightPosition = pos;
            gotRightEncoder = true;
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
                leftSetpoint  = 0.0f;
                rightSetpoint = 0.0f;
                Serial.println(">> STOP");
            } else if (buf[0] == 'D' || buf[0] == 'd') {
                Serial.println("\n=== DIAGNOSTICS ===");
                canBus.mailboxStatus();
                Serial.print("TX: "); Serial.println(frameTxCount);
                Serial.print("RX: "); Serial.println(frameRxCount);
                Serial.print("Left encoder:  ");
                Serial.println(gotLeftEncoder ? "YES" : "NO");
                Serial.print("Right encoder: ");
                Serial.println(gotRightEncoder ? "YES" : "NO");
                Serial.println("===================\n");
            } else {
                char *lp = strchr(buf, 'L');
                if (!lp) lp = strchr(buf, 'l');
                char *rp = strchr(buf, 'R');
                if (!rp) rp = strchr(buf, 'r');

                if (lp) leftSetpoint  = atof(lp + 1);
                if (rp) rightSetpoint = atof(rp + 1);
                Serial.print(">> L=");
                Serial.print(leftSetpoint, 2);
                Serial.print(" R=");
                Serial.println(rightSetpoint, 2);
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
    Serial.println(" SparkMAX Dual Motor CAN (FW 26.1.4)");
    Serial.println(" Left=ID1  Right=ID2");
    Serial.println("========================================");
    Serial.println();

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
    Serial.println("Commands: L<pct> R<pct>, S (stop), D (diag)");
    Serial.println("Example: L0.5 R-0.3");
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

    // --- Send heartbeats + motor commands every 20ms ---
    if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        lastHeartbeat = now;

        sendUniversalHeartbeat();
        sendSecondaryHeartbeat();

        sendDutyCycle(LEFT_MOTOR_ID, leftSetpoint);
        sendDutyCycle(RIGHT_MOTOR_ID, rightSetpoint);
    }

    // --- Request encoder feedback until both respond ---
    if ((!gotLeftEncoder || !gotRightEncoder) && (now - lastEncoderCfg >= 1000)) {
        lastEncoderCfg = now;
        Serial.println("[CAN] Enabling encoder feedback (STATUS_2)...");
        enableEncoderFeedback(LEFT_MOTOR_ID);
        enableEncoderFeedback(RIGHT_MOTOR_ID);
    }

    // --- Print status every 500ms ---
    if (now - lastPrint >= PRINT_INTERVAL_MS) {
        lastPrint = now;

        Serial.print("TX=");
        Serial.print(frameTxCount);
        Serial.print(" RX=");
        Serial.print(frameRxCount);
        Serial.print(" | L: vel=");
        Serial.print(leftVelocity, 2);
        Serial.print(" pos=");
        Serial.print(leftPosition, 4);
        Serial.print(" | R: vel=");
        Serial.print(rightVelocity, 2);
        Serial.print(" pos=");
        Serial.println(rightPosition, 4);
    }
}
