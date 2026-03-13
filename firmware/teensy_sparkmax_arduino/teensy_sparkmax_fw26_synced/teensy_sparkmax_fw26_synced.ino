// ============================================================================
// Teensy 4.1 -- SparkMAX Dual Velocity PID Controller (FW 26.1.4)
// ============================================================================
// Differential drive using SparkMAX built-in velocity PID.
// Each motor independently tracks its RPM setpoint -- no software sync needed.
//
// Hardware:
//   Teensy 4.1 (CAN1: CTX1=pin22, CRX1=pin23)
//   CAN transceiver (TJA1051T/3 or SN65HVD230) + 120 ohm termination
//   2x SparkMAX: CAN ID 1 (left), CAN ID 2 (right)
//
// Serial commands (115200 baud):
//   "V1000"          -> both wheels at 1000 RPM
//   "V1000 W200"     -> left=800 RPM, right=1200 RPM
//   "L500 R-500"     -> direct RPM per motor
//   "S"              -> stop both motors
//   "D"              -> diagnostics
//   "KP0.0001"       -> set kP on both motors
//   "KI0.0000005"    -> set kI
//   "KD0"            -> set kD
//   "KF0.000176"     -> set kFF (feedforward)
//   "BURN"           -> persist PID gains to SparkMAX flash
//
// PID tuning guide:
//   1. Start with only kFF = 1/freeSpeedRPM (~0.000176 for NEO)
//   2. Send "V1000", watch actual vs target in serial output
//   3. Add kP (start 0.0001, increase until responsive but not oscillating)
//   4. Add kI if steady-state error remains (start 0.0000005)
//   5. kD usually not needed for velocity
//   6. "BURN" to save gains to SparkMAX flash
// ============================================================================

#include <FlexCAN_T4.h>
#include <string.h>

// --- Motor CAN IDs ---
#define LEFT_ID   1
#define RIGHT_ID  2

// --- CAN protocol (SparkMAX 29-bit extended) ---
#define DEV_TYPE  2    // Motor Controller
#define MFG       5    // REV Robotics
#define CLS_VEL   1    // Velocity setpoint
#define IDX_SET   2    // Setpoint index
#define CLS_STATUS 46  // Status frame class (fw 25/26)
#define CLS_ENABLE 1   // Enable status frames
#define CLS_HB    11   // Secondary heartbeat class
#define CLS_PARAM 48   // Parameter access
#define IDX_BURN  2    // Burn flash sub-index
#define UNIVERSAL_HB_ID 0x01011840

// --- SparkMAX parameter IDs (PID slot 0, from REV SDK ConfigParameter enum) ---
#define PID_KP       13
#define PID_KI       14
#define PID_KD       15
#define PID_KFF      17
#define PID_IZONE    18
#define PID_OUTMIN   22
#define PID_OUTMAX   23
#define PTYPE_FLOAT  2

// --- Default PID gains for NEO velocity control ---
#define DEFAULT_KP   0.0001f
#define DEFAULT_KI   0.0000005f
#define DEFAULT_KD   0.0f
#define DEFAULT_KFF  0.000176f   // ~1/5676 (NEO free speed)

// --- Timing ---
#define CTRL_DT_MS     20    // 50 Hz control loop
#define PRINT_MS       500
#define ENCODER_CFG_MS 1000

// --- Safety ---
#define MAX_RPM 3000.0f   // well below NEO free speed (5676)

// =====================================================================
// CAN bus
// =====================================================================
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;

// =====================================================================
// State
// =====================================================================
float vCmd = 0, wCmd = 0;
float leftDirect = 0, rightDirect = 0;
bool directMode = false;

volatile float leftVel = 0, leftPos = 0;
volatile float rightVel = 0, rightPos = 0;
volatile bool gotLeft = false, gotRight = false;

float curKP = DEFAULT_KP, curKI = DEFAULT_KI;
float curKD = DEFAULT_KD, curKFF = DEFAULT_KFF;

uint32_t tCtrl = 0, tPrint = 0, tEncCfg = 0;
uint32_t txCount = 0, rxCount = 0;

// =====================================================================
// CAN helpers
// =====================================================================
uint32_t sparkId(uint8_t cls, uint8_t idx, uint8_t dev) {
    return ((uint32_t)DEV_TYPE << 24) | ((uint32_t)MFG << 16) |
           ((uint32_t)(cls & 0x3F) << 10) | ((uint32_t)(idx & 0x0F) << 6) |
           (dev & 0x3F);
}

void canSend(uint32_t id, const uint8_t *data, uint8_t len) {
    CAN_message_t m;
    m.flags.extended = 1;
    m.id = id;
    m.len = len;
    memcpy(m.buf, data, len);
    if (can.write(m) > 0) txCount++;
}

float toFloat(const uint8_t *b)    { float v; memcpy(&v, b, 4); return v; }
void fromFloat(float f, uint8_t *b) { memcpy(b, &f, 4); }

// =====================================================================
// SparkMAX parameter access
// =====================================================================
void setParam(uint8_t motor_id, uint8_t param_id, float value) {
    uint8_t d[8] = {};
    fromFloat(value, d);          // bytes 0-3: value (float LE)
    d[4] = param_id;              // byte 4: parameter ID
    d[5] = PTYPE_FLOAT;           // byte 5: type = float
    canSend(sparkId(CLS_PARAM, 0, motor_id), d, 8);
}

void burnFlash(uint8_t motor_id) {
    uint8_t d[8] = {};
    d[0] = 0xA3; d[1] = 0x3A;    // magic bytes
    canSend(sparkId(CLS_PARAM, IDX_BURN, motor_id), d, 8);
}

void configurePID(uint8_t motor_id, float kp, float ki, float kd, float kff) {
    setParam(motor_id, PID_KP,     kp);   delay(5);
    setParam(motor_id, PID_KI,     ki);   delay(5);
    setParam(motor_id, PID_KD,     kd);   delay(5);
    setParam(motor_id, PID_KFF,    kff);  delay(5);
    setParam(motor_id, PID_OUTMIN, -1.0f); delay(5);
    setParam(motor_id, PID_OUTMAX,  1.0f); delay(5);
}

// =====================================================================
// Motor commands
// =====================================================================
void sendHeartbeats() {
    uint8_t uni[] = {0x78, 0x01, 0x00, 0x12, 0x59, 0x04, 0x00, 0x60};
    canSend(UNIVERSAL_HB_ID, uni, 8);
    uint8_t sec[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    canSend(sparkId(CLS_HB, IDX_SET, 0), sec, 8);
}

void setVelocity(uint8_t id, float rpm) {
    rpm = constrain(rpm, -MAX_RPM, MAX_RPM);
    uint8_t d[8] = {};
    fromFloat(rpm, d);
    canSend(sparkId(CLS_VEL, IDX_SET, id), d, 8);
}

void enableEncoder(uint8_t id) {
    uint8_t d[8] = {};
    d[0] = 0x04; d[2] = 0x04;
    canSend(sparkId(CLS_ENABLE, 0, id), d, 8);
}

// =====================================================================
// CAN RX callback
// =====================================================================
void onRx(const CAN_message_t &msg) {
    if (!msg.flags.extended || msg.len < 8) return;
    rxCount++;

    uint8_t dev = msg.id & 0x3F;
    uint8_t cls = (msg.id >> 10) & 0x3F;
    uint8_t idx = (msg.id >> 6)  & 0x0F;
    if (cls != CLS_STATUS || idx != 2) return;

    float v = toFloat(msg.buf), p = toFloat(msg.buf + 4);
    if (dev == LEFT_ID)  { leftVel  = v; leftPos  = p; gotLeft  = true; }
    if (dev == RIGHT_ID) { rightVel = v; rightPos = p; gotRight = true; }
}

// =====================================================================
// Serial command parser
// =====================================================================
void processSerial() {
    static char buf[64];
    static uint8_t i = 0;

    while (Serial.available()) {
        char c = Serial.read();
        if (c != '\n' && c != '\r') {
            if (i < sizeof(buf) - 1) buf[i++] = c;
            continue;
        }
        buf[i] = '\0';
        if (i == 0) continue;

        char cmd = toupper(buf[0]);

        if (cmd == 'S') {
            vCmd = wCmd = leftDirect = rightDirect = 0;
            Serial.println("STOP");

        } else if (cmd == 'D') {
            float lT = directMode ? leftDirect : (vCmd - wCmd);
            float rT = directMode ? rightDirect : (vCmd + wCmd);
            Serial.printf("TX=%lu RX=%lu\n", txCount, rxCount);
            Serial.printf("  L: %6.0f / %6.0f rpm  pos=%.3f  %s\n",
                          leftVel, lT, leftPos, gotLeft ? "OK" : "--");
            Serial.printf("  R: %6.0f / %6.0f rpm  pos=%.3f  %s\n",
                          rightVel, rT, rightPos, gotRight ? "OK" : "--");
            Serial.printf("  PID: P=%.6f I=%.7f D=%.6f FF=%.6f\n",
                          curKP, curKI, curKD, curKFF);

        } else if (cmd == 'V') {
            directMode = false;
            vCmd = atof(buf + 1);
            char *wp = strchr(buf, 'W');
            if (!wp) wp = strchr(buf, 'w');
            wCmd = wp ? atof(wp + 1) : 0;
            Serial.printf("V=%.0f W=%.0f -> L=%.0f R=%.0f RPM\n",
                          vCmd, wCmd, vCmd - wCmd, vCmd + wCmd);

        } else if (cmd == 'K' && i >= 2) {
            char which = toupper(buf[1]);
            float val = atof(buf + 2);
            uint8_t pid = 0;
            float *cur = nullptr;
            const char *name = "";

            switch (which) {
            case 'P': pid = PID_KP;  cur = &curKP;  name = "kP";  break;
            case 'I': pid = PID_KI;  cur = &curKI;  name = "kI";  break;
            case 'D': pid = PID_KD;  cur = &curKD;  name = "kD";  break;
            case 'F': pid = PID_KFF; cur = &curKFF; name = "kFF"; break;
            }
            if (cur) {
                *cur = val;
                setParam(LEFT_ID, pid, val);  delay(5);
                setParam(RIGHT_ID, pid, val);
                Serial.printf("%s = %.8f\n", name, val);
            }

        } else if (cmd == 'B') {
            burnFlash(LEFT_ID);  delay(50);
            burnFlash(RIGHT_ID);
            Serial.println("PID burned to flash");

        } else {
            char *lp = strchr(buf, 'L'); if (!lp) lp = strchr(buf, 'l');
            char *rp = strchr(buf, 'R'); if (!rp) rp = strchr(buf, 'r');
            if (lp || rp) {
                directMode = true;
                if (lp) leftDirect  = atof(lp + 1);
                if (rp) rightDirect = atof(rp + 1);
                Serial.printf("L=%.0f R=%.0f RPM (direct)\n",
                              leftDirect, rightDirect);
            }
        }
        i = 0;
    }
}

// =====================================================================
// setup
// =====================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println("\n== SparkMAX Velocity PID (FW 26.1.4) ==");
    Serial.println("V<rpm> [W<rpm>] | L<rpm> R<rpm> | S | D");
    Serial.println("KP/KI/KD/KF<val> | BURN\n");

    can.begin();
    can.setBaudRate(1000000);
    can.setMaxMB(16);
    can.enableFIFO();
    can.enableFIFOInterrupt();
    can.onReceive(onRx);
    can.setFIFOFilter(ACCEPT_ALL);

    // Wake up SparkMAXes before configuring
    delay(100);
    sendHeartbeats();
    delay(50);

    // Push default PID gains to both motors
    Serial.println("[PID] Configuring...");
    configurePID(LEFT_ID,  DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_KFF);
    configurePID(RIGHT_ID, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_KFF);
    Serial.printf("  P=%.6f I=%.7f D=%.6f FF=%.6f\n",
                  DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_KFF);
    Serial.println("[PID] Ready. Tune with KP/KI/KD/KF, save with BURN.\n");
}

// =====================================================================
// loop
// =====================================================================
void loop() {
    can.events();
    processSerial();

    uint32_t now = millis();

    // --- 50 Hz control loop ---
    if (now - tCtrl >= CTRL_DT_MS) {
        tCtrl = now;
        sendHeartbeats();

        float lRPM, rRPM;

        if (directMode) {
            lRPM = leftDirect;
            rRPM = rightDirect;
        } else {
            lRPM = vCmd - wCmd;
            rRPM = vCmd + wCmd;
        }

        setVelocity(LEFT_ID, lRPM);
        setVelocity(RIGHT_ID, rRPM);
    }

    // --- Request encoder feedback until both respond ---
    if ((!gotLeft || !gotRight) && now - tEncCfg >= ENCODER_CFG_MS) {
        tEncCfg = now;
        enableEncoder(LEFT_ID);
        enableEncoder(RIGHT_ID);
    }

    // --- Status print ---
    if (now - tPrint >= PRINT_MS) {
        tPrint = now;
        float lT = directMode ? leftDirect : (vCmd - wCmd);
        float rT = directMode ? rightDirect : (vCmd + wCmd);
        Serial.printf("L: %6.0f/%6.0f  R: %6.0f/%6.0f rpm\n",
                       leftVel, lT, rightVel, rT);
    }
}
