// ============================================================================
// Teensy 4.1 — AVROS diff-drive motor bridge (SparkMAX FW 26.1.4)
// ============================================================================
// Role: USB-Serial <-> CAN bridge. The Jetson (ROS2 actuator_node) owns
// diff-drive kinematics and streams per-wheel RPM setpoints; this firmware
// forwards them to each SparkMAX's built-in velocity PID and echoes encoder
// feedback back over serial.
//
// Hardware:
//   Teensy 4.1  (CAN1: CTX1=pin22, CRX1=pin23)
//   SN65HVD230 or TJA1051T/3 transceiver, 120 ohm termination at each end
//   REV SparkMAX x 2 -- CAN ID 1 = left, ID 2 = right
//
// Host -> Teensy (115200 baud, newline-terminated):
//   L<rpm> R<rpm>     VELOCITY mode — wheel RPM setpoints (closed-loop PID)
//   UL<d> UR<d>       DUTY mode     — open-loop duty cycle, -1.0..1.0
//   S                 stop both wheels (resets to velocity mode)
//   D                 print one DIAG line
//   KP/KI/KD/KF<val>  tune SparkMAX PID slot 0 (note: param-write currently
//                     broken per REV-Specs; fix via REV Hardware Client for now)
//   BURN              persist current PID gains to SparkMAX flash
//
// Teensy -> Host:
//   E L<rpm> <pos> R<rpm> <pos>   50 Hz wheel feedback (RPM + rotations)
//   OK  ...                       command acknowledgement
//   ERR ...                       parse failure
//   DIAG ...                      response to 'D'
//   # ...                         informational log line
//
// Safety:
//   300 ms host watchdog -- if no L/R/S arrives in that window, both wheels
//   are forced to 0 RPM. SparkMAX 100 ms heartbeat timeout is a second layer.
//   MAX_RPM clamp (3000, well below NEO free speed of 5676).
// ============================================================================

#include <FlexCAN_T4.h>
#include <string.h>
#include <ctype.h>

// ---------- Configuration ----------------------------------------------------
static constexpr uint8_t  LEFT_ID        = 1;
static constexpr uint8_t  RIGHT_ID       = 2;
static constexpr uint32_t CAN_BAUD       = 1000000;
static constexpr uint32_t CTRL_DT_MS     = 20;    // 50 Hz control loop
static constexpr uint32_t FEEDBACK_DT_MS = 20;    // 50 Hz E-line to host
static constexpr uint32_t ENC_CFG_DT_MS  = 1000;  // re-kick encoder enable
static constexpr uint32_t WATCHDOG_MS    = 300;
static constexpr float    MAX_RPM        = 3000.0f;

// ---------- SparkMAX CAN protocol constants ----------------------------------
static constexpr uint8_t  SPARK_DEV_TYPE = 2;
static constexpr uint8_t  SPARK_MFG      = 5;
static constexpr uint8_t  CLS_VELOCITY   = 0;   // velocity setpoint per REV-Specs 2.1.0 (FW 25+)
static constexpr uint8_t  IDX_VELOCITY   = 0;
static constexpr uint8_t  CLS_DUTY       = 0;   // duty-cycle setpoint
static constexpr uint8_t  IDX_DUTY       = 2;
static constexpr float    MAX_DUTY       = 0.30f;  // safety cap (open-loop, bench)
static constexpr uint8_t  CLS_STATUS_CFG = 1;   // SET_STATUSES_ENABLED
static constexpr uint8_t  IDX_STATUS_CFG = 0;
static constexpr uint8_t  CLS_STATUS     = 46;  // periodic status frames
static constexpr uint8_t  IDX_STATUS_2   = 2;
static constexpr uint8_t  CLS_HB         = 11;  // REV secondary heartbeat
static constexpr uint8_t  IDX_HB         = 2;
// PARAMETER_WRITE and PERSIST_PARAMETERS per REV-Specs spark-frames-2.1.0.
// The old firmware used cls=48 for both, which does not exist in the spec —
// all previous KP/KI/KD/KF commands and BURN commands were silently discarded.
static constexpr uint8_t  CLS_PARAM      = 14;  // PARAMETER_WRITE
static constexpr uint8_t  IDX_PARAM_SET  = 0;
static constexpr uint8_t  CLS_BURN       = 63;  // PERSIST_PARAMETERS
static constexpr uint8_t  IDX_BURN       = 15;
static constexpr uint32_t UNIVERSAL_HB   = 0x01011840;  // roboRIO heartbeat

// SparkMAX PID slot-0 parameter IDs — canonical values from REV's
// SPARK-MAX-Types.proto (REVrobotics/SPARK-MAX-Server). Note: kF_0 is ID 16,
// NOT 17. ID 17 is kIZone_0 (integrator zone) — the upstream _synced.ino
// firmware wrote kFF to 17 by mistake, silently disabling the integrator
// instead of tuning feedforward. Fixed here.
static constexpr uint8_t  PID_KP      = 13;
static constexpr uint8_t  PID_KI      = 14;
static constexpr uint8_t  PID_KD      = 15;
static constexpr uint8_t  PID_KFF     = 16;

// ---------- State ------------------------------------------------------------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;

struct Wheel {
    float          cmd_rpm  = 0.0f;
    float          cmd_duty = 0.0f;
    volatile float meas_rpm = 0.0f;
    volatile float meas_pos = 0.0f;
    volatile bool  got_enc  = false;
};
static Wheel left, right;

enum ControlMode { MODE_VELOCITY, MODE_DUTY };
static ControlMode ctrl_mode = MODE_VELOCITY;

static uint32_t t_ctrl = 0, t_fb = 0, t_enc_cfg = 0, t_last_host = 0;
static uint32_t tx_count = 0, rx_count = 0;
static bool     wdt_tripped = false;
static volatile float bus_voltage = 0.0f;  // decoded from STATUS_0 (both devs report the same bus)

// ---------- CAN helpers ------------------------------------------------------
static inline uint32_t sparkId(uint8_t cls, uint8_t idx, uint8_t dev) {
    return ((uint32_t)SPARK_DEV_TYPE << 24)
         | ((uint32_t)SPARK_MFG      << 16)
         | ((uint32_t)(cls & 0x3F)   << 10)
         | ((uint32_t)(idx & 0x0F)   <<  6)
         | ((uint32_t)(dev & 0x3F));
}

static void canSend(uint32_t id, const uint8_t *data, uint8_t len) {
    CAN_message_t m;
    m.flags.extended = 1;
    m.id  = id;
    m.len = len;
    memcpy(m.buf, data, len);
    if (can.write(m) > 0) tx_count++;
}

static inline void fromFloat(float f, uint8_t *b) { memcpy(b, &f, sizeof(float)); }
static inline float toFloat(const uint8_t *b)     { float v; memcpy(&v, b, sizeof(float)); return v; }

// ---------- SparkMAX commands ------------------------------------------------
static void sendHeartbeats() {
    // roboRIO Universal (required by SparkMAX fw 25+): byte 3 = 0x12 (enabled + sysWdt)
    static const uint8_t uni[8] = {0x78, 0x01, 0x00, 0x12, 0x59, 0x04, 0x00, 0x60};
    canSend(UNIVERSAL_HB, uni, 8);
    // REV Secondary (enables all devices as a fallback)
    static const uint8_t sec[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    canSend(sparkId(CLS_HB, IDX_HB, 0), sec, 8);
}

static void setVelocity(uint8_t dev, float rpm) {
    if (rpm >  MAX_RPM) rpm =  MAX_RPM;
    if (rpm < -MAX_RPM) rpm = -MAX_RPM;
    uint8_t d[8] = {0};
    fromFloat(rpm, d);
    canSend(sparkId(CLS_VELOCITY, IDX_VELOCITY, dev), d, 8);
}

static void setDuty(uint8_t dev, float duty) {
    if (duty >  MAX_DUTY) duty =  MAX_DUTY;
    if (duty < -MAX_DUTY) duty = -MAX_DUTY;
    uint8_t d[8] = {0};
    fromFloat(duty, d);
    canSend(sparkId(CLS_DUTY, IDX_DUTY, dev), d, 8);
}

static void enableStatus2(uint8_t dev) {
    // mask=0x0004 / enable=0x0004 -> turn on STATUS_2 (encoder)
    uint8_t d[8] = {0};
    d[0] = 0x04;
    d[2] = 0x04;
    canSend(sparkId(CLS_STATUS_CFG, IDX_STATUS_CFG, dev), d, 8);
}

// PARAMETER_WRITE payload (REV-Specs 2.1.0, spark-frames spec):
// byte[0] = param_id, bytes[1:4] = value float32 LE. DLC = 5.
static void setParam(uint8_t dev, uint8_t param_id, float value) {
    uint8_t d[5] = {0};
    d[0] = param_id;
    fromFloat(value, d + 1);
    canSend(sparkId(CLS_PARAM, IDX_PARAM_SET, dev), d, 5);
}

// PERSIST_PARAMETERS magic: 15011 = 0x3AA3 as uint16 LE at bytes[0:1].
static void burnFlash(uint8_t dev) {
    uint8_t d[8] = {0};
    d[0] = 0xA3;
    d[1] = 0x3A;
    canSend(sparkId(CLS_BURN, IDX_BURN, dev), d, 8);
}

static void tuneBoth(uint8_t param, float v) {
    setParam(LEFT_ID,  param, v); delay(5);
    setParam(RIGHT_ID, param, v);
}

// ---------- CAN RX -- decode STATUS_2 ---------------------------------------
static void onCanRx(const CAN_message_t &msg) {
    if (!msg.flags.extended || msg.len < 8) return;
    rx_count++;

    uint8_t dev = msg.id & 0x3F;
    uint8_t cls = (msg.id >> 10) & 0x3F;
    uint8_t idx = (msg.id >>  6) & 0x0F;

    // STATUS_0 (cls=46 idx=0): decode bus voltage. Per REV-Specs 2.1.0:
    // bits [27:16] = VOLTAGE uint12 LE, scale × 0.007326 → V.
    if (cls == CLS_STATUS && idx == 0 && msg.len >= 4) {
        uint16_t v_raw = msg.buf[2] | ((uint16_t)(msg.buf[3] & 0x0F) << 8);
        bus_voltage = v_raw * 0.007326f;
        return;
    }

    if (cls != CLS_STATUS || idx != IDX_STATUS_2) return;

    float vel = toFloat(msg.buf);
    float pos = toFloat(msg.buf + 4);
    if (dev == LEFT_ID) {
        left.meas_rpm = vel;  left.meas_pos = pos;  left.got_enc = true;
    } else if (dev == RIGHT_ID) {
        right.meas_rpm = vel; right.meas_pos = pos; right.got_enc = true;
    }
}

// ---------- Serial parser ---------------------------------------------------
static void handleLine(char *line) {
    if (!line[0]) return;
    char cmd = toupper((unsigned char)line[0]);
    uint32_t now = millis();

    switch (cmd) {
        case 'S':
            // Stay in duty mode with 0% duty so the SparkMAX sees "idle"
            // and its Brake idle-mode setting engages. Sending velocity=0
            // via MODE_VELOCITY keeps the velocity PID running, which
            // prevents the controller from entering idle state.
            left.cmd_rpm  = right.cmd_rpm  = 0.0f;
            left.cmd_duty = right.cmd_duty = 0.0f;
            ctrl_mode = MODE_DUTY;
            t_last_host = now;
            wdt_tripped = false;
            Serial.println("OK S");
            return;

        case 'D':
            Serial.printf("DIAG tx=%lu rx=%lu wdt=%d mode=%s L=%.0f/%.0f R=%.0f/%.0f duty L=%.3f R=%.3f V=%.2f\n",
                          tx_count, rx_count, wdt_tripped ? 1 : 0,
                          ctrl_mode == MODE_DUTY ? "DUTY" : "VEL",
                          left.meas_rpm, left.cmd_rpm,
                          right.meas_rpm, right.cmd_rpm,
                          left.cmd_duty, right.cmd_duty,
                          bus_voltage);
            return;

        case 'U': {
            // Duty-cycle command:  "UL0.05 UR-0.05" / "UL0.05" / "UR-0.05"
            char *lp = strchr(line + 1, 'L'); if (!lp) lp = strchr(line + 1, 'l');
            char *rp = strchr(line + 1, 'R'); if (!rp) rp = strchr(line + 1, 'r');
            if (!lp && !rp) { Serial.println("ERR U?"); return; }
            auto clamp = [](float v) {
                if (v >  MAX_DUTY) return  MAX_DUTY;
                if (v < -MAX_DUTY) return -MAX_DUTY;
                return v;
            };
            if (lp) left.cmd_duty  = clamp(atof(lp + 1));
            if (rp) right.cmd_duty = clamp(atof(rp + 1));
            ctrl_mode = MODE_DUTY;
            t_last_host = now;
            wdt_tripped = false;
            Serial.printf("OK UL=%.3f UR=%.3f\n", left.cmd_duty, right.cmd_duty);
            return;
        }

        case 'B':
            burnFlash(LEFT_ID);  delay(50);
            burnFlash(RIGHT_ID);
            Serial.println("OK BURN");
            return;

        case 'K': {
            if (!line[1]) { Serial.println("ERR K?"); return; }
            char which = toupper((unsigned char)line[1]);
            float val  = atof(line + 2);
            uint8_t p;
            switch (which) {
                case 'P': p = PID_KP;  break;
                case 'I': p = PID_KI;  break;
                case 'D': p = PID_KD;  break;
                case 'F': p = PID_KFF; break;
                default:  Serial.println("ERR K?"); return;
            }
            tuneBoth(p, val);
            Serial.printf("OK K%c=%.8f\n", which, val);
            return;
        }

        default: {
            // Velocity command: "L500 R-500" / "L500" / "R-500"
            char *lp = strchr(line, 'L'); if (!lp) lp = strchr(line, 'l');
            char *rp = strchr(line, 'R'); if (!rp) rp = strchr(line, 'r');
            if (!lp && !rp) { Serial.println("ERR unknown"); return; }
            auto clamp = [](float v) {
                if (v >  MAX_RPM) return  MAX_RPM;
                if (v < -MAX_RPM) return -MAX_RPM;
                return v;
            };
            if (lp) left.cmd_rpm  = clamp(atof(lp + 1));
            if (rp) right.cmd_rpm = clamp(atof(rp + 1));
            ctrl_mode = MODE_VELOCITY;
            t_last_host = now;
            wdt_tripped = false;
            Serial.printf("OK L=%.0f R=%.0f\n", left.cmd_rpm, right.cmd_rpm);
            return;
        }
    }
}

static void processSerial() {
    static char buf[96];
    static uint8_t n = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (n > 0) { buf[n] = '\0'; handleLine(buf); n = 0; }
        } else if (n < sizeof(buf) - 1) {
            buf[n++] = c;
        }
    }
}

// ---------- setup / loop ----------------------------------------------------
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    Serial.println("# avros diff-drive bridge ready");
    Serial.println("# proto: L<rpm> R<rpm> | UL<d> UR<d> | S | D | K[PIDF]<v> | BURN");

    can.begin();
    can.setBaudRate(CAN_BAUD);
    can.setMaxMB(16);
    can.enableFIFO();
    can.enableFIFOInterrupt();
    can.setFIFOFilter(ACCEPT_ALL);
    can.onReceive(onCanRx);

    // Wake SparkMAXes before commanding.
    delay(100);
    sendHeartbeats();
    delay(50);

    // NOTE: intentionally no configurePID() here -- SparkMAX flash is
    // authoritative. Tune interactively with KP/KI/KD/KF, then BURN once.

    t_last_host = millis();
}

void loop() {
    can.events();
    processSerial();

    uint32_t now = millis();

    // Note: no !Serial guard. The Teensy's bool(Serial) can flicker under
    // heavy CDC traffic, which would cause momentary heartbeat gaps and push
    // the SparkMAX into blinking-magenta (disabled) state. The host watchdog
    // below (300 ms) is sufficient to stop motors on USB disconnect.

    // 50 Hz control tick
    if (now - t_ctrl >= CTRL_DT_MS) {
        t_ctrl = now;

        if (now - t_last_host > WATCHDOG_MS) {
            if (!wdt_tripped) {
                Serial.println("# WDT host-timeout stop");
                wdt_tripped = true;
            }
            left.cmd_rpm  = right.cmd_rpm  = 0.0f;
            left.cmd_duty = right.cmd_duty = 0.0f;
        }

        sendHeartbeats();
        if (ctrl_mode == MODE_DUTY) {
            setDuty(LEFT_ID,  left.cmd_duty);
            setDuty(RIGHT_ID, right.cmd_duty);
        } else {
            setVelocity(LEFT_ID,  left.cmd_rpm);
            setVelocity(RIGHT_ID, right.cmd_rpm);
        }
    }

    // STATUS_2 keepalive — SparkMAX loses the "enabled" bit on power-cycle,
    // so we re-send the enable frame every ENC_CFG_DT_MS regardless of got_enc.
    // Cheap (one frame per device per tick) and robust to SparkMAX reboots.
    if (now - t_enc_cfg >= ENC_CFG_DT_MS) {
        t_enc_cfg = now;
        enableStatus2(LEFT_ID);
        enableStatus2(RIGHT_ID);
    }

    // 50 Hz wheel feedback to host
    if (now - t_fb >= FEEDBACK_DT_MS) {
        t_fb = now;
        if (Serial.availableForWrite() >= 64) {   // non-blocking guard
            Serial.printf("E L%.0f %.4f R%.0f %.4f\n",
                          left.meas_rpm, left.meas_pos,
                          right.meas_rpm, right.meas_pos);
        }
    }
}
