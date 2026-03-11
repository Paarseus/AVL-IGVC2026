#pragma once

#include <FlexCAN_T4.h>
#include <cstring>

// ─── REV SparkMAX CAN ID bit‑fields (29‑bit extended) ───────────────────────
//  Bits [28:24]  Device Type        = 2 (motor controller)
//  Bits [23:16]  Manufacturer Code  = 5 (REV Robotics)
//  Bits [15:10]  API Class
//  Bits  [9:6]   API Index
//  Bits  [5:0]   Device ID (1‑62, 0 is broadcast)
// ─────────────────────────────────────────────────────────────────────────────

constexpr uint8_t  SPARK_DEVICE_TYPE      = 2;
constexpr uint8_t  SPARK_MANUFACTURER     = 5;

// API Class / Index pairs
constexpr uint8_t  API_PERCENT_OUTPUT_CLASS  = 0;
constexpr uint8_t  API_PERCENT_OUTPUT_INDEX  = 2;

constexpr uint8_t  API_VELOCITY_CLASS        = 1;
constexpr uint8_t  API_VELOCITY_INDEX        = 2;

constexpr uint8_t  API_POSITION_CLASS        = 3;
constexpr uint8_t  API_POSITION_INDEX        = 2;

constexpr uint8_t  API_ENCODER_CLASS         = 6;
constexpr uint8_t  API_ENCODER_INDEX         = 2;

constexpr uint8_t  API_DRIVE_ENC_CLASS       = 6;
constexpr uint8_t  API_DRIVE_ENC_INDEX       = 33;

constexpr uint8_t  API_ABS_ENC_CLASS         = 6;
constexpr uint8_t  API_ABS_ENC_INDEX         = 37;

constexpr uint8_t  API_HEARTBEAT_CLASS       = 11;
constexpr uint8_t  API_HEARTBEAT_INDEX       = 2;

constexpr uint8_t  API_PARAM_CLASS           = 48;
constexpr uint8_t  API_PARAM_INDEX           = 0;

// ─────────────────────────────────────────────────────────────────────────────
// Build a 29‑bit Extended CAN ID for a SparkMAX command / status frame
// ─────────────────────────────────────────────────────────────────────────────
inline uint32_t sparkCanId(uint8_t api_class, uint8_t api_index, uint8_t device_id) {
    return ((uint32_t)SPARK_DEVICE_TYPE  << 24) |
           ((uint32_t)SPARK_MANUFACTURER << 16) |
           ((uint32_t)(api_class & 0x3F) << 10) |
           ((uint32_t)(api_index & 0x0F) <<  6) |
           ((uint32_t)(device_id & 0x3F));
}

// ─────────────────────────────────────────────────────────────────────────────
// Decode a received 29‑bit ID back into its fields
// ─────────────────────────────────────────────────────────────────────────────
inline void parseSparkCanId(uint32_t id, uint8_t &api_class, uint8_t &api_index, uint8_t &device_id) {
    device_id = id & 0x3F;
    api_index = (id >> 6)  & 0x0F;
    api_class = (id >> 10) & 0x3F;
}

// ─────────────────────────────────────────────────────────────────────────────
// SparkMaxCAN  —  thin driver for one REV SparkMAX over CAN
// ─────────────────────────────────────────────────────────────────────────────
class SparkMaxCAN {
public:
    SparkMaxCAN() : _id(0), _bus(nullptr), _reversed(false) {}

    // Call init() after the FlexCAN bus has been started
    void init(uint8_t device_id, FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> *bus, bool reversed = false) {
        _id       = device_id;
        _bus      = bus;
        _reversed = reversed;

        // Request periodic encoder feedback at 50 ms
        uint16_t period_ms = 50;
        uint8_t data[8] = { (uint8_t)(period_ms & 0xFF), (uint8_t)(period_ms >> 8), 0,0,0,0,0,0 };
        sendMsg(API_ENCODER_INDEX, API_ENCODER_CLASS, 2, data);       // velocity / position
        sendMsg(3, API_ENCODER_CLASS, 2, data);                       // position feedback
        sendMsg(5, API_ENCODER_CLASS, 2, data);                       // absolute encoder
    }

    // ── Commands ─────────────────────────────────────────────────────────
    bool setPercentOutput(float pct) {
        // Clamp to [-1.0 , +1.0]
        if (pct >  1.0f) pct =  1.0f;
        if (pct < -1.0f) pct = -1.0f;
        if (_reversed) pct = -pct;
        uint8_t buf[8] = {0};
        floatToBytes(pct, buf);
        return sendMsg(API_PERCENT_OUTPUT_INDEX, API_PERCENT_OUTPUT_CLASS, 8, buf);
    }

    bool setVelocity(float rpm) {
        if (_reversed) rpm = -rpm;
        uint8_t buf[8] = {0};
        floatToBytes(rpm, buf);
        return sendMsg(API_VELOCITY_INDEX, API_VELOCITY_CLASS, 8, buf);
    }

    bool setPosition(float rotations) {
        if (_reversed) rotations = -rotations;
        uint8_t buf[8] = {0};
        floatToBytes(rotations, buf);
        return sendMsg(API_POSITION_INDEX, API_POSITION_CLASS, 8, buf);
    }

    // ── Heartbeat (must be sent periodically or SparkMAX will disable) ──
    bool sendHeartbeat() {
        uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00};
        // Heartbeat is broadcast (device_id = 0 in the CAN ID)
        return sendMsgRaw(API_HEARTBEAT_INDEX, API_HEARTBEAT_CLASS, 0, 8, data);
    }

    // ── Process an incoming CAN frame (call from the RX callback) ───────
    void handleFeedback(uint8_t api_class, uint8_t api_index, const uint8_t data[8]) {
        float f = readFloatLE(data);
        if (api_class == API_ABS_ENC_CLASS && api_index == 5) {
            _absolutePos = f;
        } else if (api_class == API_DRIVE_ENC_CLASS && api_index == 2) {
            _drivePos = f;
        }
    }

    // ── Getters ──────────────────────────────────────────────────────────
    float  absolutePosition() const { return _absolutePos; }
    float  drivePosition()    const { return _drivePos; }
    uint8_t id()              const { return _id; }

private:
    uint8_t  _id;
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> *_bus;
    bool     _reversed;
    volatile float _absolutePos = 0.0f;
    volatile float _drivePos    = 0.0f;

    bool sendMsg(uint8_t api_index, uint8_t api_class, uint8_t dlc, uint8_t data[8]) {
        return sendMsgRaw(api_index, api_class, _id, dlc, data);
    }

    bool sendMsgRaw(uint8_t api_index, uint8_t api_class, uint8_t dev_id, uint8_t dlc, uint8_t data[8]) {
        if (!_bus) return false;
        CAN_message_t msg;
        msg.flags.extended = 1;
        msg.id  = sparkCanId(api_class, api_index, dev_id);
        msg.len = dlc;
        memcpy(msg.buf, data, dlc);
        return _bus->write(msg) > 0;
    }

    static void floatToBytes(float f, uint8_t *out) {
        memcpy(out, &f, sizeof(float));
    }

    static float readFloatLE(const uint8_t *data) {
        float v;
        memcpy(&v, data, sizeof(float));
        return v;
    }
};
