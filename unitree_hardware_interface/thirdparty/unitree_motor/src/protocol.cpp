#include "unitree_motor/protocol.hpp"
#include "unitree_motor/crc.hpp"

#include <algorithm>
#include <cstring>

namespace unitree_motor {

// ── Helpers ──────────────────────────────────────────────────────────

static inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

static inline void write_le16(uint8_t* buf, uint16_t val) {
    buf[0] = val & 0xFF;
    buf[1] = (val >> 8) & 0xFF;
}

static inline void write_le32(uint8_t* buf, uint32_t val) {
    buf[0] = val & 0xFF;
    buf[1] = (val >> 8) & 0xFF;
    buf[2] = (val >> 16) & 0xFF;
    buf[3] = (val >> 24) & 0xFF;
}

static inline uint16_t read_le16(const uint8_t* buf) {
    return buf[0] | (static_cast<uint16_t>(buf[1]) << 8);
}

static inline uint32_t read_le32(const uint8_t* buf) {
    return buf[0]
         | (static_cast<uint32_t>(buf[1]) << 8)
         | (static_cast<uint32_t>(buf[2]) << 16)
         | (static_cast<uint32_t>(buf[3]) << 24);
}

// ── pack_command ────────────────────────────────────────────────────

void pack_command(const MotorCommand& cmd, uint8_t* buf) {
    double torque   = clamp(cmd.torque,   -127.99,  127.99);
    double speed    = clamp(cmd.speed,    -804.0,   804.0);
    double position = clamp(cmd.position, -411774.0, 411774.0);
    double kp       = clamp(cmd.kp,        0.0,     25.599);
    double kd       = clamp(cmd.kd,        0.0,     25.599);

    // Cast to int16/int32 (saturating — STM32 SATURATE equivalent)
    auto sat_int16 = [](double v) -> int16_t {
        int64_t iv = static_cast<int64_t>(v);
        if (iv > 32767)  return 32767;
        if (iv < -32768) return -32768;
        return static_cast<int16_t>(iv);
    };
    auto sat_int32 = [](double v) -> int32_t {
        int64_t iv = static_cast<int64_t>(v);
        if (iv > 2147483647)  return 2147483647;
        if (iv < -2147483647 - 1) return -2147483647 - 1;
        return static_cast<int32_t>(iv);
    };

    int16_t tor_raw = sat_int16(torque   * TORQUE_INV_SCALE);
    int16_t spd_raw = sat_int16(speed    * SPD_INV_SCALE);
    int32_t pos_raw = sat_int32(position * POS_INV_SCALE);
    int16_t kp_raw  = sat_int16(kp       * KP_INV_SCALE);
    int16_t kd_raw  = sat_int16(kd       * KSPD_INV_SCALE);

    // Mode byte: id[3:0] | status[6:4] | reserved[7]
    uint8_t mode_byte = (cmd.motor_id & 0x0F) | ((cmd.mode & 0x07) << 4);

    // Pack body (15 bytes before CRC)
    buf[0] = SEND_HEAD0;
    buf[1] = SEND_HEAD1;
    buf[2] = mode_byte;
    write_le16(buf + 3,  static_cast<uint16_t>(tor_raw));
    write_le16(buf + 5,  static_cast<uint16_t>(spd_raw));
    write_le32(buf + 7,  static_cast<uint32_t>(pos_raw));
    write_le16(buf + 11, static_cast<uint16_t>(kp_raw));
    write_le16(buf + 13, static_cast<uint16_t>(kd_raw));

    uint16_t crc = crc_ccitt(buf, 15);
    write_le16(buf + 15, crc);
}

// ── unpack_response ─────────────────────────────────────────────────

MotorState unpack_response(const uint8_t* data) {
    MotorState state;

    // Check head
    if (data[0] != RECV_HEAD0 || data[1] != RECV_HEAD1)
        return state;  // valid=false

    // Check CRC over first 14 bytes
    uint16_t expected_crc = read_le16(data + 14);
    if (crc_ccitt(data, 14) != expected_crc)
        return state;  // valid=false

    uint8_t mode_byte = data[2];

    // torque (int16 at offset 3), speed (int16 at offset 5)
    int16_t torque_raw = static_cast<int16_t>(read_le16(data + 3));
    int16_t speed_raw  = static_cast<int16_t>(read_le16(data + 5));
    // position (int32 at offset 7)
    int32_t pos_raw    = static_cast<int32_t>(read_le32(data + 7));
    // temperature (int8 at offset 11)
    int8_t  temp_raw   = static_cast<int8_t>(data[11]);
    // status (uint16 at offset 12)
    uint16_t status_raw = read_le16(data + 12);

    state.motor_id    = mode_byte & 0x0F;
    state.mode        = (mode_byte >> 4) & 0x07;
    state.torque      = torque_raw * TORQUE_SCALE;
    state.speed       = speed_raw  * SPD_SCALE;
    state.position    = pos_raw    * POS_SCALE;
    state.temperature = temp_raw;
    // Bits 3-14 of status_raw are reserved
    state.error       = status_raw & 0x07;
    state.valid       = true;

    return state;
}

} // namespace unitree_motor
