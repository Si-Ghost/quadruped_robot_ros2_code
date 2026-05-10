#pragma once

#include <cstdint>
#include <cstddef>

namespace unitree_motor {

// ── Packet sizes ────────────────────────────────────────────────────
constexpr size_t SEND_PACKET_SIZE = 17;
constexpr size_t RECV_PACKET_SIZE = 16;

// ── Head bytes ──────────────────────────────────────────────────────
constexpr uint8_t SEND_HEAD0 = 0xFE;
constexpr uint8_t SEND_HEAD1 = 0xEE;
constexpr uint8_t RECV_HEAD0 = 0xFD;
constexpr uint8_t RECV_HEAD1 = 0xEE;

// ── Broadcast motor ID ──────────────────────────────────────────────
constexpr uint8_t BROADCAST_ID = 0x0F;

// ── Motor modes ─────────────────────────────────────────────────────
constexpr uint8_t MODE_STOP      = 0;
constexpr uint8_t MODE_FOC       = 1;
constexpr uint8_t MODE_CALIBRATE = 2;

// ── Scaling constants ───────────────────────────────────────────────
constexpr double POS_SCALE      = 6.2832 / 32768.0;
constexpr double SPD_SCALE      = 6.2832 / 256.0;
constexpr double TORQUE_SCALE   = 1.0 / 256.0;
constexpr double KP_SCALE       = 25.6 / 32768.0;
constexpr double KSPD_SCALE     = 25.6 / 32768.0;
constexpr double POS_INV_SCALE  = 32768.0 / 6.2832;
constexpr double SPD_INV_SCALE  = 256.0 / 6.2832;
constexpr double TORQUE_INV_SCALE = 256.0;
constexpr double KP_INV_SCALE   = 32768.0 / 25.6;
constexpr double KSPD_INV_SCALE = 32768.0 / 25.6;

// ── Data structures ─────────────────────────────────────────────────

struct MotorCommand {
    uint8_t motor_id = 0;
    uint8_t mode     = MODE_FOC;
    double  torque   = 0.0;   // N.m,  [-127.99, 127.99]
    double  speed    = 0.0;   // rad/s, [-804.0, 804.0]
    double  position = 0.0;   // rad,   [-411774.0, 411774.0]
    double  kp       = 0.0;   // stiffness, [0, 25.599]
    double  kd       = 0.0;   // damping,   [0, 25.599]
};

struct MotorState {
    uint8_t motor_id    = 0;
    uint8_t mode        = 0;
    double  torque      = 0.0;  // N.m
    double  speed       = 0.0;  // rad/s
    double  position    = 0.0;  // rad
    int8_t  temperature = 0;    // Celsius
    uint8_t error       = 0;    // 0=ok, 1=overheat, 2=overcurrent, 3=overvoltage, 4=encoder
    bool    valid       = false;
};

// ── Pack / unpack ───────────────────────────────────────────────────

/// Pack a MotorCommand into a 17-byte send packet.
/// `buf` must be at least SEND_PACKET_SIZE bytes.
void pack_command(const MotorCommand& cmd, uint8_t* buf);

/// Unpack a 16-byte receive packet into a MotorState.
/// `data` must be at least RECV_PACKET_SIZE bytes.
MotorState unpack_response(const uint8_t* data);

} // namespace unitree_motor
