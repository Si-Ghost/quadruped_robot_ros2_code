#pragma once

#include <cstdint>
#include <cstddef>

namespace unitree_motor {

/// CRC-CCITT (XMODEM variant) — polynomial 0x1021
uint16_t crc_ccitt(const uint8_t* data, size_t len, uint16_t crc = 0);

} // namespace unitree_motor
