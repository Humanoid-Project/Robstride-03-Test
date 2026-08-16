#ifndef N100_CRC_HPP_
#define N100_CRC_HPP_

#include <cstddef>
#include <cstdint>

namespace n100 {

// Table driven CRC8 used for the 4 byte frame header.
std::uint8_t crc8(const std::uint8_t* data, std::size_t length);

// CCITT style CRC16 used for the frame payload.
std::uint16_t crc16(const std::uint8_t* data, std::size_t length);

}  // namespace n100

#endif  // N100_CRC_HPP_
