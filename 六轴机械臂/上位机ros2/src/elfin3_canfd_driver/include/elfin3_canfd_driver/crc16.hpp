#pragma once

#include <cstddef>
#include <cstdint>

#include "elfin3_canfd_driver/byte_utils.hpp"

namespace elfin3_canfd
{

inline std::uint16_t crc16_ccitt_false(
  const std::uint8_t * data, const std::size_t size)
{
  std::uint16_t crc = 0xffffU;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= static_cast<std::uint16_t>(data[index]) << 8U;
    for (std::uint8_t bit = 0; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) != 0U ?
        static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U) :
        static_cast<std::uint16_t>(crc << 1U);
    }
  }
  return crc;
}

inline bool append_crc(std::uint8_t * frame, const std::size_t size)
{
  if (size < 2U) {
    return false;
  }
  write_u16_le(frame + size - 2U, crc16_ccitt_false(frame, size - 2U));
  return true;
}

inline bool verify_crc(const std::uint8_t * frame, const std::size_t size)
{
  if (size < 2U) {
    return false;
  }
  return read_u16_le(frame + size - 2U) == crc16_ccitt_false(frame, size - 2U);
}

}  // namespace elfin3_canfd
