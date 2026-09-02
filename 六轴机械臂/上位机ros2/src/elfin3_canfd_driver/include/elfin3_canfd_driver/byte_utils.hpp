#pragma once

#include <cstdint>

namespace elfin3_canfd
{

inline std::uint16_t read_u16_le(const std::uint8_t * data)
{
  return static_cast<std::uint16_t>(data[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8U);
}

inline std::int16_t read_i16_le(const std::uint8_t * data)
{
  const auto raw = read_u16_le(data);
  if (raw <= 0x7fffU) {
    return static_cast<std::int16_t>(raw);
  }
  return static_cast<std::int16_t>(static_cast<std::int32_t>(raw) - 0x10000);
}

inline std::uint32_t read_u32_le(const std::uint8_t * data)
{
  return static_cast<std::uint32_t>(data[0]) |
         (static_cast<std::uint32_t>(data[1]) << 8U) |
         (static_cast<std::uint32_t>(data[2]) << 16U) |
         (static_cast<std::uint32_t>(data[3]) << 24U);
}

inline std::int32_t read_i32_le(const std::uint8_t * data)
{
  const auto raw = read_u32_le(data);
  if (raw <= 0x7fffffffU) {
    return static_cast<std::int32_t>(raw);
  }
  return static_cast<std::int32_t>(static_cast<std::int64_t>(raw) - 0x100000000LL);
}

inline void write_u16_le(std::uint8_t * data, const std::uint16_t value)
{
  data[0] = static_cast<std::uint8_t>(value & 0xffU);
  data[1] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
}

inline void write_i16_le(std::uint8_t * data, const std::int16_t value)
{
  write_u16_le(data, static_cast<std::uint16_t>(value));
}

inline void write_u32_le(std::uint8_t * data, const std::uint32_t value)
{
  data[0] = static_cast<std::uint8_t>(value & 0xffU);
  data[1] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
  data[2] = static_cast<std::uint8_t>((value >> 16U) & 0xffU);
  data[3] = static_cast<std::uint8_t>((value >> 24U) & 0xffU);
}

inline void write_i32_le(std::uint8_t * data, const std::int32_t value)
{
  write_u32_le(data, static_cast<std::uint32_t>(value));
}

}  // namespace elfin3_canfd
