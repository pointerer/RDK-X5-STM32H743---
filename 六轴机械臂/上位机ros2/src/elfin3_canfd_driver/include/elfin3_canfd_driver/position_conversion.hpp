#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include "elfin3_canfd_driver/protocol.hpp"

namespace elfin3_canfd
{

constexpr long double kTwoPi = 6.283185307179586476925286766559L;
constexpr long double kCountsPerRadian =
  static_cast<long double>(kJointCountsPerRevolution) / kTwoPi;
constexpr long double kRadiansPerCount =
  kTwoPi / static_cast<long double>(kJointCountsPerRevolution);

inline bool valid_direction_signs(
  const std::array<std::int8_t, kAxisCount> & direction_signs)
{
  for (const auto sign : direction_signs) {
    if (sign != -1 && sign != 1) {
      return false;
    }
  }
  return true;
}

inline bool protocol_counts_to_joint_radians(
  const std::array<std::int32_t, kAxisCount> & protocol_counts,
  const std::array<std::int32_t, kAxisCount> & joint_zero_offset_counts,
  const std::array<std::int8_t, kAxisCount> & joint_direction_signs,
  std::array<double, kAxisCount> & joint_positions_rad)
{
  if (!valid_direction_signs(joint_direction_signs)) {
    return false;
  }

  std::array<double, kAxisCount> converted{};
  for (std::size_t joint = 0; joint < kAxisCount; ++joint) {
    const auto delta = static_cast<std::int64_t>(protocol_counts[joint]) -
      static_cast<std::int64_t>(joint_zero_offset_counts[joint]);
    converted[joint] = static_cast<double>(
      static_cast<long double>(joint_direction_signs[joint]) *
      static_cast<long double>(delta) * kRadiansPerCount);
  }
  joint_positions_rad = converted;
  return true;
}

inline bool joint_radians_to_protocol_counts(
  const std::array<double, kAxisCount> & joint_positions_rad,
  const std::array<std::int32_t, kAxisCount> & joint_zero_offset_counts,
  const std::array<std::int8_t, kAxisCount> & joint_direction_signs,
  std::array<std::int32_t, kAxisCount> & protocol_counts)
{
  if (!valid_direction_signs(joint_direction_signs)) {
    return false;
  }

  std::array<std::int32_t, kAxisCount> converted{};
  for (std::size_t joint = 0; joint < kAxisCount; ++joint) {
    if (!std::isfinite(joint_positions_rad[joint])) {
      return false;
    }
    const long double raw_count =
      static_cast<long double>(joint_zero_offset_counts[joint]) +
      static_cast<long double>(joint_direction_signs[joint]) *
      static_cast<long double>(joint_positions_rad[joint]) * kCountsPerRadian;
    if (!std::isfinite(raw_count) ||
      raw_count < static_cast<long double>(std::numeric_limits<std::int32_t>::min()) ||
      raw_count > static_cast<long double>(std::numeric_limits<std::int32_t>::max()))
    {
      return false;
    }
    const auto rounded_count = static_cast<std::int32_t>(std::llround(raw_count));
    converted[joint] = rounded_count;
  }
  protocol_counts = converted;
  return true;
}

}  // namespace elfin3_canfd
