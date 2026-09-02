#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "elfin3_canfd_driver/byte_utils.hpp"
#include "elfin3_canfd_driver/crc16.hpp"
#include "elfin3_canfd_driver/protocol.hpp"

namespace elfin3_canfd
{

using CspCommandFrame = std::array<std::uint8_t, kCspCommandSize>;
using MotionControlFrame = std::array<std::uint8_t, kMotionControlSize>;
using ParameterRequestFrame = std::array<std::uint8_t, kParameterRequestSize>;
using HostHeartbeatFrame = std::array<std::uint8_t, kHostHeartbeatSize>;

inline bool valid_axes_mask(const std::uint8_t mask)
{
  return mask != 0U && (mask & static_cast<std::uint8_t>(~kAllAxesMask)) == 0U;
}

inline bool encode_csp_command(const CspCommand & command, CspCommandFrame & frame)
{
  if (command.valid_axes_mask != kAllAxesMask || command.validity_ms != kCspValidityMs) {
    return false;
  }
  if (command.flags != kResetApply && command.flags != kApply && command.flags != kHold) {
    return false;
  }

  frame.fill(0U);
  frame[0] = command.sequence;
  frame[1] = command.valid_axes_mask;
  frame[2] = command.flags;
  frame[3] = kCspMode;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    write_i32_le(frame.data() + 4U + axis * 4U, command.target_position_counts[axis]);
  }
  write_u16_le(frame.data() + 28U, command.validity_ms);
  return append_crc(frame.data(), frame.size());
}

inline bool encode_motion_control(
  const MotionControlCommand & command, MotionControlFrame & frame)
{
  if (!valid_axes_mask(command.axes_mask) || command.flags != 0U ||
    static_cast<std::uint8_t>(command.command) >
    static_cast<std::uint8_t>(MotionControl::kFaultReset))
  {
    return false;
  }

  frame.fill(0U);
  frame[0] = command.sequence;
  frame[1] = static_cast<std::uint8_t>(command.command);
  frame[2] = command.axes_mask;
  frame[3] = command.flags;
  write_u32_le(frame.data() + 4U, command.request_token);
  write_u32_le(frame.data() + 8U, command.host_time_ms);
  return append_crc(frame.data(), frame.size());
}

inline bool encode_parameter_request(
  const ParameterRequest & request, ParameterRequestFrame & frame)
{
  const auto operation = static_cast<std::uint8_t>(request.operation);
  const bool valid_axis = request.target_axis < kAxisCount || request.target_axis == 0xffU;
  if (operation > static_cast<std::uint8_t>(ParameterOperation::kWrite) ||
    !valid_axis || request.flags != 0U)
  {
    return false;
  }

  frame.fill(0U);
  frame[0] = request.sequence;
  frame[1] = operation;
  frame[2] = request.target_axis;
  frame[3] = request.flags;
  write_u16_le(frame.data() + 4U, request.parameter_id);
  write_i32_le(frame.data() + 8U, request.value);
  write_u16_le(frame.data() + 12U, request.request_token);
  return append_crc(frame.data(), frame.size());
}

inline bool encode_host_heartbeat(
  const HostHeartbeat & heartbeat, HostHeartbeatFrame & frame)
{
  if (static_cast<std::uint8_t>(heartbeat.state) >
    static_cast<std::uint8_t>(HostState::kShuttingDown) ||
    (heartbeat.flags & static_cast<std::uint8_t>(~kHostFlagMask)) != 0U)
  {
    return false;
  }

  frame.fill(0U);
  frame[0] = heartbeat.sequence;
  frame[1] = kProtocolVersion;
  frame[2] = static_cast<std::uint8_t>(heartbeat.state);
  frame[3] = heartbeat.flags;
  write_u16_le(frame.data() + 4U, heartbeat.uptime_100ms);
  return append_crc(frame.data(), frame.size());
}

}  // namespace elfin3_canfd
