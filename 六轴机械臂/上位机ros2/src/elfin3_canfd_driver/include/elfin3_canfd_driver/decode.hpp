#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "elfin3_canfd_driver/byte_utils.hpp"
#include "elfin3_canfd_driver/crc16.hpp"
#include "elfin3_canfd_driver/protocol.hpp"

namespace elfin3_canfd
{

using PositionFeedbackFrame = std::array<std::uint8_t, kPositionFeedbackSize>;
using DetailedStatusFrame = std::array<std::uint8_t, kDetailedStatusSize>;
using DiagnosticEventFrame = std::array<std::uint8_t, kDiagnosticEventSize>;
using DeviceHeartbeatFrame = std::array<std::uint8_t, kDeviceHeartbeatSize>;
using TrajectoryStatusFrame = std::array<std::uint8_t, kTrajectoryStatusSize>;

inline bool decode_position_feedback(
  const PositionFeedbackFrame & frame, PositionFeedback & output)
{
  if (!verify_crc(frame.data(), frame.size()) ||
    (frame[1] & static_cast<std::uint8_t>(~kAllAxesMask)) != 0U ||
    (frame[3] & 0x80U) != 0U)
  {
    return false;
  }

  PositionFeedback decoded;
  decoded.sequence = frame[0];
  decoded.valid_axes_mask = frame[1];
  decoded.ethercat_master_state = frame[2];
  decoded.status_flags = frame[3];
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    decoded.actual_position_counts[axis] = read_i32_le(frame.data() + 4U + axis * 4U);
  }
  decoded.ethercat_cycle_low = read_u16_le(frame.data() + 28U);
  output = decoded;
  return true;
}

inline bool decode_detailed_status(
  const DetailedStatusFrame & frame, DetailedStatus & output)
{
  if (!verify_crc(frame.data(), frame.size()) ||
    (frame[1] & static_cast<std::uint8_t>(~kAllAxesMask)) != 0U)
  {
    return false;
  }

  DetailedStatus decoded;
  decoded.sequence = frame[0];
  decoded.valid_axes_mask = frame[1];
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    decoded.actual_velocity_raw[axis] = read_i32_le(frame.data() + 2U + axis * 4U);
    decoded.actual_torque_raw[axis] = read_i16_le(frame.data() + 26U + axis * 2U);
    decoded.statusword[axis] = read_u16_le(frame.data() + 38U + axis * 2U);
    decoded.drive_error_code[axis] = read_u16_le(frame.data() + 50U + axis * 2U);
  }
  output = decoded;
  return true;
}

inline bool decode_diagnostic_event(
  const DiagnosticEventFrame & frame, DiagnosticEvent & output)
{
  if (!verify_crc(frame.data(), frame.size()) || frame[1] > 3U ||
    !(frame[2] < kAxisCount || frame[2] == 0xffU))
  {
    return false;
  }

  DiagnosticEvent decoded;
  decoded.sequence = frame[0];
  decoded.severity = static_cast<DiagnosticSeverity>(frame[1]);
  decoded.source_axis = frame[2];
  decoded.flags = frame[3];
  decoded.primary_error_code = read_u16_le(frame.data() + 4U);
  decoded.detail_error_code = read_u16_le(frame.data() + 6U);
  decoded.context_value = read_u32_le(frame.data() + 8U);
  decoded.event_count = read_u16_le(frame.data() + 12U);
  output = decoded;
  return true;
}

inline bool decode_device_heartbeat(
  const DeviceHeartbeatFrame & frame, DeviceHeartbeat & output)
{
  if (!verify_crc(frame.data(), frame.size()) || frame[1] != kProtocolVersion ||
    (frame[3] & 0xf8U) != 0U)
  {
    return false;
  }

  DeviceHeartbeat decoded;
  decoded.sequence = frame[0];
  decoded.ethercat_master_state = frame[2];
  decoded.slave_op_mask = frame[3];
  decoded.working_counter = read_u16_le(frame.data() + 4U);
  output = decoded;
  return true;
}

inline bool decode_trajectory_status(
  const TrajectoryStatusFrame & frame, TrajectoryStatus & output)
{
  if (!verify_crc(frame.data(), frame.size()) || frame[1] != kProtocolVersion ||
    frame[2] > static_cast<std::uint8_t>(TrajectoryStatusState::kHold) ||
    frame[4] == 0U || frame[3] > frame[4] || frame[13] == 0U || frame[13] > frame[4] ||
    frame[11] > static_cast<std::uint8_t>(TrajectoryRejectReason::kInternalError) ||
    frame[12] > static_cast<std::uint8_t>(TrajectoryHoldReason::kInternalError))
  {
    return false;
  }

  TrajectoryStatus decoded;
  decoded.sequence = frame[0];
  decoded.state = static_cast<TrajectoryStatusState>(frame[2]);
  decoded.queue_depth = frame[3];
  decoded.queue_capacity = frame[4];
  decoded.flags = frame[5];
  decoded.last_received_sequence = frame[6];
  decoded.last_accepted_sequence = frame[7];
  decoded.last_executed_sequence = frame[8];
  decoded.expected_sequence = frame[9];
  decoded.last_rejected_sequence = frame[10];
  decoded.reject_reason = static_cast<TrajectoryRejectReason>(frame[11]);
  decoded.hold_reason = static_cast<TrajectoryHoldReason>(frame[12]);
  decoded.prefill_target = frame[13];
  decoded.generation = read_u16_le(frame.data() + 14U);
  decoded.last_executed_ecat_cycle = read_u16_le(frame.data() + 16U);
  decoded.accepted_count = read_u16_le(frame.data() + 18U);
  decoded.executed_count = read_u16_le(frame.data() + 20U);
  decoded.rejected_count = read_u16_le(frame.data() + 22U);
  decoded.underrun_count = read_u16_le(frame.data() + 24U);
  decoded.overflow_count = read_u16_le(frame.data() + 26U);
  decoded.expired_count = read_u16_le(frame.data() + 28U);
  output = decoded;
  return true;
}

}  // namespace elfin3_canfd
