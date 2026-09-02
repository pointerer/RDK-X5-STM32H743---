#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace elfin3_canfd
{

constexpr std::size_t kAxisCount = 6;
constexpr std::uint8_t kAllAxesMask = 0x3f;
constexpr std::uint8_t kProtocolVersion = 2;
constexpr std::uint8_t kCspMode = 8;
constexpr std::uint16_t kCspValidityMs = 40;
constexpr std::int64_t kCspPeriodUs = 2000;
constexpr std::size_t kCspStartPrefillPointCount = 12U;

constexpr std::uint32_t kCspCommandId = 0x180;
constexpr std::uint32_t kMotionControlId = 0x181;
constexpr std::uint32_t kParameterRequestId = 0x182;
constexpr std::uint32_t kHostHeartbeatId = 0x183;
constexpr std::uint32_t kPositionFeedbackId = 0x200;
constexpr std::uint32_t kDetailedStatusId = 0x201;
constexpr std::uint32_t kDiagnosticEventId = 0x202;
constexpr std::uint32_t kDeviceHeartbeatId = 0x203;
constexpr std::uint32_t kTrajectoryStatusId = 0x204;

constexpr std::size_t kCspCommandSize = 32;
constexpr std::size_t kMotionControlSize = 16;
constexpr std::size_t kParameterRequestSize = 16;
constexpr std::size_t kHostHeartbeatSize = 8;
constexpr std::size_t kPositionFeedbackSize = 32;
constexpr std::size_t kDetailedStatusSize = 64;
constexpr std::size_t kDiagnosticEventSize = 16;
constexpr std::size_t kDeviceHeartbeatSize = 8;
constexpr std::size_t kTrajectoryStatusSize = 32;

constexpr std::int64_t kTrajectoryStatusActivePeriodMs = 2;
constexpr std::int64_t kTrajectoryStatusInactivePeriodMs = 20;

constexpr std::uint8_t kHostFlagTrajectoryStatusSupported = 1U << 0;
constexpr std::uint8_t kHostFlagMask = kHostFlagTrajectoryStatusSupported;

constexpr std::int64_t kMotorCountsPerRevolution = 131072;
constexpr std::int64_t kGearRatio = 101;
constexpr std::int64_t kJointCountsPerRevolution =
  kMotorCountsPerRevolution * kGearRatio;

// Protocol V2 position arrays use logical Joint1 through Joint6 order.

enum CspFlag : std::uint8_t
{
  kApply = 1U << 0,
  kHold = 1U << 1,
  kTrajectoryReset = 1U << 2,
  kResetApply = kTrajectoryReset | kApply,
};

enum class MotionControl : std::uint8_t
{
  kDisable = 0,
  kEnable = 1,
  kHold = 2,
  kQuickStop = 3,
  kFaultReset = 4,
};

enum class ParameterOperation : std::uint8_t
{
  kRead = 0,
  kWrite = 1,
};

enum class HostState : std::uint8_t
{
  kStarting = 0,
  kNotReady = 1,
  kReady = 2,
  kMoving = 3,
  kFault = 4,
  kShuttingDown = 5,
};

enum class DiagnosticSeverity : std::uint8_t
{
  kInfo = 0,
  kWarning = 1,
  kError = 2,
  kFatal = 3,
};

enum class TrajectoryStatusState : std::uint8_t
{
  kIdle = 0x00,
  kPrefill = 0x01,
  kRunning = 0x02,
  kHold = 0x03,
};

enum TrajectoryStatusFlag : std::uint8_t
{
  kTrajectoryResetRequired = 1U << 0,
  kExpectedSequenceValid = 1U << 1,
  kLastReceivedSequenceValid = 1U << 2,
  kLastAcceptedSequenceValid = 1U << 3,
  kLastExecutedSequenceValid = 1U << 4,
  kLastRejectedSequenceValid = 1U << 5,
  kSafetyHoldLatched = 1U << 6,
  kQuickStopLatched = 1U << 7,
};

enum class TrajectoryRejectReason : std::uint8_t
{
  kNone = 0x00,
  kBadFrameFormat = 0x01,
  kBadDlc = 0x02,
  kCrcError = 0x03,
  kInvalidAxisMask = 0x04,
  kInvalidFlags = 0x05,
  kInvalidMode = 0x06,
  kInvalidValidity = 0x07,
  kResetRequired = 0x08,
  kSequenceError = 0x09,
  kQueueFull = 0x0a,
  kMotionNotReady = 0x0b,
  kInternalError = 0x0c,
};

enum class TrajectoryHoldReason : std::uint8_t
{
  kNone = 0x00,
  kExplicit0x180 = 0x01,
  kExplicit0x181 = 0x02,
  kQuickStop = 0x03,
  kQueueUnderrun = 0x04,
  kQueueOverflow = 0x05,
  kSequenceError = 0x06,
  kPointExpired = 0x07,
  kCspCommandTimeout = 0x08,
  kHostHeartbeatTimeout = 0x09,
  kEthercatNotOperational = 0x0a,
  kDriveNotCsp = 0x0b,
  kDriveFault = 0x0c,
  kInternalError = 0x0d,
};

struct CspCommand
{
  std::uint8_t sequence{0};
  std::uint8_t valid_axes_mask{kAllAxesMask};
  std::uint8_t flags{kApply};
  std::array<std::int32_t, kAxisCount> target_position_counts{};
  std::uint16_t validity_ms{kCspValidityMs};
};

struct MotionControlCommand
{
  std::uint8_t sequence{0};
  MotionControl command{MotionControl::kDisable};
  std::uint8_t axes_mask{kAllAxesMask};
  std::uint8_t flags{0};
  std::uint32_t request_token{0};
  std::uint32_t host_time_ms{0};
};

struct ParameterRequest
{
  std::uint8_t sequence{0};
  ParameterOperation operation{ParameterOperation::kRead};
  std::uint8_t target_axis{0xff};
  std::uint8_t flags{0};
  std::uint16_t parameter_id{0};
  std::int32_t value{0};
  std::uint16_t request_token{0};
};

struct HostHeartbeat
{
  std::uint8_t sequence{0};
  HostState state{HostState::kStarting};
  std::uint8_t flags{0};
  std::uint16_t uptime_100ms{0};
};

struct PositionFeedback
{
  std::uint8_t sequence{0};
  std::uint8_t valid_axes_mask{0};
  std::uint8_t ethercat_master_state{0};
  std::uint8_t status_flags{0};
  std::array<std::int32_t, kAxisCount> actual_position_counts{};
  std::uint16_t ethercat_cycle_low{0};
};

struct DetailedStatus
{
  std::uint8_t sequence{0};
  std::uint8_t valid_axes_mask{0};
  std::array<std::int32_t, kAxisCount> actual_velocity_raw{};
  std::array<std::int16_t, kAxisCount> actual_torque_raw{};
  std::array<std::uint16_t, kAxisCount> statusword{};
  std::array<std::uint16_t, kAxisCount> drive_error_code{};
};

struct DiagnosticEvent
{
  std::uint8_t sequence{0};
  DiagnosticSeverity severity{DiagnosticSeverity::kInfo};
  std::uint8_t source_axis{0xff};
  std::uint8_t flags{0};
  std::uint16_t primary_error_code{0};
  std::uint16_t detail_error_code{0};
  std::uint32_t context_value{0};
  std::uint16_t event_count{0};
};

struct DeviceHeartbeat
{
  std::uint8_t sequence{0};
  std::uint8_t ethercat_master_state{0};
  std::uint8_t slave_op_mask{0};
  std::uint16_t working_counter{0};
};

struct TrajectoryStatus
{
  std::uint8_t sequence{0};
  TrajectoryStatusState state{TrajectoryStatusState::kIdle};
  std::uint8_t queue_depth{0};
  std::uint8_t queue_capacity{0};
  std::uint8_t flags{0};
  std::uint8_t last_received_sequence{0};
  std::uint8_t last_accepted_sequence{0};
  std::uint8_t last_executed_sequence{0};
  std::uint8_t expected_sequence{0};
  std::uint8_t last_rejected_sequence{0};
  TrajectoryRejectReason reject_reason{TrajectoryRejectReason::kNone};
  TrajectoryHoldReason hold_reason{TrajectoryHoldReason::kNone};
  std::uint8_t prefill_target{0};
  std::uint16_t generation{0};
  std::uint16_t last_executed_ecat_cycle{0};
  std::uint16_t accepted_count{0};
  std::uint16_t executed_count{0};
  std::uint16_t rejected_count{0};
  std::uint16_t underrun_count{0};
  std::uint16_t overflow_count{0};
  std::uint16_t expired_count{0};
};

}  // namespace elfin3_canfd
