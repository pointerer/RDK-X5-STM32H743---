#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "elfin3_canfd_driver/byte_utils.hpp"
#include "elfin3_canfd_driver/crc16.hpp"
#include "elfin3_canfd_driver/csp_scheduler.hpp"
#include "elfin3_canfd_driver/decode.hpp"
#include "elfin3_canfd_driver/encode.hpp"
#include "elfin3_canfd_driver/position_conversion.hpp"
#include "elfin3_canfd_driver/sequence_tracker.hpp"

namespace
{

void provide_remote_idle(elfin3_canfd::CspScheduler & scheduler)
{
  elfin3_canfd::TrajectoryStatus status;
  status.state = elfin3_canfd::TrajectoryStatusState::kIdle;
  status.queue_capacity = elfin3_canfd::kExpectedRemoteQueueCapacity;
  status.prefill_target = elfin3_canfd::kExpectedRemotePrefillTarget;
  status.flags = elfin3_canfd::kTrajectoryResetRequired;
  scheduler.update_remote_status(status, std::chrono::steady_clock::now());
}

bool wait_for_stream_state(
  elfin3_canfd::CspScheduler & scheduler,
  const elfin3_canfd::TrajectoryStreamState expected)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < deadline) {
    if (scheduler.stream_state() == expected) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return scheduler.stream_state() == expected;
}

void acknowledge_reset(
  elfin3_canfd::CspScheduler & scheduler, const std::uint8_t reset_sequence = 0U)
{
  elfin3_canfd::TrajectoryStatus status;
  status.state = elfin3_canfd::TrajectoryStatusState::kPrefill;
  status.queue_depth = 1U;
  status.queue_capacity = elfin3_canfd::kExpectedRemoteQueueCapacity;
  status.prefill_target = elfin3_canfd::kExpectedRemotePrefillTarget;
  status.flags = static_cast<std::uint8_t>(
    elfin3_canfd::kExpectedSequenceValid |
    elfin3_canfd::kLastAcceptedSequenceValid);
  status.last_accepted_sequence = reset_sequence;
  status.expected_sequence = static_cast<std::uint8_t>(reset_sequence + 1U);
  status.generation = 1U;
  status.accepted_count = 1U;
  scheduler.update_remote_status(status, std::chrono::steady_clock::now());
}

TEST(ByteUtils, SignedAndUnsignedRoundTrip)
{
  std::array<std::uint8_t, 4> bytes{};
  elfin3_canfd::write_u32_le(bytes.data(), 0x89abcdefU);
  EXPECT_EQ(bytes, (std::array<std::uint8_t, 4>{0xef, 0xcd, 0xab, 0x89}));
  EXPECT_EQ(elfin3_canfd::read_u32_le(bytes.data()), 0x89abcdefU);

  elfin3_canfd::write_i32_le(bytes.data(), -123456789);
  EXPECT_EQ(elfin3_canfd::read_i32_le(bytes.data()), -123456789);
  elfin3_canfd::write_i16_le(bytes.data(), std::numeric_limits<std::int16_t>::min());
  EXPECT_EQ(elfin3_canfd::read_i16_le(bytes.data()), std::numeric_limits<std::int16_t>::min());
}

TEST(Crc16, CcittFalseKnownVector)
{
  constexpr std::array<std::uint8_t, 9> input{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(elfin3_canfd::crc16_ccitt_false(input.data(), input.size()), 0x29b1U);
}

TEST(Encode, GoldenCspAndHeartbeatFrames)
{
  elfin3_canfd::CspCommand command;
  command.sequence = 0x7a;
  command.target_position_counts = {
    1, -2, 0x12345678, std::numeric_limits<std::int32_t>::min(),
    std::numeric_limits<std::int32_t>::max(), 0};
  elfin3_canfd::CspCommandFrame csp{};
  ASSERT_TRUE(elfin3_canfd::encode_csp_command(command, csp));
  EXPECT_EQ(csp[0], 0x7a);
  EXPECT_EQ(csp[1], 0x3f);
  EXPECT_EQ(csp[2], 0x01);
  EXPECT_EQ(csp[3], 0x08);
  EXPECT_EQ(csp[12], 0x78);
  EXPECT_EQ(csp[13], 0x56);
  EXPECT_EQ(csp[14], 0x34);
  EXPECT_EQ(csp[15], 0x12);
  EXPECT_EQ(csp[30], 0xab);
  EXPECT_EQ(csp[31], 0x15);

  elfin3_canfd::HostHeartbeat heartbeat;
  heartbeat.sequence = 0x2a;
  heartbeat.state = elfin3_canfd::HostState::kMoving;
  heartbeat.uptime_100ms = 0x1234;
  elfin3_canfd::HostHeartbeatFrame encoded_heartbeat{};
  ASSERT_TRUE(elfin3_canfd::encode_host_heartbeat(heartbeat, encoded_heartbeat));
  EXPECT_EQ(encoded_heartbeat,
    (elfin3_canfd::HostHeartbeatFrame{0x2a, 2, 3, 0, 0x34, 0x12, 0x67, 0x99}));
}

TEST(Encode, CspV2AcceptsOnlyDefinedMaskFlagsAndValidity)
{
  elfin3_canfd::CspCommand command;
  elfin3_canfd::CspCommandFrame frame{};

  command.flags = elfin3_canfd::kResetApply;
  EXPECT_TRUE(elfin3_canfd::encode_csp_command(command, frame));
  EXPECT_EQ(frame[1], elfin3_canfd::kAllAxesMask);
  EXPECT_EQ(frame[2], elfin3_canfd::kResetApply);
  EXPECT_EQ(elfin3_canfd::read_u16_le(frame.data() + 28U), elfin3_canfd::kCspValidityMs);

  command.flags = elfin3_canfd::kHold;
  EXPECT_TRUE(elfin3_canfd::encode_csp_command(command, frame));

  command.flags = 0x00;
  EXPECT_FALSE(elfin3_canfd::encode_csp_command(command, frame));
  command.flags = elfin3_canfd::kTrajectoryReset;
  EXPECT_FALSE(elfin3_canfd::encode_csp_command(command, frame));
  command.flags = elfin3_canfd::kApply | elfin3_canfd::kHold;
  EXPECT_FALSE(elfin3_canfd::encode_csp_command(command, frame));
  command.flags = 0x08;
  EXPECT_FALSE(elfin3_canfd::encode_csp_command(command, frame));

  command.flags = elfin3_canfd::kApply;
  command.valid_axes_mask = 0x1f;
  EXPECT_FALSE(elfin3_canfd::encode_csp_command(command, frame));
  command.valid_axes_mask = 0x7f;
  EXPECT_FALSE(elfin3_canfd::encode_csp_command(command, frame));

  command.valid_axes_mask = elfin3_canfd::kAllAxesMask;
  command.validity_ms = elfin3_canfd::kCspValidityMs - 1U;
  EXPECT_FALSE(elfin3_canfd::encode_csp_command(command, frame));
  command.validity_ms = elfin3_canfd::kCspValidityMs + 1U;
  EXPECT_FALSE(elfin3_canfd::encode_csp_command(command, frame));
}

TEST(Encode, HostHeartbeatAdvertisesOnlyDefinedCapabilities)
{
  elfin3_canfd::HostHeartbeat heartbeat;
  heartbeat.flags = elfin3_canfd::kHostFlagTrajectoryStatusSupported;
  elfin3_canfd::HostHeartbeatFrame frame{};
  ASSERT_TRUE(elfin3_canfd::encode_host_heartbeat(heartbeat, frame));
  EXPECT_EQ(frame[1], elfin3_canfd::kProtocolVersion);
  EXPECT_EQ(frame[3], elfin3_canfd::kHostFlagTrajectoryStatusSupported);

  heartbeat.flags = 0x02U;
  EXPECT_FALSE(elfin3_canfd::encode_host_heartbeat(heartbeat, frame));
}

TEST(Encode, JogPointsKeepLogicalJointOrderAndHundredCountStep)
{
  elfin3_canfd::CspCommand first;
  first.sequence = 254U;
  first.flags = elfin3_canfd::kResetApply;
  first.target_position_counts = {226000, 260000, 1666230, 1960000, 1713600, 2890000};
  auto second = first;
  second.sequence = 255U;
  second.flags = elfin3_canfd::kApply;
  second.target_position_counts[2] += 100;

  elfin3_canfd::CspCommandFrame first_frame{};
  elfin3_canfd::CspCommandFrame second_frame{};
  ASSERT_TRUE(elfin3_canfd::encode_csp_command(first, first_frame));
  ASSERT_TRUE(elfin3_canfd::encode_csp_command(second, second_frame));
  EXPECT_EQ(first_frame[0], 254U);
  EXPECT_EQ(second_frame[0], 255U);
  for (std::size_t joint = 0; joint < elfin3_canfd::kAxisCount; ++joint) {
    EXPECT_EQ(
      elfin3_canfd::read_i32_le(first_frame.data() + 4U + 4U * joint),
      first.target_position_counts[joint]);
  }
  EXPECT_EQ(
    elfin3_canfd::read_i32_le(second_frame.data() + 12U) -
    elfin3_canfd::read_i32_le(first_frame.data() + 12U),
    100);
}

TEST(Decode, DeviceHeartbeatRequiresProtocolV2)
{
  elfin3_canfd::DeviceHeartbeatFrame frame{};
  frame[0] = 7;
  frame[1] = elfin3_canfd::kProtocolVersion;
  frame[2] = 3;
  frame[3] = 0x07;
  elfin3_canfd::write_u16_le(frame.data() + 4U, 0x1234);
  ASSERT_TRUE(elfin3_canfd::append_crc(frame.data(), frame.size()));

  elfin3_canfd::DeviceHeartbeat decoded;
  ASSERT_TRUE(elfin3_canfd::decode_device_heartbeat(frame, decoded));
  EXPECT_EQ(decoded.sequence, 7);
  EXPECT_EQ(decoded.slave_op_mask, 0x07);

  frame[1] = 1;
  ASSERT_TRUE(elfin3_canfd::append_crc(frame.data(), frame.size()));
  EXPECT_FALSE(elfin3_canfd::decode_device_heartbeat(frame, decoded));
}

TEST(Decode, PositionFeedbackCommitsOnlyValidFrame)
{
  elfin3_canfd::PositionFeedbackFrame frame{};
  frame[0] = 9;
  frame[1] = 0x3f;
  frame[2] = 4;
  frame[3] = 0x03;
  for (std::size_t axis = 0; axis < elfin3_canfd::kAxisCount; ++axis) {
    elfin3_canfd::write_i32_le(frame.data() + 4U + axis * 4U,
      static_cast<std::int32_t>(axis * 100U) - 250);
  }
  elfin3_canfd::write_u16_le(frame.data() + 28U, 0xfffe);
  ASSERT_TRUE(elfin3_canfd::append_crc(frame.data(), frame.size()));

  elfin3_canfd::PositionFeedback decoded;
  ASSERT_TRUE(elfin3_canfd::decode_position_feedback(frame, decoded));
  EXPECT_EQ(decoded.sequence, 9);
  EXPECT_EQ(decoded.actual_position_counts[0], -250);
  EXPECT_EQ(decoded.actual_position_counts[5], 250);
  EXPECT_EQ(decoded.ethercat_cycle_low, 0xfffe);

  const auto previous = decoded.actual_position_counts;
  frame[4] ^= 0x01U;
  EXPECT_FALSE(elfin3_canfd::decode_position_feedback(frame, decoded));
  EXPECT_EQ(decoded.actual_position_counts, previous);
}

TEST(Decode, TrajectoryStatusMatchesStm32GoldenFrame)
{
  const elfin3_canfd::TrajectoryStatusFrame frame{
    0xa5, 0x02, 0x02, 0x08, 0x0c, 0xff, 0x10, 0x0f,
    0x0e, 0x11, 0x7f, 0x09, 0x04, 0x02, 0x78, 0x56,
    0xef, 0xcd, 0x02, 0x00, 0x34, 0x12, 0x01, 0x00,
    0xef, 0xbe, 0xfe, 0xca, 0xbe, 0xba, 0x68, 0xba};

  elfin3_canfd::TrajectoryStatus decoded;
  ASSERT_TRUE(elfin3_canfd::decode_trajectory_status(frame, decoded));
  EXPECT_EQ(decoded.sequence, 0xa5U);
  EXPECT_EQ(decoded.state, elfin3_canfd::TrajectoryStatusState::kRunning);
  EXPECT_EQ(decoded.queue_depth, 8U);
  EXPECT_EQ(decoded.queue_capacity, 12U);
  EXPECT_EQ(decoded.flags, 0xffU);
  EXPECT_EQ(decoded.last_received_sequence, 0x10U);
  EXPECT_EQ(decoded.last_accepted_sequence, 0x0fU);
  EXPECT_EQ(decoded.last_executed_sequence, 0x0eU);
  EXPECT_EQ(decoded.expected_sequence, 0x11U);
  EXPECT_EQ(decoded.last_rejected_sequence, 0x7fU);
  EXPECT_EQ(decoded.reject_reason, elfin3_canfd::TrajectoryRejectReason::kSequenceError);
  EXPECT_EQ(decoded.hold_reason, elfin3_canfd::TrajectoryHoldReason::kQueueUnderrun);
  EXPECT_EQ(decoded.prefill_target, 2U);
  EXPECT_EQ(decoded.generation, 0x5678U);
  EXPECT_EQ(decoded.last_executed_ecat_cycle, 0xcdefU);
  EXPECT_EQ(decoded.accepted_count, 2U);
  EXPECT_EQ(decoded.executed_count, 0x1234U);
  EXPECT_EQ(decoded.rejected_count, 1U);
  EXPECT_EQ(decoded.underrun_count, 0xbeefU);
  EXPECT_EQ(decoded.overflow_count, 0xcafeU);
  EXPECT_EQ(decoded.expired_count, 0xbabeU);
}

TEST(Decode, TrajectoryStatusRejectsInvalidFrameWithoutChangingOutput)
{
  elfin3_canfd::TrajectoryStatusFrame frame{
    0x00, 0x02, 0x00, 0x00, 0x0c, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb3, 0x1d};
  elfin3_canfd::TrajectoryStatus decoded;
  decoded.sequence = 0x55;

  ASSERT_TRUE(elfin3_canfd::decode_trajectory_status(frame, decoded));
  EXPECT_EQ(decoded.sequence, 0U);
  decoded.sequence = 0x55;

  frame[1] = 1U;
  ASSERT_TRUE(elfin3_canfd::append_crc(frame.data(), frame.size()));
  EXPECT_FALSE(elfin3_canfd::decode_trajectory_status(frame, decoded));
  EXPECT_EQ(decoded.sequence, 0x55U);

  frame[1] = elfin3_canfd::kProtocolVersion;
  frame[3] = 13U;
  ASSERT_TRUE(elfin3_canfd::append_crc(frame.data(), frame.size()));
  EXPECT_FALSE(elfin3_canfd::decode_trajectory_status(frame, decoded));
  EXPECT_EQ(decoded.sequence, 0x55U);

  frame[3] = 0U;
  frame[11] = 0x0dU;
  ASSERT_TRUE(elfin3_canfd::append_crc(frame.data(), frame.size()));
  EXPECT_FALSE(elfin3_canfd::decode_trajectory_status(frame, decoded));
  EXPECT_EQ(decoded.sequence, 0x55U);

  frame[11] = 0U;
  ASSERT_TRUE(elfin3_canfd::append_crc(frame.data(), frame.size()));
  frame[30] ^= 0x01U;
  EXPECT_FALSE(elfin3_canfd::decode_trajectory_status(frame, decoded));
  EXPECT_EQ(decoded.sequence, 0x55U);
}

TEST(SequenceTracker, AcceptsForwardAndRejectsDuplicateOrBackward)
{
  elfin3_canfd::SequenceTracker tracker;
  EXPECT_EQ(tracker.accept(254), elfin3_canfd::SequenceResult::kFirst);
  EXPECT_EQ(tracker.accept(255), elfin3_canfd::SequenceResult::kAccepted);
  EXPECT_EQ(tracker.accept(0), elfin3_canfd::SequenceResult::kAccepted);
  EXPECT_EQ(tracker.accept(0), elfin3_canfd::SequenceResult::kDuplicate);
  EXPECT_EQ(tracker.accept(250), elfin3_canfd::SequenceResult::kBackward);
  EXPECT_EQ(tracker.accept(2), elfin3_canfd::SequenceResult::kAcceptedWithGap);
  EXPECT_EQ(tracker.lost_count(), 1U);
}

TEST(PositionConversion, CalibratedZeroUsesLogicalJointOrder)
{
  const std::array<std::int32_t, elfin3_canfd::kAxisCount> zero_offsets{
    226000, 260000, 1666230, 1960000, 1713600, 2890000};
  const std::array<std::int8_t, elfin3_canfd::kAxisCount> directions{-1, -1, -1, -1, -1, -1};
  const auto protocol_zero = zero_offsets;

  std::array<double, elfin3_canfd::kAxisCount> positions{};
  ASSERT_TRUE(elfin3_canfd::protocol_counts_to_joint_radians(
      protocol_zero, zero_offsets, directions, positions));
  for (const auto position : positions) {
    EXPECT_DOUBLE_EQ(position, 0.0);
  }

  std::array<std::int32_t, elfin3_canfd::kAxisCount> encoded{};
  ASSERT_TRUE(elfin3_canfd::joint_radians_to_protocol_counts(
      positions, zero_offsets, directions, encoded));
  EXPECT_EQ(encoded, protocol_zero);
}

TEST(PositionConversion, PositiveJointMotionDecreasesCountsOnAllAxes)
{
  const std::array<std::int32_t, elfin3_canfd::kAxisCount> zero_offsets{
    226000, 260000, 1666230, 1960000, 1713600, 2890000};
  const std::array<std::int8_t, elfin3_canfd::kAxisCount> directions{-1, -1, -1, -1, -1, -1};
  const std::array<double, elfin3_canfd::kAxisCount> positive_positions{
    0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

  std::array<std::int32_t, elfin3_canfd::kAxisCount> counts{};
  ASSERT_TRUE(elfin3_canfd::joint_radians_to_protocol_counts(
      positive_positions, zero_offsets, directions, counts));
  for (std::size_t joint = 0; joint < elfin3_canfd::kAxisCount; ++joint) {
    EXPECT_LT(counts[joint], zero_offsets[joint]);
  }
}

TEST(PositionConversion, RoundTripPreservesJointOrderAndDirection)
{
  const std::array<std::int32_t, elfin3_canfd::kAxisCount> zero_offsets{
    226000, 260000, 1666230, 1960000, 1713600, 2890000};
  const std::array<std::int8_t, elfin3_canfd::kAxisCount> directions{1, -1, 1, -1, 1, -1};
  const std::array<double, elfin3_canfd::kAxisCount> expected{
    0.1, -0.2, 0.3, -0.4, 0.5, -0.6};

  std::array<std::int32_t, elfin3_canfd::kAxisCount> counts{};
  ASSERT_TRUE(elfin3_canfd::joint_radians_to_protocol_counts(
      expected, zero_offsets, directions, counts));
  EXPECT_GT(counts[0], zero_offsets[0]);  // field 0 carries Joint1
  EXPECT_GT(counts[1], zero_offsets[1]);  // field 1 carries Joint2

  std::array<double, elfin3_canfd::kAxisCount> actual{};
  ASSERT_TRUE(elfin3_canfd::protocol_counts_to_joint_radians(
      counts, zero_offsets, directions, actual));
  const double tolerance = static_cast<double>(elfin3_canfd::kRadiansPerCount) / 2.0;
  for (std::size_t joint = 0; joint < elfin3_canfd::kAxisCount; ++joint) {
    EXPECT_NEAR(actual[joint], expected[joint], tolerance);
  }
}

TEST(PositionConversion, RejectsInvalidInputWithoutChangingOutput)
{
  const std::array<std::int32_t, elfin3_canfd::kAxisCount> zero_offsets{};
  const std::array<std::int8_t, elfin3_canfd::kAxisCount> invalid_directions{1, 1, 0, 1, 1, 1};
  const std::array<std::int32_t, elfin3_canfd::kAxisCount> counts{};
  std::array<double, elfin3_canfd::kAxisCount> positions{1, 1, 1, 1, 1, 1};
  EXPECT_FALSE(elfin3_canfd::protocol_counts_to_joint_radians(
      counts, zero_offsets, invalid_directions, positions));
  EXPECT_EQ(positions, (std::array<double, elfin3_canfd::kAxisCount>{1, 1, 1, 1, 1, 1}));

  const std::array<std::int8_t, elfin3_canfd::kAxisCount> directions{1, 1, 1, 1, 1, 1};
  positions[0] = std::numeric_limits<double>::infinity();
  std::array<std::int32_t, elfin3_canfd::kAxisCount> output{1, 1, 1, 1, 1, 1};
  EXPECT_FALSE(elfin3_canfd::joint_radians_to_protocol_counts(
      positions, zero_offsets, directions, output));
  EXPECT_EQ(output, (std::array<std::int32_t, elfin3_canfd::kAxisCount>{1, 1, 1, 1, 1, 1}));

  positions[0] = 1.0e20;
  EXPECT_FALSE(elfin3_canfd::joint_radians_to_protocol_counts(
      positions, zero_offsets, directions, output));
  EXPECT_EQ(output, (std::array<std::int32_t, elfin3_canfd::kAxisCount>{1, 1, 1, 1, 1, 1}));
}

TEST(CspScheduler, ExposesExplicitTrajectoryLifecycle)
{
  elfin3_canfd::CspScheduler scheduler(
    [](std::uint32_t, const std::uint8_t *, std::size_t, std::string &) {
      return true;
    },
    std::chrono::microseconds(2000), std::chrono::milliseconds(20),
    elfin3_canfd::kCspValidityMs);

  EXPECT_EQ(scheduler.stream_state(), elfin3_canfd::TrajectoryStreamState::kIdle);
  EXPECT_FALSE(scheduler.start_trajectory());
  EXPECT_EQ(
    scheduler.recover_trajectory(),
    elfin3_canfd::TrajectoryRecoveryResult::kMotionGateClosed);
  scheduler.set_enabled(true);
  EXPECT_EQ(
    scheduler.recover_trajectory(),
    elfin3_canfd::TrajectoryRecoveryResult::kStateNotRecoverable);
  EXPECT_TRUE(scheduler.start_trajectory());
  EXPECT_EQ(
    scheduler.stream_state(), elfin3_canfd::TrajectoryStreamState::kStartPending);
  EXPECT_TRUE(scheduler.finish_trajectory());
  EXPECT_EQ(scheduler.stream_state(), elfin3_canfd::TrajectoryStreamState::kHolding);

  EXPECT_TRUE(scheduler.start_trajectory());
  scheduler.abort_trajectory();
  EXPECT_EQ(
    scheduler.stream_state(), elfin3_canfd::TrajectoryStreamState::kResetRequired);

  scheduler.mark_fault();
  EXPECT_EQ(scheduler.stream_state(), elfin3_canfd::TrajectoryStreamState::kFault);
  EXPECT_FALSE(scheduler.start_trajectory());
  EXPECT_EQ(
    scheduler.recover_trajectory(), elfin3_canfd::TrajectoryRecoveryResult::kSuccess);
  EXPECT_EQ(
    scheduler.stream_state(), elfin3_canfd::TrajectoryStreamState::kStartPending);

  const auto stats = scheduler.stats();
  EXPECT_EQ(stats.start_requests, 4U);
  EXPECT_EQ(stats.start_successes, 2U);
  EXPECT_EQ(stats.start_rejections, 2U);
}

TEST(CspScheduler, SendsStrictContinuousSequenceWithNaturalWraparound)
{
  std::mutex frames_mutex;
  std::vector<std::array<std::uint8_t, 2>> frames;
  elfin3_canfd::CspScheduler scheduler(
    [&frames_mutex, &frames](
      std::uint32_t, const std::uint8_t * data, std::size_t size, std::string &)
    {
      if (size != elfin3_canfd::kCspCommandSize) {
        return false;
      }
      std::lock_guard<std::mutex> lock(frames_mutex);
      frames.push_back({data[0], data[2]});
      return true;
    },
    std::chrono::microseconds(elfin3_canfd::kCspPeriodUs), std::chrono::seconds(5),
    elfin3_canfd::kCspValidityMs, 300U);

  ASSERT_TRUE(scheduler.submit_target({0, 0, 0, 0, 0, 0}));
  provide_remote_idle(scheduler);
  scheduler.set_enabled(true);
  ASSERT_TRUE(scheduler.start_trajectory());
  for (std::size_t index = 1; index < 258U; ++index) {
    ASSERT_TRUE(scheduler.submit_target(
        {static_cast<std::int32_t>(index), 0, 0, 0, 0, 0}));
  }
  scheduler.start();
  ASSERT_TRUE(wait_for_stream_state(
      scheduler, elfin3_canfd::TrajectoryStreamState::kResetAwaitingAck));
  {
    std::lock_guard<std::mutex> lock(frames_mutex);
    ASSERT_EQ(frames.size(), 1U);
  }
  acknowledge_reset(scheduler);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  std::size_t acknowledged_frame_count = 1U;
  while (std::chrono::steady_clock::now() < deadline) {
    std::size_t frame_count = 0U;
    std::uint8_t last_sequence = 0U;
    {
      std::lock_guard<std::mutex> lock(frames_mutex);
      frame_count = frames.size();
      if (!frames.empty()) {
        last_sequence = frames.back()[0];
      }
      if (frames.size() >= 258U) {
        break;
      }
    }
    if (frame_count > acknowledged_frame_count) {
      elfin3_canfd::TrajectoryStatus status;
      status.state = elfin3_canfd::TrajectoryStatusState::kRunning;
      status.queue_depth = 1U;
      status.queue_capacity = elfin3_canfd::kExpectedRemoteQueueCapacity;
      status.prefill_target = elfin3_canfd::kExpectedRemotePrefillTarget;
      status.flags = static_cast<std::uint8_t>(
        elfin3_canfd::kExpectedSequenceValid |
        elfin3_canfd::kLastAcceptedSequenceValid);
      status.last_accepted_sequence = last_sequence;
      status.expected_sequence = static_cast<std::uint8_t>(last_sequence + 1U);
      status.generation = 1U;
      status.accepted_count = static_cast<std::uint16_t>(frame_count);
      scheduler.update_remote_status(status, std::chrono::steady_clock::now());
      acknowledged_frame_count = frame_count;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  scheduler.stop();

  std::lock_guard<std::mutex> lock(frames_mutex);
  ASSERT_GE(frames.size(), 258U);
  EXPECT_EQ(frames.front()[0], 0U);
  EXPECT_EQ(frames.front()[1], elfin3_canfd::kResetApply);
  for (std::size_t index = 1; index < 258U; ++index) {
    EXPECT_EQ(frames[index][0], static_cast<std::uint8_t>(index));
    EXPECT_EQ(frames[index][1], elfin3_canfd::kApply);
  }
}

TEST(CspScheduler, QueuesTrajectoryPointsInSubmissionOrder)
{
  std::mutex frames_mutex;
  std::vector<std::array<std::int32_t, 2>> frames;
  elfin3_canfd::CspScheduler scheduler(
    [&frames_mutex, &frames](
      std::uint32_t, const std::uint8_t * data, std::size_t, std::string &)
    {
      std::lock_guard<std::mutex> lock(frames_mutex);
      frames.push_back({
        static_cast<std::int32_t>(data[2]),
        elfin3_canfd::read_i32_le(data + 4U)});
      return true;
    },
    std::chrono::microseconds(elfin3_canfd::kCspPeriodUs), std::chrono::seconds(1),
    elfin3_canfd::kCspValidityMs);

  ASSERT_TRUE(scheduler.submit_target({10, 0, 0, 0, 0, 0}));
  provide_remote_idle(scheduler);
  scheduler.set_enabled(true);
  ASSERT_TRUE(scheduler.start_trajectory());
  for (std::size_t index = 1U; index < elfin3_canfd::kCspStartPrefillPointCount; ++index) {
    const auto target = static_cast<std::int32_t>((index + 1U) * 10U);
    ASSERT_TRUE(scheduler.submit_target({target, 0, 0, 0, 0, 0}));
  }
  scheduler.start();
  ASSERT_TRUE(wait_for_stream_state(
      scheduler, elfin3_canfd::TrajectoryStreamState::kResetAwaitingAck));
  acknowledge_reset(scheduler);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(frames_mutex);
      if (frames.size() >= 3U) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  scheduler.stop();

  std::lock_guard<std::mutex> lock(frames_mutex);
  ASSERT_GE(frames.size(), 3U);
  EXPECT_EQ(frames[0], (std::array<std::int32_t, 2>{elfin3_canfd::kResetApply, 10}));
  EXPECT_EQ(frames[1], (std::array<std::int32_t, 2>{elfin3_canfd::kApply, 20}));
  EXPECT_EQ(frames[2], (std::array<std::int32_t, 2>{elfin3_canfd::kApply, 30}));
}

TEST(CspScheduler, RejectsQueueOverflowWithoutReplacingQueuedPoints)
{
  elfin3_canfd::CspScheduler scheduler(
    [](std::uint32_t, const std::uint8_t *, std::size_t, std::string &) {
      return true;
    },
    std::chrono::milliseconds(2), std::chrono::seconds(1),
    elfin3_canfd::kCspValidityMs, elfin3_canfd::kCspStartPrefillPointCount);

  ASSERT_TRUE(scheduler.submit_target({1, 0, 0, 0, 0, 0}));
  EXPECT_EQ(scheduler.queued_target_count(), 0U);
  scheduler.set_enabled(true);
  ASSERT_TRUE(scheduler.start_trajectory());
  EXPECT_EQ(scheduler.queued_target_count(), 1U);
  for (std::size_t index = 2U; index <= elfin3_canfd::kCspStartPrefillPointCount; ++index) {
    ASSERT_TRUE(scheduler.submit_target(
        {static_cast<std::int32_t>(index), 0, 0, 0, 0, 0}));
  }
  EXPECT_FALSE(scheduler.submit_target(
      {static_cast<std::int32_t>(elfin3_canfd::kCspStartPrefillPointCount + 1U),
        0, 0, 0, 0, 0}));
  EXPECT_EQ(
    scheduler.queued_target_count(), elfin3_canfd::kCspStartPrefillPointCount);
  EXPECT_EQ(scheduler.stats().queue_overflows, 1U);
}

TEST(CspScheduler, WaitsForConfiguredPrefillBeforeSendingTrajectoryReset)
{
  std::atomic<std::size_t> frame_count{0U};
  std::atomic<std::uint8_t> first_flags{0U};
  elfin3_canfd::CspScheduler scheduler(
    [&frame_count, &first_flags](
      std::uint32_t, const std::uint8_t * data, std::size_t, std::string &)
    {
      if (frame_count.fetch_add(1U) == 0U) {
        first_flags.store(data[2]);
      }
      return true;
    },
    std::chrono::microseconds(elfin3_canfd::kCspPeriodUs), std::chrono::seconds(1),
    elfin3_canfd::kCspValidityMs);

  ASSERT_TRUE(scheduler.submit_target({10, 0, 0, 0, 0, 0}));
  provide_remote_idle(scheduler);
  scheduler.set_enabled(true);
  ASSERT_TRUE(scheduler.start_trajectory());
  scheduler.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(frame_count.load(), 0U);
  EXPECT_EQ(
    scheduler.stream_state(), elfin3_canfd::TrajectoryStreamState::kStartPending);
  const auto waiting_stats = scheduler.stats();
  EXPECT_TRUE(waiting_stats.start_gate_sample_valid);
  EXPECT_GT(waiting_stats.start_pending_cycles, 0U);
  EXPECT_GT(waiting_stats.start_wait_prefill_cycles, 0U);
  EXPECT_TRUE(waiting_stats.start_last_enabled);
  EXPECT_FALSE(waiting_stats.start_last_prefill_ready);
  EXPECT_TRUE(waiting_stats.start_last_point_available);
  EXPECT_TRUE(waiting_stats.start_last_point_fresh);
  EXPECT_TRUE(waiting_stats.start_last_remote_ready);
  EXPECT_EQ(waiting_stats.start_last_local_queue_depth, 1U);

  for (std::size_t index = 1U;
    index + 1U < elfin3_canfd::kCspStartPrefillPointCount; ++index)
  {
    const auto target = static_cast<std::int32_t>((index + 1U) * 10U);
    ASSERT_TRUE(scheduler.submit_target({target, 0, 0, 0, 0, 0}));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(frame_count.load(), 0U);

  const auto final_prefill_target = static_cast<std::int32_t>(
    elfin3_canfd::kCspStartPrefillPointCount * 10U);
  ASSERT_TRUE(scheduler.submit_target({final_prefill_target, 0, 0, 0, 0, 0}));
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (frame_count.load() == 0U && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  scheduler.stop();

  ASSERT_GT(frame_count.load(), 0U);
  EXPECT_EQ(first_flags.load(), elfin3_canfd::kResetApply);
  const auto started_stats = scheduler.stats();
  EXPECT_EQ(started_stats.reset_apply_frames, 1U);
  EXPECT_TRUE(started_stats.start_last_prefill_ready);
  EXPECT_TRUE(started_stats.start_last_remote_ready);
}

TEST(CspScheduler, ReportsRemoteGateWhileStartPending)
{
  std::atomic<std::size_t> frame_count{0U};
  elfin3_canfd::CspScheduler scheduler(
    [&frame_count](
      std::uint32_t, const std::uint8_t *, std::size_t, std::string &)
    {
      frame_count.fetch_add(1U);
      return true;
    },
    std::chrono::microseconds(elfin3_canfd::kCspPeriodUs), std::chrono::seconds(1),
    elfin3_canfd::kCspValidityMs);

  ASSERT_TRUE(scheduler.submit_target({0, 0, 0, 0, 0, 0}));
  scheduler.set_enabled(true);
  ASSERT_TRUE(scheduler.start_trajectory());
  for (std::size_t index = 1U;
    index < elfin3_canfd::kCspStartPrefillPointCount; ++index)
  {
    ASSERT_TRUE(scheduler.submit_target(
        {static_cast<std::int32_t>(index), 0, 0, 0, 0, 0}));
  }

  scheduler.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  scheduler.stop();

  EXPECT_EQ(frame_count.load(), 0U);
  EXPECT_EQ(
    scheduler.stream_state(), elfin3_canfd::TrajectoryStreamState::kStartPending);
  const auto stats = scheduler.stats();
  EXPECT_TRUE(stats.start_gate_sample_valid);
  EXPECT_TRUE(stats.start_last_prefill_ready);
  EXPECT_TRUE(stats.start_last_point_available);
  EXPECT_TRUE(stats.start_last_point_fresh);
  EXPECT_FALSE(stats.start_last_remote_ready);
  EXPECT_GT(stats.start_wait_remote_cycles, 0U);
  EXPECT_EQ(
    stats.start_last_local_queue_depth,
    elfin3_canfd::kCspStartPrefillPointCount);
  EXPECT_EQ(stats.reset_apply_frames, 0U);
}

TEST(CspScheduler, StopsAtRemoteCreditLimitWithoutAcceptanceUpdates)
{
  std::atomic<std::size_t> frame_count{0U};
  elfin3_canfd::CspScheduler scheduler(
    [&frame_count](
      std::uint32_t, const std::uint8_t *, std::size_t, std::string &)
    {
      frame_count.fetch_add(1U);
      return true;
    },
    std::chrono::microseconds(elfin3_canfd::kCspPeriodUs), std::chrono::seconds(1),
    elfin3_canfd::kCspValidityMs, 20U, 4U,
    elfin3_canfd::kExpectedRemoteQueueCapacity - elfin3_canfd::kRemoteQueueReserve,
    std::chrono::microseconds(1000));

  ASSERT_TRUE(scheduler.submit_target({0, 0, 0, 0, 0, 0}));
  provide_remote_idle(scheduler);
  scheduler.set_enabled(true);
  ASSERT_TRUE(scheduler.start_trajectory());
  for (std::int32_t point = 1; point < 20; ++point) {
    ASSERT_TRUE(scheduler.submit_target(
        {static_cast<std::int32_t>(point), 0, 0, 0, 0, 0}));
  }
  scheduler.start();
  ASSERT_TRUE(wait_for_stream_state(
      scheduler, elfin3_canfd::TrajectoryStreamState::kResetAwaitingAck));
  acknowledge_reset(scheduler);
  ASSERT_TRUE(wait_for_stream_state(
      scheduler, elfin3_canfd::TrajectoryStreamState::kStreaming));
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  scheduler.stop();

  EXPECT_EQ(
    frame_count.load(),
    elfin3_canfd::kExpectedRemoteQueueCapacity - elfin3_canfd::kRemoteQueueReserve);
  EXPECT_GT(scheduler.stats().credit_stalls, 0U);
  EXPECT_EQ(
    scheduler.stats().unacknowledged_points,
    elfin3_canfd::kExpectedRemoteQueueCapacity - elfin3_canfd::kRemoteQueueReserve - 1U);
}

TEST(CspScheduler, RefillsOnlyUpToConfiguredHighWatermark)
{
  std::atomic<std::size_t> frame_count{0U};
  elfin3_canfd::CspScheduler scheduler(
    [&frame_count](
      std::uint32_t, const std::uint8_t *, std::size_t, std::string &)
    {
      frame_count.fetch_add(1U);
      return true;
    },
    std::chrono::microseconds(elfin3_canfd::kCspPeriodUs), std::chrono::seconds(1),
    elfin3_canfd::kCspValidityMs, 20U);

  ASSERT_TRUE(scheduler.submit_target({0, 0, 0, 0, 0, 0}));
  provide_remote_idle(scheduler);
  scheduler.set_enabled(true);
  ASSERT_TRUE(scheduler.start_trajectory());
  for (std::size_t point = 1U; point < 20U; ++point) {
    ASSERT_TRUE(scheduler.submit_target(
        {static_cast<std::int32_t>(point), 0, 0, 0, 0, 0}));
  }
  scheduler.start();
  ASSERT_TRUE(wait_for_stream_state(
      scheduler, elfin3_canfd::TrajectoryStreamState::kResetAwaitingAck));
  acknowledge_reset(scheduler);
  ASSERT_TRUE(wait_for_stream_state(
      scheduler, elfin3_canfd::TrajectoryStreamState::kStreaming));
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  scheduler.stop();

  EXPECT_EQ(frame_count.load(), elfin3_canfd::kDefaultRemoteHighWatermark);
  EXPECT_GT(scheduler.stats().high_watermark_stalls, 0U);
  EXPECT_GT(scheduler.stats().refill_cycles, 0U);
}

TEST(CspScheduler, BridgesBriefLocalPointGapWithFiniteDuplicateTarget)
{
  std::atomic<std::size_t> frame_count{0U};
  std::atomic<std::uint8_t> last_sequence{0U};
  std::mutex frames_mutex;
  std::vector<std::int32_t> targets;
  elfin3_canfd::CspScheduler scheduler(
    [&frame_count, &last_sequence, &frames_mutex, &targets](
      std::uint32_t, const std::uint8_t * data, std::size_t, std::string &)
    {
      {
        std::lock_guard<std::mutex> lock(frames_mutex);
        targets.push_back(elfin3_canfd::read_i32_le(data + 4U));
      }
      last_sequence.store(data[0]);
      frame_count.fetch_add(1U);
      return true;
    },
    std::chrono::microseconds(elfin3_canfd::kCspPeriodUs), std::chrono::seconds(1),
    elfin3_canfd::kCspValidityMs);

  ASSERT_TRUE(scheduler.submit_target({0, 0, 0, 0, 0, 0}));
  provide_remote_idle(scheduler);
  scheduler.set_enabled(true);
  ASSERT_TRUE(scheduler.start_trajectory());
  for (std::size_t point = 1U; point < elfin3_canfd::kCspStartPrefillPointCount; ++point) {
    ASSERT_TRUE(scheduler.submit_target(
        {static_cast<std::int32_t>(point), 0, 0, 0, 0, 0}));
  }
  scheduler.start();
  ASSERT_TRUE(wait_for_stream_state(
      scheduler, elfin3_canfd::TrajectoryStreamState::kResetAwaitingAck));
  acknowledge_reset(scheduler);
  ASSERT_TRUE(wait_for_stream_state(
      scheduler, elfin3_canfd::TrajectoryStreamState::kStreaming));

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < deadline &&
    scheduler.stream_state() != elfin3_canfd::TrajectoryStreamState::kResetRequired)
  {
    elfin3_canfd::TrajectoryStatus status;
    status.state = elfin3_canfd::TrajectoryStatusState::kRunning;
    status.queue_depth = 0U;
    status.queue_capacity = elfin3_canfd::kExpectedRemoteQueueCapacity;
    status.prefill_target = elfin3_canfd::kExpectedRemotePrefillTarget;
    status.flags = static_cast<std::uint8_t>(
      elfin3_canfd::kExpectedSequenceValid |
      elfin3_canfd::kLastAcceptedSequenceValid |
      elfin3_canfd::kLastExecutedSequenceValid);
    status.last_accepted_sequence = last_sequence.load();
    status.last_executed_sequence = last_sequence.load();
    status.expected_sequence = static_cast<std::uint8_t>(last_sequence.load() + 1U);
    status.generation = 1U;
    status.accepted_count = static_cast<std::uint16_t>(frame_count.load());
    status.executed_count = status.accepted_count;
    scheduler.update_remote_status(status, std::chrono::steady_clock::now());
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  scheduler.stop();

  ASSERT_GT(scheduler.stats().filler_frames, 0U);
  EXPECT_EQ(scheduler.stats().queue_underruns, 1U);
  EXPECT_EQ(
    scheduler.stream_state(), elfin3_canfd::TrajectoryStreamState::kResetRequired);
  EXPECT_GT(scheduler.stats().max_tx_gap_us, 0U);
  std::lock_guard<std::mutex> lock(frames_mutex);
  ASSERT_GT(targets.size(), elfin3_canfd::kCspStartPrefillPointCount);
  EXPECT_EQ(
    targets[elfin3_canfd::kCspStartPrefillPointCount - 1U],
    targets[elfin3_canfd::kCspStartPrefillPointCount]);
}

TEST(CspScheduler, NormalFinishSendsOneHoldAndEntersHolding)
{
  std::atomic<std::size_t> hold_count{0U};
  elfin3_canfd::CspScheduler scheduler(
    [&hold_count](
      std::uint32_t, const std::uint8_t * data, std::size_t, std::string &)
    {
      if (data[2] == elfin3_canfd::kHold) {
        hold_count.fetch_add(1U);
      }
      return true;
    },
    std::chrono::microseconds(elfin3_canfd::kCspPeriodUs), std::chrono::seconds(1),
    elfin3_canfd::kCspValidityMs, 16U);

  ASSERT_TRUE(scheduler.submit_target({0, 0, 0, 0, 0, 0}));
  provide_remote_idle(scheduler);
  scheduler.set_enabled(true);
  ASSERT_TRUE(scheduler.start_trajectory());
  for (std::size_t point = 1U; point < elfin3_canfd::kCspStartPrefillPointCount; ++point) {
    ASSERT_TRUE(scheduler.submit_target(
        {static_cast<std::int32_t>(point), 0, 0, 0, 0, 0}));
  }
  scheduler.start();
  ASSERT_TRUE(wait_for_stream_state(
      scheduler, elfin3_canfd::TrajectoryStreamState::kResetAwaitingAck));
  acknowledge_reset(scheduler);

  const auto streaming_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (scheduler.stream_state() != elfin3_canfd::TrajectoryStreamState::kStreaming &&
    std::chrono::steady_clock::now() < streaming_deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(scheduler.stream_state(), elfin3_canfd::TrajectoryStreamState::kStreaming);
  ASSERT_TRUE(scheduler.finish_trajectory());

  const auto holding_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (scheduler.stream_state() != elfin3_canfd::TrajectoryStreamState::kHolding &&
    std::chrono::steady_clock::now() < holding_deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  scheduler.stop();

  EXPECT_EQ(scheduler.stream_state(), elfin3_canfd::TrajectoryStreamState::kHolding);
  EXPECT_EQ(hold_count.load(), 1U);
}

TEST(CspScheduler, AbortWhileWaitingForResetAckStillSendsHold)
{
  std::atomic<std::size_t> hold_count{0U};
  elfin3_canfd::CspScheduler scheduler(
    [&hold_count](
      std::uint32_t, const std::uint8_t * data, std::size_t, std::string &)
    {
      if (data[2] == elfin3_canfd::kHold) {
        hold_count.fetch_add(1U);
      }
      return true;
    },
    std::chrono::microseconds(elfin3_canfd::kCspPeriodUs), std::chrono::seconds(1),
    elfin3_canfd::kCspValidityMs, 16U);

  ASSERT_TRUE(scheduler.submit_target({0, 0, 0, 0, 0, 0}));
  provide_remote_idle(scheduler);
  scheduler.set_enabled(true);
  ASSERT_TRUE(scheduler.start_trajectory());
  for (std::size_t point = 1U; point < elfin3_canfd::kCspStartPrefillPointCount; ++point) {
    ASSERT_TRUE(scheduler.submit_target(
        {static_cast<std::int32_t>(point), 0, 0, 0, 0, 0}));
  }
  scheduler.start();
  ASSERT_TRUE(wait_for_stream_state(
      scheduler, elfin3_canfd::TrajectoryStreamState::kResetAwaitingAck));
  scheduler.abort_trajectory();
  ASSERT_TRUE(wait_for_stream_state(
      scheduler, elfin3_canfd::TrajectoryStreamState::kResetRequired));
  scheduler.stop();

  EXPECT_EQ(hold_count.load(), 1U);
}

TEST(CspScheduler, RemoteTrajectoryHoldStopsApplyStream)
{
  std::atomic<std::size_t> hold_count{0U};
  elfin3_canfd::CspScheduler scheduler(
    [&hold_count](
      std::uint32_t, const std::uint8_t * data, std::size_t, std::string &)
    {
      if (data[2] == elfin3_canfd::kHold) {
        hold_count.fetch_add(1U);
      }
      return true;
    },
    std::chrono::microseconds(elfin3_canfd::kCspPeriodUs), std::chrono::seconds(1),
    elfin3_canfd::kCspValidityMs, 16U);

  ASSERT_TRUE(scheduler.submit_target({0, 0, 0, 0, 0, 0}));
  provide_remote_idle(scheduler);
  scheduler.set_enabled(true);
  ASSERT_TRUE(scheduler.start_trajectory());
  for (std::size_t point = 1U; point < elfin3_canfd::kCspStartPrefillPointCount; ++point) {
    ASSERT_TRUE(scheduler.submit_target(
        {static_cast<std::int32_t>(point), 0, 0, 0, 0, 0}));
  }
  scheduler.start();
  ASSERT_TRUE(wait_for_stream_state(
      scheduler, elfin3_canfd::TrajectoryStreamState::kResetAwaitingAck));
  acknowledge_reset(scheduler);
  ASSERT_TRUE(wait_for_stream_state(
      scheduler, elfin3_canfd::TrajectoryStreamState::kStreaming));

  elfin3_canfd::TrajectoryStatus remote_hold;
  remote_hold.state = elfin3_canfd::TrajectoryStatusState::kHold;
  remote_hold.queue_capacity = elfin3_canfd::kExpectedRemoteQueueCapacity;
  remote_hold.prefill_target = elfin3_canfd::kExpectedRemotePrefillTarget;
  remote_hold.flags = elfin3_canfd::kTrajectoryResetRequired;
  remote_hold.generation = 1U;
  remote_hold.accepted_count = 1U;
  scheduler.update_remote_status(remote_hold, std::chrono::steady_clock::now());

  ASSERT_TRUE(wait_for_stream_state(
      scheduler, elfin3_canfd::TrajectoryStreamState::kResetRequired));
  scheduler.stop();

  EXPECT_EQ(hold_count.load(), 1U);
  EXPECT_EQ(scheduler.stats().remote_faults, 1U);
}

TEST(CspScheduler, AbnormalAbortSendsHoldAndRequiresExplicitRecovery)
{
  std::atomic<std::size_t> hold_count{0U};
  elfin3_canfd::CspScheduler scheduler(
    [&hold_count](
      std::uint32_t, const std::uint8_t * data, std::size_t, std::string &)
    {
      if (data[2] == elfin3_canfd::kHold) {
        hold_count.fetch_add(1U);
      }
      return true;
    },
    std::chrono::microseconds(elfin3_canfd::kCspPeriodUs), std::chrono::seconds(1),
    elfin3_canfd::kCspValidityMs, 16U);

  ASSERT_TRUE(scheduler.submit_target({0, 0, 0, 0, 0, 0}));
  provide_remote_idle(scheduler);
  scheduler.set_enabled(true);
  ASSERT_TRUE(scheduler.start_trajectory());
  for (std::size_t point = 1U; point < elfin3_canfd::kCspStartPrefillPointCount; ++point) {
    ASSERT_TRUE(scheduler.submit_target(
        {static_cast<std::int32_t>(point), 0, 0, 0, 0, 0}));
  }
  scheduler.start();
  ASSERT_TRUE(wait_for_stream_state(
      scheduler, elfin3_canfd::TrajectoryStreamState::kResetAwaitingAck));
  acknowledge_reset(scheduler);

  const auto streaming_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (scheduler.stream_state() != elfin3_canfd::TrajectoryStreamState::kStreaming &&
    std::chrono::steady_clock::now() < streaming_deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(scheduler.stream_state(), elfin3_canfd::TrajectoryStreamState::kStreaming);
  scheduler.abort_trajectory();

  const auto reset_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (scheduler.stream_state() != elfin3_canfd::TrajectoryStreamState::kResetRequired &&
    std::chrono::steady_clock::now() < reset_deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  scheduler.stop();

  EXPECT_EQ(
    scheduler.stream_state(), elfin3_canfd::TrajectoryStreamState::kResetRequired);
  EXPECT_EQ(hold_count.load(), 1U);
  EXPECT_EQ(
    scheduler.recover_trajectory(), elfin3_canfd::TrajectoryRecoveryResult::kSuccess);
  EXPECT_EQ(
    scheduler.stream_state(), elfin3_canfd::TrajectoryStreamState::kStartPending);
}

}  // namespace
