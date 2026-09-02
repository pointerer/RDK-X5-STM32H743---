#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "elfin3_canfd_driver/protocol.hpp"

namespace elfin3_canfd
{

struct CspSchedulerStats
{
  std::uint64_t apply_frames{0};
  std::uint64_t hold_frames{0};
  std::uint64_t transmit_errors{0};
  std::uint64_t overruns{0};
  std::uint64_t queue_overflows{0};
  std::uint64_t queue_underruns{0};
  std::uint64_t credit_stalls{0};
  std::uint64_t high_watermark_stalls{0};
  std::uint64_t refill_cycles{0};
  std::uint64_t refill_transmissions{0};
  std::uint64_t filler_frames{0};
  std::uint64_t local_point_wait_cycles{0};
  std::uint64_t remote_faults{0};
  std::size_t remote_queue_depth{0};
  std::size_t minimum_remote_queue_depth{0xffU};
  std::size_t unacknowledged_points{0};
  std::uint64_t cycles{0};
  std::uint64_t last_tx_gap_us{0};
  std::uint64_t max_tx_gap_us{0};
  std::uint64_t last_cycle_interval_us{0};
  std::uint64_t last_cycle_jitter_us{0};
  std::uint64_t max_cycle_jitter_us{0};
  std::uint64_t last_wakeup_lateness_us{0};
  std::uint64_t max_wakeup_lateness_us{0};
  bool realtime_scheduling_active{false};
  int realtime_scheduling_error{0};
  std::uint64_t start_requests{0};
  std::uint64_t start_successes{0};
  std::uint64_t start_rejections{0};
  std::uint64_t start_pending_cycles{0};
  std::uint64_t start_wait_enabled_cycles{0};
  std::uint64_t start_wait_prefill_cycles{0};
  std::uint64_t start_wait_fresh_point_cycles{0};
  std::uint64_t start_wait_remote_cycles{0};
  std::uint64_t reset_apply_frames{0};
  bool start_gate_sample_valid{false};
  bool start_last_enabled{false};
  bool start_last_prefill_ready{false};
  bool start_last_point_available{false};
  bool start_last_point_fresh{false};
  bool start_last_remote_ready{false};
  std::size_t start_last_local_queue_depth{0};
};

inline constexpr std::size_t kDefaultCspTargetQueueCapacity = 100U;
inline constexpr std::uint8_t kExpectedRemoteQueueCapacity = 15U;
inline constexpr std::uint8_t kExpectedRemotePrefillTarget = 8U;
inline constexpr std::size_t kRemoteQueueReserve = 2U;
inline constexpr std::size_t kDefaultRemoteLowWatermark = 10U;
inline constexpr std::size_t kDefaultRemoteHighWatermark = 13U;
inline constexpr std::int64_t kDefaultCspRefillPeriodUs = 1000;
inline constexpr int kDefaultCspThreadPriority = 60;
static_assert(
  kDefaultRemoteHighWatermark + kRemoteQueueReserve <= kExpectedRemoteQueueCapacity,
  "remote high watermark must preserve the configured overflow reserve");

enum class TrajectoryStreamState : std::uint8_t
{
  kIdle,               // 空闲，尚未启动轨迹流
  kStartPending,       // 已请求启动，等待预填目标和远端 RESET 条件
  kResetAwaitingAck,   // 已发送 RESET+APPLY，等待下位机确认
  kStreaming,          // RESET 已确认，正在连续发送 APPLY 轨迹点
  kFinishing,          // 正在以最后目标发送 HOLD，等待结束轨迹流
  kHolding,            // 轨迹已结束，保持最后目标位置
  kResetRequired,      // 轨迹已中止，恢复运动前必须重新执行 RESET
  kFault,              // 调度或远端发生故障，需要显式恢复
};

enum class TrajectoryRecoveryResult : std::uint8_t
{
  kSuccess,
  kMotionGateClosed,
  kStateNotRecoverable,
};

class CspScheduler
{
public:
  using SendFunction = std::function<bool(
      std::uint32_t, const std::uint8_t *, std::size_t, std::string &)>;

  CspScheduler(
    SendFunction send, std::chrono::microseconds period,
    std::chrono::milliseconds target_timeout, std::uint16_t validity_ms,
    std::size_t target_queue_capacity = kDefaultCspTargetQueueCapacity,
    std::size_t remote_low_watermark = kDefaultRemoteLowWatermark,
    std::size_t remote_high_watermark = kDefaultRemoteHighWatermark,
    std::chrono::microseconds refill_period =
    std::chrono::microseconds(kDefaultCspRefillPeriodUs),
    int thread_priority = kDefaultCspThreadPriority);
  ~CspScheduler();

  CspScheduler(const CspScheduler &) = delete;
  CspScheduler & operator=(const CspScheduler &) = delete;

  void start();
  void stop();
  bool submit_target(
    const std::array<std::int32_t, kAxisCount> & target_position_counts,
    std::uint8_t valid_axes_mask = kAllAxesMask);
  void set_enabled(bool enabled);
  void update_remote_status(
    const TrajectoryStatus & status, std::chrono::steady_clock::time_point received_at);
  bool get_remote_status(
    TrajectoryStatus & status, std::chrono::steady_clock::time_point & received_at) const;

  bool start_trajectory();
  TrajectoryRecoveryResult recover_trajectory();
  bool finish_trajectory();
  void abort_trajectory();
  void mark_fault();

  bool is_applying() const;
  TrajectoryStreamState stream_state() const;
  std::size_t queued_target_count() const;
  CspSchedulerStats stats() const;

private:
  void begin_hold_locked(TrajectoryStreamState terminal_state);
  void run();
  bool send_command(
    const std::array<std::int32_t, kAxisCount> & positions,
    std::uint8_t axes_mask, std::uint8_t flags, std::uint8_t * sent_sequence = nullptr);

  SendFunction send_;
  const std::chrono::microseconds period_;
  const std::chrono::milliseconds target_timeout_;
  const std::uint16_t validity_ms_;
  const std::size_t target_queue_capacity_;
  const std::size_t remote_low_watermark_;
  const std::size_t remote_high_watermark_;
  const std::chrono::microseconds refill_period_;
  const int thread_priority_;

  mutable std::mutex mutex_;
  struct TargetPoint
  {
    std::array<std::int32_t, kAxisCount> positions{};
    std::chrono::steady_clock::time_point submit_time{};
  };
  std::deque<TargetPoint> target_queue_;
  std::array<std::int32_t, kAxisCount> latest_target_{};
  std::chrono::steady_clock::time_point latest_target_time_{};
  std::array<std::int32_t, kAxisCount> last_sent_target_{};
  bool target_available_{false};
  bool last_sent_target_available_{false};
  bool enabled_{false};
  TrajectoryStatus remote_status_{};
  std::chrono::steady_clock::time_point remote_status_time_{};
  bool remote_status_available_{false};
  std::uint8_t pending_reset_sequence_{0};
  std::uint16_t reset_generation_before_{0};
  std::chrono::steady_clock::time_point reset_sent_time_{};
  std::uint16_t active_generation_{0};
  std::uint16_t last_remote_accepted_count_{0};
  std::uint16_t last_remote_executed_count_{0};
  std::uint16_t active_rejected_count_{0};
  std::uint16_t active_underrun_count_{0};
  std::uint16_t active_overflow_count_{0};
  std::uint16_t active_expired_count_{0};
  std::chrono::steady_clock::time_point last_remote_progress_time_{};
  std::uint64_t apply_points_sent_{0};
  std::uint64_t apply_points_accepted_{0};
  bool accepted_count_tracking_{false};
  std::chrono::steady_clock::time_point local_queue_empty_since_{};
  bool local_queue_empty_waiting_{false};
  std::chrono::steady_clock::time_point last_successful_tx_time_{};
  bool successful_tx_available_{false};
  TrajectoryStreamState stream_state_{TrajectoryStreamState::kIdle};
  TrajectoryStreamState terminal_state_after_hold_{TrajectoryStreamState::kHolding};
  CspSchedulerStats stats_{};

  std::atomic_bool running_{false};

  // 独立 CSP 调度工作线程：执行 run()，负责状态机推进和 0x180 帧发送。
  std::thread worker_;
  
  std::uint8_t next_sequence_{0};
};

}  // namespace elfin3_canfd
