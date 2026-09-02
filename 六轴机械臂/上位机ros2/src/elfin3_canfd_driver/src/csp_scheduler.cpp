#include "elfin3_canfd_driver/csp_scheduler.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif

#include "elfin3_canfd_driver/encode.hpp"

namespace elfin3_canfd
{
namespace
{

constexpr auto kRemoteStatusTimeout = std::chrono::milliseconds(100);
constexpr auto kResetAckTimeout = std::chrono::milliseconds(50);
// ros2_control produces targets at 400 Hz while this scheduler transmits at
// 500 Hz. Permit short producer gaps and repeat the last target without
// declaring an underrun; the protocol frame validity remains 40 ms.
constexpr auto kLocalPointGrace = std::chrono::milliseconds(12);
constexpr auto kRemoteProgressTimeout = std::chrono::milliseconds(100);

bool remote_allows_reset(
  const TrajectoryStatus & status,
  const std::chrono::steady_clock::time_point received_at,
  const std::chrono::steady_clock::time_point now)
{
  const bool idle_or_hold = status.state == TrajectoryStatusState::kIdle ||
    status.state == TrajectoryStatusState::kHold;
  const auto unsafe_flags = static_cast<std::uint8_t>(kSafetyHoldLatched | kQuickStopLatched);
  const bool queue_contract_matches =
    status.queue_capacity == kExpectedRemoteQueueCapacity &&
    status.prefill_target == kExpectedRemotePrefillTarget;
  return idle_or_hold && queue_contract_matches && (status.flags & unsafe_flags) == 0U &&
         now - received_at <= kRemoteStatusTimeout;
}

}  // namespace

CspScheduler::CspScheduler(
  SendFunction send, const std::chrono::microseconds period,
  const std::chrono::milliseconds target_timeout, const std::uint16_t validity_ms,
  const std::size_t target_queue_capacity, const std::size_t remote_low_watermark,
  const std::size_t remote_high_watermark,
  const std::chrono::microseconds refill_period, const int thread_priority)
: send_(std::move(send)),
  period_(period),
  target_timeout_(target_timeout),
  validity_ms_(validity_ms),
  target_queue_capacity_(target_queue_capacity),
  remote_low_watermark_(remote_low_watermark),
  remote_high_watermark_(remote_high_watermark),
  refill_period_(refill_period),
  thread_priority_(thread_priority)
{
  if (!send_ || period_.count() != kCspPeriodUs || target_timeout_.count() <= 0 ||
    validity_ms_ != kCspValidityMs ||
    target_queue_capacity_ < kCspStartPrefillPointCount ||
    remote_low_watermark_ >= remote_high_watermark_ ||
    remote_high_watermark_ > kExpectedRemoteQueueCapacity - kRemoteQueueReserve ||
    refill_period_.count() <= 0 ||
    refill_period_ >= period_ || thread_priority_ < 0 || thread_priority_ > 99)
  {
    throw std::invalid_argument("invalid CSP scheduler configuration");
  }
}

CspScheduler::~CspScheduler()
{
  stop();
}

/**
 * @brief 启动 CSP 调度线程
 */
void CspScheduler::start()
{
  // 避免重复启动调度线程
  if (running_.exchange(true)) {
    return;
  }

  // 创建后台调度线程
  worker_ = std::thread([this]() {run();});
}

void CspScheduler::stop()
{
  running_.store(false);
  if (worker_.joinable()) {
    worker_.join();
  }
}

/**
 * @brief 提交新的 CSP 关节目标位置
 * @param target_position_counts 各轴目标编码器计数
 * @param valid_axes_mask 有效轴掩码
 * @return 是否提交成功
 */
bool CspScheduler::submit_target(
  const std::array<std::int32_t, kAxisCount> & target_position_counts,
  const std::uint8_t valid_axes_mask)
{
  // 当前协议只接受六轴全部有效的完整目标。
  if (valid_axes_mask != kAllAxesMask) {
    return false;
  }

  // ros2_control 写线程和 CSP 调度线程共享状态及队列，统一加锁保护。
  std::lock_guard<std::mutex> lock(mutex_);

  // 仅在轨迹启动、等待 RESET 确认和流式发送阶段接收新的队列点。
  const bool stream_accepts_points =
    stream_state_ == TrajectoryStreamState::kStartPending ||
    stream_state_ == TrajectoryStreamState::kResetAwaitingAck ||
    stream_state_ == TrajectoryStreamState::kStreaming;
  // 入队前检查本地容量；溢出时拒绝目标且不更新最新目标缓存。
  if (stream_accepts_points && target_queue_.size() >= target_queue_capacity_) {
    ++stats_.queue_overflows;
    return false;
  }

  // 缓存目标及其接收时间；非流式状态下该缓存供下次启动或恢复使用。
  const auto receive_time = std::chrono::steady_clock::now();
  latest_target_ = target_position_counts;
  latest_target_time_ = receive_time;
  target_available_ = true;

  // 当前状态允许接收轨迹点时，将目标加入队列等待调度线程发送。
  if (stream_accepts_points) {
    target_queue_.push_back({target_position_counts, receive_time});
  }

  // 非流式状态下返回 true 仅表示最新目标缓存已成功更新。
  return true;
}

void CspScheduler::set_enabled(const bool enabled)
{
  std::lock_guard<std::mutex> lock(mutex_);
  enabled_ = enabled;
}

void CspScheduler::update_remote_status(
  const TrajectoryStatus & status,
  const std::chrono::steady_clock::time_point received_at)
{
  std::lock_guard<std::mutex> lock(mutex_);
  remote_status_ = status;
  remote_status_time_ = received_at;
  remote_status_available_ = true;
  stats_.remote_queue_depth = status.queue_depth;
  if (status.state == TrajectoryStatusState::kRunning) {
    stats_.minimum_remote_queue_depth =
      std::min(stats_.minimum_remote_queue_depth, static_cast<std::size_t>(status.queue_depth));
  }
  if (accepted_count_tracking_ && status.generation == active_generation_ &&
    (status.flags & kLastAcceptedSequenceValid) != 0U)
  {
    const auto delta = static_cast<std::uint16_t>(
      status.accepted_count - last_remote_accepted_count_);
    if (delta < 0x8000U) {
      apply_points_accepted_ += delta;
      last_remote_accepted_count_ = status.accepted_count;
      if (delta > 0U) {
        last_remote_progress_time_ = received_at;
      }
    }
    if (status.executed_count != last_remote_executed_count_) {
      last_remote_executed_count_ = status.executed_count;
      last_remote_progress_time_ = received_at;
    }
  }
  stats_.unacknowledged_points = apply_points_sent_ > apply_points_accepted_ ?
    static_cast<std::size_t>(apply_points_sent_ - apply_points_accepted_) : 0U;
}

bool CspScheduler::get_remote_status(
  TrajectoryStatus & status,
  std::chrono::steady_clock::time_point & received_at) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!remote_status_available_) {
    return false;
  }
  status = remote_status_;
  received_at = remote_status_time_;
  return true;
}

/**
 * @brief 初始化并启动 CSP 轨迹
 * @return 是否成功进入启动等待状态
 */
bool CspScheduler::start_trajectory()
{
  // 加锁并检查轨迹启动条件
  std::lock_guard<std::mutex> lock(mutex_);
  ++stats_.start_requests;
  if (!enabled_ || stream_state_ == TrajectoryStreamState::kFault) {
    ++stats_.start_rejections;
    return false;
  }

  // 清空队列并重置轨迹统计状态
  target_queue_.clear();
  last_sent_target_available_ = false;
  accepted_count_tracking_ = false;
  apply_points_sent_ = 0U;
  apply_points_accepted_ = 0U;
  stats_.unacknowledged_points = 0U;
  stats_.minimum_remote_queue_depth = 0xffU;
  stats_.last_tx_gap_us = 0U;
  stats_.max_tx_gap_us = 0U;
  successful_tx_available_ = false;
  local_queue_empty_waiting_ = false;
  stats_.start_gate_sample_valid = false;

  // 将最新目标作为轨迹起点
  if (target_available_) {
    target_queue_.push_back({latest_target_, latest_target_time_});
  }

  // 进入轨迹启动等待状态
  stream_state_ = TrajectoryStreamState::kStartPending;
  ++stats_.start_successes;
  return true;
}

TrajectoryRecoveryResult CspScheduler::recover_trajectory()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) {
    return TrajectoryRecoveryResult::kMotionGateClosed;
  }
  if (stream_state_ != TrajectoryStreamState::kResetRequired &&
    stream_state_ != TrajectoryStreamState::kFault)
  {
    return TrajectoryRecoveryResult::kStateNotRecoverable;
  }
  target_queue_.clear();
  last_sent_target_available_ = false;
  accepted_count_tracking_ = false;
  apply_points_sent_ = 0U;
  apply_points_accepted_ = 0U;
  stats_.unacknowledged_points = 0U;
  stats_.minimum_remote_queue_depth = 0xffU;
  stats_.last_tx_gap_us = 0U;
  stats_.max_tx_gap_us = 0U;
  successful_tx_available_ = false;
  local_queue_empty_waiting_ = false;
  stats_.start_gate_sample_valid = false;
  if (target_available_) {
    target_queue_.push_back({latest_target_, latest_target_time_});
  }
  stream_state_ = TrajectoryStreamState::kStartPending;
  return TrajectoryRecoveryResult::kSuccess;
}

void CspScheduler::begin_hold_locked(const TrajectoryStreamState terminal_state)
{
  target_queue_.clear();
  local_queue_empty_waiting_ = false;
  terminal_state_after_hold_ = terminal_state;
  stream_state_ = last_sent_target_available_ ?
    TrajectoryStreamState::kFinishing : terminal_state;
}

bool CspScheduler::finish_trajectory()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (stream_state_ == TrajectoryStreamState::kStartPending) {
    target_queue_.clear();
    stream_state_ = TrajectoryStreamState::kHolding;
    return true;
  }
  if (stream_state_ == TrajectoryStreamState::kResetAwaitingAck ||
    stream_state_ == TrajectoryStreamState::kStreaming)
  {
    begin_hold_locked(TrajectoryStreamState::kHolding);
    return true;
  }
  return stream_state_ == TrajectoryStreamState::kHolding ||
         (stream_state_ == TrajectoryStreamState::kFinishing &&
         terminal_state_after_hold_ == TrajectoryStreamState::kHolding);
}

void CspScheduler::abort_trajectory()
{
  // 与 CSP 调度线程共享轨迹状态和目标队列，所有中止处理均在锁内完成。
  std::lock_guard<std::mutex> lock(mutex_);

  // 故障状态优先级最高；普通中止不能覆盖已经锁存的故障。
  if (stream_state_ == TrajectoryStreamState::kFault) {
    return;
  }

  // 如果已经在发送 HOLD 收尾，只把收尾后的终态升级为需要 RESET；
  // 若终态已经是故障则保持故障，不允许普通中止将其降级。
  if (stream_state_ == TrajectoryStreamState::kFinishing) {
    if (terminal_state_after_hold_ != TrajectoryStreamState::kFault) {
      terminal_state_after_hold_ = TrajectoryStreamState::kResetRequired;
    }
  } else if (stream_state_ == TrajectoryStreamState::kResetAwaitingAck ||
    stream_state_ == TrajectoryStreamState::kStreaming)
  {
    // RESET 已发出或轨迹正在运行时，先以最后发送目标执行 HOLD，
    // 再由调度线程把状态切换到 kResetRequired。
    begin_hold_locked(TrajectoryStreamState::kResetRequired);
  } else {
    // 其余状态尚无必须收尾的在途轨迹，直接丢弃队列并要求下次运动前 RESET。
    target_queue_.clear();
    local_queue_empty_waiting_ = false;
    stream_state_ = TrajectoryStreamState::kResetRequired;
  }
}

void CspScheduler::mark_fault()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (stream_state_ == TrajectoryStreamState::kResetAwaitingAck ||
    stream_state_ == TrajectoryStreamState::kStreaming ||
    stream_state_ == TrajectoryStreamState::kFinishing)
  {
    begin_hold_locked(TrajectoryStreamState::kFault);
  } else {
    target_queue_.clear();
    local_queue_empty_waiting_ = false;
    stream_state_ = TrajectoryStreamState::kFault;
  }
}

bool CspScheduler::is_applying() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stream_state_ == TrajectoryStreamState::kStreaming;
}

TrajectoryStreamState CspScheduler::stream_state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stream_state_;
}

std::size_t CspScheduler::queued_target_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return target_queue_.size();
}

CspSchedulerStats CspScheduler::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

bool CspScheduler::send_command(
  const std::array<std::int32_t, kAxisCount> & positions,
  const std::uint8_t axes_mask, 
  const std::uint8_t flags,
  std::uint8_t * const sent_sequence)
{
  CspCommand command;
  command.sequence = next_sequence_;
  command.valid_axes_mask = axes_mask;
  command.flags = flags;
  command.target_position_counts = positions;
  command.validity_ms = validity_ms_;
  CspCommandFrame frame{};
  std::string error;
  if (!encode_csp_command(command, frame) ||
    !send_(kCspCommandId, frame.data(), frame.size(), error))
  {
    return false;
  }
  if (sent_sequence != nullptr) {
    *sent_sequence = command.sequence;
  }
  next_sequence_ = static_cast<std::uint8_t>(next_sequence_ + 1U);
  return true;
}

/**
 * @brief 运行 CSP 周期调度与轨迹状态机
 */
void CspScheduler::run()
{
  // 定义当前周期待发送的指令类型
  enum class PendingAction
  {
    kNone,
    kResetApply,
    kApply,
    kHold,
  };

  // 配置实时线程优先级并记录结果
#ifdef __linux__
  if (thread_priority_ > 0) {
    sched_param scheduling{};
    scheduling.sched_priority = thread_priority_;
    const int scheduling_error = pthread_setschedparam(
      pthread_self(), SCHED_FIFO, &scheduling);
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.realtime_scheduling_active = scheduling_error == 0;
    stats_.realtime_scheduling_error = scheduling_error;
  }
#else
  if (thread_priority_ > 0) {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.realtime_scheduling_error = -1;
  }
#endif

  // 初始化周期调度时间
  auto next_cycle = std::chrono::steady_clock::now();
  auto previous_cycle_start = next_cycle;
  auto previous_requested_period = period_;
  bool has_previous_cycle = false;

  // 循环执行轨迹调度
  while (running_.load()) {
    const auto now = std::chrono::steady_clock::now();
    auto requested_period = period_;
    std::array<std::int32_t, kAxisCount> target{};
    PendingAction action = PendingAction::kNone;
    bool refill_action = false;
    bool filler_action = false;

    // 在互斥锁保护下运行轨迹状态机
    {
      std::lock_guard<std::mutex> lock(mutex_);

      // 记录唤醒延迟及实际周期抖动，便于诊断调度线程是否满足实时性要求。
      ++stats_.cycles;
      // 计算唤醒延迟
      const std::uint64_t wakeup_lateness_us = now > next_cycle ?
        static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
          now - next_cycle).count()) : 0U;
      stats_.last_wakeup_lateness_us = wakeup_lateness_us;
      stats_.max_wakeup_lateness_us =
        std::max(stats_.max_wakeup_lateness_us, wakeup_lateness_us);
      if (has_previous_cycle) {
        const auto interval_us = std::chrono::duration_cast<std::chrono::microseconds>(
          now - previous_cycle_start).count();
        const std::uint64_t unsigned_interval_us = interval_us > 0 ?
          static_cast<std::uint64_t>(interval_us) : 0U;
        const std::uint64_t period_us =
          static_cast<std::uint64_t>(previous_requested_period.count());
        const std::uint64_t jitter_us = unsigned_interval_us >= period_us ?
          unsigned_interval_us - period_us : period_us - unsigned_interval_us;
        stats_.last_cycle_interval_us = unsigned_interval_us;
        stats_.last_cycle_jitter_us = jitter_us;
        stats_.max_cycle_jitter_us = std::max(stats_.max_cycle_jitter_us, jitter_us);
      }
      // 只有队首目标存在且未超时，才允许将其选为本周期发送目标。
      const bool queued_point_available = !target_queue_.empty();
      const bool queued_point_fresh = queued_point_available &&
        now - target_queue_.front().submit_time <= target_timeout_;

      // 启动阶段：等待本地预填达到要求且远端允许 RESET，再发送首个 RESET+APPLY。
      if (stream_state_ == TrajectoryStreamState::kStartPending) {
        const bool prefill_ready =
          target_queue_.size() >= kCspStartPrefillPointCount;
        const bool remote_ready = remote_status_available_ && remote_allows_reset(
          remote_status_, remote_status_time_, now);

        ++stats_.start_pending_cycles;
        stats_.start_gate_sample_valid = true;
        stats_.start_last_enabled = enabled_;
        stats_.start_last_prefill_ready = prefill_ready;
        stats_.start_last_point_available = queued_point_available;
        stats_.start_last_point_fresh = queued_point_fresh;
        stats_.start_last_remote_ready = remote_ready;
        stats_.start_last_local_queue_depth = target_queue_.size();
        if (!enabled_) {
          ++stats_.start_wait_enabled_cycles;
        }
        if (!prefill_ready) {
          ++stats_.start_wait_prefill_cycles;
        }
        if (!queued_point_fresh) {
          ++stats_.start_wait_fresh_point_cycles;
        }
        if (!remote_ready) {
          ++stats_.start_wait_remote_cycles;
        }

        if (enabled_ && prefill_ready && queued_point_fresh && remote_ready) {
          reset_generation_before_ = remote_status_.generation;
          target = target_queue_.front().positions;
          target_queue_.pop_front();
          action = PendingAction::kResetApply;
        } 
        else if (enabled_ && queued_point_available && !queued_point_fresh) {
          target_queue_.clear();
          ++stats_.queue_underruns;
          stream_state_ = TrajectoryStreamState::kResetRequired;
        }
      } 
      // RESET 确认阶段：根据远端 0x204 的代次和序号确认首帧已经被接受。
      else if (stream_state_ == TrajectoryStreamState::kResetAwaitingAck) {
        const auto required_flags = static_cast<std::uint8_t>(
          kExpectedSequenceValid | kLastAcceptedSequenceValid);
        const auto unsafe_flags = static_cast<std::uint8_t>(
          kTrajectoryResetRequired | kSafetyHoldLatched | kQuickStopLatched);
        const bool active_state = remote_status_.state == TrajectoryStatusState::kPrefill ||
          remote_status_.state == TrajectoryStatusState::kRunning;
        const bool reset_accepted = remote_status_available_ &&
          now - remote_status_time_ <= kRemoteStatusTimeout && 
          active_state &&
          (remote_status_.flags & required_flags) == required_flags &&
          (remote_status_.flags & unsafe_flags) == 0U &&
          remote_status_.generation != reset_generation_before_ &&
          remote_status_.last_accepted_sequence == pending_reset_sequence_ &&
          remote_status_.expected_sequence ==
          static_cast<std::uint8_t>(pending_reset_sequence_ + 1U);
        const bool reset_safety_fault = remote_status_available_ &&
          now - remote_status_time_ <= kRemoteStatusTimeout &&
          (remote_status_.flags & static_cast<std::uint8_t>(
          kSafetyHoldLatched | kQuickStopLatched)) != 0U;

        if (!enabled_ || reset_safety_fault) {
          if (reset_safety_fault) {
            ++stats_.remote_faults;
          }
          begin_hold_locked(reset_safety_fault ?
            TrajectoryStreamState::kFault : TrajectoryStreamState::kResetRequired);
          if (last_sent_target_available_) {
            target = last_sent_target_;
            action = PendingAction::kHold;
          }
        } 
        else if (reset_accepted) {
          active_generation_ = remote_status_.generation;
          last_remote_accepted_count_ = remote_status_.accepted_count;
          last_remote_executed_count_ = remote_status_.executed_count;
          active_rejected_count_ = remote_status_.rejected_count;
          active_underrun_count_ = remote_status_.underrun_count;
          active_overflow_count_ = remote_status_.overflow_count;
          active_expired_count_ = remote_status_.expired_count;
          last_remote_progress_time_ = remote_status_time_;
          apply_points_sent_ = 0U;
          apply_points_accepted_ = 0U;
          accepted_count_tracking_ = true;
          stats_.unacknowledged_points = 0U;
          stream_state_ = TrajectoryStreamState::kStreaming;
        } 
        else if (now - reset_sent_time_ > kResetAckTimeout) {
          target_queue_.clear();
          terminal_state_after_hold_ = TrajectoryStreamState::kResetRequired;
          stream_state_ = TrajectoryStreamState::kFinishing;
          if (last_sent_target_available_) {
            target = last_sent_target_;
            action = PendingAction::kHold;
          }
        }
      } 
      // 流式阶段：依据远端队列水位和未确认点数控制后续 APPLY 的发送节奏。
      else if (stream_state_ == TrajectoryStreamState::kStreaming) {
        if (queued_point_available) {
          local_queue_empty_waiting_ = false;
        }
        //主机已经发送、但远端尚未通过 0x204 确认的 APPLY 数量
        const std::size_t unacknowledged = apply_points_sent_ > apply_points_accepted_ ?
          static_cast<std::size_t>(apply_points_sent_ - apply_points_accepted_) : 0U;

        //远端实际占用的队列深度（包括未确认的 APPLY）
        const std::size_t occupied_or_in_flight =
          static_cast<std::size_t>(remote_status_.queue_depth) + unacknowledged;

        // 远端队列容量减去保留点数后的可用信用额度
        const std::size_t credit_limit = remote_status_.queue_capacity > kRemoteQueueReserve ?
          static_cast<std::size_t>(remote_status_.queue_capacity) - kRemoteQueueReserve : 0U;

        // 远端队列高低水位的有效值，避免高水位小于低水位或超过信用额度。
        const std::size_t effective_high_watermark =
          std::min(remote_high_watermark_, credit_limit);
        const std::size_t effective_low_watermark = effective_high_watermark > 0U ?
          std::min(remote_low_watermark_, effective_high_watermark - 1U) : 0U;

        
        const auto unsafe_flags = static_cast<std::uint8_t>(
          kTrajectoryResetRequired | kSafetyHoldLatched | kQuickStopLatched);
        const bool remote_active = remote_status_.state == TrajectoryStatusState::kPrefill ||
          remote_status_.state == TrajectoryStatusState::kRunning;
        const bool remote_fresh = remote_status_available_ &&
          now - remote_status_time_ <= kRemoteStatusTimeout;
        const bool remote_event_fault = remote_status_.rejected_count != active_rejected_count_ ||
          remote_status_.underrun_count != active_underrun_count_ ||
          remote_status_.overflow_count != active_overflow_count_ ||
          remote_status_.expired_count != active_expired_count_;
        const bool remote_progress_timed_out = unacknowledged > 0U &&
          now - last_remote_progress_time_ > kRemoteProgressTimeout;
        const bool remote_fault = !remote_fresh || !remote_active ||
          (remote_status_.flags & unsafe_flags) != 0U ||
          remote_status_.generation != active_generation_ ||
          remote_status_.queue_capacity != kExpectedRemoteQueueCapacity ||
          remote_status_.prefill_target != kExpectedRemotePrefillTarget ||
          remote_event_fault ||
          remote_progress_timed_out;
        const bool credit_available = remote_status_available_ &&
          now - remote_status_time_ <= kRemoteStatusTimeout && 
          remote_active &&
          (remote_status_.flags & unsafe_flags) == 0U &&
          remote_status_.generation == active_generation_ &&
          remote_status_.queue_capacity > kRemoteQueueReserve &&
          occupied_or_in_flight < credit_limit;
        const bool below_high_watermark =
          effective_high_watermark > 0U && occupied_or_in_flight < effective_high_watermark;
        // 远端状态异常或运动门关闭时停止补点，并用最后目标发送 HOLD 收尾。
        if (remote_fault || !enabled_) {
          if (remote_fault) {
            ++stats_.remote_faults;
          }
          const bool safety_fault = remote_fresh &&
            (remote_status_.flags & static_cast<std::uint8_t>(
            kSafetyHoldLatched | kQuickStopLatched)) != 0U;
          begin_hold_locked(safety_fault ?
            TrajectoryStreamState::kFault : TrajectoryStreamState::kResetRequired);
          if (last_sent_target_available_) {
            target = last_sent_target_;
            action = PendingAction::kHold;
          }
        } 
        else {
          // 远端队列低于低水位时切换到更短的补点周期。
          if (occupied_or_in_flight <= effective_low_watermark) {
            requested_period = refill_period_;
            refill_action = true;
            ++stats_.refill_cycles;
          }
          stats_.unacknowledged_points = unacknowledged;
          if (queued_point_fresh && credit_available && below_high_watermark) {
            target = target_queue_.front().positions;
            target_queue_.pop_front();
            action = PendingAction::kApply;
          } 
          else if (queued_point_fresh && credit_available && !below_high_watermark) {
            ++stats_.high_watermark_stalls;
          } 
          else if (queued_point_fresh && !credit_available) {
            ++stats_.credit_stalls;
          } 
          else {
            // 本地短暂缺点时重复最后目标；超过宽限期则 HOLD 并要求重新 RESET。
            bool within_local_point_grace = false;
            if (!queued_point_available) {
              if (!local_queue_empty_waiting_) {
                local_queue_empty_since_ = now;
                local_queue_empty_waiting_ = true;
              }
              within_local_point_grace =
                now - local_queue_empty_since_ <= kLocalPointGrace;
              if (within_local_point_grace) {
                ++stats_.local_point_wait_cycles;
              }
            }
            const bool latest_target_fresh = target_available_ &&
              now - latest_target_time_ <= target_timeout_;
            if (within_local_point_grace && last_sent_target_available_ &&
              latest_target_fresh && credit_available && below_high_watermark)
            {
              target = last_sent_target_;
              action = PendingAction::kApply;
              filler_action = true;
            } 
            else if (!within_local_point_grace) {
              target_queue_.clear();
              local_queue_empty_waiting_ = false;
              ++stats_.queue_underruns;
              begin_hold_locked(TrajectoryStreamState::kResetRequired);
              if (last_sent_target_available_) {
                target = last_sent_target_;
                action = PendingAction::kHold;
              }
            }
          }
        }
      // 收尾阶段：发送最后目标的 HOLD，成功后进入预先指定的终态。
      } else if (stream_state_ == TrajectoryStreamState::kFinishing) {
        if (last_sent_target_available_) {
          target = last_sent_target_;
          action = PendingAction::kHold;
        } else {
          stream_state_ = terminal_state_after_hold_;
        }
      }
    }

    // 状态锁已经释放；在锁外编码并发送 0x180，避免阻塞 submit_target() 入队。
    if (action != PendingAction::kNone) {
      std::uint8_t flags = kApply;
      if (action == PendingAction::kResetApply) {
        flags = kResetApply;
      } 
      else if (action == PendingAction::kHold) {
        flags = kHold;
      }

      // RESET+APPLY、APPLY、HOLD 最终都由 send_command() 编码为 0x180 帧。
      std::uint8_t sent_sequence = 0U;
      const bool sent = send_command(target, kAllAxesMask, flags, &sent_sequence);
      const auto sent_at = std::chrono::steady_clock::now();
      // 发送完成后重新加锁，更新统计量和后续状态转换。
      std::lock_guard<std::mutex> lock(mutex_);
      if (!sent) {
        ++stats_.transmit_errors;
        stream_state_ = TrajectoryStreamState::kFault;
      } 
      else {
        if (successful_tx_available_) {
          const auto gap_us = std::chrono::duration_cast<std::chrono::microseconds>(
            sent_at - last_successful_tx_time_).count();
          stats_.last_tx_gap_us = gap_us > 0 ? static_cast<std::uint64_t>(gap_us) : 0U;
          stats_.max_tx_gap_us = std::max(stats_.max_tx_gap_us, stats_.last_tx_gap_us);
        }
        last_successful_tx_time_ = sent_at;
        successful_tx_available_ = true;
      }

      if (sent && action == PendingAction::kHold) {
        ++stats_.hold_frames;
        if (stream_state_ == TrajectoryStreamState::kFinishing) {
          stream_state_ = terminal_state_after_hold_;
        }
      } 
      else if (sent) {
        ++stats_.apply_frames;
        if (refill_action) {
          ++stats_.refill_transmissions;
        }
        if (filler_action) {
          ++stats_.filler_frames;
        }
        last_sent_target_ = target;
        last_sent_target_available_ = true;

        if (action == PendingAction::kResetApply &&
          stream_state_ == TrajectoryStreamState::kStartPending)
        {
          ++stats_.reset_apply_frames;
          pending_reset_sequence_ = sent_sequence;
          reset_sent_time_ = std::chrono::steady_clock::now();
          stream_state_ = TrajectoryStreamState::kResetAwaitingAck;
        } 
        else if (action == PendingAction::kApply) {
          ++apply_points_sent_;
          stats_.unacknowledged_points = apply_points_sent_ > apply_points_accepted_ ?
            static_cast<std::size_t>(apply_points_sent_ - apply_points_accepted_) : 0U;
        }
      }
    }

    // 采用绝对时间等待下一周期；若本周期超期则重置基准，避免连续追赶发送。
    previous_cycle_start = now;
    previous_requested_period = requested_period;
    has_previous_cycle = true;

    next_cycle += requested_period;
    const auto before_sleep = std::chrono::steady_clock::now();
    if (before_sleep > next_cycle) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.overruns;
      next_cycle = before_sleep + requested_period;
    }
    std::this_thread::sleep_until(next_cycle);
  }
}

}  // namespace elfin3_canfd
