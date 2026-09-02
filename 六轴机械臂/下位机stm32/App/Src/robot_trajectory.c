#include "robot_trajectory.h"

#include "stm32h7xx.h"

#include <string.h>

static ROBOT_TrajectoryContext robot_trajectory_context;

/* 调用时必须已进入与轨迹上下文共用的PRIMASK临界区。 */
static void ROBOT_TrajectoryClearStagedLocked(void)
{
  robot_trajectory_context.staged_valid = 0U;
  robot_trajectory_context.staged_sequence = 0U;
  robot_trajectory_context.staged_generation = 0U;
}

/**
 * @brief  清空轨迹缓冲区并初始化为等待轨迹重置状态
 *
 * @return
 * 无
 *
 * @warning
 * 该函数不使用临界区，只能在调度器启动前或轨迹生产者、消费者均已停止时调用，
 * 禁止与 ROBOT_TrajectoryPush()、ROBOT_TrajectoryTake() 等接口并发执行。调用后所有
 * 已排队轨迹点及序号状态都会丢失，状态置为 ROBOT_TRAJECTORY_IDLE 且要求重置；
 * 新轨迹必须先通过 ROBOT_TrajectoryResetAndPush() 提交首个轨迹点
 */
void ROBOT_TrajectoryInit(void)
{
  memset(&robot_trajectory_context, 0, sizeof(robot_trajectory_context));
  robot_trajectory_context.state = ROBOT_TRAJECTORY_IDLE;
  robot_trajectory_context.reset_required = 1U;
}

/**
 * @brief  清空轨迹队列、锁存首个非零原因并切换到等待重建的HOLD状态
 *
 * @param[in] hold_reason 0x204定义的HOLD原因；0表示不补充原因
 *
 * @return
 * 无
 *
 * @warning
 * 调用前必须完成 ROBOT_TrajectoryInit()，且禁止与不受临界区保护的初始化操作并发执行。
 * 本函数会在短临界区内丢弃全部已排队轨迹点并使原序列失效，后续首个轨迹点必须通过
 * ROBOT_TrajectoryResetAndPush() 提交；已有非零原因不会被覆盖。它只修改软件轨迹状态，
 * 不会直接向电机下发保持命令
 */
void ROBOT_TrajectoryRequireResetWithReason(uint8_t hold_reason)
{
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();

  if ((robot_trajectory_context.hold_reason ==
       ROBOT_TRAJECTORY_HOLD_REASON_NONE) &&
      (hold_reason != ROBOT_TRAJECTORY_HOLD_REASON_NONE))
  {
    robot_trajectory_context.hold_reason = hold_reason;
  }

  robot_trajectory_context.state = ROBOT_TRAJECTORY_HOLD;
  robot_trajectory_context.reset_required = 1U;
  robot_trajectory_context.sequence_valid = 0U;
  robot_trajectory_context.expected_sequence = 0U;
  robot_trajectory_context.head = 0U;
  robot_trajectory_context.tail = 0U;
  robot_trajectory_context.count = 0U;
  robot_trajectory_context.empty_grace_cycles = 0U;
  ROBOT_TrajectoryClearStagedLocked();

  __DMB();
  __set_PRIMASK(primask);
}

void ROBOT_TrajectoryRequireReset(void)
{
  ROBOT_TrajectoryRequireResetWithReason(
    ROBOT_TRAJECTORY_HOLD_REASON_NONE);
}

uint8_t ROBOT_TrajectoryIsHold(void)
{
  uint8_t is_hold;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  __DMB();
  is_hold = (robot_trajectory_context.state == ROBOT_TRAJECTORY_HOLD) ?
            1U : 0U;
  __set_PRIMASK(primask);

  return is_hold;
}

/**
 * @brief  原子读取轨迹运行状态及0x204所需的观测数据
 *
 * @param[out] snapshot 成功时写入一致的轨迹状态快照
 *
 * @return
 * 成功返回1；snapshot为空返回0
 *
 * @warning
 * 本函数只复制标量状态，不复制轨迹点环形缓冲区，也不修改任何轨迹状态。
 * 后续新增的观测字段写入操作必须使用同一PRIMASK临界区，才能保持快照一致性。
 */
uint8_t ROBOT_TrajectoryGetSnapshot(
  ROBOT_TrajectorySnapshot *snapshot)
{
  ROBOT_TrajectorySnapshot local_snapshot = {ROBOT_TRAJECTORY_IDLE};
  uint32_t primask;

  if (snapshot == 0)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  __DMB();

  local_snapshot.state = robot_trajectory_context.state;
  local_snapshot.reset_required = robot_trajectory_context.reset_required;
  local_snapshot.expected_sequence_valid =
    robot_trajectory_context.sequence_valid;
  local_snapshot.expected_sequence =
    robot_trajectory_context.expected_sequence;
  local_snapshot.queue_depth = robot_trajectory_context.count;
  local_snapshot.last_received_valid =
    robot_trajectory_context.last_received_valid;
  local_snapshot.last_received_sequence =
    robot_trajectory_context.last_received_sequence;
  local_snapshot.last_accepted_valid =
    robot_trajectory_context.last_accepted_valid;
  local_snapshot.last_accepted_sequence =
    robot_trajectory_context.last_accepted_sequence;
  local_snapshot.last_executed_valid =
    robot_trajectory_context.last_executed_valid;
  local_snapshot.last_executed_sequence =
    robot_trajectory_context.last_executed_sequence;
  local_snapshot.last_rejected_valid =
    robot_trajectory_context.last_rejected_valid;
  local_snapshot.last_rejected_sequence =
    robot_trajectory_context.last_rejected_sequence;
  local_snapshot.reject_reason = robot_trajectory_context.reject_reason;
  local_snapshot.hold_reason = robot_trajectory_context.hold_reason;
  local_snapshot.generation = robot_trajectory_context.generation;
  local_snapshot.last_executed_ecat_cycle =
    robot_trajectory_context.last_executed_ecat_cycle;
  local_snapshot.accepted_count = robot_trajectory_context.accepted_count;
  local_snapshot.executed_count = robot_trajectory_context.executed_count;
  local_snapshot.rejected_count = robot_trajectory_context.rejected_count;
  local_snapshot.underrun_count = robot_trajectory_context.underrun_count;
  local_snapshot.empty_grace_recovery_count =
    robot_trajectory_context.empty_grace_recovery_count;
  local_snapshot.overflow_count = robot_trajectory_context.overflow_count;
  local_snapshot.expired_count = robot_trajectory_context.expired_count;

  __DMB();
  __set_PRIMASK(primask);

  *snapshot = local_snapshot;
  return 1U;
}

/**
 * @brief  记录一帧已通过格式、长度及CRC校验的0x180序号
 *
 * @param[in] sequence 已通过基础校验的报文序号
 *
 * @return
 * 无；本函数只更新0x204观测字段，不改变轨迹状态和队列
 */
void ROBOT_TrajectoryRecordReceived(uint8_t sequence)
{
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  __DMB();

  robot_trajectory_context.last_received_sequence = sequence;
  robot_trajectory_context.last_received_valid = 1U;

  __DMB();
  __set_PRIMASK(primask);
}

/**
 * @brief  记录最近一次被拒绝的0x180及拒绝原因
 *
 * @param[in] sequence       报文序号；sequence_valid为0时忽略
 * @param[in] sequence_valid 非0表示序号已经通过基础校验、可以信任
 * @param[in] reject_reason  0x204拒绝原因枚举值
 *
 * @return
 * 无；本函数只更新拒绝观测数据和累计计数，不触发HOLD或RESET
 */
void ROBOT_TrajectoryRecordRejected(
  uint8_t sequence,
  uint8_t sequence_valid,
  uint8_t reject_reason)
{
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  __DMB();

  robot_trajectory_context.last_rejected_valid =
    (sequence_valid != 0U) ? 1U : 0U;
  robot_trajectory_context.last_rejected_sequence =
    (sequence_valid != 0U) ? sequence : 0U;
  robot_trajectory_context.reject_reason = reject_reason;
  robot_trajectory_context.rejected_count++;

  __DMB();
  __set_PRIMASK(primask);
}

/**
 * @brief  丢弃原轨迹并以指定首点重建轨迹序列
 *
 * @param[in] point 新轨迹的首个轨迹点
 *
 * @return
 * 成功重建并压入首点返回1，point 为空返回0
 *
 * @warning
 * 调用前必须完成 ROBOT_TrajectoryInit()，point 的目标位置、有效期和时间戳必须由调用方
 * 预先校验，本函数仅检查空指针。调用会在短临界区内无条件丢弃全部已排队轨迹点，
 * 将下一期望序号设为 point->sequence + 1 并进入 ROBOT_TRAJECTORY_PREFILL；首点不会立即
 * 输出，队列达到 ROBOT_TRAJECTORY_PREFILL_COUNT 后才开始运行。禁止与初始化操作并发调用
 */
uint8_t ROBOT_TrajectoryResetAndPush(const ROBOT_TrajectoryPoint *point)
{
  uint32_t primask;

  if (point == 0)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  __DMB();

  robot_trajectory_context.head = 0U;
  robot_trajectory_context.tail = 1U;
  robot_trajectory_context.count = 1U;
  robot_trajectory_context.empty_grace_cycles = 0U;
  robot_trajectory_context.points[0] = *point;
  robot_trajectory_context.expected_sequence =
    (uint8_t)(point->sequence + 1U);
  robot_trajectory_context.sequence_valid = 1U;
  robot_trajectory_context.reset_required = 0U;

  /* 合法RESET原子地建立新轨迹代次，并记录该首点已接受。 */
  robot_trajectory_context.generation++;
  robot_trajectory_context.last_accepted_valid = 1U;
  robot_trajectory_context.last_accepted_sequence = point->sequence;
  robot_trajectory_context.accepted_count++;

  /* 上一代且无独立代次标签的瞬时观测信息不再有效。 */
  robot_trajectory_context.hold_reason = 0U;
  robot_trajectory_context.last_rejected_valid = 0U;
  robot_trajectory_context.last_rejected_sequence = 0U;
  robot_trajectory_context.reject_reason = 0U;
  robot_trajectory_context.last_executed_valid = 0U;
  robot_trajectory_context.last_executed_sequence = 0U;
  robot_trajectory_context.last_executed_ecat_cycle = 0U;
  ROBOT_TrajectoryClearStagedLocked();

  robot_trajectory_context.state = ROBOT_TRAJECTORY_PREFILL;

  __DMB();
  __set_PRIMASK(primask);
  return 1U;
}

/**
 * @brief  按期望序号向轨迹环形队列追加一个轨迹点
 *
 * @param[in] point 待追加的轨迹点
 *
 * @return
 * 追加成功返回 ROBOT_TRAJECTORY_PUSH_OK；point 为空返回 ROBOT_TRAJECTORY_PUSH_INVALID；
 * 轨迹尚未重建或当前状态不允许入队时返回 ROBOT_TRAJECTORY_PUSH_RESET_REQUIRED；序号
 * 不连续返回 ROBOT_TRAJECTORY_PUSH_SEQUENCE_ERROR；队列已满返回 ROBOT_TRAJECTORY_PUSH_FULL
 *
 * @warning
 * 调用前必须完成 ROBOT_TrajectoryInit()，并先通过 ROBOT_TrajectoryResetAndPush() 建立首点
 * 和期望序号；point 的目标位置、有效期和时间戳必须由调用方校验。序号错误或队列已满时，
 * 本函数会在短临界区内清空全部轨迹点、切换到 ROBOT_TRAJECTORY_HOLD 并要求重新建轨迹，
 * 而非保留原队列。禁止与不受临界区保护的初始化操作并发调用
 */
ROBOT_TrajectoryPushResult ROBOT_TrajectoryPush(
  const ROBOT_TrajectoryPoint *point)
{
  ROBOT_TrajectoryPushResult result;
  uint32_t primask;

  if (point == 0)
  {
    return ROBOT_TRAJECTORY_PUSH_INVALID;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  if ((robot_trajectory_context.reset_required != 0U) ||
      (robot_trajectory_context.sequence_valid == 0U) ||
      ((robot_trajectory_context.state != ROBOT_TRAJECTORY_PREFILL) &&
       (robot_trajectory_context.state != ROBOT_TRAJECTORY_RUNNING)))
  {
    result = ROBOT_TRAJECTORY_PUSH_RESET_REQUIRED;
  }
  else if (point->sequence != robot_trajectory_context.expected_sequence)
  {
    if (robot_trajectory_context.hold_reason ==
        ROBOT_TRAJECTORY_HOLD_REASON_NONE)
    {
      robot_trajectory_context.hold_reason =
        ROBOT_TRAJECTORY_HOLD_REASON_SEQUENCE_ERROR;
    }
    robot_trajectory_context.state = ROBOT_TRAJECTORY_HOLD;
    robot_trajectory_context.reset_required = 1U;
    robot_trajectory_context.sequence_valid = 0U;
    robot_trajectory_context.expected_sequence = 0U;
    robot_trajectory_context.head = 0U;
    robot_trajectory_context.tail = 0U;
    robot_trajectory_context.count = 0U;
    robot_trajectory_context.empty_grace_cycles = 0U;
    ROBOT_TrajectoryClearStagedLocked();
    result = ROBOT_TRAJECTORY_PUSH_SEQUENCE_ERROR;
  }
  else if (robot_trajectory_context.count >=
           ROBOT_TRAJECTORY_BUFFER_CAPACITY)
  {
    if (robot_trajectory_context.hold_reason ==
        ROBOT_TRAJECTORY_HOLD_REASON_NONE)
    {
      robot_trajectory_context.hold_reason =
        ROBOT_TRAJECTORY_HOLD_REASON_QUEUE_OVERFLOW;
    }
    robot_trajectory_context.overflow_count++;
    robot_trajectory_context.state = ROBOT_TRAJECTORY_HOLD;
    robot_trajectory_context.reset_required = 1U;
    robot_trajectory_context.sequence_valid = 0U;
    robot_trajectory_context.expected_sequence = 0U;
    robot_trajectory_context.head = 0U;
    robot_trajectory_context.tail = 0U;
    robot_trajectory_context.count = 0U;
    robot_trajectory_context.empty_grace_cycles = 0U;
    ROBOT_TrajectoryClearStagedLocked();
    result = ROBOT_TRAJECTORY_PUSH_FULL;
  }
  else
  {
    if ((robot_trajectory_context.state == ROBOT_TRAJECTORY_RUNNING) &&
        (robot_trajectory_context.count == 0U) &&
        (robot_trajectory_context.empty_grace_cycles != 0U))
    {
      robot_trajectory_context.empty_grace_recovery_count++;
      robot_trajectory_context.empty_grace_cycles = 0U;
    }
    robot_trajectory_context.points[robot_trajectory_context.tail] = *point;
    robot_trajectory_context.tail++;
    if (robot_trajectory_context.tail >= ROBOT_TRAJECTORY_BUFFER_CAPACITY)
    {
      robot_trajectory_context.tail = 0U;
    }
    robot_trajectory_context.count++;
    robot_trajectory_context.expected_sequence =
      (uint8_t)(point->sequence + 1U);

    /* 普通APPLY成功入队后，原子记录最近接受点及累计数量。 */
    robot_trajectory_context.last_accepted_valid = 1U;
    robot_trajectory_context.last_accepted_sequence = point->sequence;
    robot_trajectory_context.accepted_count++;

    result = ROBOT_TRAJECTORY_PUSH_OK;
  }

  __DMB();
  __set_PRIMASK(primask);
  return result;
}

ROBOT_TrajectoryTakeResult ROBOT_TrajectoryTake(
  ROBOT_TrajectoryPoint *point,
  uint32_t current_tick_ms,
  uint32_t *generation)
{
  ROBOT_TrajectoryPoint *queued_point;
  ROBOT_TrajectoryTakeResult result;
  uint32_t primask;

  if ((point == 0) || (generation == 0))
  {
    return ROBOT_TRAJECTORY_TAKE_INVALID;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  if ((robot_trajectory_context.reset_required != 0U) ||
      (robot_trajectory_context.state == ROBOT_TRAJECTORY_HOLD))
  {
    result = ROBOT_TRAJECTORY_TAKE_HOLD;
  }
  else
  {
    if ((robot_trajectory_context.state == ROBOT_TRAJECTORY_PREFILL) &&
        (robot_trajectory_context.count >= ROBOT_TRAJECTORY_PREFILL_COUNT))
    {
      robot_trajectory_context.state = ROBOT_TRAJECTORY_RUNNING;
      robot_trajectory_context.empty_grace_cycles = 0U;
    }

    if (robot_trajectory_context.state != ROBOT_TRAJECTORY_RUNNING)
    {
      result = ROBOT_TRAJECTORY_TAKE_WAIT;
    }
    else if (robot_trajectory_context.count == 0U)
    {
      if (robot_trajectory_context.empty_grace_cycles <
          ROBOT_TRAJECTORY_EMPTY_GRACE_CYCLES)
      {
        robot_trajectory_context.empty_grace_cycles++;
        result = ROBOT_TRAJECTORY_TAKE_GRACE;
      }
      else
      {
        if (robot_trajectory_context.hold_reason ==
            ROBOT_TRAJECTORY_HOLD_REASON_NONE)
        {
          robot_trajectory_context.hold_reason =
            ROBOT_TRAJECTORY_HOLD_REASON_QUEUE_UNDERRUN;
        }
        robot_trajectory_context.underrun_count++;
        robot_trajectory_context.state = ROBOT_TRAJECTORY_HOLD;
        robot_trajectory_context.reset_required = 1U;
        robot_trajectory_context.sequence_valid = 0U;
        robot_trajectory_context.expected_sequence = 0U;
        robot_trajectory_context.head = 0U;
        robot_trajectory_context.tail = 0U;
        robot_trajectory_context.empty_grace_cycles = 0U;
        ROBOT_TrajectoryClearStagedLocked();
        result = ROBOT_TRAJECTORY_TAKE_HOLD;
      }
    }
    else
    {
      robot_trajectory_context.empty_grace_cycles = 0U;
      queued_point =
        &robot_trajectory_context.points[robot_trajectory_context.head];
      if ((current_tick_ms - queued_point->received_tick_ms) >=
          (uint32_t)queued_point->validity_ms)
      {
        if (robot_trajectory_context.hold_reason ==
            ROBOT_TRAJECTORY_HOLD_REASON_NONE)
        {
          robot_trajectory_context.hold_reason =
            ROBOT_TRAJECTORY_HOLD_REASON_POINT_EXPIRED;
        }
        robot_trajectory_context.expired_count++;
        robot_trajectory_context.state = ROBOT_TRAJECTORY_HOLD;
        robot_trajectory_context.reset_required = 1U;
        robot_trajectory_context.sequence_valid = 0U;
        robot_trajectory_context.expected_sequence = 0U;
        robot_trajectory_context.head = 0U;
        robot_trajectory_context.tail = 0U;
        robot_trajectory_context.count = 0U;
        robot_trajectory_context.empty_grace_cycles = 0U;
        ROBOT_TrajectoryClearStagedLocked();
        result = ROBOT_TRAJECTORY_TAKE_HOLD;
      }
      else
      {
        *point = *queued_point;
        robot_trajectory_context.head++;
        if (robot_trajectory_context.head >=
            ROBOT_TRAJECTORY_BUFFER_CAPACITY)
        {
          robot_trajectory_context.head = 0U;
        }
        robot_trajectory_context.count--;
        *generation = robot_trajectory_context.generation;
        result = ROBOT_TRAJECTORY_TAKE_POINT;
      }
    }
  }

  __DMB();
  __set_PRIMASK(primask);
  return result;
}

uint8_t ROBOT_TrajectoryStageExecution(
  uint8_t sequence,
  uint32_t generation)
{
  uint8_t staged = 0U;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  __DMB();

  if ((robot_trajectory_context.staged_valid == 0U) &&
      (generation == robot_trajectory_context.generation) &&
      (robot_trajectory_context.reset_required == 0U) &&
      (robot_trajectory_context.state == ROBOT_TRAJECTORY_RUNNING))
  {
    robot_trajectory_context.staged_sequence = sequence;
    robot_trajectory_context.staged_generation = generation;
    robot_trajectory_context.staged_valid = 1U;
    staged = 1U;
  }

  __DMB();
  __set_PRIMASK(primask);
  return staged;
}

uint8_t ROBOT_TrajectoryConfirmExecution(uint32_t ecat_cycle)
{
  uint8_t confirmed = 0U;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  __DMB();

  if (robot_trajectory_context.staged_valid != 0U)
  {
    if ((robot_trajectory_context.staged_generation ==
         robot_trajectory_context.generation) &&
        (robot_trajectory_context.reset_required == 0U) &&
        (robot_trajectory_context.state == ROBOT_TRAJECTORY_RUNNING))
    {
      robot_trajectory_context.last_executed_valid = 1U;
      robot_trajectory_context.last_executed_sequence =
        robot_trajectory_context.staged_sequence;
      robot_trajectory_context.last_executed_ecat_cycle = ecat_cycle;
      robot_trajectory_context.executed_count++;
      confirmed = 1U;
    }
    ROBOT_TrajectoryClearStagedLocked();
  }

  __DMB();
  __set_PRIMASK(primask);
  return confirmed;
}
