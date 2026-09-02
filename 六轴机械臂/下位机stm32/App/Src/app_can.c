#include "app_can.h"

#include "ECAT.h"
#include "bsp_can.h"
#include "main.h"
#include "robot_joint.h"
#include "robot_trajectory.h"
#include <stddef.h>
#include <string.h>

#define APP_CAN_CRC16_POLYNOMIAL            0x1021U
#define APP_CAN_CRC16_INITIAL_VALUE         0xFFFFU
#define APP_CAN_CRC_SIZE                    2U
#define APP_CAN_SEQUENCE_FORWARD_MAX        127U
#define APP_CAN_TRAJECTORY_STATUS_PAYLOAD_OFFSET 2U
#define APP_CAN_TRAJECTORY_STATUS_PAYLOAD_SIZE   28U

typedef struct
{
  bool received;
  uint8_t last_value;
} APP_CAN_SequenceState;

typedef enum
{
  APP_CAN_PARSE_OK = 0,
  APP_CAN_PARSE_PAYLOAD_ERROR,
  APP_CAN_PARSE_SEQUENCE_ERROR
} APP_CAN_ParseResult;

static bool app_can_initialized = false;
static APP_CAN_CspCommand app_can_csp_commands[2]; /* 最新CSP命令快照的双缓冲区 */
static APP_CAN_MotionControl app_can_motion_controls[2]; /* 最新运动控制命令快照的双缓冲区 */
static APP_CAN_ParameterRequest app_can_parameter_requests[2]; /* 最新参数请求快照的双缓冲区 */
static APP_CAN_HostHeartbeat app_can_host_heartbeats[2]; /* 最新上位机心跳快照的双缓冲区 */
static volatile uint8_t app_can_csp_index = 0U;
static volatile uint8_t app_can_motion_index = 0U;
static volatile uint8_t app_can_parameter_index = 0U;
static volatile uint8_t app_can_heartbeat_index = 0U;
static volatile bool app_can_csp_valid = false;
static volatile bool app_can_motion_valid = false;
static volatile bool app_can_parameter_valid = false;
static volatile bool app_can_heartbeat_valid = false;
static volatile bool app_can_command_timed_out = true;
static volatile bool app_can_host_online = false;
static volatile uint8_t app_can_safety_request = APP_CAN_SAFETY_REQUEST_NONE;
static bool app_can_command_timeout_reported = false;
static bool app_can_heartbeat_timeout_reported = false;
static bool app_can_tx_schedule_started = false;
static uint32_t app_can_last_position_tx_tick = 0U;
static uint32_t app_can_last_status_tx_tick = 0U;
static uint32_t app_can_last_trajectory_status_tx_tick = 0U;
static uint32_t app_can_last_heartbeat_tx_tick = 0U;
static uint8_t app_can_position_tx_sequence = 0U;
static uint8_t app_can_status_tx_sequence = 0U;
static uint8_t app_can_trajectory_status_tx_sequence = 0U;
static uint8_t app_can_diagnostic_tx_sequence = 0U;
static uint8_t app_can_heartbeat_tx_sequence = 0U;
static uint8_t app_can_last_trajectory_status_payload[
  APP_CAN_TRAJECTORY_STATUS_PAYLOAD_SIZE];
static bool app_can_last_trajectory_status_payload_valid = false;
static APP_CAN_Diagnostic app_can_pending_diagnostic;
static volatile bool app_can_diagnostic_pending = false;
static APP_CAN_SequenceState app_can_sequence_states[4]; /* 0x180～0x183报文的序号校验状态 */
static APP_CAN_Stats app_can_stats;

static uint16_t APP_CAN_ReadU16Le(const uint8_t *data)
{
  return (uint16_t)((uint16_t)data[0] |
                    ((uint16_t)data[1] << 8U));
}

static uint32_t APP_CAN_ReadU32Le(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

static int32_t APP_CAN_ReadS32Le(const uint8_t *data)
{
  return (int32_t)APP_CAN_ReadU32Le(data);
}

static void APP_CAN_WriteU16Le(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
}

static void APP_CAN_WriteU32Le(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
  data[2] = (uint8_t)(value >> 16U);
  data[3] = (uint8_t)(value >> 24U);
}

/**
 * @brief  清零并初始化标准 CAN FD+BRS 发送帧头
 *
 * @param[out] frame      待初始化的 BSP CAN 帧
 * @param[in]  identifier 11 位标准 CAN 标识符
 * @param[in]  length     有效数据长度，单位：字节
 *
 * @return
 * 无
 *
 * @warning
 * frame 必须指向有效且可写的 BSP_CAN_Frame；identifier 必须不大于 0x7FF，length 必须
 * 是 BSP_CAN_Send() 支持的标准 DLC 长度，本函数不会执行参数校验。调用会清除 frame 的
 * 全部原有内容，只负责设置帧头，不填充业务载荷、追加 CRC 或发送报文
 */
static void APP_CAN_PrepareTxFrame(BSP_CAN_Frame *frame,
                                   uint32_t identifier,
                                   uint8_t length)
{
  /* 统一生成11位标准CAN FD+BRS数据帧头，并清零未使用字节。 */
  memset(frame, 0, sizeof(*frame));
  frame->id = identifier;
  frame->length = length;
  frame->is_fd = 1U;
  frame->bit_rate_switch = 1U;
  frame->id_type = BSP_CAN_ID_STANDARD;
}

static uint16_t APP_CAN_CalculateCrc16(const uint8_t *data, uint8_t length)
{
  uint16_t crc = APP_CAN_CRC16_INITIAL_VALUE;
  uint8_t byte_index;
  uint8_t bit_index;

  for (byte_index = 0U; byte_index < length; byte_index++)
  {
    crc ^= (uint16_t)data[byte_index] << 8U;

    for (bit_index = 0U; bit_index < 8U; bit_index++)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1U) ^ APP_CAN_CRC16_POLYNOMIAL);
      }
      else
      {
        crc <<= 1U;
      }
    }
  }

  return crc;
}

static void APP_CAN_AppendFrameCrc(BSP_CAN_Frame *frame)
{
  uint16_t crc;

  crc = APP_CAN_CalculateCrc16(frame->data,
                               (uint8_t)(frame->length - APP_CAN_CRC_SIZE));
  APP_CAN_WriteU16Le(&frame->data[frame->length - APP_CAN_CRC_SIZE], crc);
}

/**
 * @brief  按关节映射表将 EtherCAT 从站有效掩码展开为六轴有效掩码
 *
 * @param[in] slave_mask 从站有效掩码，位0～位2分别对应从站1～从站3
 *
 * @return
 * 六轴有效掩码，位0～位5分别对应 Joint1～Joint6；有效从站上的全部映射关节均置位
 *
 * @warning
 * 本函数依赖 ROBOT_JOINT_GetConfig() 返回的 slave 编号位于1～APP_CAN_SLAVE_COUNT，
 * 不会单独校验映射表中的从站范围；slave_mask 中未被关节映射引用的位将被忽略
 */
static uint8_t APP_CAN_SlaveMaskToAxisMask(uint8_t slave_mask)
{
  uint8_t axis_mask = 0U;
  uint8_t joint_index;
  const ROBOT_JOINT_Config *config;

  for (joint_index = 0U; joint_index < ROBOT_JOINT_COUNT; joint_index++)
  {
    config = ROBOT_JOINT_GetConfig(joint_index);
    if ((config != 0) &&
        ((slave_mask & (uint8_t)(1U << (config->slave - 1U))) != 0U))
    {
      axis_mask |= (uint8_t)(1U << joint_index);
    }
  }

  return axis_mask;
}

/**
 * @brief  非阻塞提交已打包 CAN 帧并更新应用层发送统计
 *
 * @param[in]     frame           已完成帧头、载荷及 CRC 打包的 CAN 帧
 * @param[in,out] success_counter 对应报文类型的成功提交计数器
 *
 * @return
 * 报文成功加入 FDCAN Tx FIFO 时递增 success_counter 并返回true；发送忙或发生其他
 * BSP 发送错误时更新对应失败统计并返回false
 *
 * @warning
 * 调用前必须完成 APP_CAN_Init()，frame 和 success_counter 必须分别指向有效的完整帧与
 * 可写计数器。本函数返回true只表示已提交到硬件发送队列，不代表总线发送已经完成；
 * BSP_CAN_BUSY 会累计 tx_busy，其他失败统一累计 tx_errors，函数自身不会等待或重试。
 * 统计字段按单一发送任务设计，禁止多个任务或中断并发调用
 */
static bool APP_CAN_SendPreparedFrame(const BSP_CAN_Frame *frame,
                                      uint32_t *success_counter)
{
  BSP_CAN_Result result = BSP_CAN_Send(frame);

  if (result == BSP_CAN_OK)
  {
    (*success_counter)++;
    return true;
  }

  if (result == BSP_CAN_BUSY)
  {
    app_can_stats.tx_busy++;
  }
  else
  {
    app_can_stats.tx_errors++;
  }

  return false;
}

/**
 * @brief  校验 CAN 应用协议帧末尾的 CRC16
 *
 * @param[in] frame 待校验的 CAN 帧
 *
 * @return
 * 帧末尾小端序 CRC16 与前置数据计算结果一致时返回true，否则返回false
 *
 * @warning
 * frame 必须有效，且调用方必须先确认 frame->length 位于 APP_CAN_CRC_SIZE～
 * BSP_CAN_MAX_DATA_LEN 范围内；本函数不执行指针和长度检查。CRC 计算不包含末尾两个
 * 校验字节，使用多项式 0x1021 和初值 0xFFFF
 */
static bool APP_CAN_CheckFrameCrc(const BSP_CAN_Frame *frame)
{
  uint16_t received_crc;
  uint16_t calculated_crc;

  received_crc = APP_CAN_ReadU16Le(&frame->data[frame->length - APP_CAN_CRC_SIZE]);
  calculated_crc = APP_CAN_CalculateCrc16(frame->data,
                                          (uint8_t)(frame->length - APP_CAN_CRC_SIZE));

  return received_crc == calculated_crc;
}

static bool APP_CAN_AcceptSequence(uint8_t state_index, uint8_t sequence)
{
  APP_CAN_SequenceState *state = &app_can_sequence_states[state_index];
  uint8_t forward_distance;

  if (!state->received)
  {
    state->received = true;
    state->last_value = sequence;
    return true;
  }

  forward_distance = (uint8_t)(sequence - state->last_value);

  /* 接受前向递增及自然回绕，拒绝重复帧和明显回退帧。 */
  if ((forward_distance == 0U) ||
      (forward_distance > APP_CAN_SEQUENCE_FORWARD_MAX))
  {
    return false;
  }

  state->last_value = sequence;
  return true;
}

/**
 * @brief  通过双缓冲发布最新的完整 CSP 命令快照
 *
 * @param[in] command 待复制发布的 CSP 命令
 *
 * @return
 * 无
 *
 * @warning
 * 调用前必须完成 APP_CAN_Init()，command 必须指向有效且内容完整的 APP_CAN_CspCommand。
 * 本函数先写非活动缓冲区，再通过内存屏障更新活动索引和有效标志；只允许单一接收任务
 * 作为发布者调用，禁止多个任务或中断并发发布。读取方必须使用
 * APP_CAN_GetLatestCspCommand() 获取快照，禁止直接访问内部缓冲区和索引
 */
static void APP_CAN_PublishCspCommand(const APP_CAN_CspCommand *command)
{
  uint8_t next_index = app_can_csp_index ^ 1U;  // ^ 是按位异或

  /* 先写完备用缓冲，再一次性发布完整的六轴目标。 */
  app_can_csp_commands[next_index] = *command;
  __DMB();
  app_can_csp_index = next_index;
  app_can_csp_valid = true;
}

static void APP_CAN_PublishMotionControl(const APP_CAN_MotionControl *control)
{
  uint8_t next_index = app_can_motion_index ^ 1U;

  app_can_motion_controls[next_index] = *control;
  __DMB();
  app_can_motion_index = next_index;
  app_can_motion_valid = true;
}

static void APP_CAN_PublishParameterRequest(const APP_CAN_ParameterRequest *request)
{
  uint8_t next_index = app_can_parameter_index ^ 1U;

  app_can_parameter_requests[next_index] = *request;
  __DMB();
  app_can_parameter_index = next_index;
  app_can_parameter_valid = true;
}

static void APP_CAN_PublishHostHeartbeat(const APP_CAN_HostHeartbeat *heartbeat)
{
  uint8_t next_index = app_can_heartbeat_index ^ 1U;

  app_can_host_heartbeats[next_index] = *heartbeat;
  __DMB();
  app_can_heartbeat_index = next_index;
  app_can_heartbeat_valid = true;
}

/**
 * @brief  解析六轴 CSP 目标命令并更新轨迹缓冲区与最新命令快照
 *
 * @param[in] frame 已完成通用格式、固定长度及 CRC 校验的 0x180 CAN 帧
 *
 * @return
 * 命令成功处理返回 APP_CAN_PARSE_OK；基本字段、有效期或轨迹入队状态无效时返回
 * APP_CAN_PARSE_PAYLOAD_ERROR；普通 APPLY 命令的轨迹序号不连续时返回
 * APP_CAN_PARSE_SEQUENCE_ERROR
 *
 * @warning
 * frame 必须有效且长度为 APP_CAN_DLC_CSP_COMMAND，本函数不重复校验 ID、帧格式、长度及
 * CRC，只允许由接收任务在 ROBOT_TrajectoryInit() 完成后调用。HOLD 会清空轨迹并要求后续
 * 使用 RESET+APPLY 重建；RESET+APPLY 会丢弃原轨迹并以当前点建立新序列；普通 APPLY
 * 遇到序号错误或轨迹队列满时，即使返回错误也会使轨迹进入 HOLD 并清空已排队轨迹点
 */
static APP_CAN_ParseResult APP_CAN_ParseCspCommand(const BSP_CAN_Frame *frame)
{
  APP_CAN_CspCommand command;
  ROBOT_TrajectoryPoint point;
  ROBOT_TrajectoryPushResult push_result;
  uint8_t axis_index;

  /* 初始化命令对象，并提取序号、轴掩码、命令标志和 CSP 工作模式。 */
  memset(&command, 0, sizeof(command));
  command.sequence = frame->data[0];
  command.valid_axis_mask = frame->data[1];
  command.flags = frame->data[2];
  command.mode = frame->data[3];

  /* 按固定优先级校验字段，确保0x204只报告首个拒绝原因。 */
  if (command.valid_axis_mask != APP_CAN_AXIS_VALID_MASK)
  {
    ROBOT_TrajectoryRecordRejected(
      command.sequence,
      1U,
      (uint8_t)APP_CAN_TRAJECTORY_REJECT_INVALID_AXIS_MASK);
    return APP_CAN_PARSE_PAYLOAD_ERROR;
  }

  if ((command.flags != APP_CAN_CSP_FLAG_APPLY) &&
      (command.flags != APP_CAN_CSP_FLAG_HOLD) &&
      (command.flags != APP_CAN_CSP_FLAG_RESET_APPLY))
  {
    ROBOT_TrajectoryRecordRejected(
      command.sequence,
      1U,
      (uint8_t)APP_CAN_TRAJECTORY_REJECT_INVALID_FLAGS);
    return APP_CAN_PARSE_PAYLOAD_ERROR;
  }

  if (command.mode != APP_CAN_CSP_MODE)
  {
    ROBOT_TrajectoryRecordRejected(
      command.sequence,
      1U,
      (uint8_t)APP_CAN_TRAJECTORY_REJECT_INVALID_MODE);
    return APP_CAN_PARSE_PAYLOAD_ERROR;
  }

  /* HOLD 不使用目标位置、有效期和序号，立即清空轨迹并发布保持命令。 */
  if (command.flags == APP_CAN_CSP_FLAG_HOLD)
  {
    ROBOT_TrajectoryRequireResetWithReason(
      ROBOT_TRAJECTORY_HOLD_REASON_EXPLICIT_0X180);
    command.received_tick_ms = HAL_GetTick();
    APP_CAN_PublishCspCommand(&command);
    app_can_stats.accepted_csp_commands++;
    return APP_CAN_PARSE_OK;
  }

  /* APPLY 类命令按小端序解析 Joint1～Joint6 的 32 位绝对目标位置。 */
  for (axis_index = 0U; axis_index < APP_CAN_AXIS_COUNT; axis_index++)
  {
    command.target_position[axis_index] =
      APP_CAN_ReadS32Le(&frame->data[4U + ((uint32_t)axis_index * 4U)]);
  }

  /* 解析轨迹点有效期，拒绝会立即失效的 0 ms 配置。 */
  command.validity_ms = APP_CAN_ReadU16Le(&frame->data[28]);
  if (command.validity_ms == 0U)
  {
    ROBOT_TrajectoryRecordRejected(
      command.sequence,
      1U,
      (uint8_t)APP_CAN_TRAJECTORY_REJECT_INVALID_VALIDITY);
    return APP_CAN_PARSE_PAYLOAD_ERROR;
  }

  /* 记录统一接收时刻，并将应用命令转换为轨迹模块使用的轨迹点。 */
  command.received_tick_ms = HAL_GetTick();
  point.sequence = command.sequence;
  memcpy(point.target_position,
         command.target_position,
         sizeof(point.target_position));
  point.validity_ms = command.validity_ms;
  point.received_tick_ms = command.received_tick_ms;

  /* RESET+APPLY：清空旧轨迹，并以当前点重新建立轨迹序列。 */
  if (command.flags == APP_CAN_CSP_FLAG_RESET_APPLY)
  {
    if (ROBOT_TrajectoryResetAndPush(&point) == 0U)
    {
      ROBOT_TrajectoryRecordRejected(
        command.sequence,
        1U,
        (uint8_t)APP_CAN_TRAJECTORY_REJECT_INTERNAL_ERROR);
      return APP_CAN_PARSE_PAYLOAD_ERROR;
    }

    /* 轨迹重建成功后发布最新命令，并累计已接受命令数。 */
    APP_CAN_PublishCspCommand(&command);
    app_can_stats.accepted_csp_commands++;
    return APP_CAN_PARSE_OK;
  }

  /* APPLY：保持现有轨迹序列，按期望序号追加当前轨迹点。 */
  push_result = ROBOT_TrajectoryPush(&point);

  /* 将轨迹模块结果映射为稳定的0x204拒绝原因。 */
  switch (push_result)
  {
    case ROBOT_TRAJECTORY_PUSH_OK:
      break;

    case ROBOT_TRAJECTORY_PUSH_RESET_REQUIRED:
      ROBOT_TrajectoryRecordRejected(
        command.sequence,
        1U,
        (uint8_t)APP_CAN_TRAJECTORY_REJECT_RESET_REQUIRED);
      return APP_CAN_PARSE_PAYLOAD_ERROR;

    case ROBOT_TRAJECTORY_PUSH_SEQUENCE_ERROR:
      ROBOT_TrajectoryRecordRejected(
        command.sequence,
        1U,
        (uint8_t)APP_CAN_TRAJECTORY_REJECT_SEQUENCE_ERROR);
      return APP_CAN_PARSE_SEQUENCE_ERROR;

    case ROBOT_TRAJECTORY_PUSH_FULL:
      ROBOT_TrajectoryRecordRejected(
        command.sequence,
        1U,
        (uint8_t)APP_CAN_TRAJECTORY_REJECT_QUEUE_FULL);
      return APP_CAN_PARSE_PAYLOAD_ERROR;

    case ROBOT_TRAJECTORY_PUSH_INVALID:
    default:
      ROBOT_TrajectoryRecordRejected(
        command.sequence,
        1U,
        (uint8_t)APP_CAN_TRAJECTORY_REJECT_INTERNAL_ERROR);
      return APP_CAN_PARSE_PAYLOAD_ERROR;
  }

  /* 普通入队成功后发布最新命令，并累计已接受命令数。 */
  APP_CAN_PublishCspCommand(&command);
  app_can_stats.accepted_csp_commands++;

  return APP_CAN_PARSE_OK;
}

static APP_CAN_ParseResult APP_CAN_ParseMotionControl(const BSP_CAN_Frame *frame)
{
  APP_CAN_MotionControl control;
  uint32_t primask;

  control.sequence = frame->data[0];
  control.command = (APP_CAN_MotionCommandType)frame->data[1];
  control.axis_mask = frame->data[2];
  control.flags = frame->data[3];

  /* 校验语义化命令及其六轴作用范围，禁止透传原始控制字。 */
  if ((control.command > APP_CAN_MOTION_FAULT_RESET) ||
      (control.axis_mask == 0U) ||
      ((control.axis_mask & (uint8_t)(~APP_CAN_AXIS_VALID_MASK)) != 0U) ||
      (APP_CAN_ReadU16Le(&frame->data[12]) != 0U))
  {
    return APP_CAN_PARSE_PAYLOAD_ERROR;
  }

  control.request_token = APP_CAN_ReadU32Le(&frame->data[4]);
  control.host_time_ms = APP_CAN_ReadU32Le(&frame->data[8]);

  /* Quick Stop独立于轨迹和运动命令sequence，重复接收仍立即锁存停止。 */
  if (control.command == APP_CAN_MOTION_QUICK_STOP)
  {
    ROBOT_TrajectoryRequireResetWithReason(
      ROBOT_TRAJECTORY_HOLD_REASON_QUICK_STOP);

    primask = __get_PRIMASK();
    __disable_irq();
    app_can_safety_request |=
      (uint8_t)(APP_CAN_SAFETY_REQUEST_HOLD_POSITION |
                APP_CAN_SAFETY_REQUEST_QUICK_STOP);
    __DMB();
    __set_PRIMASK(primask);

    control.received_tick_ms = HAL_GetTick();
    APP_CAN_PublishMotionControl(&control);
    app_can_stats.accepted_motion_controls++;
    return APP_CAN_PARSE_OK;
  }

  if (!APP_CAN_AcceptSequence(1U, control.sequence))
  {
    return APP_CAN_PARSE_SEQUENCE_ERROR;
  }

  control.received_tick_ms = HAL_GetTick();
  APP_CAN_PublishMotionControl(&control);
  app_can_stats.accepted_motion_controls++;

  return APP_CAN_PARSE_OK;
}

static APP_CAN_ParseResult APP_CAN_ParseParameterRequest(const BSP_CAN_Frame *frame)
{
  APP_CAN_ParameterRequest request;

  request.sequence = frame->data[0];
  request.operation = (APP_CAN_ParameterOperation)frame->data[1];
  request.target_axis = frame->data[2];
  request.flags = frame->data[3];

  /* 参数目标仅允许六个单轴或全局对象，保留字段必须为零。 */
  if ((request.operation > APP_CAN_PARAMETER_WRITE) ||
      ((request.target_axis >= APP_CAN_AXIS_COUNT) &&
       (request.target_axis != APP_CAN_PARAMETER_TARGET_GLOBAL)) ||
      (APP_CAN_ReadU16Le(&frame->data[6]) != 0U))
  {
    return APP_CAN_PARSE_PAYLOAD_ERROR;
  }

  request.parameter_id = APP_CAN_ReadU16Le(&frame->data[4]);
  request.value = APP_CAN_ReadS32Le(&frame->data[8]);
  request.request_token = APP_CAN_ReadU16Le(&frame->data[12]);

  if (!APP_CAN_AcceptSequence(2U, request.sequence))
  {
    return APP_CAN_PARSE_SEQUENCE_ERROR;
  }

  request.received_tick_ms = HAL_GetTick();
  APP_CAN_PublishParameterRequest(&request);
  app_can_stats.accepted_parameter_requests++;

  return APP_CAN_PARSE_OK;
}

static APP_CAN_ParseResult APP_CAN_ParseHostHeartbeat(const BSP_CAN_Frame *frame)
{
  APP_CAN_HostHeartbeat heartbeat;

  heartbeat.sequence = frame->data[0];
  heartbeat.protocol_version = frame->data[1];
  heartbeat.host_state = frame->data[2];
  heartbeat.flags = frame->data[3];

  /* 协议版本不一致时拒绝心跳，避免上下位机误用字段定义。 */
  if (heartbeat.protocol_version != APP_CAN_PROTOCOL_VERSION)
  {
    return APP_CAN_PARSE_PAYLOAD_ERROR;
  }

  heartbeat.host_uptime_100ms = APP_CAN_ReadU16Le(&frame->data[4]);

  if (!APP_CAN_AcceptSequence(3U, heartbeat.sequence))
  {
    return APP_CAN_PARSE_SEQUENCE_ERROR;
  }

  heartbeat.received_tick_ms = HAL_GetTick();
  APP_CAN_PublishHostHeartbeat(&heartbeat);
  app_can_stats.accepted_heartbeats++;

  return APP_CAN_PARSE_OK;
}

/**
 * @brief  校验并分发一帧 CAN 应用协议接收报文
 *
 * @param[in] frame 待校验和解析的 BSP CAN 帧
 *
 * @return
 * 无；解析结果通过命令或心跳快照、轨迹与安全状态以及 APP CAN 分类统计体现
 *
 * @warning
 * 调用前必须完成 APP_CAN_Init()，frame 必须指向有效且内容完整的 BSP_CAN_Frame。
 * 本函数仅接受 ID 0x180～0x183 的 11 位标准 CAN FD+BRS 数据帧，并依次校验固定长度、
 * CRC、业务字段及序列号；成功解析可能发布新快照、更新轨迹或锁存 Quick Stop 安全请求。
 * 仅允许由单一接收任务通过 APP_CAN_ProcessRx() 调用，禁止在中断或多个任务中并发解析
 */
static void APP_CAN_ParseFrame(const BSP_CAN_Frame *frame)
{
  APP_CAN_ParseResult parse_result;
  uint8_t expected_length;

  /* 应用协议仅接收11位标准CAN FD+BRS数据帧。 */
  if ((frame->id_type != BSP_CAN_ID_STANDARD) ||
      (frame->is_fd == 0U) ||
      (frame->bit_rate_switch == 0U))
  {
    app_can_stats.invalid_formats++;
    if (frame->id == APP_CAN_ID_CSP_COMMAND)
    {
      ROBOT_TrajectoryRecordRejected(
        0U,
        0U,
        (uint8_t)APP_CAN_TRAJECTORY_REJECT_BAD_FRAME_FORMAT);
    }
    return;
  }

  switch (frame->id)
  {
    case APP_CAN_ID_CSP_COMMAND:
      expected_length = APP_CAN_DLC_CSP_COMMAND;
      break;

    case APP_CAN_ID_MOTION_CONTROL:
      expected_length = APP_CAN_DLC_MOTION_CONTROL;
      break;

    case APP_CAN_ID_PARAMETER_REQUEST:
      expected_length = APP_CAN_DLC_PARAMETER_REQUEST;
      break;

    case APP_CAN_ID_HOST_HEARTBEAT:
      expected_length = APP_CAN_DLC_HOST_HEARTBEAT;
      break;

    default:
      app_can_stats.invalid_ids++;
      return;
  }

  /* 先校验固定DLC及CRC，再解析具体业务字段。 */
  if (frame->length != expected_length)
  {
    app_can_stats.invalid_lengths++;
    if (frame->id == APP_CAN_ID_CSP_COMMAND)
    {
      ROBOT_TrajectoryRecordRejected(
        0U,
        0U,
        (uint8_t)APP_CAN_TRAJECTORY_REJECT_BAD_DLC);
    }
    return;
  }

  if (!APP_CAN_CheckFrameCrc(frame))
  {
    app_can_stats.crc_errors++;
    if (frame->id == APP_CAN_ID_CSP_COMMAND)
    {
      ROBOT_TrajectoryRecordRejected(
        0U,
        0U,
        (uint8_t)APP_CAN_TRAJECTORY_REJECT_CRC_ERROR);
    }
    return;
  }

  if (frame->id == APP_CAN_ID_CSP_COMMAND)
  {
    ROBOT_TrajectoryRecordReceived(frame->data[0]);
  }

  switch (frame->id)
  {
    case APP_CAN_ID_CSP_COMMAND:
      parse_result = APP_CAN_ParseCspCommand(frame);
      break;

    case APP_CAN_ID_MOTION_CONTROL:
      parse_result = APP_CAN_ParseMotionControl(frame);
      break;

    case APP_CAN_ID_PARAMETER_REQUEST:
      parse_result = APP_CAN_ParseParameterRequest(frame);
      break;

    case APP_CAN_ID_HOST_HEARTBEAT:
      parse_result = APP_CAN_ParseHostHeartbeat(frame);
      break;

    default:
      return;
  }

  /* 分开统计字段错误和序号错误，避免同一拒收帧重复归类。 */
  if (parse_result == APP_CAN_PARSE_PAYLOAD_ERROR)
  {
    app_can_stats.payload_errors++;
  }
  else if (parse_result == APP_CAN_PARSE_SEQUENCE_ERROR)
  {
    app_can_stats.sequence_errors++;
  }
}

/**
 * @brief  初始化 CAN 应用协议状态并启动 FDCAN 通信
 *
 * @return
 * 应用状态、接收过滤器及 FDCAN 均初始化成功时返回 APP_CAN_OK；FDCAN 底层尚未
 * 就绪时返回 APP_CAN_NOT_READY；BSP 配置或启动失败时返回 APP_CAN_BSP_ERROR
 *
 * @warning
 * 调用前必须完成 MX_FDCAN1_Init()，并应在调度器启动前或所有 CAN 处理均已停止时
 * 调用，禁止与接收中断及 APP_CAN_ProcessRx()/APP_CAN_ProcessTx() 并发执行。本函数
 * 会重置轨迹模块、全部已发布命令和心跳、序号校验、统计、超时与安全请求、诊断及
 * 发送调度状态，并将硬件接收过滤范围配置为标准 ID 0x180～0x183；失败时应用层保持
 * 未初始化，已清除的运行数据不会恢复
 */
APP_CAN_Result APP_CAN_Init(void)
{
  BSP_CAN_Config config;
  BSP_CAN_Result bsp_result;

  ROBOT_TrajectoryInit();
  memset(app_can_csp_commands, 0, sizeof(app_can_csp_commands));
  memset(app_can_motion_controls, 0, sizeof(app_can_motion_controls));
  memset(app_can_parameter_requests, 0, sizeof(app_can_parameter_requests));
  memset(app_can_host_heartbeats, 0, sizeof(app_can_host_heartbeats));
  memset(app_can_sequence_states, 0, sizeof(app_can_sequence_states));
  APP_CAN_ResetStats();

  app_can_csp_index = 0U;
  app_can_motion_index = 0U;
  app_can_parameter_index = 0U;
  app_can_heartbeat_index = 0U;
  app_can_csp_valid = false;
  app_can_motion_valid = false;
  app_can_parameter_valid = false;
  app_can_heartbeat_valid = false;
  app_can_command_timed_out = true;
  app_can_host_online = false;
  app_can_safety_request = APP_CAN_SAFETY_REQUEST_NONE;
  app_can_command_timeout_reported = false;
  app_can_heartbeat_timeout_reported = false;
  app_can_tx_schedule_started = false;
  app_can_last_position_tx_tick = 0U;
  app_can_last_status_tx_tick = 0U;
  app_can_last_trajectory_status_tx_tick = 0U;
  app_can_last_heartbeat_tx_tick = 0U;
  app_can_position_tx_sequence = 0U;
  app_can_status_tx_sequence = 0U;
  app_can_trajectory_status_tx_sequence = 0U;
  app_can_diagnostic_tx_sequence = 0U;
  app_can_heartbeat_tx_sequence = 0U;
  memset(app_can_last_trajectory_status_payload,
         0,
         sizeof(app_can_last_trajectory_status_payload));
  app_can_last_trajectory_status_payload_valid = false;
  memset(&app_can_pending_diagnostic, 0, sizeof(app_can_pending_diagnostic));
  app_can_diagnostic_pending = false;
  app_can_initialized = false;

  /* 应用层确定协议ID范围，BSP层只负责配置对应硬件过滤器。 */
  config.standard_filter_id = APP_CAN_ID_CSP_COMMAND;
  config.standard_filter_mask = APP_CAN_RX_FILTER_MASK;

  bsp_result = BSP_CAN_Init(&config);
  if (bsp_result == BSP_CAN_NOT_READY)
  {
    return APP_CAN_NOT_READY;
  }
  if (bsp_result != BSP_CAN_OK)
  {
    return APP_CAN_BSP_ERROR;
  }

  bsp_result = BSP_CAN_Start();
  if (bsp_result == BSP_CAN_NOT_READY)
  {
    return APP_CAN_NOT_READY;
  }
  if (bsp_result != BSP_CAN_OK)
  {
    return APP_CAN_BSP_ERROR;
  }

  app_can_initialized = true;
  return APP_CAN_OK;
}

void APP_CAN_ProcessRx(void)
{
  BSP_CAN_Frame frame;

  if (!app_can_initialized)
  {
    return;
  }

  /* 单一任务持续取帧，直到排空BSP软件接收队列。 */
  while (BSP_CAN_TryReceive(&frame))
  {
    app_can_stats.rx_frames++;
    APP_CAN_ParseFrame(&frame);
  }
}

bool APP_CAN_GetLatestCspCommand(APP_CAN_CspCommand *command)
{
  uint32_t primask;

  if ((command == NULL) || (!app_can_csp_valid))
  {
    return false;
  }

  /* 短临界区内复制已发布快照，避免与下一次发布交叉。 */
  primask = __get_PRIMASK();
  __disable_irq();
  __DMB();
  *command = app_can_csp_commands[app_can_csp_index];
  __DMB();
  __set_PRIMASK(primask);

  return true;
}

bool APP_CAN_GetLatestMotionControl(APP_CAN_MotionControl *control)
{
  uint32_t primask;

  if ((control == NULL) || (!app_can_motion_valid))
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  __DMB();
  *control = app_can_motion_controls[app_can_motion_index];
  __DMB();
  __set_PRIMASK(primask);

  return true;
}

bool APP_CAN_GetLatestParameterRequest(APP_CAN_ParameterRequest *request)
{
  uint32_t primask;

  if ((request == NULL) || (!app_can_parameter_valid))
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  __DMB();
  *request = app_can_parameter_requests[app_can_parameter_index];
  __DMB();
  __set_PRIMASK(primask);

  return true;
}

bool APP_CAN_GetLatestHostHeartbeat(APP_CAN_HostHeartbeat *heartbeat)
{
  uint32_t primask;

  if ((heartbeat == NULL) || (!app_can_heartbeat_valid))
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  __DMB();
  *heartbeat = app_can_host_heartbeats[app_can_heartbeat_index];
  __DMB();
  __set_PRIMASK(primask);

  return true;
}

/**
 * @brief  将六轴位置反馈打包为 0x200 标准 CAN FD+BRS 数据帧
 *
 * @param[in]  feedback 待编码的位置反馈快照
 * @param[out] frame    成功时写入固定 32 字节的完整 CAN 帧及帧末 CRC16
 *
 * @return
 * 打包成功返回 APP_CAN_OK；任一指针为空、有效轴掩码越界或包含未定义状态标志时返回
 * APP_CAN_INVALID_ARGUMENT，且不会修改 frame
 *
 * @warning
 * feedback 和 frame 必须分别指向有效的输入、可写输出对象。本函数仅校验轴掩码与状态
 * 标志范围，不校验 EtherCAT 状态及各轴位置语义；无论 valid_axis_mask 是否置位，六个
 * actual_position 都会按 Joint1～Joint6 顺序以小端补码写入，接收方必须依据有效轴掩码
 * 判定数据是否可用。本函数只生成报文，不负责调用 BSP_CAN_Send() 发送
 */
APP_CAN_Result APP_CAN_PackPositionFeedback(
  const APP_CAN_PositionFeedback *feedback,
  BSP_CAN_Frame *frame)
{
  uint8_t axis_index;

  /* 校验反馈指针、六轴有效掩码及当前已定义的状态标志。 */
  if ((feedback == NULL) ||
      (frame == NULL) ||
      ((feedback->valid_axis_mask & (uint8_t)(~APP_CAN_AXIS_VALID_MASK)) != 0U) ||
      ((feedback->flags & (uint8_t)(~APP_CAN_FEEDBACK_FLAG_MASK)) != 0U))
  {
    return APP_CAN_INVALID_ARGUMENT;
  }

  APP_CAN_PrepareTxFrame(frame,
                         APP_CAN_ID_POSITION_FEEDBACK,
                         APP_CAN_DLC_POSITION_FEEDBACK);

  frame->data[0] = feedback->sequence;
  frame->data[1] = feedback->valid_axis_mask;
  frame->data[2] = feedback->ecat_state;
  frame->data[3] = feedback->flags;

  /* 按固定轴序将六轴实际位置写入同一帧，保证上位机获得一致快照。 */
  for (axis_index = 0U; axis_index < APP_CAN_AXIS_COUNT; axis_index++)
  {
    APP_CAN_WriteU32Le(&frame->data[4U + ((uint32_t)axis_index * 4U)],
                       (uint32_t)feedback->actual_position[axis_index]);
  }

  APP_CAN_WriteU16Le(&frame->data[28], feedback->cycle_counter);
  APP_CAN_AppendFrameCrc(frame);

  return APP_CAN_OK;
}

/**
 * @brief  将六轴详细状态打包为 0x201 标准 CAN FD+BRS 数据帧
 *
 * @param[in]  status 待编码的六轴速度、转矩、状态字及错误码快照
 * @param[out] frame  成功时写入固定 64 字节的完整 CAN 帧及帧末 CRC16
 *
 * @return
 * 打包成功返回 APP_CAN_OK；任一指针为空或有效轴掩码越界时返回
 * APP_CAN_INVALID_ARGUMENT，且不会修改 frame
 *
 * @warning
 * status 和 frame 必须分别指向有效的输入、可写输出对象。本函数仅校验有效轴掩码，
 * 不校验各轴反馈值的业务语义；无论 valid_axis_mask 是否置位，六轴速度、转矩、状态字
 * 和错误码都会按 Joint1～Joint6 顺序以小端序写入，带符号量使用补码编码，接收方必须
 * 依据有效轴掩码判定数据是否可用。本函数只生成报文，不负责调用 BSP_CAN_Send() 发送
 */
APP_CAN_Result APP_CAN_PackAxisStatus(
  const APP_CAN_AxisStatusFeedback *status,
  BSP_CAN_Frame *frame)
{
  uint8_t axis_index;

  if ((status == NULL) ||
      (frame == NULL) ||
      ((status->valid_axis_mask & (uint8_t)(~APP_CAN_AXIS_VALID_MASK)) != 0U))
  {
    return APP_CAN_INVALID_ARGUMENT;
  }

  APP_CAN_PrepareTxFrame(frame,
                         APP_CAN_ID_AXIS_STATUS,
                         APP_CAN_DLC_AXIS_STATUS);

  frame->data[0] = status->sequence;
  frame->data[1] = status->valid_axis_mask;

  /* 依次打包速度、转矩、状态字和错误码，字段总长固定为64字节。 */
  for (axis_index = 0U; axis_index < APP_CAN_AXIS_COUNT; axis_index++)
  {
    APP_CAN_WriteU32Le(&frame->data[2U + ((uint32_t)axis_index * 4U)],
                       (uint32_t)status->actual_velocity[axis_index]);
    APP_CAN_WriteU16Le(&frame->data[26U + ((uint32_t)axis_index * 2U)],
                       (uint16_t)status->actual_torque[axis_index]);
    APP_CAN_WriteU16Le(&frame->data[38U + ((uint32_t)axis_index * 2U)],
                       status->statusword[axis_index]);
    APP_CAN_WriteU16Le(&frame->data[50U + ((uint32_t)axis_index * 2U)],
                       status->error_code[axis_index]);
  }

  APP_CAN_AppendFrameCrc(frame);
  return APP_CAN_OK;
}

APP_CAN_Result APP_CAN_PackDiagnostic(
  const APP_CAN_Diagnostic *diagnostic,
  BSP_CAN_Frame *frame)
{
  if ((diagnostic == NULL) ||
      (frame == NULL) ||
      (diagnostic->severity > APP_CAN_DIAGNOSTIC_FATAL) ||
      ((diagnostic->source_axis >= APP_CAN_AXIS_COUNT) &&
       (diagnostic->source_axis != APP_CAN_DIAGNOSTIC_SOURCE_GLOBAL)))
  {
    return APP_CAN_INVALID_ARGUMENT;
  }

  APP_CAN_PrepareTxFrame(frame,
                         APP_CAN_ID_DIAGNOSTIC,
                         APP_CAN_DLC_DIAGNOSTIC);

  /* 诊断帧保存来源、错误码和现场值，便于ROS2关联具体轴故障。 */
  frame->data[0] = diagnostic->sequence;
  frame->data[1] = (uint8_t)diagnostic->severity;
  frame->data[2] = diagnostic->source_axis;
  frame->data[3] = diagnostic->flags;
  APP_CAN_WriteU16Le(&frame->data[4], diagnostic->error_code);
  APP_CAN_WriteU16Le(&frame->data[6], diagnostic->detail_code);
  APP_CAN_WriteU32Le(&frame->data[8], diagnostic->context);
  APP_CAN_WriteU16Le(&frame->data[12], diagnostic->event_counter);
  APP_CAN_AppendFrameCrc(frame);

  return APP_CAN_OK;
}

/**
 * @brief  将设备心跳打包为 0x203 标准 CAN FD+BRS 数据帧
 *
 * @param[in]  heartbeat 待编码的 EtherCAT 主站状态、OP 从站掩码及最近 WKC
 * @param[out] frame     成功时写入固定 8 字节的完整 CAN 帧及帧末 CRC16
 *
 * @return
 * 打包成功返回 APP_CAN_OK；任一指针为空或 OP 从站掩码包含未定义位时返回
 * APP_CAN_INVALID_ARGUMENT，且不会修改 frame
 *
 * @warning
 * heartbeat 和 frame 必须分别指向有效的输入、可写输出对象。本函数只校验
 * operational_slave_mask 的低三位范围，不校验 EtherCAT 状态和 WKC 的业务语义；协议版本
 * 字段固定写入 APP_CAN_PROTOCOL_VERSION，WKC 以小端序编码。本函数只生成报文，不负责
 * 调用 BSP_CAN_Send() 发送
 */
APP_CAN_Result APP_CAN_PackDeviceHeartbeat(
  const APP_CAN_DeviceHeartbeat *heartbeat,
  BSP_CAN_Frame *frame)
{
  if ((heartbeat == NULL) ||
      (frame == NULL) ||
      ((heartbeat->operational_slave_mask &
        (uint8_t)(~APP_CAN_SLAVE_VALID_MASK)) != 0U))
  {
    return APP_CAN_INVALID_ARGUMENT;
  }

  APP_CAN_PrepareTxFrame(frame,
                         APP_CAN_ID_DEVICE_HEARTBEAT,
                         APP_CAN_DLC_DEVICE_HEARTBEAT);

  /* 心跳固定携带协议版本、EtherCAT状态、从站掩码和最近WKC。 */
  frame->data[0] = heartbeat->sequence;
  frame->data[1] = APP_CAN_PROTOCOL_VERSION;
  frame->data[2] = heartbeat->ecat_state;
  frame->data[3] = heartbeat->operational_slave_mask;
  APP_CAN_WriteU16Le(&frame->data[4], heartbeat->working_counter);
  APP_CAN_AppendFrameCrc(frame);

  return APP_CAN_OK;
}

/**
 * @brief  将轨迹状态打包为0x204标准CAN FD+BRS数据帧
 * @param[in]  status 待编码的轨迹状态快照
 * @param[out] frame  成功时写入固定32字节帧及帧末CRC16
 * @return 成功返回APP_CAN_OK；参数为空或状态字段不自洽时返回APP_CAN_INVALID_ARGUMENT
 *
 * @warning 本函数只生成报文，不负责读取轨迹快照、维护发送序号或调用BSP_CAN_Send()。
 */
APP_CAN_Result APP_CAN_PackTrajectoryStatus(
  const APP_CAN_TrajectoryStatus *status,
  BSP_CAN_Frame *frame)
{
  if ((status == NULL) ||
      (frame == NULL) ||
      ((uint32_t)status->state >
       (uint32_t)APP_CAN_TRAJECTORY_STATUS_HOLD) ||
      ((uint32_t)status->reject_reason >
       (uint32_t)APP_CAN_TRAJECTORY_REJECT_INTERNAL_ERROR) ||
      ((uint32_t)status->hold_reason >
       (uint32_t)APP_CAN_TRAJECTORY_HOLD_INTERNAL_ERROR) ||
      (status->queue_capacity == 0U) ||
      (status->queue_depth > status->queue_capacity) ||
      (status->prefill_target == 0U) ||
      (status->prefill_target > status->queue_capacity))
  {
    return APP_CAN_INVALID_ARGUMENT;
  }

  APP_CAN_PrepareTxFrame(frame,
                         APP_CAN_ID_TRAJECTORY_STATUS,
                         APP_CAN_DLC_TRAJECTORY_STATUS);

  frame->data[0] = status->sequence;
  frame->data[1] = APP_CAN_PROTOCOL_VERSION;
  frame->data[2] = (uint8_t)status->state;
  frame->data[3] = status->queue_depth;
  frame->data[4] = status->queue_capacity;
  frame->data[5] = status->flags;
  frame->data[6] = status->last_received_sequence;
  frame->data[7] = status->last_accepted_sequence;
  frame->data[8] = status->last_executed_sequence;
  frame->data[9] = status->expected_sequence;
  frame->data[10] = status->last_rejected_sequence;
  frame->data[11] = (uint8_t)status->reject_reason;
  frame->data[12] = (uint8_t)status->hold_reason;
  frame->data[13] = status->prefill_target;
  APP_CAN_WriteU16Le(&frame->data[14], status->generation);
  APP_CAN_WriteU16Le(&frame->data[16],
                     status->last_executed_ecat_cycle);
  APP_CAN_WriteU16Le(&frame->data[18], status->accepted_count);
  APP_CAN_WriteU16Le(&frame->data[20], status->executed_count);
  APP_CAN_WriteU16Le(&frame->data[22], status->rejected_count);
  APP_CAN_WriteU16Le(&frame->data[24], status->underrun_count);
  APP_CAN_WriteU16Le(&frame->data[26], status->overflow_count);
  APP_CAN_WriteU16Le(&frame->data[28], status->expired_count);
  APP_CAN_AppendFrameCrc(frame);

  return APP_CAN_OK;
}

/**
 * @brief  检查 CSP 命令与上位机心跳新鲜度并更新安全状态
 *
 * @param[in] current_tick_ms 当前系统毫秒时刻，应与 HAL_GetTick() 使用同一时间基准
 *
 * @return
 * 无；应用层未初始化时不执行操作，结果通过超时、在线、安全请求状态及统计信息体现
 *
 * @warning
 * 调用前必须成功完成 APP_CAN_Init()，并应由单一 CAN 服务任务在 APP_CAN_ProcessRx() 后
 * 周期调用，禁止在中断或多个任务中并发执行。轨迹处于 HOLD 时不要求持续接收 CSP 命令；
 * 尚未收到首条命令或心跳时只报告未就绪或离线，不累计超时事件。已接收报文过期时会
 * 一次性累计统计并锁存 HOLD_POSITION 与 QUICK_STOP 请求，但不会直接修改 PDO 或轨迹；
 * 后续通信恢复只复位超时报告条件，不会自动清除安全请求，必须由上层在条件恢复后调用
 * APP_CAN_ClearSafetyRequest()。无符号时间差允许 current_tick_ms 自然回绕
 */
void APP_CAN_CheckTimeout(uint32_t current_tick_ms)
{
  APP_CAN_CspCommand command;
  APP_CAN_HostHeartbeat heartbeat;
  bool command_available;
  bool heartbeat_available;
  uint32_t elapsed_ms;

  if (!app_can_initialized)
  {
    return;
  }

  command_available = APP_CAN_GetLatestCspCommand(&command);
  heartbeat_available = APP_CAN_GetLatestHostHeartbeat(&heartbeat);

  if (ROBOT_TrajectoryIsHold() != 0U)
  {
    /* HOLD已锁存时不要求持续发送0x180，但上位机心跳仍在下方独立检查。 */
    app_can_command_timed_out = false;
    app_can_command_timeout_reported = false;
  }
  else if (command_available)
  {
    /* 使用无符号时间差，系统Tick回绕时超时判断仍然成立。 */
    elapsed_ms = current_tick_ms - command.received_tick_ms;
    app_can_command_timed_out = elapsed_ms >= APP_CAN_COMMAND_TIMEOUT_MS;

    if (app_can_command_timed_out)
    {
      if (!app_can_command_timeout_reported)
      {
        /* 命令超时后锁存保持和Quick Stop请求，不直接修改PDO。 */
        app_can_safety_request |=
          (uint8_t)(APP_CAN_SAFETY_REQUEST_HOLD_POSITION |
                    APP_CAN_SAFETY_REQUEST_QUICK_STOP);
        app_can_stats.command_timeouts++;
        app_can_command_timeout_reported = true;
      }
    }
    else
    {
      app_can_command_timeout_reported = false;
    }
  }
  else
  {
    /* 尚未收到首条位置命令时保持未就绪，但不产生虚假超时事件。 */
    app_can_command_timed_out = true;
  }

  if (heartbeat_available)
  {
    elapsed_ms = current_tick_ms - heartbeat.received_tick_ms;
    app_can_host_online = elapsed_ms < APP_CAN_HOST_HEARTBEAT_TIMEOUT_MS;

    if (!app_can_host_online)
    {
      if (!app_can_heartbeat_timeout_reported)
      {
        /* 心跳失联同样锁存安全停机请求，等待控制状态机处理。 */
        app_can_safety_request |=
          (uint8_t)(APP_CAN_SAFETY_REQUEST_HOLD_POSITION |
                    APP_CAN_SAFETY_REQUEST_QUICK_STOP);
        app_can_stats.heartbeat_timeouts++;
        app_can_heartbeat_timeout_reported = true;
      }
    }
    else
    {
      app_can_heartbeat_timeout_reported = false;
    }
  }
  else
  {
    /* 启动后尚未收到首个心跳时仅报告离线，不重复累计超时。 */
    app_can_host_online = false;
  }
}

bool APP_CAN_IsCommandTimedOut(void)
{
  return app_can_command_timed_out;
}

bool APP_CAN_IsHostOnline(void)
{
  return app_can_host_online;
}

APP_CAN_SafetyRequest APP_CAN_GetSafetyRequest(void)
{
  return (APP_CAN_SafetyRequest)app_can_safety_request;
}

bool APP_CAN_ClearSafetyRequest(void)
{
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();

  /* 仅在命令和心跳均恢复后允许上层显式清除，禁止自动恢复使能。 */
  if ((!app_can_initialized) ||
      app_can_command_timed_out ||
      (!app_can_host_online))
  {
    __set_PRIMASK(primask);
    return false;
  }

  if (app_can_safety_request != APP_CAN_SAFETY_REQUEST_NONE)
  {
    app_can_safety_request = APP_CAN_SAFETY_REQUEST_NONE;
    app_can_stats.safety_request_clears++;
  }

  __DMB();
  __set_PRIMASK(primask);
  return true;
}

bool APP_CAN_QueueDiagnostic(const APP_CAN_Diagnostic *diagnostic)
{
  BSP_CAN_Frame validation_frame;
  uint32_t primask;

  if ((!app_can_initialized) ||
      (diagnostic == NULL) ||
      (APP_CAN_PackDiagnostic(diagnostic, &validation_frame) != APP_CAN_OK))
  {
    return false;
  }

  primask = __get_PRIMASK();
  __disable_irq();

  /* 单槽已占用时保留旧事件，禁止静默覆盖尚未发送的故障。 */
  if (app_can_diagnostic_pending)
  {
    app_can_stats.diagnostic_queue_full++;
    __set_PRIMASK(primask);
    return false;
  }

  app_can_pending_diagnostic = *diagnostic;
  __DMB();
  app_can_diagnostic_pending = true;
  __set_PRIMASK(primask);

  return true;
}

/**
 * @brief  汇总 EtherCAT 反馈并调度发送 CAN 周期反馈、诊断和设备心跳
 *
 * @param[in] current_tick_ms 当前系统毫秒时刻，应与 HAL_GetTick() 使用同一时间基准
 *
 * @return
 * 无；应用层未初始化时不执行操作，发送结果通过发送序号、待发送状态和统计信息体现
 *
 * @warning
 * 调用前必须成功完成 APP_CAN_Init()，并应由单一 CAN 服务任务周期调用，禁止在中断或
 * 多个任务中并发执行；为使超时标志及时反映到反馈帧，应先使用同一时刻调用
 * APP_CAN_CheckTimeout()。首次调用会立即尝试发送全部周期报文，之后使用无符号时间差
 * 独立调度；发送忙或失败时不会推进对应序号和时间戳，待下次调用重试，诊断也只在发送
 * 成功后清除。无有效 EtherCAT 快照时，反馈有效轴掩码为0，PDO反馈字段保持零值
 */
void APP_CAN_ProcessTx(uint32_t current_tick_ms)
{
  ECAT_FeedbackSnapshot snapshot;
  APP_CAN_PositionFeedback position_feedback;
  APP_CAN_AxisStatusFeedback axis_status;
  ROBOT_TrajectorySnapshot trajectory_snapshot;
  APP_CAN_TrajectoryStatus trajectory_status;
  APP_CAN_DeviceHeartbeat heartbeat;
  APP_CAN_Diagnostic diagnostic;
  BSP_CAN_Frame frame;
  BSP_CAN_Frame trajectory_frame;
  ECAT_State ecat_state;
  APP_CAN_SafetyRequest safety_request;
  uint32_t primask;
  uint8_t snapshot_available;
  uint8_t joint_index;
  uint8_t slave_index;
  uint8_t enabled_axis_mask = 0U;
  uint8_t trajectory_snapshot_available;
  bool diagnostic_pending;
  bool trajectory_frame_ready = false;
  bool trajectory_status_changed;
  uint32_t trajectory_status_period_ms;
  const ROBOT_JOINT_Config *config;
  const AppEtherCAT_ServoTxPdo *joint_feedback;

  if (!app_can_initialized)
  {
    return;
  }

  if (!app_can_tx_schedule_started)
  {
    /* 首次调用立即发送各周期报文，之后按各自周期独立调度。 */
    app_can_last_position_tx_tick =
      current_tick_ms - APP_CAN_POSITION_FEEDBACK_PERIOD_MS;
    app_can_last_status_tx_tick =
      current_tick_ms - APP_CAN_AXIS_STATUS_PERIOD_MS;
    app_can_last_trajectory_status_tx_tick =
      current_tick_ms - APP_CAN_TRAJECTORY_STATUS_ACTIVE_PERIOD_MS;
    app_can_last_heartbeat_tx_tick =
      current_tick_ms - APP_CAN_DEVICE_HEARTBEAT_PERIOD_MS;
    app_can_last_trajectory_status_payload_valid = false;
    app_can_tx_schedule_started = true;
  }

  memset(&snapshot, 0, sizeof(snapshot));
  memset(&position_feedback, 0, sizeof(position_feedback));
  memset(&axis_status, 0, sizeof(axis_status));
  memset(&trajectory_snapshot, 0, sizeof(trajectory_snapshot));
  memset(&trajectory_status, 0, sizeof(trajectory_status));
  memset(&trajectory_frame, 0, sizeof(trajectory_frame));
  memset(&heartbeat, 0, sizeof(heartbeat));

  snapshot_available = ECAT_GetFeedbackSnapshot(&snapshot);
  ecat_state = (snapshot_available != 0U) ? snapshot.state : ECAT_GetState();

  position_feedback.valid_axis_mask =
    (snapshot_available != 0U) ?
    APP_CAN_SlaveMaskToAxisMask(snapshot.valid_slave_mask) : 0U;
  position_feedback.ecat_state = (uint8_t)ecat_state;
  position_feedback.cycle_counter =
    (snapshot_available != 0U) ? (uint16_t)snapshot.cycle_counter : 0U;
  axis_status.valid_axis_mask = position_feedback.valid_axis_mask;

  heartbeat.ecat_state = (uint8_t)ecat_state;
  heartbeat.operational_slave_mask =
    (snapshot_available != 0U) ? snapshot.operational_slave_mask : 0U;
  heartbeat.working_counter =
    (snapshot_available != 0U) ? snapshot.working_counter : 0U;

  if (ecat_state == ECAT_STATE_OPERATIONAL)
  {
    position_feedback.flags |= APP_CAN_FEEDBACK_FLAG_OPERATIONAL;
  }

  if (APP_CAN_IsCommandTimedOut())
  {
    position_feedback.flags |= APP_CAN_FEEDBACK_FLAG_CMD_TIMEOUT;
  }

  safety_request = APP_CAN_GetSafetyRequest();
  if ((safety_request & APP_CAN_SAFETY_REQUEST_QUICK_STOP) != 0U)
  {
    position_feedback.flags |= APP_CAN_FEEDBACK_FLAG_QUICK_STOP;
  }

  trajectory_snapshot_available =
    ROBOT_TrajectoryGetSnapshot(&trajectory_snapshot);
  if (trajectory_snapshot_available != 0U)
  {
    switch (trajectory_snapshot.state)
    {
      case ROBOT_TRAJECTORY_IDLE:
        trajectory_status.state = APP_CAN_TRAJECTORY_STATUS_IDLE;
        trajectory_frame_ready = true;
        break;
      case ROBOT_TRAJECTORY_PREFILL:
        trajectory_status.state = APP_CAN_TRAJECTORY_STATUS_PREFILL;
        trajectory_frame_ready = true;
        break;
      case ROBOT_TRAJECTORY_RUNNING:
        trajectory_status.state = APP_CAN_TRAJECTORY_STATUS_RUNNING;
        trajectory_frame_ready = true;
        break;
      case ROBOT_TRAJECTORY_HOLD:
        trajectory_status.state = APP_CAN_TRAJECTORY_STATUS_HOLD;
        trajectory_frame_ready = true;
        break;
      default:
        break;
    }
  }

  if (trajectory_frame_ready)
  {
    trajectory_status.sequence = app_can_trajectory_status_tx_sequence;
    trajectory_status.queue_depth = trajectory_snapshot.queue_depth;
    trajectory_status.queue_capacity = ROBOT_TRAJECTORY_BUFFER_CAPACITY;
    trajectory_status.prefill_target = ROBOT_TRAJECTORY_PREFILL_COUNT;

    if (trajectory_snapshot.reset_required != 0U)
    {
      trajectory_status.flags |=
        APP_CAN_TRAJECTORY_STATUS_FLAG_RESET_REQUIRED;
    }
    if (trajectory_snapshot.expected_sequence_valid != 0U)
    {
      trajectory_status.flags |=
        APP_CAN_TRAJECTORY_STATUS_FLAG_EXPECTED_SEQUENCE_VALID;
      trajectory_status.expected_sequence =
        trajectory_snapshot.expected_sequence;
    }
    if (trajectory_snapshot.last_received_valid != 0U)
    {
      trajectory_status.flags |=
        APP_CAN_TRAJECTORY_STATUS_FLAG_LAST_RECEIVED_VALID;
      trajectory_status.last_received_sequence =
        trajectory_snapshot.last_received_sequence;
    }
    if (trajectory_snapshot.last_accepted_valid != 0U)
    {
      trajectory_status.flags |=
        APP_CAN_TRAJECTORY_STATUS_FLAG_LAST_ACCEPTED_VALID;
      trajectory_status.last_accepted_sequence =
        trajectory_snapshot.last_accepted_sequence;
    }
    if (trajectory_snapshot.last_executed_valid != 0U)
    {
      trajectory_status.flags |=
        APP_CAN_TRAJECTORY_STATUS_FLAG_LAST_EXECUTED_VALID;
      trajectory_status.last_executed_sequence =
        trajectory_snapshot.last_executed_sequence;
    }
    if (trajectory_snapshot.last_rejected_valid != 0U)
    {
      trajectory_status.flags |=
        APP_CAN_TRAJECTORY_STATUS_FLAG_LAST_REJECTED_VALID;
      trajectory_status.last_rejected_sequence =
        trajectory_snapshot.last_rejected_sequence;
    }
    if ((safety_request & APP_CAN_SAFETY_REQUEST_HOLD_POSITION) != 0U)
    {
      trajectory_status.flags |=
        APP_CAN_TRAJECTORY_STATUS_FLAG_SAFETY_HOLD_LATCHED;
    }
    if ((safety_request & APP_CAN_SAFETY_REQUEST_QUICK_STOP) != 0U)
    {
      trajectory_status.flags |=
        APP_CAN_TRAJECTORY_STATUS_FLAG_QUICK_STOP_LATCHED;
    }

    trajectory_status.reject_reason =
      (APP_CAN_TrajectoryRejectReason)trajectory_snapshot.reject_reason;
    trajectory_status.hold_reason =
      (APP_CAN_TrajectoryHoldReason)trajectory_snapshot.hold_reason;
    trajectory_status.generation = (uint16_t)trajectory_snapshot.generation;
    trajectory_status.last_executed_ecat_cycle =
      (uint16_t)trajectory_snapshot.last_executed_ecat_cycle;
    trajectory_status.accepted_count =
      (uint16_t)trajectory_snapshot.accepted_count;
    trajectory_status.executed_count =
      (uint16_t)trajectory_snapshot.executed_count;
    trajectory_status.rejected_count =
      (uint16_t)trajectory_snapshot.rejected_count;
    trajectory_status.underrun_count =
      (uint16_t)trajectory_snapshot.underrun_count;
    trajectory_status.overflow_count =
      (uint16_t)trajectory_snapshot.overflow_count;
    trajectory_status.expired_count =
      (uint16_t)trajectory_snapshot.expired_count;

    if (APP_CAN_PackTrajectoryStatus(&trajectory_status,
                                     &trajectory_frame) != APP_CAN_OK)
    {
      trajectory_frame_ready = false;
    }
  }

  if (snapshot_available != 0U)
  {
    /* 按关节映射表将物理PDO反馈重排为Joint1..Joint6逻辑顺序。 */
    for (joint_index = 0U; joint_index < ROBOT_JOINT_COUNT; joint_index++)
    {
      config = ROBOT_JOINT_GetConfig(joint_index);
      if (config == 0)
      {
        continue;
      }

      slave_index = (uint8_t)(config->slave - 1U);
      joint_feedback = &snapshot.slave[slave_index];
      if (config->axis == ROBOT_JOINT_AXIS_1)
      {
        position_feedback.actual_position[joint_index] =
          joint_feedback->position_actual_value;
        axis_status.actual_velocity[joint_index] =
          joint_feedback->velocity_actual_value;
        axis_status.actual_torque[joint_index] =
          joint_feedback->torque_actual_value;
        axis_status.statusword[joint_index] = joint_feedback->statusword;
        axis_status.error_code[joint_index] = joint_feedback->error_code;
      }
      else
      {
        position_feedback.actual_position[joint_index] =
          joint_feedback->axis2_position_actual_value;
        axis_status.actual_velocity[joint_index] =
          joint_feedback->axis2_velocity_actual_value;
        axis_status.actual_torque[joint_index] =
          joint_feedback->axis2_torque_actual_value;
        axis_status.statusword[joint_index] =
          joint_feedback->axis2_statusword;
        axis_status.error_code[joint_index] =
          joint_feedback->axis2_error_code;
      }

      if ((axis_status.statusword[joint_index] & 0x006FU) == 0x0027U)
      {
        enabled_axis_mask |= (uint8_t)(1U << joint_index);
      }

      if ((axis_status.statusword[joint_index] & 0x0080U) != 0U)
      {
        position_feedback.flags |= APP_CAN_FEEDBACK_FLAG_WARNING;
      }
      if (((axis_status.statusword[joint_index] & 0x0008U) != 0U) ||
          (axis_status.error_code[joint_index] != 0U))
      {
        position_feedback.flags |= APP_CAN_FEEDBACK_FLAG_FAULT;
      }
    }
  }

  if ((position_feedback.valid_axis_mask == APP_CAN_AXIS_VALID_MASK) &&
      (enabled_axis_mask == APP_CAN_AXIS_VALID_MASK))
  {
    position_feedback.flags |= APP_CAN_FEEDBACK_FLAG_AXES_ENABLED;
  }

  if ((current_tick_ms - app_can_last_position_tx_tick) >=
      APP_CAN_POSITION_FEEDBACK_PERIOD_MS)
  {
    position_feedback.sequence = app_can_position_tx_sequence;
    if ((APP_CAN_PackPositionFeedback(&position_feedback, &frame) == APP_CAN_OK) &&
        APP_CAN_SendPreparedFrame(&frame,
                                  &app_can_stats.tx_position_feedbacks))
    {
      app_can_position_tx_sequence++;
      app_can_last_position_tx_tick = current_tick_ms;
    }
  }

  if (trajectory_frame_ready)
  {
    trajectory_status_changed =
      (!app_can_last_trajectory_status_payload_valid) ||
      (memcmp(&trajectory_frame.data[
                APP_CAN_TRAJECTORY_STATUS_PAYLOAD_OFFSET],
              app_can_last_trajectory_status_payload,
              APP_CAN_TRAJECTORY_STATUS_PAYLOAD_SIZE) != 0);

    if (trajectory_status_changed)
    {
      /* 变化报文绕过20 ms保活周期，但仍保持2 ms最小发送间隔。 */
      trajectory_status_period_ms =
        APP_CAN_TRAJECTORY_STATUS_ACTIVE_PERIOD_MS;
    }
    else if ((trajectory_status.state ==
                APP_CAN_TRAJECTORY_STATUS_PREFILL) ||
             (trajectory_status.state ==
                APP_CAN_TRAJECTORY_STATUS_RUNNING))
    {
      trajectory_status_period_ms =
        APP_CAN_TRAJECTORY_STATUS_ACTIVE_PERIOD_MS;
    }
    else
    {
      trajectory_status_period_ms =
        APP_CAN_TRAJECTORY_STATUS_INACTIVE_PERIOD_MS;
    }

    if ((current_tick_ms - app_can_last_trajectory_status_tx_tick) >=
        trajectory_status_period_ms)
    {
      if (APP_CAN_SendPreparedFrame(
            &trajectory_frame,
            &app_can_stats.tx_trajectory_statuses))
      {
        app_can_trajectory_status_tx_sequence++;
        app_can_last_trajectory_status_tx_tick = current_tick_ms;
        memcpy(app_can_last_trajectory_status_payload,
               &trajectory_frame.data[
                 APP_CAN_TRAJECTORY_STATUS_PAYLOAD_OFFSET],
               APP_CAN_TRAJECTORY_STATUS_PAYLOAD_SIZE);
        app_can_last_trajectory_status_payload_valid = true;
      }
    }
  }

  if ((current_tick_ms - app_can_last_status_tx_tick) >=
      APP_CAN_AXIS_STATUS_PERIOD_MS)
  {
    axis_status.sequence = app_can_status_tx_sequence;
    if ((APP_CAN_PackAxisStatus(&axis_status, &frame) == APP_CAN_OK) &&
        APP_CAN_SendPreparedFrame(&frame,
                                  &app_can_stats.tx_axis_statuses))
    {
      app_can_status_tx_sequence++;
      app_can_last_status_tx_tick = current_tick_ms;
    }
  }

  primask = __get_PRIMASK();
  __disable_irq();
  diagnostic_pending = app_can_diagnostic_pending;
  diagnostic = app_can_pending_diagnostic;
  __set_PRIMASK(primask);

  if (diagnostic_pending)
  {
    diagnostic.sequence = app_can_diagnostic_tx_sequence;
    if ((APP_CAN_PackDiagnostic(&diagnostic, &frame) == APP_CAN_OK) &&
        APP_CAN_SendPreparedFrame(&frame, &app_can_stats.tx_diagnostics))
    {
      primask = __get_PRIMASK();
      __disable_irq();
      app_can_diagnostic_pending = false;
      __DMB();
      __set_PRIMASK(primask);
      app_can_diagnostic_tx_sequence++;
    }
  }

  if ((current_tick_ms - app_can_last_heartbeat_tx_tick) >=
      APP_CAN_DEVICE_HEARTBEAT_PERIOD_MS)
  {
    heartbeat.sequence = app_can_heartbeat_tx_sequence;
    if ((APP_CAN_PackDeviceHeartbeat(&heartbeat, &frame) == APP_CAN_OK) &&
        APP_CAN_SendPreparedFrame(&frame, &app_can_stats.tx_heartbeats))
    {
      app_can_heartbeat_tx_sequence++;
      app_can_last_heartbeat_tx_tick = current_tick_ms;
    }
  }
}

void APP_CAN_GetStats(APP_CAN_Stats *stats)
{
  uint32_t primask;

  if (stats == NULL)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  __DMB();
  *stats = app_can_stats;
  __DMB();
  __set_PRIMASK(primask);
}

/**
 * @brief  清零全部 CAN 应用层运行统计
 *
 * @return
 * 无
 *
 * @warning
 * 本函数会在短临界区内不可恢复地清除 app_can_stats，可在 APP_CAN_Init() 前后调用；
 * 它不会清除 BSP CAN 统计，也不会改变已发布命令、心跳、报文序号、超时与安全状态、
 * 待发送诊断或发送序号。调用方如需保留当前统计，应先通过 APP_CAN_GetStats() 获取快照
 */
void APP_CAN_ResetStats(void)
{
  APP_CAN_Stats empty_stats = {0};
  uint32_t primask;

  /* 统计清零不影响已发布命令及各报文序号状态。 */
  primask = __get_PRIMASK();
  __disable_irq();
  __DMB();
  app_can_stats = empty_stats;
  __DMB();
  __set_PRIMASK(primask);
}
