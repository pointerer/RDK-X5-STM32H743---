#ifndef __APP_CAN_H__
#define __APP_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "bsp_can.h"

#define APP_CAN_AXIS_COUNT                  6U
#define APP_CAN_AXIS_VALID_MASK             0x3FU
#define APP_CAN_PROTOCOL_VERSION            2U
#define APP_CAN_CSP_MODE                    8U

#define APP_CAN_ID_CSP_COMMAND              0x180U /* 接收：六轴 CSP 轨迹目标命令 */
#define APP_CAN_ID_MOTION_CONTROL           0x181U /* 接收：使能、保持、快停等运动控制命令 */
#define APP_CAN_ID_PARAMETER_REQUEST        0x182U /* 接收：参数读写请求 */
#define APP_CAN_ID_HOST_HEARTBEAT            0x183U /* 接收：上位机心跳 */
#define APP_CAN_RX_FILTER_MASK              0x7FCU /* 忽略标准 ID 最低两位，匹配 0x180～0x183 */

#define APP_CAN_ID_POSITION_FEEDBACK        0x200U
#define APP_CAN_ID_AXIS_STATUS              0x201U
#define APP_CAN_ID_DIAGNOSTIC               0x202U
#define APP_CAN_ID_DEVICE_HEARTBEAT          0x203U
#define APP_CAN_ID_TRAJECTORY_STATUS         0x204U

#define APP_CAN_DLC_CSP_COMMAND             32U
#define APP_CAN_DLC_MOTION_CONTROL          16U
#define APP_CAN_DLC_PARAMETER_REQUEST       16U
#define APP_CAN_DLC_HOST_HEARTBEAT          8U

#define APP_CAN_DLC_POSITION_FEEDBACK       32U
#define APP_CAN_DLC_AXIS_STATUS             64U
#define APP_CAN_DLC_DIAGNOSTIC              16U
#define APP_CAN_DLC_DEVICE_HEARTBEAT         8U
#define APP_CAN_DLC_TRAJECTORY_STATUS       32U

#define APP_CAN_HOST_FLAG_TRAJECTORY_STATUS_SUPPORTED 0x01U
#define APP_CAN_HOST_FLAG_MASK                        0x01U

#define APP_CAN_CSP_FLAG_APPLY              0x01U /* 按连续序号追加当前轨迹点 */
#define APP_CAN_CSP_FLAG_HOLD               0x02U /* 清空轨迹并请求位置保持 */
#define APP_CAN_CSP_FLAG_TRAJECTORY_RESET   0x04U /* 轨迹重建标志，仅与 APPLY 组合使用 */
#define APP_CAN_CSP_FLAG_RESET_APPLY        \
  (APP_CAN_CSP_FLAG_TRAJECTORY_RESET | APP_CAN_CSP_FLAG_APPLY) /* 清空旧轨迹并以当前点重建序列 */

#define APP_CAN_PARAMETER_TARGET_GLOBAL     0xFFU
#define APP_CAN_DIAGNOSTIC_SOURCE_GLOBAL    0xFFU

#define APP_CAN_FEEDBACK_FLAG_OPERATIONAL   0x01U
#define APP_CAN_FEEDBACK_FLAG_AXES_ENABLED  0x02U
#define APP_CAN_FEEDBACK_FLAG_CMD_TIMEOUT   0x04U
#define APP_CAN_FEEDBACK_FLAG_QUICK_STOP    0x08U
#define APP_CAN_FEEDBACK_FLAG_WARNING       0x10U
#define APP_CAN_FEEDBACK_FLAG_FAULT         0x20U
#define APP_CAN_FEEDBACK_FLAG_BUS_OFF       0x40U
#define APP_CAN_FEEDBACK_FLAG_MASK          0x7FU

#define APP_CAN_TRAJECTORY_STATUS_FLAG_RESET_REQUIRED          0x01U
#define APP_CAN_TRAJECTORY_STATUS_FLAG_EXPECTED_SEQUENCE_VALID 0x02U
#define APP_CAN_TRAJECTORY_STATUS_FLAG_LAST_RECEIVED_VALID     0x04U
#define APP_CAN_TRAJECTORY_STATUS_FLAG_LAST_ACCEPTED_VALID     0x08U
#define APP_CAN_TRAJECTORY_STATUS_FLAG_LAST_EXECUTED_VALID     0x10U
#define APP_CAN_TRAJECTORY_STATUS_FLAG_LAST_REJECTED_VALID     0x20U
#define APP_CAN_TRAJECTORY_STATUS_FLAG_SAFETY_HOLD_LATCHED     0x40U
#define APP_CAN_TRAJECTORY_STATUS_FLAG_QUICK_STOP_LATCHED      0x80U
#define APP_CAN_TRAJECTORY_STATUS_FLAG_MASK                    0xFFU

#define APP_CAN_SLAVE_COUNT                 3U
#define APP_CAN_SLAVE_VALID_MASK            0x07U

#define APP_CAN_COMMAND_TIMEOUT_MS          20U
#define APP_CAN_HOST_HEARTBEAT_TIMEOUT_MS   300U
#define APP_CAN_POSITION_FEEDBACK_PERIOD_MS 2U
#define APP_CAN_AXIS_STATUS_PERIOD_MS       10U
#define APP_CAN_DEVICE_HEARTBEAT_PERIOD_MS  100U
#define APP_CAN_TRAJECTORY_STATUS_ACTIVE_PERIOD_MS   2U
#define APP_CAN_TRAJECTORY_STATUS_INACTIVE_PERIOD_MS 20U

typedef enum
{
  APP_CAN_OK = 0,
  APP_CAN_INVALID_ARGUMENT,
  APP_CAN_NOT_READY,
  APP_CAN_BSP_ERROR
} APP_CAN_Result;

typedef enum
{
  APP_CAN_TRAJECTORY_STATUS_IDLE    = 0x00U,
  APP_CAN_TRAJECTORY_STATUS_PREFILL = 0x01U,
  APP_CAN_TRAJECTORY_STATUS_RUNNING = 0x02U,
  APP_CAN_TRAJECTORY_STATUS_HOLD    = 0x03U
} APP_CAN_TrajectoryStatusState;

typedef enum
{
  APP_CAN_TRAJECTORY_REJECT_NONE              = 0x00U,
  APP_CAN_TRAJECTORY_REJECT_BAD_FRAME_FORMAT  = 0x01U,
  APP_CAN_TRAJECTORY_REJECT_BAD_DLC           = 0x02U,
  APP_CAN_TRAJECTORY_REJECT_CRC_ERROR         = 0x03U,
  APP_CAN_TRAJECTORY_REJECT_INVALID_AXIS_MASK = 0x04U,
  APP_CAN_TRAJECTORY_REJECT_INVALID_FLAGS     = 0x05U,
  APP_CAN_TRAJECTORY_REJECT_INVALID_MODE      = 0x06U,
  APP_CAN_TRAJECTORY_REJECT_INVALID_VALIDITY  = 0x07U,
  APP_CAN_TRAJECTORY_REJECT_RESET_REQUIRED    = 0x08U,
  APP_CAN_TRAJECTORY_REJECT_SEQUENCE_ERROR    = 0x09U,
  APP_CAN_TRAJECTORY_REJECT_QUEUE_FULL        = 0x0AU,
  APP_CAN_TRAJECTORY_REJECT_MOTION_NOT_READY  = 0x0BU,
  APP_CAN_TRAJECTORY_REJECT_INTERNAL_ERROR    = 0x0CU
} APP_CAN_TrajectoryRejectReason;

typedef enum
{
  APP_CAN_TRAJECTORY_HOLD_NONE                   = 0x00U,
  APP_CAN_TRAJECTORY_HOLD_EXPLICIT_0X180         = 0x01U,
  APP_CAN_TRAJECTORY_HOLD_EXPLICIT_0X181         = 0x02U,
  APP_CAN_TRAJECTORY_HOLD_QUICK_STOP             = 0x03U,
  APP_CAN_TRAJECTORY_HOLD_QUEUE_UNDERRUN         = 0x04U,
  APP_CAN_TRAJECTORY_HOLD_QUEUE_OVERFLOW         = 0x05U,
  APP_CAN_TRAJECTORY_HOLD_SEQUENCE_ERROR         = 0x06U,
  APP_CAN_TRAJECTORY_HOLD_POINT_EXPIRED          = 0x07U,
  APP_CAN_TRAJECTORY_HOLD_CSP_COMMAND_TIMEOUT    = 0x08U,
  APP_CAN_TRAJECTORY_HOLD_HOST_HEARTBEAT_TIMEOUT = 0x09U,
  APP_CAN_TRAJECTORY_HOLD_ETHERCAT_NOT_OP        = 0x0AU,
  APP_CAN_TRAJECTORY_HOLD_DRIVE_NOT_CSP          = 0x0BU,
  APP_CAN_TRAJECTORY_HOLD_DRIVE_FAULT            = 0x0CU,
  APP_CAN_TRAJECTORY_HOLD_INTERNAL_ERROR          = 0x0DU
} APP_CAN_TrajectoryHoldReason;

typedef enum
{
  APP_CAN_MOTION_DISABLE = 0,
  APP_CAN_MOTION_ENABLE,
  APP_CAN_MOTION_HOLD,
  APP_CAN_MOTION_QUICK_STOP,
  APP_CAN_MOTION_FAULT_RESET
} APP_CAN_MotionCommandType;

typedef enum
{
  APP_CAN_PARAMETER_READ = 0,
  APP_CAN_PARAMETER_WRITE
} APP_CAN_ParameterOperation;

typedef enum
{
  APP_CAN_DIAGNOSTIC_INFO = 0,
  APP_CAN_DIAGNOSTIC_WARNING,
  APP_CAN_DIAGNOSTIC_ERROR,
  APP_CAN_DIAGNOSTIC_FATAL
} APP_CAN_DiagnosticSeverity;

typedef enum
{
  APP_CAN_SAFETY_REQUEST_NONE          = 0x00U,
  APP_CAN_SAFETY_REQUEST_HOLD_POSITION = 0x01U,
  APP_CAN_SAFETY_REQUEST_QUICK_STOP    = 0x02U
} APP_CAN_SafetyRequest;

typedef struct
{
  uint8_t sequence;                         /* 报文循环序号 */
  uint8_t valid_axis_mask;                  /* 六轴目标有效掩码 */
  uint8_t flags;                            /* 应用、保持及轨迹重置标志 */
  uint8_t mode;                             /* 固定为CSP模式8 */
  int32_t target_position[APP_CAN_AXIS_COUNT]; /* Joint1..Joint6原始编码器绝对计数 */
  uint16_t validity_ms;                     /* 本条目标的有效时间 */
  uint32_t received_tick_ms;                /* 本地接收时刻 */
} APP_CAN_CspCommand;

typedef struct
{
  uint8_t sequence;                         /* 报文循环序号 */
  APP_CAN_MotionCommandType command;        /* 语义化运动控制命令 */
  uint8_t axis_mask;                        /* 命令作用轴掩码 */
  uint8_t flags;                            /* 命令附加标志 */
  uint32_t request_token;                   /* 上位机请求标识 */
  uint32_t host_time_ms;                    /* 上位机命令时间 */
  uint32_t received_tick_ms;                /* 本地接收时刻 */
} APP_CAN_MotionControl;

typedef struct
{
  uint8_t sequence;                         /* 报文循环序号 */
  APP_CAN_ParameterOperation operation;     /* 参数读取或写入 */
  uint8_t target_axis;                      /* 目标轴号或全局目标 */
  uint8_t flags;                            /* 参数请求附加标志 */
  uint16_t parameter_id;                    /* 应用层参数编号 */
  int32_t value;                            /* 参数写入值 */
  uint16_t request_token;                   /* 参数请求标识 */
  uint32_t received_tick_ms;                /* 本地接收时刻 */
} APP_CAN_ParameterRequest;

typedef struct
{
  uint8_t sequence;                         /* 报文循环序号 */
  uint8_t protocol_version;                 /* 上位机协议版本 */
  uint8_t host_state;                       /* ROS2上位机状态 */
  uint8_t flags;                            /* 心跳附加状态标志 */
  uint16_t host_uptime_100ms;               /* 上位机运行时间低16位 */
  uint32_t received_tick_ms;                /* 本地接收时刻 */
} APP_CAN_HostHeartbeat;

typedef struct
{
  uint8_t sequence;                         /* 报文循环序号 */
  uint8_t valid_axis_mask;                  /* 六轴反馈有效掩码 */
  uint8_t ecat_state;                       /* EtherCAT主站运行状态 */
  uint8_t flags;                            /* 使能、超时及故障状态 */
  int32_t actual_position[APP_CAN_AXIS_COUNT]; /* Joint1..Joint6原始编码器绝对计数 */
  uint16_t cycle_counter;                   /* EtherCAT周期计数低16位 */
} APP_CAN_PositionFeedback;

typedef struct
{
  uint8_t sequence;                         /* 报文循环序号 */
  uint8_t valid_axis_mask;                  /* 六轴状态有效掩码 */
  int32_t actual_velocity[APP_CAN_AXIS_COUNT]; /* 六轴实际速度 */
  int16_t actual_torque[APP_CAN_AXIS_COUNT];   /* 六轴实际转矩 */
  uint16_t statusword[APP_CAN_AXIS_COUNT];  /* 六轴CiA402状态字 */
  uint16_t error_code[APP_CAN_AXIS_COUNT];  /* 六轴驱动器错误码 */
} APP_CAN_AxisStatusFeedback;

typedef struct
{
  uint8_t sequence;                         /* 报文循环序号 */
  APP_CAN_DiagnosticSeverity severity;      /* 诊断严重等级 */
  uint8_t source_axis;                      /* 来源轴号或全局来源 */
  uint8_t flags;                            /* 诊断附加标志 */
  uint16_t error_code;                      /* 主错误码 */
  uint16_t detail_code;                     /* 详细错误码 */
  uint32_t context;                         /* 错误现场值 */
  uint16_t event_counter;                   /* 诊断事件计数 */
} APP_CAN_Diagnostic;

typedef struct
{
  uint8_t sequence;                         /* 报文循环序号 */
  uint8_t ecat_state;                       /* EtherCAT主站运行状态 */
  uint8_t operational_slave_mask;           /* 三个从站OP状态掩码 */
  uint16_t working_counter;                 /* 最近一次有效WKC */
} APP_CAN_DeviceHeartbeat;

/*
 * 0x204轨迹状态逻辑数据。后续打包函数在报文字节1写入固定协议版本，并在
 * 字节30～31追加CRC16。枚举大小和结构体填充由编译器决定，禁止使用memcpy
 * 将本结构体直接复制为CAN报文，必须由打包函数逐字段编码。
 */
typedef struct
{
  uint8_t sequence;                         /* 报文循环序号，对应字节0 */
  APP_CAN_TrajectoryStatusState state;      /* 轨迹运行状态，对应字节2 */
  uint8_t queue_depth;                      /* 当前轨迹队列深度，对应字节3 */
  uint8_t queue_capacity;                   /* 轨迹队列总容量，对应字节4 */
  uint8_t flags;                            /* 轨迹状态有效位及锁存标志，对应字节5 */
  uint8_t last_received_sequence;           /* 最近通过基础校验的序号，对应字节6 */
  uint8_t last_accepted_sequence;           /* 最近成功入队的轨迹点序号，对应字节7 */
  uint8_t last_executed_sequence;           /* 最近完成EtherCAT执行的序号，对应字节8 */
  uint8_t expected_sequence;                /* 下一条期望接收的严格连续序号，对应字节9 */
  uint8_t last_rejected_sequence;           /* 最近被拒绝的轨迹点序号，对应字节10 */
  APP_CAN_TrajectoryRejectReason reject_reason; /* 最近一次拒绝原因，对应字节11 */
  APP_CAN_TrajectoryHoldReason hold_reason; /* 当前锁存的HOLD原因，对应字节12 */
  uint8_t prefill_target;                   /* 启动前要求达到的预缓存点数，对应字节13 */
  uint16_t generation;                      /* 轨迹代次低16位，对应字节14～15 */
  uint16_t last_executed_ecat_cycle;        /* 最近执行点的EtherCAT周期低16位，对应字节16～17 */
  uint16_t accepted_count;                  /* 成功接受轨迹点计数低16位，对应字节18～19 */
  uint16_t executed_count;                  /* 成功执行轨迹点计数低16位，对应字节20～21 */
  uint16_t rejected_count;                  /* 被拒绝报文计数低16位，对应字节22～23 */
  uint16_t underrun_count;                  /* 轨迹队列欠载次数低16位，对应字节24～25 */
  uint16_t overflow_count;                  /* 轨迹队列溢出次数低16位，对应字节26～27 */
  uint16_t expired_count;                   /* 轨迹点过期次数低16位，对应字节28～29 */
} APP_CAN_TrajectoryStatus;

typedef struct
{
  uint32_t rx_frames;                       /* 应用层取出的总帧数 */
  uint32_t accepted_csp_commands;           /* 有效CSP位置命令数 */
  uint32_t accepted_motion_controls;        /* 有效运动控制命令数 */
  uint32_t accepted_parameter_requests;     /* 有效参数请求数 */
  uint32_t accepted_heartbeats;             /* 有效上位机心跳数 */
  uint32_t invalid_ids;                     /* 非协议ID帧数 */
  uint32_t invalid_formats;                 /* 非标准FD+BRS帧数 */
  uint32_t invalid_lengths;                 /* DLC不匹配帧数 */
  uint32_t crc_errors;                      /* CRC校验失败帧数 */
  uint32_t sequence_errors;                 /* 重复或回退序号帧数 */
  uint32_t payload_errors;                  /* 数据字段非法帧数 */
  uint32_t command_timeouts;                /* CSP命令超时次数 */
  uint32_t heartbeat_timeouts;              /* 上位机心跳超时次数 */
  uint32_t safety_request_clears;            /* 安全请求清除次数 */
  uint32_t tx_position_feedbacks;            /* 六轴位置反馈发送数 */
  uint32_t tx_axis_statuses;                 /* 六轴详细状态发送数 */
  uint32_t tx_trajectory_statuses;           /* 轨迹状态反馈发送数 */
  uint32_t tx_diagnostics;                   /* 诊断事件发送数 */
  uint32_t tx_heartbeats;                    /* 下位机心跳发送数 */
  uint32_t tx_busy;                          /* 发送FIFO忙次数 */
  uint32_t tx_errors;                        /* 应用层发送失败次数 */
  uint32_t diagnostic_queue_full;            /* 诊断事件槽占用次数 */
} APP_CAN_Stats;

APP_CAN_Result APP_CAN_Init(void);
void APP_CAN_ProcessRx(void);
bool APP_CAN_GetLatestCspCommand(APP_CAN_CspCommand *command);
bool APP_CAN_GetLatestMotionControl(APP_CAN_MotionControl *control);
bool APP_CAN_GetLatestParameterRequest(APP_CAN_ParameterRequest *request);
bool APP_CAN_GetLatestHostHeartbeat(APP_CAN_HostHeartbeat *heartbeat);
APP_CAN_Result APP_CAN_PackPositionFeedback(
  const APP_CAN_PositionFeedback *feedback,
  BSP_CAN_Frame *frame);
APP_CAN_Result APP_CAN_PackAxisStatus(
  const APP_CAN_AxisStatusFeedback *status,
  BSP_CAN_Frame *frame);
APP_CAN_Result APP_CAN_PackDiagnostic(
  const APP_CAN_Diagnostic *diagnostic,
  BSP_CAN_Frame *frame);
APP_CAN_Result APP_CAN_PackDeviceHeartbeat(
  const APP_CAN_DeviceHeartbeat *heartbeat,
  BSP_CAN_Frame *frame);
APP_CAN_Result APP_CAN_PackTrajectoryStatus(
  const APP_CAN_TrajectoryStatus *status,
  BSP_CAN_Frame *frame);
void APP_CAN_CheckTimeout(uint32_t current_tick_ms);
bool APP_CAN_IsCommandTimedOut(void);
bool APP_CAN_IsHostOnline(void);
APP_CAN_SafetyRequest APP_CAN_GetSafetyRequest(void);
bool APP_CAN_ClearSafetyRequest(void);
bool APP_CAN_QueueDiagnostic(const APP_CAN_Diagnostic *diagnostic);
void APP_CAN_ProcessTx(uint32_t current_tick_ms);
void APP_CAN_GetStats(APP_CAN_Stats *stats);
void APP_CAN_ResetStats(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CAN_H__ */
