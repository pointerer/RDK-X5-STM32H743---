#ifndef ROBOT_TRAJECTORY_H
#define ROBOT_TRAJECTORY_H

#include <stdint.h>

#include "robot_joint.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOT_TRAJECTORY_BUFFER_CAPACITY  15U
#define ROBOT_TRAJECTORY_PREFILL_COUNT      8U
#define ROBOT_TRAJECTORY_EMPTY_GRACE_CYCLES 3U

/* 数值与0x204的HOLD原因字段保持一致。 */
#define ROBOT_TRAJECTORY_HOLD_REASON_NONE             0x00U
#define ROBOT_TRAJECTORY_HOLD_REASON_EXPLICIT_0X180   0x01U
#define ROBOT_TRAJECTORY_HOLD_REASON_EXPLICIT_0X181   0x02U
#define ROBOT_TRAJECTORY_HOLD_REASON_QUICK_STOP       0x03U
#define ROBOT_TRAJECTORY_HOLD_REASON_QUEUE_UNDERRUN   0x04U
#define ROBOT_TRAJECTORY_HOLD_REASON_QUEUE_OVERFLOW   0x05U
#define ROBOT_TRAJECTORY_HOLD_REASON_SEQUENCE_ERROR   0x06U
#define ROBOT_TRAJECTORY_HOLD_REASON_POINT_EXPIRED    0x07U
#define ROBOT_TRAJECTORY_HOLD_REASON_CSP_TIMEOUT      0x08U
#define ROBOT_TRAJECTORY_HOLD_REASON_HOST_TIMEOUT     0x09U
#define ROBOT_TRAJECTORY_HOLD_REASON_ETHERCAT_NOT_OP  0x0AU
#define ROBOT_TRAJECTORY_HOLD_REASON_DRIVE_NOT_CSP    0x0BU
#define ROBOT_TRAJECTORY_HOLD_REASON_DRIVE_FAULT      0x0CU
#define ROBOT_TRAJECTORY_HOLD_REASON_INTERNAL_ERROR   0x0DU

typedef enum
{
  ROBOT_TRAJECTORY_IDLE = 0,
  ROBOT_TRAJECTORY_PREFILL,
  ROBOT_TRAJECTORY_RUNNING,
  ROBOT_TRAJECTORY_HOLD
} ROBOT_TrajectoryState;

typedef enum
{
  ROBOT_TRAJECTORY_PUSH_OK = 0,
  ROBOT_TRAJECTORY_PUSH_INVALID,
  ROBOT_TRAJECTORY_PUSH_RESET_REQUIRED,
  ROBOT_TRAJECTORY_PUSH_SEQUENCE_ERROR,
  ROBOT_TRAJECTORY_PUSH_FULL
} ROBOT_TrajectoryPushResult;

typedef enum
{
  ROBOT_TRAJECTORY_TAKE_WAIT = 0,
  ROBOT_TRAJECTORY_TAKE_POINT,
  ROBOT_TRAJECTORY_TAKE_GRACE,
  ROBOT_TRAJECTORY_TAKE_HOLD,
  ROBOT_TRAJECTORY_TAKE_INVALID
} ROBOT_TrajectoryTakeResult;

typedef struct
{
  uint8_t sequence;
  int32_t target_position[ROBOT_JOINT_COUNT];
  uint16_t validity_ms;
  uint32_t received_tick_ms;
} ROBOT_TrajectoryPoint;

typedef struct
{
  ROBOT_TrajectoryState state;              /* 当前轨迹运行状态 */
  uint8_t reset_required;                   /* 非0表示继续入队前必须重建轨迹序列 */
  uint8_t sequence_valid;                   /* expected_sequence 是否有效 */
  uint8_t expected_sequence;                /* 下一轨迹点应携带的循环序号 */
  uint8_t last_executed_sequence;           /* 最近一次EtherCAT确认执行的轨迹点序号 */
  uint8_t head;                             /* 环形缓冲区下一待读取槽索引 */
  uint8_t tail;                             /* 环形缓冲区下一待写入槽索引 */
  uint8_t count;                            /* 当前已排队的有效轨迹点数量 */
  uint8_t empty_grace_cycles;               /* RUNNING空队列已保持的连续EtherCAT周期数 */
  ROBOT_TrajectoryPoint points[ROBOT_TRAJECTORY_BUFFER_CAPACITY]; /* 轨迹点环形缓冲区 */

  /* 0x204轨迹状态反馈所需的只读观测数据；后续步骤再接入更新和快照接口。 */
  uint8_t last_received_valid;              /* 最近接收序号是否有效 */
  uint8_t last_received_sequence;           /* 最近通过帧格式、长度和CRC校验的0x180序号 */
  uint8_t last_accepted_valid;              /* 最近接受序号是否有效 */
  uint8_t last_accepted_sequence;           /* 最近成功入队的轨迹点序号 */
  uint8_t last_executed_valid;              /* 最近EtherCAT确认执行序号是否有效 */
  uint8_t staged_valid;                     /* 是否有等待EtherCAT确认的轨迹点 */
  uint8_t staged_sequence;                  /* 已写入下一次RxPDO输出映像的序号 */
  uint32_t staged_generation;               /* 已暂存轨迹点所属的完整代次 */
  uint8_t last_rejected_valid;              /* 最近拒绝序号是否有效 */
  uint8_t last_rejected_sequence;           /* 最近被拒绝的0x180序号 */
  uint8_t reject_reason;                    /* 最近拒绝原因，数值对应0x204拒绝原因枚举 */
  uint8_t hold_reason;                      /* 锁存的HOLD原因，数值对应0x204 HOLD原因枚举 */
  uint32_t generation;                      /* 轨迹代次；0x204发送低16位 */
  uint32_t last_executed_ecat_cycle;        /* 最近确认执行点的EtherCAT周期；发送低16位 */
  uint32_t accepted_count;                  /* 成功接受轨迹点累计数；发送低16位 */
  uint32_t executed_count;                  /* 成功执行轨迹点累计数；发送低16位 */
  uint32_t rejected_count;                  /* 被拒绝0x180报文累计数；发送低16位 */
  uint32_t underrun_count;                  /* 轨迹队列欠载累计数；发送低16位 */
  uint32_t empty_grace_recovery_count;      /* 宽限期内收到新点并恢复的累计次数 */
  uint32_t overflow_count;                  /* 轨迹队列溢出累计数；发送低16位 */
  uint32_t expired_count;                   /* 轨迹点过期累计数；发送低16位 */
} ROBOT_TrajectoryContext;

typedef struct
{
  ROBOT_TrajectoryState state;              /* 当前轨迹运行状态 */
  uint8_t reset_required;                   /* 是否必须通过RESET重建轨迹 */
  uint8_t expected_sequence_valid;          /* 下一期望序号是否有效 */
  uint8_t expected_sequence;                /* 下一条期望接收的严格连续序号 */
  uint8_t queue_depth;                      /* 当前已排队轨迹点数量 */
  uint8_t last_received_valid;              /* 最近接收序号是否有效 */
  uint8_t last_received_sequence;           /* 最近通过基础校验的0x180序号 */
  uint8_t last_accepted_valid;              /* 最近接受序号是否有效 */
  uint8_t last_accepted_sequence;           /* 最近成功入队的轨迹点序号 */
  uint8_t last_executed_valid;              /* 最近确认执行序号是否有效 */
  uint8_t last_executed_sequence;           /* 最近确认完成EtherCAT执行的序号 */
  uint8_t last_rejected_valid;              /* 最近拒绝序号是否有效 */
  uint8_t last_rejected_sequence;           /* 最近被拒绝的0x180序号 */
  uint8_t reject_reason;                    /* 最近拒绝原因 */
  uint8_t hold_reason;                      /* 当前锁存的HOLD原因 */
  uint32_t generation;                      /* 轨迹代次 */
  uint32_t last_executed_ecat_cycle;        /* 最近确认执行点的EtherCAT周期 */
  uint32_t accepted_count;                  /* 成功接受轨迹点累计数 */
  uint32_t executed_count;                  /* 成功执行轨迹点累计数 */
  uint32_t rejected_count;                  /* 被拒绝0x180报文累计数 */
  uint32_t underrun_count;                  /* 轨迹队列欠载累计数 */
  uint32_t empty_grace_recovery_count;      /* 宽限期内收到新点并恢复的累计次数 */
  uint32_t overflow_count;                  /* 轨迹队列溢出累计数 */
  uint32_t expired_count;                   /* 轨迹点过期累计数 */
} ROBOT_TrajectorySnapshot;

void ROBOT_TrajectoryInit(void);
void ROBOT_TrajectoryRequireReset(void);
void ROBOT_TrajectoryRequireResetWithReason(uint8_t hold_reason);
uint8_t ROBOT_TrajectoryIsHold(void);
uint8_t ROBOT_TrajectoryGetSnapshot(
  ROBOT_TrajectorySnapshot *snapshot);
void ROBOT_TrajectoryRecordReceived(uint8_t sequence);
void ROBOT_TrajectoryRecordRejected(
  uint8_t sequence,
  uint8_t sequence_valid,
  uint8_t reject_reason);
uint8_t ROBOT_TrajectoryResetAndPush(const ROBOT_TrajectoryPoint *point);
ROBOT_TrajectoryPushResult ROBOT_TrajectoryPush(
  const ROBOT_TrajectoryPoint *point);
ROBOT_TrajectoryTakeResult ROBOT_TrajectoryTake(
  ROBOT_TrajectoryPoint *point,
  uint32_t current_tick_ms,
  uint32_t *generation);
uint8_t ROBOT_TrajectoryStageExecution(
  uint8_t sequence,
  uint32_t generation);
uint8_t ROBOT_TrajectoryConfirmExecution(uint32_t ecat_cycle);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_TRAJECTORY_H */
