#include "ECAT_runtime.h"

#include "ECAT_diag.h"
#include "app_can.h"
#include "bsp_soem_timebase.h"
#include "cmsis_compiler.h"
#include "robot_joint.h"
#include "robot_trajectory.h"
#include "stm32h7xx_hal.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define ECAT_RUNTIME_COMMUNICATION_TEST_ONLY 0U

#define ECAT_RUNTIME_GROUP                0U
#define ECAT_RUNTIME_OP_WAIT_CYCLES       50U
#define ECAT_RUNTIME_OP_STABLE_CYCLES     3U
#define ECAT_RUNTIME_OP_CYCLE_US          2000U
#define ECAT_RUNTIME_WKC_FAULT_LIMIT      100U
#define ECAT_RUNTIME_WKC_ROLLBACK_LIMIT   3U
#define ECAT_RUNTIME_TXPDO_PARSE_INTERVAL 1000U
#define ECAT_RUNTIME_CW06_DELAY_CYCLES    2500U
#define ECAT_RUNTIME_CW06_READY_TIMEOUT   1000U
#define ECAT_RUNTIME_CW06_READY_LOSS      10U
#define ECAT_RUNTIME_CW07_SWITCH_TIMEOUT  1000U
#define ECAT_RUNTIME_CW07_SWITCH_LOSS     10U
#define ECAT_RUNTIME_CW0F_TARGET_TIMEOUT  1000U
#define ECAT_RUNTIME_CW0F_ENABLE_TIMEOUT  1000U
#define ECAT_RUNTIME_CW0F_CW07_TIMEOUT    1000U
#define ECAT_RUNTIME_CW0F_POSITION_TOLERANCE 1U
#define ECAT_RUNTIME_CW0F_VELOCITY_TOLERANCE 1U
#define ECAT_RUNTIME_CW0F_TARGET_STABLE_CYCLES 3U
#define ECAT_RUNTIME_BRAKE_DELAY_US              10000U
#define ECAT_RUNTIME_BRAKE_OPEN_COMMAND          0x11000000U
#define ECAT_RUNTIME_BRAKE_OPENED_FEEDBACK       0x00300000U
#define ECAT_RUNTIME_BRAKE_EXECUTE_COMMAND       0x22000000U
#define ECAT_RUNTIME_BRAKE_CLEAR_COMMAND         0x33000000U
#define ECAT_RUNTIME_BRAKE_EXECUTED_FEEDBACK     0x00400000U
/* 两轴抱闸流程完成后开始0x0006使能序列。 */
#define ECAT_RUNTIME_AFTER_BRAKE_STATE ECAT_RUNTIME_CW06_WAIT_DELAY

/* Temporary counters for observing runtime brake SDO submission/rollback. */
volatile uint32 ecat_dbg_sdo_write_submits;
volatile uint32 ecat_dbg_sdo_write_cancels;
volatile uint32 ecat_dbg_txpdo_report_dropped;

/* Only ECAT_RuntimeProcessTxPdoReport() uses this single-task report buffer. */
static char ecat_runtime_txpdo_report_buffer[ECAT_LOG_BLOCK_BUFFER_SIZE];

typedef enum
{
  ECAT_RUNTIME_PDO_CYCLE_OK = 0,
  ECAT_RUNTIME_PDO_CYCLE_SLAVE_MISSING,
  ECAT_RUNTIME_PDO_CYCLE_EXCHANGE_FAILED
} ECAT_RuntimePdoCycleResult;

typedef enum
{
  ECAT_RUNTIME_BRAKE_STEP_IDLE = 0,                   /* 初始化抱闸流程 */
  ECAT_RUNTIME_BRAKE_STEP_SET_MODE_11,                /* 下发模式11 */
  ECAT_RUNTIME_BRAKE_STEP_WRITE_OPEN_COMMAND,         /* 写入开抱闸命令 */
  ECAT_RUNTIME_BRAKE_STEP_WAIT_OPEN_FEEDBACK,         /* 等待开抱闸完成反馈 */
  ECAT_RUNTIME_BRAKE_STEP_WAIT_OPEN_10_MS,            /* 开抱闸后等待10ms */
  ECAT_RUNTIME_BRAKE_STEP_WRITE_OPEN_CLEAR_COMMAND,   /* 清除开抱闸反馈 */
  ECAT_RUNTIME_BRAKE_STEP_WAIT_OPEN_FEEDBACK_CLEAR,   /* 等待开抱闸反馈清零 */
  ECAT_RUNTIME_BRAKE_STEP_WRITE_EXECUTE_COMMAND,      /* 写入关抱闸命令 */
  ECAT_RUNTIME_BRAKE_STEP_WAIT_EXECUTE_FEEDBACK,      /* 等待关抱闸完成反馈 */
  ECAT_RUNTIME_BRAKE_STEP_WAIT_10_MS,                 /* 关抱闸后等待10ms */
  ECAT_RUNTIME_BRAKE_STEP_WRITE_CLEAR_COMMAND,        /* 清除关抱闸反馈 */
  ECAT_RUNTIME_BRAKE_STEP_WAIT_FEEDBACK_CLEAR,        /* 等待关抱闸反馈清零 */
  ECAT_RUNTIME_BRAKE_STEP_WRITE_COMMAND_ZERO,         /* 清零命令寄存器 */
  ECAT_RUNTIME_BRAKE_STEP_RELATCH_TARGET_AND_SET_MODE_8, /* 重锁目标并恢复模式8 */
  ECAT_RUNTIME_BRAKE_STEP_DONE                        /* 抱闸流程完成 */
} ECAT_RuntimeBrakeStep;

typedef enum
{
  ECAT_RUNTIME_BRAKE_SDO_WAIT = 0,
  ECAT_RUNTIME_BRAKE_SDO_COMPLETE,
  ECAT_RUNTIME_BRAKE_SDO_FAILED
} ECAT_RuntimeBrakeSdoResult;

static uint8 ECAT_RuntimeDecodeSlaveTxPdo(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave,
  AppEtherCAT_ServoTxPdo *txpdo);
static uint8 ECAT_RuntimeAppendTxPdoReport(
  uint32 *used,
  const char *format,
  ...);
static uint8 ECAT_RuntimeAllManagedAxesMatch(
  const ecx_contextt *context,
  uint16 state_mask,
  uint16 state_value,
  uint8 *fault_found);
static uint8 ECAT_RuntimeSetAllManagedControlwords(
  ecx_contextt *context,
  uint16 controlword);
static ECAT_RuntimeResult ECAT_RuntimeRollbackToZero(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime);
static ECAT_RuntimeResult ECAT_RuntimeRollbackToSwitchedOn(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime);
static uint8 ECAT_RuntimeRelatchAllManagedTargets(
  ecx_contextt *context);
static uint8 ECAT_RuntimeAllManagedAxesPassEnableCheck(const ecx_contextt *context,
  uint32 position_tolerance, uint32 velocity_tolerance, uint8 *fault_found);
static uint8 ECAT_RuntimeCaptureTxPdoSnapshots(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime);
static uint8 ECAT_RuntimePrepareSafeRxPdo(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave,
  const AppEtherCAT_ServoTxPdo *txpdo);
static ECAT_RuntimeResult ECAT_RuntimeRequestSafeOpState(
  ecx_contextt *context,
  ECAT_LogContext *log);
static ECAT_RuntimeResult ECAT_RuntimeExchangeSafeOpPdo(
  ecx_contextt *context,
  ECAT_LogContext *log,
  ECAT_RuntimeContext *runtime);
static uint8 ECAT_RuntimePreloadSafeOutputs(
  ecx_contextt *context,
  ECAT_LogContext *log,
  int expected_wkc);
static uint32 ECAT_RuntimeWaitForOpStable(
  ecx_contextt *context,
  int expected_wkc,
  int *last_wkc,
  uint16 *last_state);
static void ECAT_RuntimeFallbackSafeOp(
  ecx_contextt *context,
  ECAT_LogContext *log,
  int expected_wkc,
  uint32 stable_cycles,
  int last_wkc,
  uint16 last_state);
static ECAT_RuntimePdoCycleResult ECAT_RuntimeExchangePdoCycle(
  ecx_contextt *context,
  int expected_wkc,
  int *send_ret,
  int *receive_wkc);
static ECAT_RuntimeResult ECAT_RuntimeHandlePdoExchangeFailure(
  ecx_contextt *context,
  ECAT_LogContext *log,
  ECAT_RuntimeContext *runtime,
  int send_ret,
  int receive_wkc);
static void ECAT_RuntimeHandleWaitDelay(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime);
static ECAT_RuntimeResult ECAT_RuntimeHandleWaitReady(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime);
static ECAT_RuntimeResult ECAT_RuntimeHandleReadyToSwitchOn(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime);
static ECAT_RuntimeResult ECAT_RuntimeHandleWaitSwitchedOn(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime);
static ECAT_RuntimeResult ECAT_RuntimeHandleSwitchedOn(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime);
static ECAT_RuntimeResult ECAT_RuntimeHandleWaitTargetStable(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime);
static ECAT_RuntimeResult ECAT_RuntimeHandleWaitOperationEnabled(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime);
static ECAT_RuntimeResult ECAT_RuntimeHandleOperationEnabled(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime);
static uint8 ECAT_RuntimeSetBrakeAxisMode(
  ecx_contextt *context,
  uint16 slave,
  uint8 axis,
  int8_t mode);
static ECAT_RuntimeBrakeSdoResult ECAT_RuntimeBrakeProcessSdo(
  ECAT_RuntimeSdoJob *job,
  ECAT_RuntimeSdoJobState operation,
  uint16 slave,
  uint16 index,
  uint32 request_wire,
  uint32 *response_wire);
static uint8 ECAT_RuntimePrepareBrakeRollback(
  ECAT_RuntimeContext *runtime,
  uint8 execute_write_pending);
static ECAT_RuntimeResult ECAT_RuntimeHandleAxis1BrakeRelease(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime);
static ECAT_RuntimeResult ECAT_RuntimeHandleAxis2BrakeRelease(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime);
static ECAT_RuntimeResult ECAT_RuntimeHandleBrakeRelease(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime,
  uint8 axis);
static ECAT_RuntimeResult ECAT_RuntimeHandleTest(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime);
static ECAT_RuntimeResult ECAT_RuntimeHandleRollbackToSwitchedOn(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime);
static void ECAT_RuntimeHandleRollbackZero(
  ECAT_RuntimeContext *runtime);
static void ECAT_RuntimeHandleFailed(
  ecx_contextt *context);
static ECAT_RuntimeResult ECAT_RuntimeDispatchDriveState(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime);

void ECAT_RuntimeReset(ECAT_RuntimeContext *runtime)
{
  memset(runtime, 0, sizeof(*runtime));
}

void ECAT_RuntimeResetAfterError(ECAT_RuntimeContext *runtime)
{
  runtime->expected_wkc = 0;
  runtime->bad_wkc_continuous = 0U;
  runtime->pdo_cycle_count = 0U;
  runtime->successful_pdo_count = 0U;
  runtime->drive_state = ECAT_RUNTIME_CW06_DISABLED;
  runtime->state_cycle_count = 0U;
  runtime->target_stable_cycle_count = 0U;
  runtime->test_position_offset = 0U;
  runtime->test_completed = 0U;
  runtime->test_keys_armed = 0U;
  memset(&runtime->brake, 0, sizeof(runtime->brake));
  memset(&runtime->sdo_job, 0, sizeof(runtime->sdo_job));
  runtime->enable_operation_request = 0U;
  runtime->txpdo_snapshot_consumed_sequence =
    runtime->txpdo_snapshot_publish_sequence;
}

uint8 ECAT_RuntimeRequestEnableOperation(ECAT_RuntimeContext *runtime)
{
  uint32 primask;
  uint8 accepted = 0U;
  if (runtime == 0)
  {
    return 0U;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  if ((runtime->drive_state == ECAT_RUNTIME_CW07_DONE) &&
      (runtime->enable_operation_request == 0U))
  {
    runtime->enable_operation_request = 1U;
    __DMB();
    accepted = 1U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
  return accepted;
}

/**
 * @brief  刷新并输出 SOEM 主站及全部已发现从站的 AL 状态
 *
 * @param[in,out] context   SOEM 主站上下文，读取结果会更新从站状态列表
 * @param[in,out] log       EtherCAT 日志上下文
 *
 * @return
 * 无
 *
 * @warning
 * context 和 log 必须有效，且调用前必须完成从站发现；该函数会同步访问 EtherCAT
 * 总线，ecx_readstate() 的结果仅记录到日志，不通过返回值向调用方报告
 */
void ECAT_RuntimeReadAllStates(ecx_contextt *context,
                                         ECAT_LogContext *log)
{
  int read_state_ret;
  int slave;

  if (context->slavecount <= 0)
  {
    ECAT_LogPrintf(log, "[SOEM] readstate skip: no slave\r\n");
    return;
  }

  read_state_ret = ecx_readstate(context);
  ECAT_LogPrintf(
    log,
    "[SOEM] ecx_readstate ret=0x%04X slavecount=%d\r\n",
    (unsigned int)read_state_ret,
    context->slavecount);
  ECAT_LogPrintf(log,
                          "[SOEM] slave0 state=0x%04X AL=0x%04X\r\n",
                          context->slavelist[0].state,
                          context->slavelist[0].ALstatuscode);

  for (slave = 1; slave <= context->slavecount; slave++)
  {
    ECAT_LogPrintf(
      log,
      "[SOEM] slave%d state=0x%04X AL=0x%04X configadr=0x%04X\r\n",
      slave,
      context->slavelist[slave].state,
      context->slavelist[slave].ALstatuscode,
      context->slavelist[slave].configadr);
  }
}

/**
 * @brief  检查缓存中的全部已发现从站是否处于 PRE-OP 状态
 *
 * @param[in]     context   SOEM 主站上下文，提供从站数量及状态缓存
 * @param[in,out] log       EtherCAT 日志上下文
 *
 * @return
 * 全部从站均处于PRE-OP返回1，任一从站状态不符返回0
 *
 * @warning
 * context 和 log 必须有效，调用前应通过 ECAT_RuntimeReadAllStates() 刷新状态；
 * context->slavecount 必须大于0，否则空从站列表会被判定为通过
 */
uint8 ECAT_RuntimeCheckPreOp(ecx_contextt *context,
                                      ECAT_LogContext *log)
{
  int slave;
  int fail_count = 0;

  for (slave = 1; slave <= context->slavecount; slave++)
  {
    if (context->slavelist[slave].state != EC_STATE_PRE_OP)
    {
      fail_count++;
      ECAT_LogPrintf(
        log,
        "[SOEM] PRE-OP FAIL slave%d state=0x%04X AL=0x%04X configadr=0x%04X\r\n",
        slave,
        context->slavelist[slave].state,
        context->slavelist[slave].ALstatuscode,
        context->slavelist[slave].configadr);
    }
  }

  if (fail_count == 0)
  {
    ECAT_LogPrintf(log,
                            "[SOEM] PRE-OP PASS all slaves=%d\r\n",
                            context->slavecount);
    return 1U;
  }

  ECAT_LogPrintf(log,
                          "[SOEM] PRE-OP FAIL count=%d/%d\r\n",
                          fail_count,
                          context->slavecount);
  return 0U;
}

/**
 * @brief  请求全部受管从站进入 SAFE-OP 并完成首次安全过程数据交换
 *
 * @param[in,out] context SOEM 主站上下文，函数会更新从站状态并访问过程数据区
 * @param[in,out] log     EtherCAT 日志上下文
 * @param[in,out] runtime 运行时上下文，函数会记录期望工作计数
 *
 * @return
 * 成功返回 ECAT_RUNTIME_RESULT_OK；从站缺失、映射后未保持 PRE-OP、SAFE-OP 请求失败、
 * TxPDO 解码失败或安全 RxPDO 准备失败时返回对应的 ECAT_RuntimeResult 错误码
 *
 * @warning
 * context、log 和 runtime 必须有效，调用前必须完成从站发现及 PDO 映射，且全部受管
 * 从站应处于 PRE-OP。本函数会阻塞执行状态读写、状态等待及一次过程数据收发，
 * 并根据接收的 TxPDO 写入安全 RxPDO；失败时不负责恢复从站状态
 */
ECAT_RuntimeResult ECAT_RuntimeEnterSafeOp(
  ecx_contextt *context,
  ECAT_LogContext *log,
  ECAT_RuntimeContext *runtime)
{
  ECAT_RuntimeResult result;

  if (context->slavecount < (int)ECAT_LAST_SLAVE)
  {
    ECAT_LogPrintf(
      log,
      "[SOEM] SAFE-OP skip: slavecount=%d need=%u\r\n",
      context->slavecount,
      (unsigned int)ECAT_LAST_SLAVE);
    return ECAT_RUNTIME_RESULT_SAFEOP_SLAVE_MISSING;
  }

  result = ECAT_RuntimeRequestSafeOpState(context, log);
  if (result != ECAT_RUNTIME_RESULT_OK)
  {
    return result;
  }

  result = ECAT_RuntimeExchangeSafeOpPdo(context, log, runtime);
  if (result != ECAT_RUNTIME_RESULT_OK)
  {
    return result;
  }

  ECAT_LogPrintf(log,
                          "[SOEM] SAFE-OP processdata done\r\n");
  return ECAT_RUNTIME_RESULT_OK;
}

/**
 * @brief  复核 PRE-OP 状态并请求全部已发现从站进入 SAFE-OP
 *
 * @param[in,out] context SOEM 主站上下文，函数会刷新并修改从站状态缓存
 * @param[in,out] log     EtherCAT 日志上下文
 *
 * @return
 * 全部从站进入 SAFE-OP 返回 ECAT_RUNTIME_RESULT_OK；映射后未保持 PRE-OP 返回
 * ECAT_RUNTIME_RESULT_MAPPING_LEFT_PREOP；状态等待失败返回 ECAT_RUNTIME_RESULT_SAFEOP_FAILED
 *
 * @warning
 * context 和 log 必须有效，调用前必须完成从站发现及 PDO 映射，且 slavecount 必须
 * 大于0。本函数会通过从站 0 广播状态请求并按 EC_TIMEOUTSTATE 阻塞等待；
 * ecx_writestate() 的返回值仅记录日志，失败时不会主动恢复到 PRE-OP
 */
static ECAT_RuntimeResult ECAT_RuntimeRequestSafeOpState(
  ecx_contextt *context,
  ECAT_LogContext *log)
{
  int write_state_ret;
  uint16 check_state_ret;
  uint8 preop_ok;

  ECAT_LogSectionLine(log);
  ECAT_LogPrintf(log, "[SOEM] SAFE-OP request start\r\n");
  ECAT_LogPrintf(log,
                          "[SOEM] state before SAFE-OP request\r\n");
  ECAT_RuntimeReadAllStates(context, log);

  preop_ok = ECAT_RuntimeCheckPreOp(context, log);
  ECAT_LogPrintf(log,
                          "[SOEM] SAFE-OP sequence PRE-OP=%s\r\n",
                          (preop_ok != 0U) ? "PASS" : "FAIL");
  if (preop_ok == 0U)
  {
    ECAT_LogPrintf(
      log,
      "[SOEM] SAFE-OP abort: mapping did not remain in PRE-OP\r\n");
    return ECAT_RUNTIME_RESULT_MAPPING_LEFT_PREOP;
  }

  context->slavelist[0].state = EC_STATE_SAFE_OP;
  write_state_ret = ecx_writestate(context, 0);
  ECAT_LogPrintf(log,
                          "[SOEM] request SAFE-OP write_ret=%d\r\n",
                          write_state_ret);

  check_state_ret = ecx_statecheck(context,
                                   0,
                                   EC_STATE_SAFE_OP,
                                   EC_TIMEOUTSTATE);
  ECAT_LogPrintf(log,
                          "[SOEM] statecheck SAFE-OP ret=0x%04X %s\r\n",
                          check_state_ret,
                          (check_state_ret == EC_STATE_SAFE_OP) ?
                            "PASS" : "FAIL");
  ECAT_LogPrintf(log,
                          "[SOEM] state after SAFE-OP request\r\n");
  ECAT_RuntimeReadAllStates(context, log);

  if (check_state_ret != EC_STATE_SAFE_OP)
  {
    ECAT_LogPrintf(log,
                            "[SOEM] processdata skip: SAFE-OP failed\r\n");
    ECAT_LogPrintf(log, "[SOEM] OP not requested\r\n");
    return ECAT_RUNTIME_RESULT_SAFEOP_FAILED;
  }

  return ECAT_RUNTIME_RESULT_OK;
}

/**
 * @brief  在 SAFE-OP 下交换一次 PDO、计算期望 WKC 并准备安全 RxPDO
 *
 * @param[in,out] context SOEM 主站上下文，函数会读取输入区并写入输出区
 * @param[in,out] log     EtherCAT 日志上下文
 * @param[in,out] runtime 运行时上下文，函数会更新 expected_wkc
 *
 * @return
 * 所有受管从站的 TxPDO 均解码成功且安全 RxPDO 准备完成返回
 * ECAT_RUNTIME_RESULT_OK；否则返回首先遇到的 ECAT_RUNTIME_RESULT_TXPDO_DECODE_FAILED
 * 或 ECAT_RUNTIME_RESULT_SAFE_RXPDO_PREPARE_FAILED
 *
 * @warning
 * context、log 和 runtime 必须有效，全部受管从站应已进入 SAFE-OP 且过程数据指针及
 * 长度有效；本函数会按 EC_TIMEOUTRET 阻塞接收并改写各从站输出区。发送返回值和实际
 * WKC 仅记录日志、不参与失败判定；TxPDO 解码失败时会尝试写入全零模式及位置的安全输出
 */
static ECAT_RuntimeResult ECAT_RuntimeExchangeSafeOpPdo(
  ecx_contextt *context,
  ECAT_LogContext *log,
  ECAT_RuntimeContext *runtime)
{
  const uint8 group = ECAT_RUNTIME_GROUP;
  ECAT_RuntimeResult result = ECAT_RUNTIME_RESULT_OK;
  int send_ret;
  int receive_wkc;
  uint16 slave;
  AppEtherCAT_ServoTxPdo txpdo;

  send_ret = ecx_send_processdata(context);
  receive_wkc = ecx_receive_processdata(context, EC_TIMEOUTRET);
  runtime->expected_wkc =
    ((int)context->grouplist[group].outputsWKC * 2) +
    (int)context->grouplist[group].inputsWKC;

  ECAT_LogPrintf(log,
                          "[SOEM] send_processdata ret=%d\r\n",
                          send_ret);
  ECAT_LogPrintf(
    log,
    "[SOEM] receive_processdata WKC=%d expected=%d\r\n",
    receive_wkc,
    runtime->expected_wkc);
  for (slave = ECAT_FIRST_SLAVE; slave <= ECAT_LAST_SLAVE; slave++)
  {
    ECAT_DiagPrintInputsRaw(log,
                           slave,
                           context->slavelist[slave].inputs,
                           context->slavelist[slave].Ibytes,
                           APP_ETHERCAT_SERVO_TXPDO_SIZE);

    if (ECAT_RuntimeDecodeSlaveTxPdo(context,
                                     log,
                                     slave,
                                     &txpdo) == 0U)
    {
      if (result == ECAT_RUNTIME_RESULT_OK)
      {
        result = ECAT_RUNTIME_RESULT_TXPDO_DECODE_FAILED;
      }
      (void)ECAT_RuntimePrepareSafeRxPdo(context, log, slave, 0);
      continue;
    }

    if (ECAT_RuntimePrepareSafeRxPdo(context,
                                     log,
                                     slave,
                                     &txpdo) == 0U)
    {
      ECAT_LogPrintf(
        log,
        "[SOEM] slave%u OP safe test skip: safe RxPDO prepare failed\r\n",
        (unsigned int)slave);
      if (result == ECAT_RUNTIME_RESULT_OK)
      {
        result = ECAT_RUNTIME_RESULT_SAFE_RXPDO_PREPARE_FAILED;
      }
    }
  }

  return result;
}

/**
 * @brief  预发送安全输出并请求全部受管从站稳定进入 OPERATIONAL
 *
 * @param[in,out] context SOEM 主站上下文，函数会请求状态并持续交换过程数据
 * @param[in,out] log     EtherCAT 日志上下文
 * @param[in,out] runtime 运行时上下文，提供期望 WKC 并接收运行计数的初始化结果
 *
 * @return
 * OP 状态和期望 WKC 连续达到规定稳定周期返回 ECAT_RUNTIME_RESULT_OK；从站缺失、
 * 安全输出预发送失败或稳定等待超时返回 ECAT_RUNTIME_RESULT_OP_REQUEST_FAILED
 *
 * @warning
 * context、log 和 runtime 必须有效，全部受管从站必须已进入 SAFE-OP，过程数据映射
 * 有效且 runtime->expected_wkc 已正确计算。函数最多执行 ECAT_RUNTIME_OP_WAIT_CYCLES
 * 个阻塞交换周期；失败时仅尝试回退 SAFE-OP，成功时会复位运行计数并启动 CW06 延时流程
 */
ECAT_RuntimeResult ECAT_RuntimeRequestOp(
  ecx_contextt *context,
  ECAT_LogContext *log,
  ECAT_RuntimeContext *runtime)
{
  uint32 stable_cycles;
  int last_wkc;
  uint16 last_state;
  uint16 slave;
  uint8 operational_mask;
  uint8 expected_operational_mask;

  if (context->slavecount < (int)ECAT_LAST_SLAVE)
  {
    ECAT_LogPrintf(
      log,
      "[SOEM] OP request skip: slavecount=%d need=%u\r\n",
      context->slavecount,
      (unsigned int)ECAT_LAST_SLAVE);
    return ECAT_RUNTIME_RESULT_OP_REQUEST_FAILED;
  }

  if (ECAT_RuntimePreloadSafeOutputs(context,
                                               log,
                                               runtime->expected_wkc) == 0U)
  {
    return ECAT_RUNTIME_RESULT_OP_REQUEST_FAILED;
  }

  ECAT_LogPrintf(
    log,
    "[SOEM] request OP, require %u stable cycles at %u us\r\n",
    (unsigned int)ECAT_RUNTIME_OP_STABLE_CYCLES,
    (unsigned int)ECAT_RUNTIME_OP_CYCLE_US);
  context->slavelist[0].state = EC_STATE_OPERATIONAL;
  (void)ecx_writestate(context, 0);

  stable_cycles = ECAT_RuntimeWaitForOpStable(
    context,
    runtime->expected_wkc,
    &last_wkc,
    &last_state);
  if (stable_cycles < ECAT_RUNTIME_OP_STABLE_CYCLES)
  {
    ECAT_RuntimeFallbackSafeOp(context,
                                        log,
                                        runtime->expected_wkc,
                                        stable_cycles,
                                        last_wkc,
                                        last_state);
    return ECAT_RUNTIME_RESULT_OP_REQUEST_FAILED;
  }

  /*
   * ecx_statecheck(context, 0, ...) only refreshes the aggregate slave0
   * state.  Refresh every slave once before the drive sequence so CAN
   * heartbeat byte 3 reflects the actual managed-slave OP state.
   */
  ECAT_RuntimeReadAllStates(context, log);
  operational_mask = 0U;
  for (slave = ECAT_FIRST_SLAVE; slave <= ECAT_LAST_SLAVE; slave++)
  {
    if (context->slavelist[slave].state == EC_STATE_OPERATIONAL)
    {
      operational_mask |=
        (uint8)(1U << (slave - ECAT_FIRST_SLAVE));
    }
  }
  expected_operational_mask = (uint8)((1U << ECAT_SLAVE_COUNT) - 1U);
  ECAT_LogPrintf(
    log,
    "[SOEM] managed OP mask=0x%02X expected=0x%02X\r\n",
    (unsigned int)operational_mask,
    (unsigned int)expected_operational_mask);
  if (operational_mask != expected_operational_mask)
  {
    ECAT_RuntimeFallbackSafeOp(context,
                               log,
                               runtime->expected_wkc,
                               stable_cycles,
                               last_wkc,
                               last_state);
    return ECAT_RUNTIME_RESULT_OP_REQUEST_FAILED;
  }

  runtime->bad_wkc_continuous = 0U;
  runtime->pdo_cycle_count = 0U;
  runtime->successful_pdo_count = 0U;
  runtime->state_cycle_count = 0U;
  runtime->target_stable_cycle_count = 0U;
  runtime->test_position_offset = 0U;
  runtime->test_completed = 0U;
  runtime->test_keys_armed = 0U;
  memset(&runtime->brake, 0, sizeof(runtime->brake));
  memset(&runtime->sdo_job, 0, sizeof(runtime->sdo_job));
  runtime->enable_operation_request = 0U;
#if ECAT_RUNTIME_COMMUNICATION_TEST_ONLY
  /*
   * Communication-only firmware: keep all drives disabled after EtherCAT
   * reaches OP.  This prevents entry into both brake-release state machines;
   * PDO exchange and CAN feedback continue with the safe controlword 0x0000.
   */
  runtime->drive_state = ECAT_RUNTIME_CW06_DISABLED;
  ECAT_LogPrintf(
    log,
    "[SOEM] COMMUNICATION TEST ONLY: brake-release sequence inhibited\r\n");
#else
  runtime->drive_state = ECAT_RUNTIME_AXIS1_BRAKE_RELEASE;
#endif
  runtime->txpdo_snapshot_publish_sequence = 0U;
  runtime->txpdo_snapshot_consumed_sequence = 0U;
  return ECAT_RUNTIME_RESULT_OK;
}

/**
 * @brief  在请求 OP 前发送一次已准备好的安全 RxPDO 输出
 *
 * @param[in,out] context      SOEM 主站上下文
 * @param[in,out] log          EtherCAT 日志上下文
 * @param[in]     expected_wkc 期望工作计数，仅用于诊断日志
 *
 * @return
 * 过程数据发送返回值和接收 WKC 均大于0返回1，否则记录失败日志并返回0
 *
 * @warning
 * context 和 log 必须有效，全部受管从站应处于 SAFE-OP，且输出映像必须已填充控制字
 * 为 0x0000 的安全 RxPDO。本函数会按 EC_TIMEOUTRET 阻塞接收；它不要求实际 WKC
 * 等于 expected_wkc，非零但不完整的 WKC 仍会被判定为成功
 */
static uint8 ECAT_RuntimePreloadSafeOutputs(
  ecx_contextt *context,
  ECAT_LogContext *log,
  int expected_wkc)
{
  int send_ret;
  int receive_wkc;

  ECAT_LogSectionLine(log);
  ECAT_LogPrintf(
    log,
    "[SOEM] OP request start, controlword stays 0x0000\r\n");
  send_ret = ecx_send_processdata(context);
  receive_wkc = ecx_receive_processdata(context, EC_TIMEOUTRET);
  ECAT_LogPrintf(
    log,
    "[SOEM] before OP safe PDO send=%d WKC=%d expected=%d\r\n",
    send_ret,
    receive_wkc,
    expected_wkc);

  if ((send_ret <= 0) || (receive_wkc <= 0))
  {
    ECAT_LogPrintf(log,
                            "[SOEM] OP preload FAIL: send=%d WKC=%d\r\n",
                            send_ret,
                            receive_wkc);
    return 0U;
  }

  return 1U;
}

/**
 * @brief  循环交换过程数据并等待 OP 状态与 WKC 连续稳定
 *
 * @param[in,out] context      SOEM 主站上下文
 * @param[in]     expected_wkc 期望工作计数
 * @param[out]    last_wkc     最后一次接收的工作计数
 * @param[out]    last_state   最后一次检查到的主站状态
 *
 * @return
 * 返回退出循环时累计的连续稳定周期数；达到 ECAT_RUNTIME_OP_STABLE_CYCLES 表示成功，
 * 否则表示在 ECAT_RUNTIME_OP_WAIT_CYCLES 内未稳定
 *
 * @warning
 * context、last_wkc 和 last_state 必须有效，expected_wkc 必须已正确计算，且调用前应已
 * 广播请求 OP。本函数会反复按 EC_TIMEOUTRET 阻塞收包和检查状态，并在周期间延时；
 * 任一状态或 WKC 不匹配都会清零连续计数，ecx_send_processdata() 的返回值会被忽略
 */
static uint32 ECAT_RuntimeWaitForOpStable(
  ecx_contextt *context,
  int expected_wkc,
  int *last_wkc,
  uint16 *last_state)
{
  uint32 wait_cycle;
  uint32 stable_cycles = 0U;

  *last_wkc = 0;
  *last_state = 0U;
  for (wait_cycle = 0U;
       wait_cycle < ECAT_RUNTIME_OP_WAIT_CYCLES;
       wait_cycle++)
  {
    (void)ecx_send_processdata(context);
    *last_wkc = ecx_receive_processdata(context, EC_TIMEOUTRET);
    *last_state = ecx_statecheck(context,
                                 0,
                                 EC_STATE_OPERATIONAL,
                                 EC_TIMEOUTRET);

    if ((*last_state == EC_STATE_OPERATIONAL) &&
        (*last_wkc == expected_wkc))
    {
      stable_cycles++;
    }
    else
    {
      stable_cycles = 0U;
    }

    if (stable_cycles >= ECAT_RUNTIME_OP_STABLE_CYCLES)
    {
      break;
    }

    BSP_SOEM_Timebase_DelayUs(ECAT_RUNTIME_OP_CYCLE_US);
  }

  return stable_cycles;
}

/**
 * @brief  记录 OP 稳定失败信息并尝试将全部从站回退到 SAFE-OP
 *
 * @param[in,out] context       SOEM 主站上下文，函数会修改并刷新从站状态缓存
 * @param[in,out] log           EtherCAT 日志上下文
 * @param[in]     expected_wkc  期望工作计数，用于失败诊断
 * @param[in]     stable_cycles 退出等待时的连续稳定周期数
 * @param[in]     last_wkc      最后一次接收的工作计数
 * @param[in]     last_state    最后一次检查到的主站状态
 *
 * @return
 * 无
 *
 * @warning
 * context 和 log 必须有效，且至少存在一个已发现从站；本函数通过从站 0 广播回退请求，
 * 并按 EC_TIMEOUTSTATE 阻塞检查结果。写入和状态检查结果仅记录日志，不保证回退成功；
 * 函数不会修改安全输出映像或复位运行时上下文
 */
static void ECAT_RuntimeFallbackSafeOp(
  ecx_contextt *context,
  ECAT_LogContext *log,
  int expected_wkc,
  uint32 stable_cycles,
  int last_wkc,
  uint16 last_state)
{
  int write_state_ret;
  uint16 safe_state_ret;

  ECAT_LogPrintf(
    log,
    "[SOEM] OP stable FAIL: state=0x%04X WKC=%d expected=%d stable=%lu/%u\r\n",
    last_state,
    last_wkc,
    expected_wkc,
    (unsigned long)stable_cycles,
    (unsigned int)ECAT_RUNTIME_OP_STABLE_CYCLES);
  context->slavelist[0].state = EC_STATE_SAFE_OP;
  write_state_ret = ecx_writestate(context, 0);
  safe_state_ret = ecx_statecheck(context,
                                  0,
                                  EC_STATE_SAFE_OP,
                                  EC_TIMEOUTSTATE);
  ECAT_LogPrintf(
    log,
    "[SOEM] fallback SAFE-OP write_ret=%d state=0x%04X %s\r\n",
    write_state_ret,
    safe_state_ret,
    (safe_state_ret == EC_STATE_SAFE_OP) ? "PASS" : "FAIL");
  ECAT_RuntimeReadAllStates(context, log);
}

/* 执行一次 PDO 收发并按发送结果及 WKC 判定本周期是否有效。 */
static ECAT_RuntimePdoCycleResult ECAT_RuntimeExchangePdoCycle(
  ecx_contextt *context,
  int expected_wkc,
  int *send_ret,
  int *receive_wkc)
{
  *send_ret = 0;
  *receive_wkc = 0;
  if (context->slavecount < (int)ECAT_LAST_SLAVE)
  {
    return ECAT_RUNTIME_PDO_CYCLE_SLAVE_MISSING;
  }

  *send_ret = ecx_send_processdata(context);
  *receive_wkc = ecx_receive_processdata(context, EC_TIMEOUTRET);
  if ((*send_ret > 0) && (*receive_wkc == expected_wkc))
  {
    return ECAT_RUNTIME_PDO_CYCLE_OK;
  }

  return ECAT_RUNTIME_PDO_CYCLE_EXCHANGE_FAILED;
}

/* 暂存 0x0000，写入成功后统一进入回零过渡状态。 */
static ECAT_RuntimeResult ECAT_RuntimeRollbackToZero(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime)
{
  if (ECAT_RuntimeSetAllManagedControlwords(context, 0x0000U) == 0U)
  {
    return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
  }

  runtime->drive_state = ECAT_RUNTIME_CW06_ROLLBACK_ZERO;
  return ECAT_RUNTIME_RESULT_OK;
}

/* 暂存 0x0007，写入成功后统一进入 Switched On 回退等待状态。 */
static ECAT_RuntimeResult ECAT_RuntimeRollbackToSwitchedOn(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime)
{
  if (ECAT_RuntimeSetAllManagedControlwords(context, 0x0007U) == 0U)
  {
    return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
  }

  runtime->drive_state = ECAT_RUNTIME_CW0F_ROLLBACK_CW07;
  return ECAT_RUNTIME_RESULT_OK;
}

/* 处理单周期 PDO 通信异常，并执行原有容错及安全回退。 */
static ECAT_RuntimeResult ECAT_RuntimeHandlePdoExchangeFailure(
  ecx_contextt *context,
  ECAT_LogContext *log,
  ECAT_RuntimeContext *runtime,
  int send_ret,
  int receive_wkc)
{
  runtime->bad_wkc_continuous++;
  if (runtime->bad_wkc_continuous < ECAT_RUNTIME_WKC_ROLLBACK_LIMIT)
  {
    return ECAT_RUNTIME_RESULT_OK;
  }
  if (runtime->drive_state == ECAT_RUNTIME_CW06_WAIT_DELAY)
  {
    runtime->state_cycle_count = 0U;
  }
  else if ((runtime->drive_state == ECAT_RUNTIME_CW06_WAIT_READY) ||
           (runtime->drive_state == ECAT_RUNTIME_CW06_DONE) ||
           (runtime->drive_state == ECAT_RUNTIME_CW07_WAIT_SWITCHED_ON) ||
           (runtime->drive_state == ECAT_RUNTIME_CW07_DONE) ||
           (runtime->drive_state == ECAT_RUNTIME_CW0F_WAIT_TARGET_STABLE) ||
           (runtime->drive_state == ECAT_RUNTIME_CW0F_WAIT_OPERATION_ENABLED) ||
           (runtime->drive_state == ECAT_RUNTIME_CW0F_DONE) ||
           (runtime->drive_state == ECAT_RUNTIME_AXIS1_BRAKE_RELEASE) ||
           (runtime->drive_state == ECAT_RUNTIME_AXIS2_BRAKE_RELEASE) ||
           (runtime->drive_state == ECAT_RUNTIME_TEST) ||
           (runtime->drive_state == ECAT_RUNTIME_CW0F_ROLLBACK_CW07))
  {
    runtime->state_cycle_count = 0U;
    runtime->target_stable_cycle_count = 0U;
    runtime->enable_operation_request = 0U;
    if (runtime->drive_state == ECAT_RUNTIME_TEST)
    {
      ROBOT_TrajectoryRequireResetWithReason(
        ROBOT_TRAJECTORY_HOLD_REASON_ETHERCAT_NOT_OP);
    }
    if ((runtime->drive_state == ECAT_RUNTIME_AXIS1_BRAKE_RELEASE) ||
        (runtime->drive_state == ECAT_RUNTIME_AXIS2_BRAKE_RELEASE))
    {
      if ((runtime->brake.step >
           ECAT_RUNTIME_BRAKE_STEP_WRITE_OPEN_COMMAND) &&
          (runtime->brake.step <=
           ECAT_RUNTIME_BRAKE_STEP_RELATCH_TARGET_AND_SET_MODE_8))
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
      if (ECAT_RuntimePrepareBrakeRollback(
            runtime,
            ((runtime->brake.step ==
              ECAT_RUNTIME_BRAKE_STEP_WRITE_OPEN_COMMAND) ||
             (runtime->brake.step ==
              ECAT_RUNTIME_BRAKE_STEP_WRITE_EXECUTE_COMMAND)) ? 1U : 0U) == 0U)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
      if ((runtime->brake.current_slave >= ECAT_FIRST_SLAVE) &&
          (runtime->brake.current_slave <= ECAT_LAST_SLAVE) &&
          (ECAT_RuntimeSetBrakeAxisMode(context,
                                        runtime->brake.current_slave,
                                        (runtime->drive_state ==
                                         ECAT_RUNTIME_AXIS1_BRAKE_RELEASE) ?
                                        1U : 2U,
                                        8) == 0U))
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }

      memset(&runtime->brake, 0, sizeof(runtime->brake));
    }
    if (ECAT_RuntimeRollbackToZero(context, runtime) !=
        ECAT_RUNTIME_RESULT_OK)
    {
      return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
    }
  }
  if (runtime->bad_wkc_continuous <
      ECAT_RUNTIME_WKC_FAULT_LIMIT)
  {
    return ECAT_RUNTIME_RESULT_OK;
  }

  ECAT_LogPrintf(
    log,
    "[SOEM] OP processdata lost send=%d WKC=%d expected=%d continuous=%lu\r\n",
    send_ret,
    receive_wkc,
    runtime->expected_wkc,
    (unsigned long)runtime->bad_wkc_continuous);
  return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
}

/* 等待规定的连续成功周期，然后暂存 0x0006 并进入下一状态。 */
static void ECAT_RuntimeHandleWaitDelay(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime)
{
  runtime->state_cycle_count++;
  if (runtime->state_cycle_count >= ECAT_RUNTIME_CW06_DELAY_CYCLES)
  {
    runtime->state_cycle_count = 0U;
    if ((ECAT_RuntimeAllManagedAxesMatch(context,
                                         0x004FU,
                                         0x0040U,
                                         0) != 0U) &&
        (ECAT_RuntimeSetAllManagedControlwords(context,
                                               0x0006U) != 0U))
    {
      runtime->drive_state = ECAT_RUNTIME_CW06_WAIT_READY;
    }
    else
    {
      runtime->drive_state = ECAT_RUNTIME_CW06_FAILED;
    }
  }
}

/* 等待所有受管轴达到 0x0021，故障或超时时暂存 0x0000。 */
static ECAT_RuntimeResult ECAT_RuntimeHandleWaitReady(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime)
{
  uint8 fault_found;

  if (ECAT_RuntimeAllManagedAxesMatch(context,
                                      0x006FU,
                                      0x0021U,
                                      &fault_found) != 0U)
  {
    runtime->state_cycle_count = 0U;
    runtime->drive_state = ECAT_RUNTIME_CW06_DONE;
  }
  else
  {
    runtime->state_cycle_count++;
    if ((fault_found != 0U) ||
        (runtime->state_cycle_count >= ECAT_RUNTIME_CW06_READY_TIMEOUT))
    {
      runtime->state_cycle_count = 0U;
      if (ECAT_RuntimeRollbackToZero(context, runtime) !=
          ECAT_RUNTIME_RESULT_OK)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
    }
  }

  return ECAT_RUNTIME_RESULT_OK;
}

/* 再次确认 0x0021 后暂存 0x0007，状态丢失时回退到 0x0000。 */
static ECAT_RuntimeResult ECAT_RuntimeHandleReadyToSwitchOn(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime)
{
  uint8 fault_found;

  if (ECAT_RuntimeAllManagedAxesMatch(context,
                                      0x006FU,
                                      0x0021U,
                                      &fault_found) != 0U)
  {
    runtime->state_cycle_count = 0U;
    if (ECAT_RuntimeSetAllManagedControlwords(context, 0x0007U) == 0U)
    {
      return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
    }
    runtime->drive_state = ECAT_RUNTIME_CW07_WAIT_SWITCHED_ON;
  }
  else
  {
    runtime->state_cycle_count++;
    if ((fault_found != 0U) ||
        (runtime->state_cycle_count >= ECAT_RUNTIME_CW06_READY_LOSS))
    {
      runtime->state_cycle_count = 0U;
      if (ECAT_RuntimeRollbackToZero(context, runtime) !=
          ECAT_RUNTIME_RESULT_OK)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
    }
  }

  return ECAT_RUNTIME_RESULT_OK;
}

/* 等待所有受管轴达到 0x0023，故障或超时时暂存 0x0000。 */
static ECAT_RuntimeResult ECAT_RuntimeHandleWaitSwitchedOn(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime)
{
  uint8 fault_found;

  if (ECAT_RuntimeAllManagedAxesMatch(context,
                                      0x006FU,
                                      0x0023U,
                                      &fault_found) != 0U)
  {
    runtime->state_cycle_count = 0U;
    runtime->drive_state = ECAT_RUNTIME_CW07_DONE;
  }
  else
  {
    runtime->state_cycle_count++;
    if ((fault_found != 0U) ||
        (runtime->state_cycle_count >= ECAT_RUNTIME_CW07_SWITCH_TIMEOUT))
    {
      runtime->state_cycle_count = 0U;
      if (ECAT_RuntimeRollbackToZero(context, runtime) !=
          ECAT_RUNTIME_RESULT_OK)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
    }
  }

  return ECAT_RUNTIME_RESULT_OK;
}

/* 保持 0x0023，并在收到使能请求后重新锁存安全目标。 */
static ECAT_RuntimeResult ECAT_RuntimeHandleSwitchedOn(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime)
{
  uint8 fault_found;

  if (ECAT_RuntimeAllManagedAxesMatch(context,
                                      0x006FU,
                                      0x0023U,
                                      &fault_found) != 0U)
  {
    runtime->state_cycle_count = 0U;
    if (runtime->enable_operation_request != 0U)
    {
      __DMB();
      runtime->enable_operation_request = 0U;
      runtime->target_stable_cycle_count = 0U;
      if (ECAT_RuntimeRelatchAllManagedTargets(context) == 0U)
      {
        if (ECAT_RuntimeRollbackToZero(context, runtime) !=
            ECAT_RUNTIME_RESULT_OK)
        {
          return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
        }
      }
      else
      {
        runtime->drive_state = ECAT_RUNTIME_CW0F_WAIT_TARGET_STABLE;
      }
    }
  }
  else
  {
    runtime->enable_operation_request = 0U;
    runtime->state_cycle_count++;
    if ((fault_found != 0U) ||
        (runtime->state_cycle_count >= ECAT_RUNTIME_CW07_SWITCH_LOSS))
    {
      runtime->state_cycle_count = 0U;
      if (ECAT_RuntimeRollbackToZero(context, runtime) !=
          ECAT_RUNTIME_RESULT_OK)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
    }
  }

  return ECAT_RUNTIME_RESULT_OK;
}

/* 等待目标连续稳定 3 个周期，再暂存 0x000F。 */
static ECAT_RuntimeResult ECAT_RuntimeHandleWaitTargetStable(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime)
{
  uint8 fault_found;

  runtime->state_cycle_count++;
  if (ECAT_RuntimeAllManagedAxesPassEnableCheck(context,
        ECAT_RUNTIME_CW0F_POSITION_TOLERANCE,
        ECAT_RUNTIME_CW0F_VELOCITY_TOLERANCE, &fault_found) != 0U)
  {
    if (runtime->target_stable_cycle_count < ECAT_RUNTIME_CW0F_TARGET_STABLE_CYCLES)
    {
      runtime->target_stable_cycle_count++;
    }
    if (runtime->target_stable_cycle_count >= ECAT_RUNTIME_CW0F_TARGET_STABLE_CYCLES)
    {
      runtime->state_cycle_count = 0U;
      runtime->target_stable_cycle_count = 0U;
      if (ECAT_RuntimeSetAllManagedControlwords(context, 0x000FU) == 0U)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
      runtime->drive_state = ECAT_RUNTIME_CW0F_WAIT_OPERATION_ENABLED;
    }
  }
  else
  {
    runtime->target_stable_cycle_count = 0U;
  }
  if ((fault_found != 0U) ||
      (runtime->state_cycle_count >= ECAT_RUNTIME_CW0F_TARGET_TIMEOUT))
  {
    runtime->state_cycle_count = 0U;
    runtime->target_stable_cycle_count = 0U;
    if (ECAT_RuntimeRollbackToZero(context, runtime) !=
        ECAT_RUNTIME_RESULT_OK)
    {
      return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
    }
  }

  return ECAT_RUNTIME_RESULT_OK;
}

/* 等待所有受管轴达到 0x0027，故障时回零，超时时回退至 0x0007。 */
static ECAT_RuntimeResult ECAT_RuntimeHandleWaitOperationEnabled(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime)
{
  uint8 fault_found;

  if (ECAT_RuntimeAllManagedAxesMatch(context,
                                      0x006FU,
                                      0x0027U,
                                      &fault_found) != 0U)
  {
    runtime->state_cycle_count = 0U;
    runtime->target_stable_cycle_count = 0U;
    runtime->drive_state = ECAT_RUNTIME_CW0F_DONE;
  }
  else
  {
    runtime->state_cycle_count++;
    if (fault_found != 0U)
    {
      runtime->state_cycle_count = 0U;
      runtime->target_stable_cycle_count = 0U;
      if (ECAT_RuntimeRollbackToZero(context, runtime) !=
          ECAT_RUNTIME_RESULT_OK)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
    }
    else if (runtime->state_cycle_count >= ECAT_RUNTIME_CW0F_ENABLE_TIMEOUT)
    {
      runtime->state_cycle_count = 0U;
      runtime->target_stable_cycle_count = 0U;
      if (ECAT_RuntimeRollbackToSwitchedOn(context, runtime) !=
          ECAT_RUNTIME_RESULT_OK)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
    }
  }

  return ECAT_RUNTIME_RESULT_OK;
}

/* 保持 0x0027；状态丢失时按 Fault 与非 Fault 分别回退。 */
static ECAT_RuntimeResult ECAT_RuntimeHandleOperationEnabled(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime)
{
  uint8 fault_found;

  if (ECAT_RuntimeAllManagedAxesMatch(context,
                                      0x006FU,
                                      0x0027U,
                                      &fault_found) != 0U)
  {
    runtime->target_stable_cycle_count = 0U;
    runtime->state_cycle_count = 0U;
    if (runtime->test_completed == 0U)
    {
      runtime->test_keys_armed = 0U;
      runtime->drive_state = ECAT_RUNTIME_TEST;
    }
  }
  else
  {
    runtime->state_cycle_count = 0U;
    runtime->target_stable_cycle_count = 0U;
    runtime->enable_operation_request = 0U;
    if (fault_found != 0U)
    {
      if (ECAT_RuntimeRollbackToZero(context, runtime) !=
          ECAT_RUNTIME_RESULT_OK)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
    }
    else
    {
      if (ECAT_RuntimeRollbackToSwitchedOn(context, runtime) !=
          ECAT_RUNTIME_RESULT_OK)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
    }
  }

  return ECAT_RUNTIME_RESULT_OK;
}

static uint8 ECAT_RuntimeSetBrakeAxisMode(
  ecx_contextt *context,
  uint16 slave,
  uint8 axis,
  int8_t mode)
{
  AppEtherCAT_ServoRxPdo rxpdo;
  ec_slavet *slave_info;

  if ((context == 0) || (slave < ECAT_FIRST_SLAVE) ||
      (slave > ECAT_LAST_SLAVE) || (context->slavecount < (int)slave))
  {
    return 0U;
  }

  slave_info = &context->slavelist[slave];
  if ((slave_info->outputs == 0) ||
      (slave_info->Obytes != APP_ETHERCAT_SERVO_RXPDO_SIZE))
  {
    return 0U;
  }

  memcpy(&rxpdo, slave_info->outputs, sizeof(rxpdo));
  if (axis == 1U)
  {
    rxpdo.modes_of_operation = mode;
  }
  else if (axis == 2U)
  {
    rxpdo.axis2_modes_of_operation = mode;
  }
  else
  {
    return 0U;
  }
  memcpy(slave_info->outputs, &rxpdo, sizeof(rxpdo));
  return 1U;
}

/**
 * @brief  提交或取回制动流程使用的异步 4 字节 SDO 作业
 *
 * @param[in,out] job           与低优先级 SDO Worker 共享的作业槽
 * @param[in]     operation     READ_PENDING 或 WRITE_PENDING 作业类型
 * @param[in]     slave         目标从站索引
 * @param[in]     index         目标对象索引，子索引固定为0
 * @param[in]     request_wire  写作业的 4 字节线序数据，读作业忽略该参数
 * @param[out]    response_wire 可选的读结果缓冲区，按线序返回；写作业忽略该参数
 *
 * @return
 * 新作业已提交或 Worker 尚未完成时返回 ECAT_RUNTIME_BRAKE_SDO_WAIT；完成结果的
 * 从站、索引、子索引、长度及 WKC 有效，且写缓存与请求一致时返回
 * ECAT_RUNTIME_BRAKE_SDO_COMPLETE；结果校验失败时返回 ECAT_RUNTIME_BRAKE_SDO_FAILED
 *
 * @warning
 * job 必须有效并由单一调用流程与 SDO Worker 按既定状态握手共享，operation 只能为
 * READ_PENDING 或 WRITE_PENDING；等待期间必须以相同的 slave、index、operation 和
 * request_wire 重复调用。函数固定传输4字节且不进行大小端转换，读取失败或等待时
 * 不修改 response_wire；DONE 结果无论校验成功与否都会被消费并将作业重置为空闲
 */
static ECAT_RuntimeBrakeSdoResult ECAT_RuntimeBrakeProcessSdo(
  ECAT_RuntimeSdoJob *job,
  ECAT_RuntimeSdoJobState operation,
  uint16 slave,
  uint16 index,
  uint32 request_wire,
  uint32 *response_wire)
{
  ECAT_RuntimeSdoJobState state;
  uint8 data[ECAT_RUNTIME_SDO_DATA_SIZE];
  uint8 subindex = 0U;
  uint16 result_slave = 0U;
  uint16 result_index = 0U;
  uint32 primask;
  int size = 0;
  int wkc = 0;

  primask = __get_PRIMASK();
  __disable_irq();
  state = job->state;
  if (state == ECAT_RUNTIME_SDO_JOB_IDLE)
  {
    job->slave = slave;
    job->index = index;
    job->subindex = 0U;
    if (operation == ECAT_RUNTIME_SDO_JOB_WRITE_PENDING)
    {
      memcpy(job->data, &request_wire, sizeof(request_wire));
    }
    else
    {
      memset(job->data, 0, sizeof(job->data));
    }
    job->size = (int)ECAT_RUNTIME_SDO_DATA_SIZE;
    job->wkc = 0;
    __DMB();
    job->state = operation;
    if (operation == ECAT_RUNTIME_SDO_JOB_WRITE_PENDING)
    {
      ecat_dbg_sdo_write_submits++;
    }
  }
  else if (state == ECAT_RUNTIME_SDO_JOB_DONE)
  {
    __DMB();
    result_slave = job->slave;
    result_index = job->index;
    subindex = job->subindex;
    size = job->size;
    wkc = job->wkc;
    memcpy(data, job->data, sizeof(data));
    job->state = ECAT_RUNTIME_SDO_JOB_IDLE;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (state != ECAT_RUNTIME_SDO_JOB_DONE)
  {
    return ECAT_RUNTIME_BRAKE_SDO_WAIT;
  }
  if ((result_slave != slave) || (result_index != index) ||
      (subindex != 0U) || (wkc <= 0))
  {
    return ECAT_RUNTIME_BRAKE_SDO_FAILED;
  }
  if (operation == ECAT_RUNTIME_SDO_JOB_READ_PENDING)
  {
    if ((size <= 0) || (size > (int)ECAT_RUNTIME_SDO_DATA_SIZE))
    {
      return ECAT_RUNTIME_BRAKE_SDO_FAILED;
    }
    if (response_wire != 0)
    {
      memset(response_wire, 0, sizeof(*response_wire));
      memcpy(response_wire, data, (size_t)size);
    }
  }
  else if ((size != (int)ECAT_RUNTIME_SDO_DATA_SIZE) ||
           (memcmp(data, &request_wire, sizeof(request_wire)) != 0))
  {
    return ECAT_RUNTIME_BRAKE_SDO_FAILED;
  }
  return ECAT_RUNTIME_BRAKE_SDO_COMPLETE;
}

/**
 * @brief  按制动回退阶段安全取消或清理异步 SDO 作业
 *
 * @param[in,out] runtime               运行时上下文，函数会检查并可能重置 SDO 作业状态
 * @param[in]     execute_write_pending 非0时采用执行命令写入阶段的严格取消策略
 *
 * @return
 * SDO 作业原本为空闲，或按所选策略成功重置为空闲时返回1；作业正在执行，或当前
 * 状态不允许由所选策略取消时返回0
 *
 * @warning
 * runtime 必须有效。execute_write_pending 非0时仅撤销尚未被 Worker 领取的
 * ECAT_RUNTIME_SDO_JOB_WRITE_PENDING；为0时可清理 READ_PENDING、WRITE_PENDING
 * 和 DONE。函数绝不会中止 RUNNING 作业，也不会清除作业中的参数及结果字段；
 * 调用方必须在返回1后才能继续制动回退流程
 */
static uint8 ECAT_RuntimePrepareBrakeRollback(
  ECAT_RuntimeContext *runtime,
  uint8 execute_write_pending)
{
  ECAT_RuntimeSdoJobState state;
  uint32 primask;

  primask = __get_PRIMASK();
  __disable_irq();
  state = runtime->sdo_job.state;
  if (execute_write_pending != 0U)
  {
    if (state == ECAT_RUNTIME_SDO_JOB_WRITE_PENDING)
    {
      ecat_dbg_sdo_write_cancels++;
      runtime->sdo_job.state = ECAT_RUNTIME_SDO_JOB_IDLE;
      state = ECAT_RUNTIME_SDO_JOB_IDLE;
    }
  }
  else if ((state == ECAT_RUNTIME_SDO_JOB_READ_PENDING) ||
           (state == ECAT_RUNTIME_SDO_JOB_WRITE_PENDING) ||
           (state == ECAT_RUNTIME_SDO_JOB_DONE))
  {
    if (state == ECAT_RUNTIME_SDO_JOB_WRITE_PENDING)
    {
      ecat_dbg_sdo_write_cancels++;
    }
    runtime->sdo_job.state = ECAT_RUNTIME_SDO_JOB_IDLE;
    state = ECAT_RUNTIME_SDO_JOB_IDLE;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
  return (state == ECAT_RUNTIME_SDO_JOB_IDLE) ? 1U : 0U;
}

/**
 * @brief  分周期依次执行三个受管从站指定轴的开、关抱闸及运行模式恢复流程
 *
 * @param[in,out] context SOEM 主站上下文，函数会读取 TxPDO 并修改目标轴的 RxPDO
 * @param[in,out] runtime 运行时上下文，用于维护制动步骤、异步 SDO 作业及驱动状态
 * @param[in]     axis    目标轴编号：1使用CH1对象，2使用CH2对象
 *
 * @return
 * 当前步骤完成、仍在等待 SDO/反馈/延时，或三个从站完成并转入下一顶层状态时返回
 * ECAT_RUNTIME_RESULT_OK；检测到可安全处理的前期异常并成功启动回退时也返回该值。
 * 关键 PDO/SDO 操作失败、执行命令后的轴状态异常，或无法
 * 完成安全回退时返回 ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST
 *
 * @warning
 * context 和 runtime 必须有效，axis 只能为1或2，且仅应在对应的 AXIS1/AXIS2
 * BRAKE_RELEASE 状态下，于每次成功的 OP PDO 交换后调用；异步 SDO Worker 必须持续
 * 处理 runtime->sdo_job，SOEM 时间基准也必须可用。Axis1使用0x3023/0x2023/0x2024，
 * Axis2使用0x3033/0x2033/0x2034；禁止并发修改 PDO 映像、制动上下文或绕过既定握手
 * 访问 SDO 作业。返回 OK 仅表示本周期处理正常，不代表抱闸流程已经完成
 */
static ECAT_RuntimeResult ECAT_RuntimeHandleBrakeRelease(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime,
  uint8 axis)
{
  AppEtherCAT_ServoRxPdo rxpdo;
  AppEtherCAT_ServoTxPdo txpdo;
  ECAT_RuntimeBrakeSdoResult sdo_result;
  ECAT_RuntimeSdoJob *job = &runtime->sdo_job;
  ECAT_RuntimeSdoJobState job_state;
  ec_slavet *slave_info;
  uint8 fault_found;
  uint8 rollback_to_zero = 0U;
  uint8 restore_mode = 0U;
  uint16 command_index;
  uint16 feedback_index_1;
  uint16 feedback_index_2;
  uint16 expected_index = 0U;
  uint32 feedback_wire = 0U;
  uint32 feedback_value;
  uint32 command_wire;
  uint32 now_us;
  uint32 primask;

  if (axis == 1U)
  {
    command_index = 0x3023U;
    feedback_index_1 = 0x2023U;
    feedback_index_2 = 0x2024U;
  }
  else if (axis == 2U)
  {
    command_index = 0x3033U;
    feedback_index_1 = 0x2033U;
    feedback_index_2 = 0x2034U;
  }
  else
  {
    return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
  }

  /*
   * 抱闸流程现在位于使能序列之前，因此全部受管轴必须保持
   * Switch on disabled(0x0040)，流程完成后才开始下发0x0006。
   */
  if (ECAT_RuntimeAllManagedAxesMatch(context,
                                      0x004FU,
                                      0x0040U,
                                      &fault_found) == 0U)
  {
    /*
     * 任一抱闸命令已经完成、但对应反馈清理尚未全部完成时，
     * 驱动内部命令状态可能处于中间态。此时禁止简单恢复模式8或进入普通回退，
     * 直接上报运行交换丢失，由上层停止邮箱服务并执行安全关闭。
     */
    if (((runtime->brake.step >
          ECAT_RUNTIME_BRAKE_STEP_WRITE_OPEN_COMMAND) &&
         (runtime->brake.step <=
          ECAT_RUNTIME_BRAKE_STEP_RELATCH_TARGET_AND_SET_MODE_8)))
    {
      return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
    }
    /*
     * 若正准备发送0x11000000或0x22000000，只允许原子取消尚未被Worker领取的WRITE_PENDING。
     * 若SDO已进入RUNNING或DONE，命令可能已经到达驱动，不能把它当作未执行后回退。
     * 其他非危险阶段则可清理尚未执行的读写请求或已消费完成的结果。
     */
    if (ECAT_RuntimePrepareBrakeRollback(
          runtime,
          ((runtime->brake.step ==
            ECAT_RUNTIME_BRAKE_STEP_WRITE_OPEN_COMMAND) ||
           (runtime->brake.step ==
            ECAT_RUNTIME_BRAKE_STEP_WRITE_EXECUTE_COMMAND)) ? 1U : 0U) == 0U)
    {
      return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
    }
    /* 使能前的抱闸流程异常统一保持控制字0x0000。 */
    rollback_to_zero = 1U;
    /* 已经开始设置模式11时，普通回退前需要先把当前轴的模式对象恢复为8。 */
    restore_mode =
      (runtime->brake.step >= ECAT_RUNTIME_BRAKE_STEP_SET_MODE_11) ? 1U : 0U;
    goto rollback;
  }

  /* 模块二：按brake.step每个成功PDO周期只推进一个抱闸子步骤。 */
  switch ((ECAT_RuntimeBrakeStep)runtime->brake.step)
  {
    case ECAT_RUNTIME_BRAKE_STEP_IDLE:
      /*
       * 初始化抱闸流程。只有共享SDO单槽为空闲时才开始，避免覆盖上一笔事务；
       * 遗留的DONE结果在这里丢弃，RUNNING/PENDING则继续等待Worker收尾。
       */
      primask = __get_PRIMASK();/* 保存进入临界区前的中断状态 */
      __disable_irq();
      job_state = job->state;
      /* 清理上一笔已完成但尚未释放的SDO结果。 */
      if (job_state == ECAT_RUNTIME_SDO_JOB_DONE)
      {
        job->state = ECAT_RUNTIME_SDO_JOB_IDLE;
        job_state = ECAT_RUNTIME_SDO_JOB_IDLE;
      }
      /* 进入前允许中断时，退出临界区后恢复中断。 */
      if (primask == 0U)
      {
        __enable_irq();
      }
      /* SDO单槽仍忙时不覆盖任务，本周期继续等待。 */
      if (job_state != ECAT_RUNTIME_SDO_JOB_IDLE)
      {
        return ECAT_RUNTIME_RESULT_OK;
      }

      runtime->brake.current_slave = ECAT_FIRST_SLAVE;
      runtime->brake.retry_count = 0U;
      runtime->brake.feedback_2023 = 0U;
      runtime->brake.feedback_2024 = 0U;
      runtime->brake.wait_start_us = 0U;
      runtime->brake.step = ECAT_RUNTIME_BRAKE_STEP_SET_MODE_11;
      return ECAT_RUNTIME_RESULT_OK;

    case ECAT_RUNTIME_BRAKE_STEP_SET_MODE_11:
      /*
       * 仅修改当前从站当前轴的RxPDO模式对象为11，其余PDO字段保持不变。
       * 本周期只修改输出映像，实际值会在下一次PDO发送时下发到驱动。
       */
      if (ECAT_RuntimeSetBrakeAxisMode(context,
                                       runtime->brake.current_slave,
                                       axis,
                                       11) == 0U)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
      runtime->state_cycle_count = 0U;
      runtime->brake.retry_count = 0U;
      runtime->brake.step =
        ECAT_RUNTIME_BRAKE_STEP_WRITE_OPEN_COMMAND;
      return ECAT_RUNTIME_RESULT_OK;

    case ECAT_RUNTIME_BRAKE_STEP_WRITE_OPEN_COMMAND:
      /* 向当前通道命令对象写0x11000000，打开当前从站当前轴抱闸。 */
      command_wire = htoel(ECAT_RUNTIME_BRAKE_OPEN_COMMAND);
      sdo_result = ECAT_RuntimeBrakeProcessSdo(
        job,
        ECAT_RUNTIME_SDO_JOB_WRITE_PENDING,
        runtime->brake.current_slave,
        command_index,
        command_wire,
        0);
      if (sdo_result == ECAT_RUNTIME_BRAKE_SDO_WAIT)
      {
        return ECAT_RUNTIME_RESULT_OK;
      }
      if (sdo_result != ECAT_RUNTIME_BRAKE_SDO_COMPLETE)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
      runtime->brake.retry_count = 0U;
      runtime->brake.feedback_2023 = 0U;
      runtime->brake.feedback_2024 = 0U;
      runtime->state_cycle_count = 0U;
      runtime->brake.step = ECAT_RUNTIME_BRAKE_STEP_WAIT_OPEN_FEEDBACK;
      return ECAT_RUNTIME_RESULT_OK;

    case ECAT_RUNTIME_BRAKE_STEP_WAIT_OPEN_FEEDBACK:
      /* 轮询当前通道的两个反馈对象，等待两者同时反馈0x00300000。 */
      expected_index = (runtime->brake.retry_count == 0U) ?
                       feedback_index_1 : feedback_index_2;
      sdo_result = ECAT_RuntimeBrakeProcessSdo(
        job,
        ECAT_RUNTIME_SDO_JOB_READ_PENDING,
        runtime->brake.current_slave,
        expected_index,
        0U,
        &feedback_wire);
      if (sdo_result == ECAT_RUNTIME_BRAKE_SDO_WAIT)
      {
        return ECAT_RUNTIME_RESULT_OK;
      }
      if (sdo_result != ECAT_RUNTIME_BRAKE_SDO_COMPLETE)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
      feedback_value = etohl(feedback_wire);
      if (runtime->brake.retry_count == 0U)
      {
        runtime->brake.feedback_2023 = feedback_value;
        runtime->brake.retry_count = 1U;
      }
      else
      {
        runtime->brake.feedback_2024 = feedback_value;
        runtime->brake.retry_count = 0U;
        if ((runtime->brake.feedback_2023 ==
             ECAT_RUNTIME_BRAKE_OPENED_FEEDBACK) &&
            (runtime->brake.feedback_2024 ==
             ECAT_RUNTIME_BRAKE_OPENED_FEEDBACK))
        {
          runtime->brake.wait_start_us =
            (uint32)BSP_SOEM_Timebase_GetUs();
          runtime->state_cycle_count = 0U;
          runtime->brake.step = ECAT_RUNTIME_BRAKE_STEP_WAIT_OPEN_10_MS;
        }
      }
      return ECAT_RUNTIME_RESULT_OK;

    case ECAT_RUNTIME_BRAKE_STEP_WAIT_OPEN_10_MS:
      /* 开抱闸反馈确认后，非阻塞等待至少10ms。 */
      now_us = (uint32)BSP_SOEM_Timebase_GetUs();
      if ((uint32)(now_us - runtime->brake.wait_start_us) >=
          ECAT_RUNTIME_BRAKE_DELAY_US)
      {
        runtime->brake.step =
          ECAT_RUNTIME_BRAKE_STEP_WRITE_OPEN_CLEAR_COMMAND;
      }
      return ECAT_RUNTIME_RESULT_OK;

    case ECAT_RUNTIME_BRAKE_STEP_WRITE_OPEN_CLEAR_COMMAND:
      /* 写0x33000000，清除开抱闸操作产生的反馈值。 */
      command_wire = htoel(ECAT_RUNTIME_BRAKE_CLEAR_COMMAND);
      sdo_result = ECAT_RuntimeBrakeProcessSdo(
        job,
        ECAT_RUNTIME_SDO_JOB_WRITE_PENDING,
        runtime->brake.current_slave,
        command_index,
        command_wire,
        0);
      if (sdo_result == ECAT_RUNTIME_BRAKE_SDO_WAIT)
      {
        return ECAT_RUNTIME_RESULT_OK;
      }
      if (sdo_result != ECAT_RUNTIME_BRAKE_SDO_COMPLETE)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
      runtime->brake.retry_count = 0U;
      runtime->brake.feedback_2023 = 0U;
      runtime->brake.feedback_2024 = 0U;
      runtime->state_cycle_count = 0U;
      runtime->brake.step =
        ECAT_RUNTIME_BRAKE_STEP_WAIT_OPEN_FEEDBACK_CLEAR;
      return ECAT_RUNTIME_RESULT_OK;

    case ECAT_RUNTIME_BRAKE_STEP_WAIT_OPEN_FEEDBACK_CLEAR:
      /* 轮询当前通道的两个反馈对象，等待两者同时清零。 */
      expected_index = (runtime->brake.retry_count == 0U) ?
                       feedback_index_1 : feedback_index_2;
      sdo_result = ECAT_RuntimeBrakeProcessSdo(
        job,
        ECAT_RUNTIME_SDO_JOB_READ_PENDING,
        runtime->brake.current_slave,
        expected_index,
        0U,
        &feedback_wire);
      if (sdo_result == ECAT_RUNTIME_BRAKE_SDO_WAIT)
      {
        return ECAT_RUNTIME_RESULT_OK;
      }
      if (sdo_result != ECAT_RUNTIME_BRAKE_SDO_COMPLETE)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
      feedback_value = etohl(feedback_wire);
      if (runtime->brake.retry_count == 0U)
      {
        runtime->brake.feedback_2023 = feedback_value;
        runtime->brake.retry_count = 1U;
      }
      else
      {
        runtime->brake.feedback_2024 = feedback_value;
        runtime->brake.retry_count = 0U;
        if ((runtime->brake.feedback_2023 == 0U) &&
            (runtime->brake.feedback_2024 == 0U))
        {
          runtime->state_cycle_count = 0U;
          runtime->brake.step =
            ECAT_RUNTIME_BRAKE_STEP_WRITE_EXECUTE_COMMAND;
        }
      }
      return ECAT_RUNTIME_RESULT_OK;

    case ECAT_RUNTIME_BRAKE_STEP_WRITE_EXECUTE_COMMAND:
      /*
       * 向当前通道命令对象写0x22000000，触发当前从站当前轴关闭抱闸。
       * 该写操作只允许提交一次；若写结果不确定，禁止盲目重发，直接进入错误处理。
       */
      command_wire = htoel(ECAT_RUNTIME_BRAKE_EXECUTE_COMMAND);
      sdo_result = ECAT_RuntimeBrakeProcessSdo(
        job,
        ECAT_RUNTIME_SDO_JOB_WRITE_PENDING,
        runtime->brake.current_slave,
        command_index,
        command_wire,
        0);
      if (sdo_result == ECAT_RUNTIME_BRAKE_SDO_WAIT)
      {
        return ECAT_RUNTIME_RESULT_OK;
      }
      if (sdo_result != ECAT_RUNTIME_BRAKE_SDO_COMPLETE)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
      runtime->brake.retry_count = 0U;
      runtime->brake.feedback_2023 = 0U;
      runtime->brake.feedback_2024 = 0U;
      runtime->state_cycle_count = 0U;
      runtime->brake.step =
        ECAT_RUNTIME_BRAKE_STEP_WAIT_EXECUTE_FEEDBACK;
      return ECAT_RUNTIME_RESULT_OK;

    case ECAT_RUNTIME_BRAKE_STEP_WAIT_EXECUTE_FEEDBACK:
      /*
       * 成对轮询当前通道的两个反馈对象。只有两者在同一轮读取中都等于0x00400000，
       * 才能确认关抱闸动作已执行。反馈尚未到目标值时继续轮询，不重复写0x22000000。
       */
      expected_index = (runtime->brake.retry_count == 0U) ?
                       feedback_index_1 : feedback_index_2;
      sdo_result = ECAT_RuntimeBrakeProcessSdo(
        job,
        ECAT_RUNTIME_SDO_JOB_READ_PENDING,
        runtime->brake.current_slave,
        expected_index,
        0U,
        &feedback_wire);
      if (sdo_result == ECAT_RUNTIME_BRAKE_SDO_WAIT)
      {
        return ECAT_RUNTIME_RESULT_OK;
      }
      if (sdo_result != ECAT_RUNTIME_BRAKE_SDO_COMPLETE)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
      feedback_value = etohl(feedback_wire);
      if (runtime->brake.retry_count == 0U)
      {
        runtime->brake.feedback_2023 = feedback_value;
        runtime->brake.retry_count = 1U;
      }
      else
      {
        runtime->brake.feedback_2024 = feedback_value;
        runtime->brake.retry_count = 0U;
        if ((runtime->brake.feedback_2023 ==
             ECAT_RUNTIME_BRAKE_EXECUTED_FEEDBACK) &&
            (runtime->brake.feedback_2024 ==
             ECAT_RUNTIME_BRAKE_EXECUTED_FEEDBACK))
        {
          runtime->brake.wait_start_us =
            (uint32)BSP_SOEM_Timebase_GetUs();
          runtime->state_cycle_count = 0U;
          runtime->brake.step = ECAT_RUNTIME_BRAKE_STEP_WAIT_10_MS;
        }
      }
      return ECAT_RUNTIME_RESULT_OK;

    case ECAT_RUNTIME_BRAKE_STEP_WAIT_10_MS:
      /*
       * 从两个执行反馈都确认成功的时刻开始，非阻塞等待至少10ms。
       * 使用无符号时间差可正确处理32位微秒计数回绕，同时不阻塞2ms PDO任务。
       */
      now_us = (uint32)BSP_SOEM_Timebase_GetUs();
      if ((uint32)(now_us - runtime->brake.wait_start_us) >=
          ECAT_RUNTIME_BRAKE_DELAY_US)
      {
        runtime->brake.step =
          ECAT_RUNTIME_BRAKE_STEP_WRITE_CLEAR_COMMAND;
      }
      return ECAT_RUNTIME_RESULT_OK;

    case ECAT_RUNTIME_BRAKE_STEP_WRITE_CLEAR_COMMAND:
      /*
       * 向当前通道命令对象写0x33000000，请求驱动清除两个反馈对象的执行反馈。
       * 写完成只代表命令已应答，后续仍必须读取两个反馈并确认它们都回到0。
       */
      command_wire = htoel(ECAT_RUNTIME_BRAKE_CLEAR_COMMAND);
      sdo_result = ECAT_RuntimeBrakeProcessSdo(
        job,
        ECAT_RUNTIME_SDO_JOB_WRITE_PENDING,
        runtime->brake.current_slave,
        command_index,
        command_wire,
        0);
      if (sdo_result == ECAT_RUNTIME_BRAKE_SDO_WAIT)
      {
        return ECAT_RUNTIME_RESULT_OK;
      }
      if (sdo_result != ECAT_RUNTIME_BRAKE_SDO_COMPLETE)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
      runtime->brake.retry_count = 0U;
      runtime->brake.feedback_2023 = 0U;
      runtime->brake.feedback_2024 = 0U;
      runtime->state_cycle_count = 0U;
      runtime->brake.step =
        ECAT_RUNTIME_BRAKE_STEP_WAIT_FEEDBACK_CLEAR;
      return ECAT_RUNTIME_RESULT_OK;

    case ECAT_RUNTIME_BRAKE_STEP_WAIT_FEEDBACK_CLEAR:
      /*
       * 成对轮询当前通道的两个反馈对象，两个值都为0才表示驱动重新进入Ready状态。
       * 任一值非0时继续轮询，但不重复写0x33000000。
       */
      expected_index = (runtime->brake.retry_count == 0U) ?
                       feedback_index_1 : feedback_index_2;
      sdo_result = ECAT_RuntimeBrakeProcessSdo(
        job,
        ECAT_RUNTIME_SDO_JOB_READ_PENDING,
        runtime->brake.current_slave,
        expected_index,
        0U,
        &feedback_wire);
      if (sdo_result == ECAT_RUNTIME_BRAKE_SDO_WAIT)
      {
        return ECAT_RUNTIME_RESULT_OK;
      }
      if (sdo_result != ECAT_RUNTIME_BRAKE_SDO_COMPLETE)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
      feedback_value = etohl(feedback_wire);
      if (runtime->brake.retry_count == 0U)
      {
        runtime->brake.feedback_2023 = feedback_value;
        runtime->brake.retry_count = 1U;
      }
      else
      {
        runtime->brake.feedback_2024 = feedback_value;
        runtime->brake.retry_count = 0U;
        if ((runtime->brake.feedback_2023 == 0U) &&
            (runtime->brake.feedback_2024 == 0U))
        {
          runtime->state_cycle_count = 0U;
          runtime->brake.step =
            ECAT_RUNTIME_BRAKE_STEP_WRITE_COMMAND_ZERO;
        }
      }
      return ECAT_RUNTIME_RESULT_OK;

    case ECAT_RUNTIME_BRAKE_STEP_WRITE_COMMAND_ZERO:
      /*
       * 在两个反馈均清零后，向当前通道命令对象写0，清空命令寄存器本身。
       * 此步骤成功后才允许开始恢复CSP模式和重新锁存目标位置。
       */
      command_wire = htoel(0U);
      sdo_result = ECAT_RuntimeBrakeProcessSdo(
        job,
        ECAT_RUNTIME_SDO_JOB_WRITE_PENDING,
        runtime->brake.current_slave,
        command_index,
        command_wire,
        0);
      if (sdo_result == ECAT_RUNTIME_BRAKE_SDO_WAIT)
      {
        return ECAT_RUNTIME_RESULT_OK;
      }
      if (sdo_result != ECAT_RUNTIME_BRAKE_SDO_COMPLETE)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
      runtime->brake.step =
        ECAT_RUNTIME_BRAKE_STEP_RELATCH_TARGET_AND_SET_MODE_8;
      return ECAT_RUNTIME_RESULT_OK;

    case ECAT_RUNTIME_BRAKE_STEP_RELATCH_TARGET_AND_SET_MODE_8:
      /*
       * 将当前轴的目标位置重新锁存为实际位置，并在同一RxPDO映像中把模式恢复为8。
       * 只修改当前从站当前轴的目标位置和模式，控制字、另一轴及其他从站保持不变。
       */
      if ((runtime->brake.current_slave < ECAT_FIRST_SLAVE) ||
          (runtime->brake.current_slave > ECAT_LAST_SLAVE))
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
      slave_info = &context->slavelist[runtime->brake.current_slave];
      if ((slave_info->outputs == 0) || (slave_info->inputs == 0) ||
          (slave_info->Obytes != APP_ETHERCAT_SERVO_RXPDO_SIZE) ||
          (slave_info->Ibytes != APP_ETHERCAT_SERVO_TXPDO_SIZE))
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
      memcpy(&rxpdo, slave_info->outputs, sizeof(rxpdo));
      memcpy(&txpdo, slave_info->inputs, sizeof(txpdo));
      if (axis == 1U)
      {
        rxpdo.target_position = txpdo.position_actual_value;
        rxpdo.modes_of_operation = 8;
      }
      else
      {
        rxpdo.axis2_target_position = txpdo.axis2_position_actual_value;
        rxpdo.axis2_modes_of_operation = 8;
      }
      memcpy(slave_info->outputs, &rxpdo, sizeof(rxpdo));
      runtime->state_cycle_count = 0U;
      runtime->brake.step = ECAT_RUNTIME_BRAKE_STEP_DONE;
      return ECAT_RUNTIME_RESULT_OK;

    case ECAT_RUNTIME_BRAKE_STEP_DONE:
      /*
       * 上一周期写入的模式8已经随本周期PDO发送；确认SDO单槽空闲后完成流程。
       * 最后重锁目标位置、清除抱闸上下文并进入配置的抱闸完成状态。
       */
      slave_info = &context->slavelist[runtime->brake.current_slave];
      if ((slave_info->outputs == 0) || (slave_info->inputs == 0) ||
          (slave_info->Obytes != APP_ETHERCAT_SERVO_RXPDO_SIZE) ||
          (slave_info->Ibytes != APP_ETHERCAT_SERVO_TXPDO_SIZE))
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
      memcpy(&rxpdo, slave_info->outputs, sizeof(rxpdo));
      memcpy(&txpdo, slave_info->inputs, sizeof(txpdo));
      primask = __get_PRIMASK();
      __disable_irq();
      job_state = job->state;
      if (primask == 0U)
      {
        __enable_irq();
      }
      if (job_state != ECAT_RUNTIME_SDO_JOB_IDLE)
      {
        return ECAT_RUNTIME_RESULT_OK;
      }
      if (axis == 1U)
      {
        rxpdo.target_position = txpdo.position_actual_value;
      }
      else
      {
        rxpdo.axis2_target_position = txpdo.axis2_position_actual_value;
      }
      memcpy(slave_info->outputs, &rxpdo, sizeof(rxpdo));
      if (runtime->brake.current_slave < ECAT_LAST_SLAVE)
      {
        runtime->brake.current_slave++;
        runtime->brake.retry_count = 0U;
        runtime->brake.feedback_2023 = 0U;
        runtime->brake.feedback_2024 = 0U;
        runtime->brake.wait_start_us = 0U;
        runtime->state_cycle_count = 0U;
        runtime->brake.step = ECAT_RUNTIME_BRAKE_STEP_SET_MODE_11;
        return ECAT_RUNTIME_RESULT_OK;
      }
      memset(&runtime->brake, 0, sizeof(runtime->brake));
      runtime->state_cycle_count = 0U;
      runtime->target_stable_cycle_count = 0U;
      runtime->test_position_offset = 0U;
      runtime->drive_state = (axis == 1U) ?
                             ECAT_RUNTIME_AXIS2_BRAKE_RELEASE :
                             ECAT_RUNTIME_AFTER_BRAKE_STATE;
      return ECAT_RUNTIME_RESULT_OK;

    default:
      /* 非法子步骤不继续执行抱闸命令，尝试恢复模式8并走统一安全回退。 */
      restore_mode = 1U;
      goto rollback;
  }

rollback:
  /*
   * 模块三：仅处理尚未进入危险命令中间态的统一回退。
   * 先恢复模式8，再确认SDO单槽可以安全清理，最后清除抱闸上下文；
   * 有Fault时回控制字0x0000，普通状态异常时回控制字0x0007。
   */
  if ((restore_mode != 0U) &&
      (runtime->brake.current_slave >= ECAT_FIRST_SLAVE) &&
      (runtime->brake.current_slave <= ECAT_LAST_SLAVE) &&
      (ECAT_RuntimeSetBrakeAxisMode(context,
                                    runtime->brake.current_slave,
                                    axis,
                                    8) == 0U))
  {
    return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
  }

  if (ECAT_RuntimePrepareBrakeRollback(runtime, 0U) == 0U)
  {
    return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
  }

  memset(&runtime->brake, 0, sizeof(runtime->brake));
  runtime->state_cycle_count = 0U;
  runtime->target_stable_cycle_count = 0U;
  runtime->enable_operation_request = 0U;
  if (rollback_to_zero != 0U)
  {
    return ECAT_RuntimeRollbackToZero(context, runtime);
  }
  return ECAT_RuntimeRollbackToSwitchedOn(context, runtime);
}

static ECAT_RuntimeResult ECAT_RuntimeHandleAxis1BrakeRelease(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime)
{
  return ECAT_RuntimeHandleBrakeRelease(context, runtime, 1U);
}

static ECAT_RuntimeResult ECAT_RuntimeHandleAxis2BrakeRelease(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime)
{
  return ECAT_RuntimeHandleBrakeRelease(context, runtime, 2U);
}

/* Apply one complete logical six-joint trajectory point per successful PDO cycle. */
static ECAT_RuntimeResult ECAT_RuntimeHandleTest(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime)
{
  AppEtherCAT_ServoRxPdo rxpdo[ECAT_SLAVE_COUNT];
  AppEtherCAT_ServoTxPdo txpdo[ECAT_SLAVE_COUNT];
  ROBOT_TrajectoryPoint point;
  ROBOT_TrajectoryTakeResult take_result;
  uint32_t point_generation = 0U;
  const ROBOT_JOINT_Config *config;
  ec_slavet *slave;
  uint8 fault_found;
  uint8 command_timed_out;
  uint8 host_online;
  uint8 joint_index;
  uint8 slave_index;

  /* 同一PDO周期使用一致的通信状态快照来选择首个HOLD原因。 */
  command_timed_out = APP_CAN_IsCommandTimedOut() ? 1U : 0U;
  host_online = APP_CAN_IsHostOnline() ? 1U : 0U;

  /* Quick Stop动作保持最高优先级，但0x204保留导致它的首个根因。 */
  if ((APP_CAN_GetSafetyRequest() & APP_CAN_SAFETY_REQUEST_QUICK_STOP) != 0U)
  {
    if (host_online == 0U)
    {
      ROBOT_TrajectoryRequireResetWithReason(
        ROBOT_TRAJECTORY_HOLD_REASON_HOST_TIMEOUT);
    }
    else if (command_timed_out != 0U)
    {
      ROBOT_TrajectoryRequireResetWithReason(
        ROBOT_TRAJECTORY_HOLD_REASON_CSP_TIMEOUT);
    }
    else
    {
      ROBOT_TrajectoryRequireResetWithReason(
        ROBOT_TRAJECTORY_HOLD_REASON_QUICK_STOP);
    }
    if (ECAT_RuntimeSetAllManagedControlwords(context, 0x000BU) == 0U)
    {
      return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
    }
    return ECAT_RUNTIME_RESULT_OK;
  }

  if (ECAT_RuntimeAllManagedAxesMatch(context,
                                      0x006FU,
                                      0x0027U,
                                      &fault_found) == 0U)
  {
    ROBOT_TrajectoryRequireResetWithReason(
      (fault_found != 0U) ?
      ROBOT_TRAJECTORY_HOLD_REASON_DRIVE_FAULT :
      ROBOT_TRAJECTORY_HOLD_REASON_INTERNAL_ERROR);
    runtime->state_cycle_count = 0U;
    runtime->target_stable_cycle_count = 0U;
    runtime->enable_operation_request = 0U;
    if (fault_found != 0U)
    {
      return ECAT_RuntimeRollbackToZero(context, runtime);
    }
    return ECAT_RuntimeRollbackToSwitchedOn(context, runtime);
  }

  /* Copy all six PDO images first; commit outputs only after all checks pass. */
  for (slave_index = 0U; slave_index < ECAT_SLAVE_COUNT; slave_index++)
  {
    slave = &context->slavelist[ECAT_FIRST_SLAVE + slave_index];
    if ((slave->outputs == 0) || (slave->inputs == 0) ||
        (slave->Obytes != APP_ETHERCAT_SERVO_RXPDO_SIZE) ||
        (slave->Ibytes != APP_ETHERCAT_SERVO_TXPDO_SIZE))
    {
      ROBOT_TrajectoryRequireResetWithReason(
        ROBOT_TRAJECTORY_HOLD_REASON_INTERNAL_ERROR);
      return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
    }

    memcpy(&rxpdo[slave_index], slave->outputs, sizeof(rxpdo[slave_index]));
    memcpy(&txpdo[slave_index], slave->inputs, sizeof(txpdo[slave_index]));

    if ((rxpdo[slave_index].modes_of_operation != 8) ||
        (rxpdo[slave_index].axis2_modes_of_operation != 8) ||
        (txpdo[slave_index].modes_of_operation_display != 8) ||
        (txpdo[slave_index].axis2_modes_of_operation_display != 8))
    {
      ROBOT_TrajectoryRequireResetWithReason(
        ROBOT_TRAJECTORY_HOLD_REASON_DRIVE_NOT_CSP);
      return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
    }
  }

  if ((command_timed_out != 0U) || (host_online == 0U))
  {
    ROBOT_TrajectoryRequireResetWithReason(
      (host_online == 0U) ?
      ROBOT_TRAJECTORY_HOLD_REASON_HOST_TIMEOUT :
      ROBOT_TRAJECTORY_HOLD_REASON_CSP_TIMEOUT);
    take_result = ROBOT_TRAJECTORY_TAKE_HOLD;
  }
  else
  {
    take_result = ROBOT_TrajectoryTake(&point,
                                       HAL_GetTick(),
                                       &point_generation);
  }
  if ((take_result != ROBOT_TRAJECTORY_TAKE_POINT) &&
      (take_result != ROBOT_TRAJECTORY_TAKE_GRACE))
  {
    /* HOLD/WAIT: relatch every target to the current raw position. */
    for (slave_index = 0U; slave_index < ECAT_SLAVE_COUNT; slave_index++)
    {
      rxpdo[slave_index].target_position =
        txpdo[slave_index].position_actual_value;
      rxpdo[slave_index].axis2_target_position =
        txpdo[slave_index].axis2_position_actual_value;
    }
  }
  else if (take_result == ROBOT_TRAJECTORY_TAKE_POINT)
  {
    /* A trajectory point always contains Joint1..Joint6 raw absolute counts. */
    for (joint_index = 0U; joint_index < ROBOT_JOINT_COUNT; joint_index++)
    {
      config = ROBOT_JOINT_GetConfig(joint_index);
      if ((config == 0) ||
          (config->slave < ECAT_FIRST_SLAVE) ||
          (config->slave > ECAT_LAST_SLAVE))
      {
        ROBOT_TrajectoryRequireResetWithReason(
          ROBOT_TRAJECTORY_HOLD_REASON_INTERNAL_ERROR);
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }

      slave_index = (uint8)(config->slave - ECAT_FIRST_SLAVE);
      if (config->axis == ROBOT_JOINT_AXIS_1)
      {
        rxpdo[slave_index].target_position =
          point.target_position[joint_index];
        rxpdo[slave_index].controlword = 0x001FU;
      }
      else if (config->axis == ROBOT_JOINT_AXIS_2)
      {
        rxpdo[slave_index].axis2_target_position =
          point.target_position[joint_index];
        rxpdo[slave_index].axis2_controlword = 0x001FU;
      }
      else
      {
        ROBOT_TrajectoryRequireResetWithReason(
          ROBOT_TRAJECTORY_HOLD_REASON_INTERNAL_ERROR);
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
    }
  }
  else
  {
    /*
     * A transient empty queue keeps the previous RxPDO target unchanged.
     * Never extrapolate motion; a fourth consecutive empty EtherCAT cycle
     * is converted to the normal latched QUEUE_UNDERRUN path by Take().
     */
  }

  if ((take_result == ROBOT_TRAJECTORY_TAKE_POINT) &&
      (ROBOT_TrajectoryStageExecution(point.sequence,
                                      point_generation) == 0U))
  {
    ROBOT_TrajectoryRequireResetWithReason(
      ROBOT_TRAJECTORY_HOLD_REASON_INTERNAL_ERROR);
    return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
  }

  for (slave_index = 0U; slave_index < ECAT_SLAVE_COUNT; slave_index++)
  {
    slave = &context->slavelist[ECAT_FIRST_SLAVE + slave_index];
    memcpy(slave->outputs, &rxpdo[slave_index], sizeof(rxpdo[slave_index]));
  }

  return ECAT_RUNTIME_RESULT_OK;
}

/* 等待所有受管轴回到 0x0023，故障或超时时进一步回零。 */
static ECAT_RuntimeResult ECAT_RuntimeHandleRollbackToSwitchedOn(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime)
{
  uint8 fault_found;

  if (ECAT_RuntimeAllManagedAxesMatch(context,
                                      0x006FU,
                                      0x0023U,
                                      &fault_found) != 0U)
  {
    runtime->state_cycle_count = 0U;
    runtime->target_stable_cycle_count = 0U;
    runtime->enable_operation_request = 0U;
    runtime->drive_state = ECAT_RUNTIME_CW07_DONE;
  }
  else
  {
    runtime->state_cycle_count++;
    if ((fault_found != 0U) ||
        (runtime->state_cycle_count >= ECAT_RUNTIME_CW0F_CW07_TIMEOUT))
    {
      runtime->state_cycle_count = 0U;
      runtime->target_stable_cycle_count = 0U;
      runtime->enable_operation_request = 0U;
      if (ECAT_RuntimeRollbackToZero(context, runtime) !=
          ECAT_RUNTIME_RESULT_OK)
      {
        return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
      }
    }
  }

  return ECAT_RUNTIME_RESULT_OK;
}

/* 在回零控制字已暂存后的下一成功周期进入 FAILED。 */
static void ECAT_RuntimeHandleRollbackZero(
  ECAT_RuntimeContext *runtime)
{
  runtime->drive_state = ECAT_RUNTIME_CW06_FAILED;
}

/* FAILED 状态持续暂存 0x0000，并保持原有的忽略写入结果行为。 */
static void ECAT_RuntimeHandleFailed(
  ecx_contextt *context)
{
  (void)ECAT_RuntimeSetAllManagedControlwords(context, 0x0000U);
}

/**
 * @brief  按当前驱动状态分派并执行一次状态机处理 - 使能从站
 *
 * @param[in,out] context SOEM 主站上下文，状态处理器会读取 TxPDO 并可能修改 RxPDO
 * @param[in,out] runtime 运行时上下文，函数会读取并更新驱动状态、计数器及辅助流程
 *
 * @return
 * 当前状态处理成功或无需处理时返回 ECAT_RUNTIME_RESULT_OK；状态处理器执行 PDO、
 * 控制字或制动器相关操作失败时返回对应的 ECAT_RuntimeResult 错误码
 *
 * @warning
 * context 和 runtime 必须有效，且仅应在一次成功的 OP PDO 交换后调用；函数按调用
 * 时的 drive_state 最多执行一个状态处理器，新状态通常在下一成功周期继续处理。
 * 若本次处理进入 ECAT_RUNTIME_CW06_FAILED，会立即将全部受管轴控制字暂存为0x0000；
 * 调用期间不得并发修改运行时上下文或过程数据映像
 */
static ECAT_RuntimeResult ECAT_RuntimeDispatchDriveState(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime)
{
  ECAT_RuntimeResult result = ECAT_RUNTIME_RESULT_OK;

  switch (runtime->drive_state)
  {
    case ECAT_RUNTIME_AXIS1_BRAKE_RELEASE:
      result = ECAT_RuntimeHandleAxis1BrakeRelease(context, runtime);
      break;

    case ECAT_RUNTIME_AXIS2_BRAKE_RELEASE:
      result = ECAT_RuntimeHandleAxis2BrakeRelease(context, runtime);
      break;

    case ECAT_RUNTIME_CW06_DISABLED:
      break;

    case ECAT_RUNTIME_CW06_WAIT_DELAY:
      ECAT_RuntimeHandleWaitDelay(context, runtime);
      break;

    case ECAT_RUNTIME_CW06_WAIT_READY:
      result = ECAT_RuntimeHandleWaitReady(context, runtime);
      break;

    case ECAT_RUNTIME_CW06_DONE:
      result = ECAT_RuntimeHandleReadyToSwitchOn(context, runtime);
      break;

    case ECAT_RUNTIME_CW07_WAIT_SWITCHED_ON:
      result = ECAT_RuntimeHandleWaitSwitchedOn(context, runtime);
      break;

    case ECAT_RUNTIME_CW07_DONE:
      result = ECAT_RuntimeHandleSwitchedOn(context, runtime);
      break;

    case ECAT_RUNTIME_CW0F_WAIT_TARGET_STABLE:
      result = ECAT_RuntimeHandleWaitTargetStable(context, runtime);
      break;

    case ECAT_RUNTIME_CW0F_WAIT_OPERATION_ENABLED:
      result = ECAT_RuntimeHandleWaitOperationEnabled(context, runtime);
      break;

    case ECAT_RUNTIME_CW0F_DONE:
      result = ECAT_RuntimeHandleOperationEnabled(context, runtime);
      break;

    case ECAT_RUNTIME_TEST:
      result = ECAT_RuntimeHandleTest(context, runtime);
      break;

    case ECAT_RUNTIME_CW0F_ROLLBACK_CW07:
      result = ECAT_RuntimeHandleRollbackToSwitchedOn(context, runtime);
      break;

    case ECAT_RUNTIME_CW06_ROLLBACK_ZERO:
      ECAT_RuntimeHandleRollbackZero(runtime);
      break;

    case ECAT_RUNTIME_CW06_FAILED:
      break;

    default:
      break;
  }

  if (result != ECAT_RUNTIME_RESULT_OK)
  {
    return result;
  }

  if (runtime->drive_state == ECAT_RUNTIME_CW06_FAILED)
  {
    ECAT_RuntimeHandleFailed(context);
  }

  return ECAT_RUNTIME_RESULT_OK;
}

/**
 * @brief  执行一个 OP 周期的 PDO 交换、通信监控及 CW06 控制流程
 *
 * @param[in,out] context SOEM 主站上下文，函数会访问过程数据区并可能修改控制字
 * @param[in,out] log     EtherCAT 日志上下文
 * @param[in,out] runtime 运行时上下文，用于维护 WKC、控制流程及快照状态
 *
 * @return
 * 正常交换或连续异常尚未达到阈值返回 ECAT_RUNTIME_RESULT_OK；受管从站缺失、
 * 控制字安全回退失败或连续通信异常达到 ECAT_RUNTIME_WKC_FAULT_LIMIT 时返回
 * ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST
 *
 * @warning
 * context、log 和 runtime 必须有效，全部受管从站必须保持 OP，PDO 指针及长度有效，
 * runtime->expected_wkc 必须正确。本函数会在规定成功周期后把双轴控制字写为 0x0006，
 * 并在故障或超时时回写 0x0000；短暂的发送或 WKC 异常会被容忍并仍返回成功，调用方
 * 必须按固定周期持续调用，且不得并发改写同一过程数据或运行时上下文
 */
ECAT_RuntimeResult ECAT_RuntimeExchangeOperational(
  ecx_contextt *context,
  ECAT_LogContext *log,
  ECAT_RuntimeContext *runtime)
{
  int send_ret;
  int receive_wkc;
  ECAT_RuntimePdoCycleResult cycle_result;
  ECAT_RuntimeDriveState drive_state_before;

  cycle_result = ECAT_RuntimeExchangePdoCycle(context,
                                               runtime->expected_wkc,
                                               &send_ret,
                                               &receive_wkc);
  if (cycle_result == ECAT_RUNTIME_PDO_CYCLE_SLAVE_MISSING)
  {
    ROBOT_TrajectoryRequireResetWithReason(
      ROBOT_TRAJECTORY_HOLD_REASON_ETHERCAT_NOT_OP);
    ECAT_LogPrintf(
      log,
      "[SOEM] OP exchange skip: slavecount=%d need=%u\r\n",
      context->slavecount,
      (unsigned int)ECAT_LAST_SLAVE);
    return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
  }

  if (cycle_result == ECAT_RUNTIME_PDO_CYCLE_OK)
  {
    runtime->bad_wkc_continuous = 0U;
    runtime->pdo_cycle_count++;
    (void)ROBOT_TrajectoryConfirmExecution(runtime->pdo_cycle_count);
    runtime->successful_pdo_count++;
    drive_state_before = runtime->drive_state;
    if (ECAT_RuntimeDispatchDriveState(context, runtime) !=
        ECAT_RUNTIME_RESULT_OK)
    {
      return ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST;
    }
    if ((drive_state_before == ECAT_RUNTIME_AXIS1_BRAKE_RELEASE) &&
        (runtime->drive_state == ECAT_RUNTIME_AXIS2_BRAKE_RELEASE))
    {
      ECAT_LogPrintf(log,
                     "[BRAKE] slave1-3 axis1 CLOSE done\r\n");
    }
    if ((drive_state_before == ECAT_RUNTIME_AXIS2_BRAKE_RELEASE) &&
        (runtime->drive_state == ECAT_RUNTIME_AFTER_BRAKE_STATE))
    {
      ECAT_LogPrintf(log,
                     "[BRAKE] slave1-3 axis2 CLOSE done\r\n");
    }
    if (runtime->successful_pdo_count >=
        ECAT_RUNTIME_TXPDO_PARSE_INTERVAL)
    {
      runtime->successful_pdo_count = 0U;
      (void)ECAT_RuntimeCaptureTxPdoSnapshots(context, runtime);
    }
    return ECAT_RUNTIME_RESULT_OK;
  }

  return ECAT_RuntimeHandlePdoExchangeFailure(context,
                                               log,
                                               runtime,
                                               send_ret,
                                               receive_wkc);
}

static uint8 ECAT_RuntimeAppendTxPdoReport(
  uint32 *used,
  const char *format,
  ...)
{
  va_list arguments;
  uint32 remaining;
  int written;

  if ((used == 0) ||
      (format == 0) ||
      (*used >= ECAT_LOG_BLOCK_BUFFER_SIZE))
  {
    return 0U;
  }

  remaining = ECAT_LOG_BLOCK_BUFFER_SIZE - *used;
  va_start(arguments, format);
  written = vsnprintf(&ecat_runtime_txpdo_report_buffer[*used],
                      remaining,
                      format,
                      arguments);
  va_end(arguments);

  if ((written < 0) || ((uint32)written >= remaining))
  {
    return 0U;
  }

  *used += (uint32)written;
  return 1U;
}

void ECAT_RuntimeProcessTxPdoReport(
  ECAT_LogContext *log,
  ECAT_RuntimeContext *runtime)
{
  AppEtherCAT_ServoTxPdo txpdo[ECAT_SLAVE_COUNT];
  uint32 buffer_index;
  uint32 publish_sequence;
  uint32 report_count;
  uint32 retry;
  uint32 slave_index;
  uint32 axis_index;
  uint32 report_length = 0U;
  uint16 statusword;
  uint8 ready_mask = 0U;
  uint8 switched_on_mask = 0U;
  uint8 operation_enabled_mask = 0U;
  uint8 mode8_mask = 0U;
  uint8 fault_mask = 0U;
  ECAT_RuntimeDriveState drive_state;

  for (retry = 0U; retry < 3U; retry++)
  {
    publish_sequence = runtime->txpdo_snapshot_publish_sequence;
    if (publish_sequence == runtime->txpdo_snapshot_consumed_sequence)
    {
      return;
    }

    buffer_index = publish_sequence %
                   ECAT_RUNTIME_SNAPSHOT_BUFFER_COUNT;
    __DMB();
    memcpy(txpdo,
           runtime->txpdo_snapshot[buffer_index],
           sizeof(txpdo));
    report_count = runtime->txpdo_snapshot_report[buffer_index];
    drive_state = runtime->txpdo_snapshot_drive_state[buffer_index];
    __DMB();
    if (runtime->txpdo_snapshot_publish_sequence == publish_sequence)
    {
      break;
    }
  }

  if (retry >= 3U)
  {
    return;
  }

  runtime->txpdo_snapshot_consumed_sequence = publish_sequence;

  for (slave_index = 0U; slave_index < ECAT_SLAVE_COUNT; slave_index++)
  {
    axis_index = slave_index * 2U;
    statusword = txpdo[slave_index].statusword;
    if ((statusword & 0x006FU) == 0x0021U)
    {
      ready_mask |= (uint8)(1U << axis_index);
    }
    if ((statusword & 0x006FU) == 0x0023U)
    {
      switched_on_mask |= (uint8)(1U << axis_index);
    }
    if ((statusword & 0x006FU) == 0x0027U)
    {
      operation_enabled_mask |= (uint8)(1U << axis_index);
    }
    if (txpdo[slave_index].modes_of_operation_display == 8)
    {
      mode8_mask |= (uint8)(1U << axis_index);
    }
    if ((statusword & 0x0008U) != 0U)
    {
      fault_mask |= (uint8)(1U << axis_index);
    }

    statusword = txpdo[slave_index].axis2_statusword;
    if ((statusword & 0x006FU) == 0x0021U)
    {
      ready_mask |= (uint8)(1U << (axis_index + 1U));
    }
    if ((statusword & 0x006FU) == 0x0023U)
    {
      switched_on_mask |= (uint8)(1U << (axis_index + 1U));
    }
    if ((statusword & 0x006FU) == 0x0027U)
    {
      operation_enabled_mask |=
        (uint8)(1U << (axis_index + 1U));
    }
    if (txpdo[slave_index].axis2_modes_of_operation_display == 8)
    {
      mode8_mask |= (uint8)(1U << (axis_index + 1U));
    }
    if ((statusword & 0x0008U) != 0U)
    {
      fault_mask |= (uint8)(1U << (axis_index + 1U));
    }
  }

  if (ECAT_RuntimeAppendTxPdoReport(
        &report_length,
        "[SOEM] TxPDO-A1 #%lu state=%u R=%02X S=%02X OP=%02X "
        "M8=%02X F=%02X I=%u\r\n",
        (unsigned long)report_count,
        (unsigned int)drive_state,
        (unsigned int)ready_mask,
        (unsigned int)switched_on_mask,
        (unsigned int)operation_enabled_mask,
        (unsigned int)mode8_mask,
        (unsigned int)fault_mask,
        (unsigned int)ECAT_RUNTIME_TXPDO_PARSE_INTERVAL) == 0U)
  {
    ecat_dbg_txpdo_report_dropped++;
    return;
  }

  for (slave_index = 0U;
       slave_index < ECAT_SLAVE_COUNT;
       slave_index++)
  {
    if (ECAT_RuntimeAppendTxPdoReport(
          &report_length,
          " S%u SW=%04X M=%d E=%04X P=%ld V=%ld T=%d\r\n",
          (unsigned int)(ECAT_FIRST_SLAVE + slave_index),
          (unsigned int)txpdo[slave_index].statusword,
          (int)txpdo[slave_index].modes_of_operation_display,
          (unsigned int)txpdo[slave_index].error_code,
          (long)txpdo[slave_index].position_actual_value,
          (long)txpdo[slave_index].velocity_actual_value,
          (int)txpdo[slave_index].torque_actual_value) == 0U)
    {
      ecat_dbg_txpdo_report_dropped++;
      return;
    }
  }

  if (ECAT_LogWriteBlock(log,
                         ecat_runtime_txpdo_report_buffer,
                         (uint16_t)report_length) == 0U)
  {
    ecat_dbg_txpdo_report_dropped++;
  }
}

const char *ECAT_RuntimeResultReason(
  ECAT_RuntimeResult result)
{
  switch (result)
  {
    case ECAT_RUNTIME_RESULT_SAFEOP_SLAVE_MISSING:
      return "SAFE-OP slave missing";

    case ECAT_RUNTIME_RESULT_MAPPING_LEFT_PREOP:
      return "mapping left PRE-OP";

    case ECAT_RUNTIME_RESULT_SAFEOP_FAILED:
      return "SAFE-OP failed";

    case ECAT_RUNTIME_RESULT_TXPDO_DECODE_FAILED:
      return "TxPDO decode failed";

    case ECAT_RUNTIME_RESULT_SAFE_RXPDO_PREPARE_FAILED:
      return "safe RxPDO prepare failed";

    case ECAT_RUNTIME_RESULT_OP_REQUEST_FAILED:
      return "OP request failed";

    case ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST:
      return "OP PDO exchange lost";

    case ECAT_RUNTIME_RESULT_OK:
    default:
      return "runtime failed";
  }
}

/**
 * @brief  将指定从站的输入过程数据解码为伺服 TxPDO 结构并输出诊断信息
 *
 * @param[in]     context SOEM 主站上下文
 * @param[in,out] log     EtherCAT 日志上下文
 * @param[in]     slave   要解码的从站索引
 * @param[out]    txpdo   用于接收 TxPDO 数据的结构体
 *
 * @return
 * 输入指针有效且输入长度足以复制完整 TxPDO 时返回1，否则返回0
 *
 * @warning
 * context、log 和 txpdo 必须有效，slave 必须位于已发现从站范围内；输入区必须采用
 * 与 AppEtherCAT_ServoTxPdo 完全一致的布局和字节序，并在复制期间保持稳定。函数会先
 * 清零 txpdo，失败时其内容保持为零；成功后会同步输出完整 TxPDO 诊断日志
 */
static uint8 ECAT_RuntimeDecodeSlaveTxPdo(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave,
  AppEtherCAT_ServoTxPdo *txpdo)
{
  uint8 *inputs;
  uint32 input_len;

  if (txpdo == 0)
  {
    return 0U;
  }

  memset(txpdo, 0, sizeof(*txpdo));
  inputs = context->slavelist[slave].inputs;
  input_len = context->slavelist[slave].Ibytes;

  if (inputs == 0)
  {
    ECAT_LogPrintf(log,
                            "[SOEM] TxPDO decode skip: slave%d inputs NULL\r\n",
                            slave);
    return 0U;
  }

  if (input_len < sizeof(*txpdo))
  {
    ECAT_LogPrintf(log,
                            "[SOEM] TxPDO decode skip: slave%u len=%lu need=%u\r\n",
                            (unsigned int)slave,
                            (unsigned long)input_len,
                            (unsigned int)sizeof(*txpdo));
    return 0U;
  }

  memcpy(txpdo, inputs, sizeof(*txpdo));
  ECAT_DiagPrintTxPdo(log, slave, txpdo);
  return 1U;
}

/**
 * @brief  检查所有受管从站的双轴状态字是否满足指定掩码条件
 *
 * @param[in]  context     SOEM 主站上下文
 * @param[in]  state_mask  应用于每个轴状态字的位掩码
 * @param[in]  state_value 掩码后期望得到的状态值
 * @param[out] fault_found 可选故障标志；任一轴状态字的故障位有效时置1，可为空
 *
 * @return
 * 全部受管轴均满足 (statusword & state_mask) == state_value 返回1；上下文、从站输入
 * 映射无效或任一轴不匹配时返回0
 *
 * @warning
 * context 必须有效且包含全部受管从站，每个输入区长度必须精确等于
 * APP_ETHERCAT_SERVO_TXPDO_SIZE，并采用匹配的结构布局与字节序；输入数据在复制期间
 * 必须保持稳定。fault_found 的检测结果不会独立改变返回值，除非故障位也包含在状态掩码中
 */
static uint8 ECAT_RuntimeAllManagedAxesMatch(
  const ecx_contextt *context,
  uint16 state_mask,
  uint16 state_value,
  uint8 *fault_found)
{
  const ec_slavet *slave_info;
  AppEtherCAT_ServoTxPdo txpdo;
  uint32 slave_index;
  uint16 slave;
  uint8 all_match = 1U;

  if (fault_found != 0)
  {
    *fault_found = 0U;
  }

  if ((context == 0) ||
      (context->slavecount < (int)ECAT_LAST_SLAVE))
  {
    return 0U;
  }

  for (slave_index = 0U;
       slave_index < ECAT_SLAVE_COUNT;
       slave_index++)
  {
    slave = (uint16)(ECAT_FIRST_SLAVE + slave_index);
    slave_info = &context->slavelist[slave];
    if ((slave_info->inputs == 0) ||
        (slave_info->Ibytes != APP_ETHERCAT_SERVO_TXPDO_SIZE))
    {
      return 0U;
    }

    memcpy(&txpdo, slave_info->inputs, sizeof(txpdo));
    if ((fault_found != 0) &&
        (((txpdo.statusword | txpdo.axis2_statusword) & 0x0008U) != 0U))
    {
      *fault_found = 1U;
    }
    if (((txpdo.statusword & state_mask) != state_value) ||
        ((txpdo.axis2_statusword & state_mask) != state_value))
    {
      all_match = 0U;
    }
  }

  return all_match;
}

/**
 * @brief  将全部受管从站的双轴控制字统一更新为指定值
 *
 * @param[in,out] context     SOEM 主站上下文，函数会改写各从站输出过程数据区
 * @param[in]     controlword 要写入两个轴的 CiA402 控制字
 *
 * @return
 * 全部从站及输出映射均有效并完成统一更新返回1，否则不提交任何更新并返回0
 *
 * @warning
 * context 必须有效并包含全部受管从站，每个输出区长度必须精确等于
 * APP_ETHERCAT_SERVO_RXPDO_SIZE，且布局与 AppEtherCAT_ServoRxPdo 一致。函数会整结构
 * 写回本地输出映像但不会立即发送，controlword 必须符合当前 CiA402 状态；禁止并发
 * 修改同一输出区，否则其他字段的更新可能被暂存映像覆盖
 */
static uint8 ECAT_RuntimeSetAllManagedControlwords(
  ecx_contextt *context,
  uint16 controlword)
{
  AppEtherCAT_ServoRxPdo image[ECAT_SLAVE_COUNT];
  uint32 slave_index;
  uint16 slave;

  if ((context == 0) ||
      (context->slavecount < (int)ECAT_LAST_SLAVE))
  {
    return 0U;
  }

  for (slave_index = 0U; slave_index < ECAT_SLAVE_COUNT; slave_index++)
  {
    slave = (uint16)(ECAT_FIRST_SLAVE + slave_index);
    if ((context->slavelist[slave].outputs == 0) ||
        (context->slavelist[slave].Obytes != APP_ETHERCAT_SERVO_RXPDO_SIZE))
    {
      return 0U;
    }

    memcpy(&image[slave_index], context->slavelist[slave].outputs,
           sizeof(image[slave_index]));
    image[slave_index].controlword = controlword;
    image[slave_index].axis2_controlword = controlword;
  }

  for (slave_index = 0U; slave_index < ECAT_SLAVE_COUNT; slave_index++)
  {
    slave = (uint16)(ECAT_FIRST_SLAVE + slave_index);
    memcpy(context->slavelist[slave].outputs, &image[slave_index],
           sizeof(image[slave_index]));
  }

  return 1U;
}

/**
 * @brief  捕获全部受管从站的 TxPDO 并发布一组双缓冲快照
 *
 * @param[in]     context SOEM 主站上下文，提供各从站输入过程数据
 * @param[in,out] runtime 运行时上下文，用于保存快照、元数据及发布序列
 *
 * @return
 * 全部从站存在且输入缓冲区足以复制完整 TxPDO 时发布快照并返回1，否则返回0
 *
 * @warning
 * context 和 runtime 必须有效，输入过程数据应在一次成功接收后保持稳定，且只能由
 * 单一生产者调用。本函数先校验全部输入再复制，并使用 __DMB() 后更新发布序列；
 * 双缓冲不保证保存每一帧，消费者处理过慢时旧的未消费快照可能被后续发布覆盖
 */
static uint8 ECAT_RuntimeCaptureTxPdoSnapshots(
  ecx_contextt *context,
  ECAT_RuntimeContext *runtime)
{
  uint32 buffer_index;
  uint32 next_sequence;
  uint32 slave_index;
  uint16 slave;
  const uint8 *inputs;
  uint32 input_len;

  if (context->slavecount < (int)ECAT_LAST_SLAVE)
  {
    return 0U;
  }

  for (slave_index = 0U;
       slave_index < ECAT_SLAVE_COUNT;
       slave_index++)
  {
    slave = (uint16)(ECAT_FIRST_SLAVE + slave_index);
    inputs = context->slavelist[slave].inputs;
    input_len = context->slavelist[slave].Ibytes;
    if ((inputs == 0) ||
        (input_len < sizeof(AppEtherCAT_ServoTxPdo)))
    {
      return 0U;
    }
  }

  next_sequence = runtime->txpdo_snapshot_publish_sequence + 1U;
  buffer_index = next_sequence % ECAT_RUNTIME_SNAPSHOT_BUFFER_COUNT;
  for (slave_index = 0U;
       slave_index < ECAT_SLAVE_COUNT;
       slave_index++)
  {
    slave = (uint16)(ECAT_FIRST_SLAVE + slave_index);
    memcpy(&runtime->txpdo_snapshot[buffer_index][slave_index],
           context->slavelist[slave].inputs,
           sizeof(AppEtherCAT_ServoTxPdo));
  }

  runtime->txpdo_report_count++;
  runtime->txpdo_snapshot_report[buffer_index] =
    runtime->txpdo_report_count;
  runtime->txpdo_snapshot_drive_state[buffer_index] = runtime->drive_state;
  __DMB();
  runtime->txpdo_snapshot_publish_sequence = next_sequence;
  return 1U;
}

/**
 * @brief  为指定从站构造并写入双轴安全 RxPDO 输出映像
 *
 * @param[in,out] context SOEM 主站上下文，函数会改写目标从站的输出过程数据区
 * @param[in,out] log     EtherCAT 日志上下文
 * @param[in]     slave   目标从站索引
 * @param[in]     txpdo   当前 TxPDO；可为空，为空时模式和目标位置均使用0
 *
 * @return
 * 输出指针有效且空间足以容纳完整 RxPDO 时返回1，否则记录日志并返回0
 *
 * @warning
 * context 和 log 必须有效，slave 必须位于已映射从站范围内；输出区必须采用与
 * AppEtherCAT_ServoRxPdo 一致的布局和字节序。函数将双轴控制字保持为0，目标位置取
 * 当前实际位置并清零转矩及速度前馈；它只更新本地输出映像，不会立即发送过程数据
 */
static uint8 ECAT_RuntimePrepareSafeRxPdo(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave,
  const AppEtherCAT_ServoTxPdo *txpdo)
{
  AppEtherCAT_ServoRxPdo rxpdo;
  uint8 *outputs;
  uint32 output_len;

  outputs = context->slavelist[slave].outputs;
  output_len = context->slavelist[slave].Obytes;

  if (outputs == 0)
  {
    ECAT_LogPrintf(log,
                            "[SOEM] safe RxPDO skip: slave%d outputs NULL\r\n",
                            slave);
    return 0U;
  }

  if (output_len < sizeof(rxpdo))
  {
    ECAT_LogPrintf(log,
                            "[SOEM] safe RxPDO skip: slave%u len=%lu need=%u\r\n",
                            (unsigned int)slave,
                            (unsigned long)output_len,
                            (unsigned int)sizeof(rxpdo));
    return 0U;
  }

  memset(&rxpdo, 0, sizeof(rxpdo));
  rxpdo.controlword = 0x0000U;
  rxpdo.modes_of_operation =
    (txpdo != 0) ? txpdo->modes_of_operation_display : 0;
  rxpdo.target_position = (txpdo != 0) ? txpdo->position_actual_value : 0;
  rxpdo.target_torque = 0;
  rxpdo.axis1_velocity_feed_forward = 0;
  rxpdo.axis2_controlword = 0x0000U;
  rxpdo.axis2_modes_of_operation =
    (txpdo != 0) ? txpdo->axis2_modes_of_operation_display : 0;
  rxpdo.axis2_target_position =
    (txpdo != 0) ? txpdo->axis2_position_actual_value : 0;
  rxpdo.axis2_target_torque = 0;
  rxpdo.axis2_velocity_feed_forward = 0;

  memcpy(outputs, &rxpdo, sizeof(rxpdo));

  ECAT_LogPrintf(log,
                          "[SOEM] safe RxPDO prepared slave%d len=%u\r\n",
                          slave,
                          (unsigned int)sizeof(rxpdo));
  ECAT_LogPrintf(
    log,
    "[SOEM] slave%u Axis1 safe 0x6040 CW=0x%04X 0x6060 mode=%d 0x607A target=%ld\r\n",
    (unsigned int)slave,
    (unsigned int)rxpdo.controlword,
    (int)rxpdo.modes_of_operation,
    (long)rxpdo.target_position);
  ECAT_LogPrintf(
    log,
    "[SOEM] slave%u Axis2 safe 0x6840 CW=0x%04X 0x6860 mode=%d 0x687A target=%ld\r\n",
    (unsigned int)slave,
    (unsigned int)rxpdo.axis2_controlword,
    (int)rxpdo.axis2_modes_of_operation,
    (long)rxpdo.axis2_target_position);
  ECAT_LogPrintf(
    log,
    "[SOEM] slave%u safe RxPDO target torque=0 velocity feed-forward=0\r\n",
    (unsigned int)slave);
  return 1U;
}

/**
 * @brief  将全部受管轴的目标位置重新锁存为当前实际位置
 *
 * @param[in,out] context SOEM 主站上下文，函数会读取 TxPDO 并改写 RxPDO 输出映像
 *
 * @return
 * 全部从站的映射、控制字、CSP 模式及 Switched On 状态均符合要求并完成统一更新
 * 返回1；任一条件不满足时不提交任何更新并返回0
 *
 * @warning
 * context 必须有效并包含全部受管从站，输入输出长度必须精确匹配工程 PDO 结构；
 * 调用时双轴控制字必须为 0x0007、命令及显示模式均为8，状态字掩码结果必须为
 * 0x0023。函数会整结构写回，将目标位置设为反馈位置并清零转矩与速度前馈，但不会
 * 立即发送过程数据；禁止并发更新输入或输出映像
 */
static uint8 ECAT_RuntimeRelatchAllManagedTargets(
  ecx_contextt *context)
{
  AppEtherCAT_ServoRxPdo image[ECAT_SLAVE_COUNT];
  AppEtherCAT_ServoTxPdo feedback;
  ec_slavet *slave_info;
  uint32 slave_index;
  uint16 slave;
  if ((context == 0) || (context->slavecount < (int)ECAT_LAST_SLAVE))
  {
    return 0U;
  }
  for (slave_index = 0U; slave_index < ECAT_SLAVE_COUNT; slave_index++)
  {
    slave = (uint16)(ECAT_FIRST_SLAVE + slave_index);
    slave_info = &context->slavelist[slave];
    if ((slave_info->outputs == 0) || (slave_info->inputs == 0) ||
        (slave_info->Obytes != APP_ETHERCAT_SERVO_RXPDO_SIZE) ||
        (slave_info->Ibytes != APP_ETHERCAT_SERVO_TXPDO_SIZE))
    {
      return 0U;
    }
    memcpy(&image[slave_index], slave_info->outputs,
           sizeof(image[slave_index]));
    memcpy(&feedback, slave_info->inputs, sizeof(feedback));
    if ((image[slave_index].controlword != 0x0007U) ||
        (image[slave_index].axis2_controlword != 0x0007U) ||
        (image[slave_index].modes_of_operation != 8) ||
        (image[slave_index].axis2_modes_of_operation != 8) ||
        (feedback.modes_of_operation_display != 8) ||
        (feedback.axis2_modes_of_operation_display != 8) ||
        ((feedback.statusword & 0x006FU) != 0x0023U) ||
        ((feedback.axis2_statusword & 0x006FU) != 0x0023U))
    {
      return 0U;
    }
    image[slave_index].target_position = feedback.position_actual_value;
    image[slave_index].target_torque = 0;
    image[slave_index].axis1_velocity_feed_forward = 0;
    image[slave_index].axis2_target_position =
      feedback.axis2_position_actual_value;
    image[slave_index].axis2_target_torque = 0;
    image[slave_index].axis2_velocity_feed_forward = 0;
  }
  for (slave_index = 0U; slave_index < ECAT_SLAVE_COUNT; slave_index++)
  {
    slave = (uint16)(ECAT_FIRST_SLAVE + slave_index);
    memcpy(context->slavelist[slave].outputs, &image[slave_index],
           sizeof(image[slave_index]));
  }
  return 1U;
}

/**
 * @brief  检查全部受管轴是否满足进入 Operation Enabled 前的安全条件
 *
 * @param[in]  context            SOEM 主站上下文
 * @param[in]  position_tolerance 目标位置与实际位置允许的最大绝对偏差
 * @param[in]  velocity_tolerance 实际速度允许的最大绝对值
 * @param[out] fault_found        可选故障标志；任一轴状态字的 Fault 位有效时置1，可为空
 *
 * @return
 * 全部轴均处于 Switched On 状态，控制字为0x0007、CSP模式为8、驱动无错误，
 * 且转矩与速度前馈为零、位置偏差及实际速度均在容差内时返回1；否则返回0
 *
 * @warning
 * context 必须有效且包含全部受管从站，输入输出PDO指针、长度、结构布局及字节序
 * 必须正确；调用前应完成有效的过程数据接收，并保证检查期间PDO映像不被并发修改。
 * fault_found 仅表示状态字 Fault 位，不会涵盖映射无效、模式不符或超出容差等失败原因
 */
static uint8 ECAT_RuntimeAllManagedAxesPassEnableCheck(const ecx_contextt *context,
  uint32 position_tolerance, uint32 velocity_tolerance, uint8 *fault_found)
{
  AppEtherCAT_ServoRxPdo rxpdo;
  AppEtherCAT_ServoTxPdo txpdo;
  int64_t position_error;
  int64_t velocity;
  uint32 slave_index;
  uint16 slave;
  if (ECAT_RuntimeAllManagedAxesMatch(context, 0x006FU, 0x0023U,
                                      fault_found) == 0U)
  {
    return 0U;
  }
  for (slave_index = 0U; slave_index < ECAT_SLAVE_COUNT; slave_index++)
  {
    slave = (uint16)(ECAT_FIRST_SLAVE + slave_index);
    if ((context->slavelist[slave].outputs == 0) ||
        (context->slavelist[slave].Obytes != APP_ETHERCAT_SERVO_RXPDO_SIZE))
    {
      return 0U;
    }
    memcpy(&rxpdo, context->slavelist[slave].outputs, sizeof(rxpdo));
    memcpy(&txpdo, context->slavelist[slave].inputs, sizeof(txpdo));
    if ((rxpdo.controlword != 0x0007U) || (rxpdo.axis2_controlword != 0x0007U) ||
        (rxpdo.modes_of_operation != 8) || (rxpdo.axis2_modes_of_operation != 8) ||
        (txpdo.modes_of_operation_display != 8) ||
        (txpdo.axis2_modes_of_operation_display != 8) ||
        (txpdo.error_code != 0U) || (txpdo.axis2_error_code != 0U) ||
        (rxpdo.target_torque != 0) || (rxpdo.axis2_target_torque != 0) ||
        (rxpdo.axis1_velocity_feed_forward != 0) ||
        (rxpdo.axis2_velocity_feed_forward != 0))
    {
      return 0U;
    }
    position_error = (int64_t)rxpdo.target_position -
                     (int64_t)txpdo.position_actual_value;
    velocity = (int64_t)txpdo.velocity_actual_value;
    if ((position_error > (int64_t)position_tolerance) ||
        (position_error < -(int64_t)position_tolerance) ||
        (velocity > (int64_t)velocity_tolerance) ||
        (velocity < -(int64_t)velocity_tolerance))
    {
      return 0U;
    }
    position_error = (int64_t)rxpdo.axis2_target_position -
                     (int64_t)txpdo.axis2_position_actual_value;
    velocity = (int64_t)txpdo.axis2_velocity_actual_value;
    if ((position_error > (int64_t)position_tolerance) ||
        (position_error < -(int64_t)position_tolerance) ||
        (velocity > (int64_t)velocity_tolerance) ||
        (velocity < -(int64_t)velocity_tolerance))
    {
      return 0U;
    }
  }
  return 1U;
}
