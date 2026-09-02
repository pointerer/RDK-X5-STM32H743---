/*
 * 文件作用：
 *   本文件用于验证 STM32H743 上的 SOEM 主站移植和 EtherCAT 安全过程数据通道，
 *   不承担正式主站控制业务。当前拓扑要求发现 3 个从站；状态切换面向全部从站，
 *   mailbox、PDO 映射以及双轴伺服数据检查覆盖 slave1~3。
 *
 * 当前调试流程：
 *   1. 等待 LAN8720 PHY LinkUp；等待期间每秒打印一次 PHY BSR。
 *   2. 通过 ecx_init() 初始化 SOEM 上下文及 STM32 ETH 裸帧网卡移植层。
 *   3. 最多发送 50 次 BRD 广播读探测帧，检查 EtherType 0x88A4、TX/RX 路径和 WKC；
 *      WKC 等于预期从站数 3 后继续。
 *   4. 通过 ecx_config_init() 扫描并严格校验 3 个从站，逐个打印配置地址、
 *      Vendor ID、Product Code 和名称。
 *   5. 请求全部从站进入 PRE-OP，读取各从站状态和 AL Status Code。
 *   6. 检查 slave1~3 mailbox/CoE 能力，逐个读取 0x1000 和完整的 0x1018 身份信息。
 *   7. 读取 slave1~3 的 0x1C12/0x1C13 PDO assignment，展开并缓存各 PDO mapping entry。
 *   8. 临时启用 manualstatechange 调用 ecx_config_map_group() 生成 IOmap，随后恢复原值；
 *      校验 IOmap 大小、各从站输入/输出指针以及 22 字节 RxPDO/42 字节 TxPDO，并打印
 *      每个映射项在 IOmap 中解析出的方向、位偏移和值。
 *   9. 请求 SAFE-OP 并完成一次过程数据交换，打印并解码三个从站的双轴 TxPDO；构造安全
 *      RxPDO 时，轴1目标位置取当前实际位置、模式取当前显示值，其余字段清零，双轴
 *      controlword 均保持 0x0000。
 *  10. 预发送安全 RxPDO 后请求 OP；最多等待 50 个周期，要求 OP 状态和期望 WKC 连续
 *      稳定 3 个 2 ms 周期。失败时先回退 SAFE-OP，再进入 ERROR。
 *  11. 进入 OPERATIONAL 后由 defaultTask 每 2 ms 持续交换 PDO，不主动退出 OP；每累计
 *      1000 次成功交换批量抓取三个从站的 TxPDO 快照，由 txPdoReportTask 异步打印各从站轴1。
 *      连续 100 次发送失败或 WKC 异常时关闭 SOEM 上下文并进入 ERROR。
 *
 * 安全边界与限制：
 *   - 只验证 controlword=0x0000 的安全 OP 通道，不执行 CiA402 使能序列，不下发运动目标。
 *   - 当前有效流程不配置 DC-SYNC0。
 *   - ECAT_ctx.iomap[] 是调试专用 IOmap，SOEM 会把从站 outputs/inputs 指针映射到其中。
 *   - 所有关键节点和错误诊断均通过 ECAT_log -> APP_UART_LOG_Write() 入队输出。
 *
 * 主要调用关系：
 *   freertos.c/MX_FREERTOS_Init()
 *     -> APP_UART_LOG_Init() -> ECAT_Init()
 *
 *   freertos.c/defaultTask
 *     -> ECAT_Process()
 *        -> WAIT_LINK / SOEM_INIT / PROBE / DISCOVER / PREOP
 *        -> SDO / PDO_ASSIGN / MAP / SAFEOP / OP_REQUEST / OPERATIONAL
 *
 *   freertos.c/txPdoReportTask
 *     -> ECAT_ProcessTxPdoReport()
 */
#include "ECAT.h"
#include "ECAT_diag.h"
#include "ECAT_log.h"
#include "ECAT_pdo_config.h"
#include "ECAT_runtime.h"
#include "ECAT_runtime_sdo.h"
#include "ECAT_sdo.h"

#include "bsp_lan8720.h"
#include "bsp_soem_timebase.h"
#include "cmsis_compiler.h"
#include "soem/soem.h"

#include <string.h>

#define ECAT_PROBE_COUNT      200U
#define ECAT_EXPECTED_SLAVES  ((int)ECAT_SLAVE_COUNT)
#define ECAT_WAIT_TIMEOUT_US  EC_TIMEOUTRET
#define ECAT_PHY_PRINT_US     1000000ULL
#define ECAT_IOMAP_SIZE       512U
#define ECAT_MBX_SERVICE_GROUP  0U
#define ECAT_MBX_SERVICE_LIMIT  1

typedef struct
{
  ecx_contextt soem;                         /* SOEM 主站协议栈上下文，包含端口和从站表 */
  volatile ECAT_State state;                /* EtherCAT 应用状态机当前状态 */
  ECAT_DiagContext diag;                    /* PHY 链路诊断打印节流状态 */
  uint8 iomap[ECAT_IOMAP_SIZE];             /* 从站输入、输出 PDO 的 IOmap 存储区 */
  ECAT_LogContext log;                      /* EtherCAT 日志接口上下文 */
  volatile uint8 context_open;              /* SOEM 上下文已成功打开标志 */
  volatile uint8 cyclic_mbx_service_ready; /* 周期邮箱映射并启用成功标志 */
  volatile uint8 sdo_worker_busy;           /* 异步 SDO Worker 正在执行作业标志 */
  volatile uint8 close_pending;             /* 等待 SDO Worker 空闲后关闭上下文的挂起标志 */
  volatile uint32 can_feedback_cycle_count; /* 成功 OP PDO 周期计数，用于标识 CAN 数据新鲜度 */
  uint32 probe_attempt_count;               /* 已执行的BRD探测次数，跨ECAT_Process周期保留 */
  ECAT_RuntimeContext runtime;              /* PDO 交换、驱动控制、SDO 作业及快照运行上下文 */
  ECAT_PdoConfigContext pdo_config;         /* 各受管从站 PDO 映射缓存与 PO2SO 配置上下文 */
} ECAT_Context;

typedef enum
{
  ECAT_STATE_EVENT_RESET = 0,
  ECAT_STATE_EVENT_LINK_UP,
  ECAT_STATE_EVENT_SOEM_INIT_OK,
  ECAT_STATE_EVENT_PROBE_OK,
  ECAT_STATE_EVENT_DISCOVER_OK,
  ECAT_STATE_EVENT_PREOP_OK,
  ECAT_STATE_EVENT_SDO_DONE,
  ECAT_STATE_EVENT_PDO_ASSIGN_OK,
  ECAT_STATE_EVENT_MAP_OK,
  ECAT_STATE_EVENT_SAFEOP_OK,
  ECAT_STATE_EVENT_OP_STABLE,
  ECAT_STATE_EVENT_FATAL_ERROR
} ECAT_StateEvent;

typedef struct
{
  ECAT_State current_state;
  ECAT_StateEvent event;
  ECAT_State next_state;
} ECAT_StateTransition;

static const ECAT_StateTransition ECAT_state_transitions[] =
{
  {ECAT_STATE_WAIT_LINK,
   ECAT_STATE_EVENT_LINK_UP,
   ECAT_STATE_SOEM_INIT},
  {ECAT_STATE_SOEM_INIT,
   ECAT_STATE_EVENT_SOEM_INIT_OK,
   ECAT_STATE_PROBE},
  {ECAT_STATE_PROBE,
   ECAT_STATE_EVENT_PROBE_OK,
   ECAT_STATE_DISCOVER},
  {ECAT_STATE_DISCOVER,
   ECAT_STATE_EVENT_DISCOVER_OK,
   ECAT_STATE_PREOP},
  {ECAT_STATE_PREOP,
   ECAT_STATE_EVENT_PREOP_OK,
   ECAT_STATE_SDO},
  {ECAT_STATE_SDO,
   ECAT_STATE_EVENT_SDO_DONE,
   ECAT_STATE_PDO_ASSIGN},
  {ECAT_STATE_PDO_ASSIGN,
   ECAT_STATE_EVENT_PDO_ASSIGN_OK,
   ECAT_STATE_MAP},
  {ECAT_STATE_MAP,
   ECAT_STATE_EVENT_MAP_OK,
   ECAT_STATE_SAFEOP},
  {ECAT_STATE_SAFEOP,
   ECAT_STATE_EVENT_SAFEOP_OK,
   ECAT_STATE_OP_REQUEST},
  {ECAT_STATE_OP_REQUEST,
   ECAT_STATE_EVENT_OP_STABLE,
   ECAT_STATE_OPERATIONAL}
};

/* Keep the large SOEM context in ZI; ECAT_Init() sets the first runtime state. */
static ECAT_Context ECAT_ctx;

/* Temporary counters for observing the asynchronous runtime SDO handoff. */
volatile uint32 ecat_dbg_sdo_worker_calls;
volatile uint32 ecat_dbg_sdo_write_pending_seen;
volatile uint32 ecat_dbg_sdo_claims;
volatile uint32 ecat_dbg_sdo_last_operation;
volatile uint32 ecat_dbg_sdo_last_slave;
volatile uint32 ecat_dbg_sdo_last_index;
volatile uint32 ecat_dbg_sdo_last_subindex;
volatile uint32 ecat_dbg_sdo_last_size;
volatile uint32 ecat_dbg_sdo_last_elapsed_us;
volatile int ecat_dbg_sdo_last_wkc;
volatile uint32 ecat_dbg_sdo_last_pdo_cycle_before;
volatile uint32 ecat_dbg_sdo_last_pdo_cycle_after;
volatile uint32 ecat_dbg_sdo_attempt_count;
volatile uint32 ecat_dbg_sdo_complete_count;
volatile uint32 ecat_dbg_sdo_error_flag;
volatile uint32 ecat_dbg_sdo_error_head_before;
volatile uint32 ecat_dbg_sdo_error_head_after;
volatile uint32 ecat_dbg_sdo_error_tail_after;
volatile uint32 ecat_dbg_sdo_error_new_count;
volatile uint32 ecat_dbg_sdo_error_present;
volatile uint32 ecat_dbg_sdo_error_matched;
volatile uint32 ecat_dbg_sdo_error_type;
volatile uint32 ecat_dbg_sdo_error_slave;
volatile uint32 ecat_dbg_sdo_error_index;
volatile uint32 ecat_dbg_sdo_error_subindex;
volatile uint32 ecat_dbg_sdo_error_code;

#define ECAT_PrintSectionLine() \
  ECAT_LogSectionLine(&ECAT_ctx.log)
#define ECAT_Printf(...) \
  ECAT_LogPrintf(&ECAT_ctx.log, __VA_ARGS__)

static void ECAT_StepSoemInit(void);
static void ECAT_StepProbe(void);
static void ECAT_StepDiscover(void);
static void ECAT_StepSafeOp(void);
static void ECAT_StepOpRequest(void);
static void ECAT_StepOperational(void);
static void ECAT_EnterError(const char *reason);
static void ECAT_CloseContextWhenWorkerIdle(void);
static void ECAT_CommitState(ECAT_State next_state,
                                      ECAT_StateEvent event);
static void ECAT_Transition(ECAT_StateEvent event);
static void ECAT_ResetState(void);
static int ECAT_RunProbe(uint32 probe_index);
static uint8 ECAT_PrepareProbeFrame(uint16 *read_data);
static void ECAT_PrintProbeFrame(uint8 idx);
static void ECAT_PrintProbeResult(int wait_ret);
static void ECAT_ReadStateAndRequestPreOp(void);
static void ECAT_ReadBasicSdoInfo(void);
static void ECAT_ReadDeviceTypeSdo(uint16 slave);
static void ECAT_ReadIdentitySdo(uint16 slave);
static void ECAT_StepPdoAssignment(void);
static void ECAT_StepPdoConfig(void);
static uint8 ECAT_ServiceCyclicMailbox(void);
static int ECAT_PO2SOconfig(ecx_contextt *context, uint16 slave);
static uint16 ECAT_ReadTxEtherType(uint8 idx);

/**
 * @brief  提交 EtherCAT 主站状态并记录状态迁移日志
 *
 * @param[in] next_state   要写入的目标状态
 * @param[in] event        触发状态迁移的事件，仅用于日志记录
 *
 * @return
 * 无
 *
 * @warning
 * 该函数不校验状态迁移是否合法，并会直接改写全局状态；常规状态迁移应通过
 * ECAT_Transition() 完成，禁止并发调用
 */
static void ECAT_CommitState(ECAT_State next_state,
                                      ECAT_StateEvent event)
{
  ECAT_State previous_state = ECAT_ctx.state;

  ECAT_ctx.state = next_state;
  ECAT_Printf("[SOEM] app state %u->%u event=%u\r\n",
                       (unsigned int)previous_state,
                       (unsigned int)next_state,
                       (unsigned int)event);
}

/**
 * @brief  根据当前状态和事件推进 EtherCAT 主站状态机
 *
 * @param[in] event   触发状态迁移的事件
 *
 * @return
 * 无
 *
 * @warning
 * 调用前必须完成 ECAT_Init()，且 event 必须与当前状态匹配；非法状态迁移
 * 会使主站进入 ECAT_STATE_ERROR 状态
 */
static void ECAT_Transition(ECAT_StateEvent event)
{
  uint32 transition_index;

  for (transition_index = 0U;
       transition_index <
         (sizeof(ECAT_state_transitions) /
          sizeof(ECAT_state_transitions[0]));
       transition_index++)
  {
    const ECAT_StateTransition *transition =
      &ECAT_state_transitions[transition_index];

    if ((transition->current_state == ECAT_ctx.state) &&
        (transition->event == event))
    {
      ECAT_CommitState(transition->next_state, event);
      return;
    }
  }

  ECAT_Printf("[SOEM] invalid app state transition state=%u event=%u\r\n",
                       (unsigned int)ECAT_ctx.state,
                       (unsigned int)event);
  ECAT_EnterError("invalid app state transition");
}

static void ECAT_ResetState(void)
{
  ECAT_CommitState(ECAT_STATE_WAIT_LINK,
                            ECAT_STATE_EVENT_RESET);
}

/**
 * @brief  初始化 EtherCAT 调试主站运行上下文
 *
 * @return
 * 无
 *
 * @warning
 * 调用该函数前必须完成 SOEM 微秒时基、以太网外设及日志串口初始化；
 * 仅可在调度器启动前，或 ERROR 状态的关闭清理完成后调用，禁止与
 * ECAT_Process() 并发调用；SDO Worker 或关闭流程忙时本次不做重置，需稍后重试
 */
void ECAT_Init(void)
{
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  if ((ECAT_ctx.sdo_worker_busy != 0U) ||
      (ECAT_ctx.close_pending != 0U) ||
      ((ECAT_ctx.state != ECAT_STATE_IDLE) &&
       (ECAT_ctx.state != ECAT_STATE_ERROR)))
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return;
  }
  ECAT_ctx.cyclic_mbx_service_ready = 0U;
  ECAT_ctx.close_pending = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (ECAT_ctx.context_open != 0U)
  {
    ecx_close(&ECAT_ctx.soem);
  }

  memset(&ECAT_ctx.soem, 0, sizeof(ECAT_ctx.soem));
  memset(ECAT_ctx.iomap, 0, sizeof(ECAT_ctx.iomap));
  ECAT_PdoConfigReset(&ECAT_ctx.pdo_config);
  ECAT_RuntimeReset(&ECAT_ctx.runtime);
  ECAT_ctx.context_open = 0U;
  ECAT_ctx.can_feedback_cycle_count = 0U;
  ECAT_ctx.probe_attempt_count = 0U;
  ECAT_ResetState();
  ECAT_DiagReset(&ECAT_ctx.diag);

  ECAT_PrintSectionLine();
  ECAT_Printf("\r\n[SOEM] debug start\r\n");
  ECAT_Printf("[SOEM] time_us=%llu\r\n",
                       (unsigned long long)BSP_SOEM_Timebase_GetUs());
  ECAT_Printf("[SOEM] wait PHY link up\r\n");
}

/**
 * @brief  推进 EtherCAT 主站状态机并执行当前状态处理
 *
 * @return
 * 无
 *
 * @warning
 * 调用该函数前必须完成 ECAT_Init()；进入 OPERATIONAL 状态后应由同一任务
 * 每 2 ms 周期调用，禁止并发重入
 */
void ECAT_Process(void)
{
  switch (ECAT_ctx.state)
  {
    case ECAT_STATE_WAIT_LINK:
      if (LAN8720_IsLinkUp() == 0U)
      {
        ECAT_DiagProcessPhyMonitor(
          &ECAT_ctx.diag,
          &ECAT_ctx.log,
          ECAT_PHY_PRINT_US);
        break;
      }

      ECAT_PrintSectionLine();
      ECAT_Printf("[SOEM] PHY link up, start SOEM init\r\n");
      ECAT_Transition(ECAT_STATE_EVENT_LINK_UP);
      break;

    case ECAT_STATE_SOEM_INIT:
      ECAT_StepSoemInit();
      break;

    case ECAT_STATE_PROBE:
      ECAT_StepProbe();
      break;

    case ECAT_STATE_DISCOVER:
      ECAT_StepDiscover();
      break;

    case ECAT_STATE_PREOP:
      ECAT_ReadStateAndRequestPreOp();
      break;

    case ECAT_STATE_SDO:
      ECAT_ReadBasicSdoInfo();
      break;

    case ECAT_STATE_PDO_ASSIGN:
      ECAT_StepPdoAssignment();
      break;

    case ECAT_STATE_MAP:
      ECAT_StepPdoConfig();
      break;

    case ECAT_STATE_SAFEOP:
      ECAT_StepSafeOp();
      break;

    case ECAT_STATE_OP_REQUEST:
      ECAT_StepOpRequest();
      break;

    case ECAT_STATE_OPERATIONAL:
      ECAT_StepOperational();
      break;

    case ECAT_STATE_ERROR:
      ECAT_CloseContextWhenWorkerIdle();
      break;

    case ECAT_STATE_DONE:
    case ECAT_STATE_IDLE:
    default:
      break;
  }
}

ECAT_State ECAT_GetState(void)
{
  return ECAT_ctx.state;
}

uint8_t ECAT_GetFeedbackSnapshot(ECAT_FeedbackSnapshot *snapshot)
{
  ECAT_FeedbackSnapshot local_snapshot;
  uint32_t primask;
  uint32_t slave_index;
  uint16_t slave;

  if (snapshot == 0)
  {
    return 0U;
  }

  memset(&local_snapshot, 0, sizeof(local_snapshot));
  primask = __get_PRIMASK();
  __disable_irq();

  /* 仅复制最近一次WKC有效的OP输入映像，避免上报半更新PDO。 */
  if ((ECAT_ctx.state != ECAT_STATE_OPERATIONAL) ||
      (ECAT_ctx.context_open == 0U) ||
      (ECAT_ctx.runtime.bad_wkc_continuous != 0U) ||
      (ECAT_ctx.soem.slavecount < (int)ECAT_LAST_SLAVE))
  {
    __set_PRIMASK(primask);
    return 0U;
  }

  for (slave_index = 0U; slave_index < ECAT_SLAVE_COUNT; slave_index++)
  {
    slave = (uint16_t)(ECAT_FIRST_SLAVE + slave_index);
    if ((ECAT_ctx.soem.slavelist[slave].inputs == 0) ||
        (ECAT_ctx.soem.slavelist[slave].Ibytes !=
         APP_ETHERCAT_SERVO_TXPDO_SIZE))
    {
      __set_PRIMASK(primask);
      return 0U;
    }

    memcpy(&local_snapshot.slave[slave_index],
           ECAT_ctx.soem.slavelist[slave].inputs,
           sizeof(local_snapshot.slave[slave_index]));
    local_snapshot.valid_slave_mask |= (uint8_t)(1U << slave_index);

    if (ECAT_ctx.soem.slavelist[slave].state == EC_STATE_OPERATIONAL)
    {
      local_snapshot.operational_slave_mask |=
        (uint8_t)(1U << slave_index);
    }
  }

  local_snapshot.state = ECAT_ctx.state;
  local_snapshot.cycle_counter = ECAT_ctx.can_feedback_cycle_count;
  local_snapshot.working_counter =
    (uint16_t)ECAT_ctx.runtime.expected_wkc;
  __DMB();
  __set_PRIMASK(primask);

  *snapshot = local_snapshot;
  return 1U;
}

/**
 * @brief  请求全部受管轴异步进入 Operation Enabled 状态
 *
 * @return
 * 全部轴已完成 Switched On 阶段且本次请求被接受时返回1；运行阶段未就绪或已有
 * 待处理请求时返回0
 *
 * @warning
 * 调用该函数前必须完成 ECAT_Init()，且主站应已进入 ECAT_STATE_OPERATIONAL、
 * 全部受管轴已完成控制字0x0007切换。本函数仅设置请求标志，实际目标重锁存、
 * 安全条件检查及控制字0x000F切换由后续 ECAT_Process() 周期异步执行
 */
uint8_t ECAT_RequestEnableOperation(void)
{
  return (uint8_t)ECAT_RuntimeRequestEnableOperation(&ECAT_ctx.runtime);
}

void ECAT_ProcessTxPdoReport(void)
{
  ECAT_RuntimeProcessTxPdoReport(&ECAT_ctx.log,
                                 &ECAT_ctx.runtime);
}

/**
 * @brief  查询运行期异步 SDO Worker 是否有待领取作业
 *
 * @return
 * sdo_job 为 READ_PENDING 或 WRITE_PENDING 时返回1，否则返回0
 *
 * @warning
 * 本函数只读取作业状态，不领取或修改作业。调用方可据此唤醒唯一的 SDO Worker；
 * 真正领取仍由 ECAT_ProcessRuntimeSdoWorker() 在同一临界区内完成。
 */
uint8_t ECAT_IsRuntimeSdoWorkPending(void)
{
  ECAT_RuntimeSdoJobState state;
  uint32 primask;

  primask = __get_PRIMASK();
  __disable_irq();
  state = ECAT_ctx.runtime.sdo_job.state;
  if (primask == 0U)
  {
    __enable_irq();
  }

  return ((state == ECAT_RUNTIME_SDO_JOB_READ_PENDING) ||
          (state == ECAT_RUNTIME_SDO_JOB_WRITE_PENDING)) ? 1U : 0U;
}

/**
 * @brief  领取并执行一个运行期异步 SDO 作业
 *
 * @return
 * 无
 *
 * @warning
 * 调用前必须完成 ECAT_Init()，并由独立的低优先级任务在收到作业通知后调用；只有主站处于
 * ECAT_STATE_OPERATIONAL、SOEM 上下文已打开且循环邮箱服务就绪时才会领取作业。
 * 本函数每次最多处理一个 READ_PENDING 或 WRITE_PENDING 作业，底层 SDO 读写最多
 * 可阻塞 ECAT_RUNTIME_SDO_RESPONSE_TIMEOUT_US，禁止从 2 ms EtherCAT 周期任务、
 * 中断或多个 Worker 并发调用。完成后无论成功失败都会发布 DONE，调用方应通过
 * 作业的 WKC、长度和数据判断结果；不得绕过作业状态握手修改共享字段
 */
void ECAT_ProcessRuntimeSdoWorker(void)
{
  ECAT_RuntimeSdoJob *job = &ECAT_ctx.runtime.sdo_job;
  ECAT_RuntimeSdoJobState operation;
  uint16 slave;
  uint16 index;
  uint8 subindex;
  uint8 data[ECAT_RUNTIME_SDO_DATA_SIZE];
  int size;
  int wkc;
  uint32 primask;
  uint8 claimed = 0U;
  uint64 start_us;
  int16 error_head_before;
  int16 error_head_after;
  int16 error_cursor;
  uint32 error_scan_count;
  const ec_errort *sdo_error;

  primask = __get_PRIMASK();
  __disable_irq();
  operation = job->state;
  ecat_dbg_sdo_worker_calls++;
  if (operation == ECAT_RUNTIME_SDO_JOB_WRITE_PENDING)
  {
    ecat_dbg_sdo_write_pending_seen++;
  }
  if ((ECAT_ctx.state == ECAT_STATE_OPERATIONAL) &&
      (ECAT_ctx.context_open != 0U) &&
      (ECAT_ctx.cyclic_mbx_service_ready != 0U) &&
      (ECAT_ctx.sdo_worker_busy == 0U) &&
      ((operation == ECAT_RUNTIME_SDO_JOB_READ_PENDING) ||
       (operation == ECAT_RUNTIME_SDO_JOB_WRITE_PENDING)))
  {
    __DMB();
    slave = job->slave;
    index = job->index;
    subindex = job->subindex;
    size = job->size;
    memcpy(data, job->data, sizeof(data));
    job->state = ECAT_RUNTIME_SDO_JOB_RUNNING;
    ECAT_ctx.sdo_worker_busy = 1U;
    ecat_dbg_sdo_claims++;
    claimed = 1U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (claimed == 0U)
  {
    return;
  }

  ecat_dbg_sdo_last_operation = (uint32)operation;
  ecat_dbg_sdo_last_slave = (uint32)slave;
  ecat_dbg_sdo_last_index = (uint32)index;
  ecat_dbg_sdo_last_subindex = (uint32)subindex;
  ecat_dbg_sdo_last_size = (uint32)size;
  ecat_dbg_sdo_last_pdo_cycle_before =
    ECAT_ctx.can_feedback_cycle_count;
  error_head_before = ECAT_ctx.soem.elist.head;
  ecat_dbg_sdo_error_flag = 0U;
  ecat_dbg_sdo_error_head_before = (uint32)error_head_before;
  ecat_dbg_sdo_error_head_after = (uint32)error_head_before;
  ecat_dbg_sdo_error_tail_after =
    (uint32)ECAT_ctx.soem.elist.tail;
  ecat_dbg_sdo_error_new_count = 0U;
  ecat_dbg_sdo_error_present = 0U;
  ecat_dbg_sdo_error_matched = 0U;
  ecat_dbg_sdo_error_type = 0U;
  ecat_dbg_sdo_error_slave = 0U;
  ecat_dbg_sdo_error_index = 0U;
  ecat_dbg_sdo_error_subindex = 0U;
  ecat_dbg_sdo_error_code = 0U;
  __DMB();
  ecat_dbg_sdo_attempt_count++;

  start_us = BSP_SOEM_Timebase_GetUs();
  wkc = 0;
  if ((size > 0) && (size <= (int)sizeof(data)))
  {
    if (operation == ECAT_RUNTIME_SDO_JOB_READ_PENDING)
    {
      wkc = ECAT_RuntimeSdoRead(&ECAT_ctx.soem,
                                slave,
                                index,
                                subindex,
                                data,
                                &size);
    }
    else
    {
      wkc = ECAT_RuntimeSdoWrite(&ECAT_ctx.soem,
                                 slave,
                                 index,
                                 subindex,
                                 data,
                                 size);
    }
  }

  ecat_dbg_sdo_last_elapsed_us =
    (uint32)(BSP_SOEM_Timebase_GetUs() - start_us);
  ecat_dbg_sdo_last_size = (uint32)size;
  ecat_dbg_sdo_last_wkc = wkc;
  ecat_dbg_sdo_last_pdo_cycle_after =
    ECAT_ctx.can_feedback_cycle_count;
  error_head_after = ECAT_ctx.soem.elist.head;
  ecat_dbg_sdo_error_flag = (uint32)ECAT_ctx.soem.ecaterror;
  ecat_dbg_sdo_error_head_after = (uint32)error_head_after;
  ecat_dbg_sdo_error_tail_after =
    (uint32)ECAT_ctx.soem.elist.tail;

  if (wkc <= 0)
  {
    error_cursor = error_head_before;
    error_scan_count = 0U;
    while ((error_cursor != error_head_after) &&
           (error_scan_count <= EC_MAXELIST))
    {
      sdo_error = &ECAT_ctx.soem.elist.Error[error_cursor];
      ecat_dbg_sdo_error_present = 1U;
      ecat_dbg_sdo_error_new_count++;

      if ((ecat_dbg_sdo_error_matched == 0U) ||
          (((sdo_error->Etype == EC_ERR_TYPE_SDO_ERROR) ||
            (sdo_error->Etype == EC_ERR_TYPE_PACKET_ERROR)) &&
           (sdo_error->Slave == slave) &&
           (sdo_error->Index == index) &&
           (sdo_error->SubIdx == subindex)))
      {
        ecat_dbg_sdo_error_type = (uint32)sdo_error->Etype;
        ecat_dbg_sdo_error_slave = (uint32)sdo_error->Slave;
        ecat_dbg_sdo_error_index = (uint32)sdo_error->Index;
        ecat_dbg_sdo_error_subindex = (uint32)sdo_error->SubIdx;
        if (sdo_error->Etype == EC_ERR_TYPE_SDO_ERROR)
        {
          ecat_dbg_sdo_error_code = (uint32)sdo_error->AbortCode;
        }
        else
        {
          ecat_dbg_sdo_error_code = (uint32)sdo_error->ErrorCode;
        }
      }

      if (((sdo_error->Etype == EC_ERR_TYPE_SDO_ERROR) ||
           (sdo_error->Etype == EC_ERR_TYPE_PACKET_ERROR)) &&
          (sdo_error->Slave == slave) &&
          (sdo_error->Index == index) &&
          (sdo_error->SubIdx == subindex))
      {
        ecat_dbg_sdo_error_matched = 1U;
      }

      error_cursor++;
      if (error_cursor > EC_MAXELIST)
      {
        error_cursor = 0;
      }
      error_scan_count++;
    }
  }
  __DMB();
  ecat_dbg_sdo_complete_count++;

  primask = __get_PRIMASK();
  __disable_irq();
  if ((operation == ECAT_RUNTIME_SDO_JOB_READ_PENDING) && (wkc > 0))
  {
    memcpy(job->data, data, sizeof(job->data));
  }
  job->size = size;
  job->wkc = wkc;
  __DMB();
  job->state = ECAT_RUNTIME_SDO_JOB_DONE;
  __DMB();
  ECAT_ctx.sdo_worker_busy = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/**
 * @brief  初始化 SOEM 主站上下文并打开 STM32 以太网接口
 *
 * @return
 * 无
 *
 * @warning
 * 仅应在 ECAT_STATE_SOEM_INIT 状态且 PHY 链路已建立时调用；该函数会清空
 * SOEM 上下文和 IOmap，初始化失败时主站将进入 ECAT_STATE_ERROR 状态
 */
static void ECAT_StepSoemInit(void)
{
  int init_ret;

  memset(&ECAT_ctx.soem, 0, sizeof(ECAT_ctx.soem));
  memset(ECAT_ctx.iomap, 0, sizeof(ECAT_ctx.iomap));

  init_ret = ecx_init(&ECAT_ctx.soem, "stm32_eth");
  ECAT_Printf("[SOEM] ecx_init ret=%d %s\r\n",
                       init_ret,
                       (init_ret > 0) ? "PASS" : "FAIL");

  if (init_ret <= 0)
  {
    ECAT_EnterError("ecx_init failed");
    return;
  }

  ECAT_ctx.context_open = 1U;
  ECAT_Transition(ECAT_STATE_EVENT_SOEM_INIT_OK);
}

/**
 * @brief  通过 BRD 帧探测预期数量的 EtherCAT 从站
 *
 * @return
 * 无
 *
 * @warning
 * 仅应在 ECAT_STATE_PROBE 状态且 SOEM 上下文已打开时调用；每次调用只执行一次
 * 探测，失败后由后续 ECAT_Process() 周期继续，累计 ECAT_PROBE_COUNT 次均失败时
 * 主站进入 ECAT_STATE_ERROR 状态
 */
static void ECAT_StepProbe(void)
{
  int probe_wkc;

  ECAT_ctx.probe_attempt_count++;
  probe_wkc = ECAT_RunProbe(ECAT_ctx.probe_attempt_count);

  if (probe_wkc == ECAT_EXPECTED_SLAVES)
  {
    ECAT_PrintSectionLine();
    ECAT_Printf("[SOEM] BRD WKC=%d expected=%d PASS, start config\r\n",
                         probe_wkc,
                         ECAT_EXPECTED_SLAVES);
    ECAT_Transition(ECAT_STATE_EVENT_PROBE_OK);
    return;
  }

  if (ECAT_ctx.probe_attempt_count < ECAT_PROBE_COUNT)
  {
    return;
  }

  ECAT_Printf("[SOEM] BRD probe failed after %u attempts WKC=%d expected=%d\r\n",
                       (unsigned int)ECAT_ctx.probe_attempt_count,
                       probe_wkc,
                       ECAT_EXPECTED_SLAVES);
  ECAT_EnterError("BRD probe failed");
}

/**
 * @brief  记录错误原因、释放 SOEM 资源并将主站切换至错误状态
 *
 * @param[in] reason   以空字符结尾的错误原因字符串
 *
 * @return
 * 无
 *
 * @warning
 * reason 必须指向有效字符串；该函数会关闭已打开的 SOEM 上下文，并复位运行时、
 * PDO 配置结果及诊断状态，进入错误状态后需重新调用 ECAT_Init() 才能恢复流程
 */
static void ECAT_EnterError(const char *reason)
{
  uint32 primask;

  if (ECAT_ctx.state == ECAT_STATE_ERROR)
  {
    return;
  }

  ECAT_Printf("[SOEM] ERROR: %s\r\n", reason);
  primask = __get_PRIMASK();
  __disable_irq();
  ECAT_ctx.cyclic_mbx_service_ready = 0U;
  ECAT_ctx.close_pending = 1U;
  if (primask == 0U)
  {
    __enable_irq();
  }
  ECAT_CommitState(ECAT_STATE_ERROR,
                            ECAT_STATE_EVENT_FATAL_ERROR);
  ECAT_CloseContextWhenWorkerIdle();
}

static void ECAT_CloseContextWhenWorkerIdle(void)
{
  uint32 primask;
  uint8 close_context = 0U;
  uint8 cleanup = 0U;

  primask = __get_PRIMASK();
  __disable_irq();
  if ((ECAT_ctx.close_pending != 0U) &&
      (ECAT_ctx.sdo_worker_busy == 0U))
  {
    cleanup = 1U;
    if (ECAT_ctx.context_open != 0U)
    {
      ECAT_ctx.context_open = 0U;
      close_context = 1U;
    }
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (cleanup == 0U)
  {
    return;
  }

  if (close_context != 0U)
  {
    ecx_close(&ECAT_ctx.soem);
  }
  ECAT_RuntimeResetAfterError(&ECAT_ctx.runtime);
  ECAT_PdoConfigResetPo2SoResult(&ECAT_ctx.pdo_config);
  ECAT_DiagReset(&ECAT_ctx.diag);

  primask = __get_PRIMASK();
  __disable_irq();
  ECAT_ctx.close_pending = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/**
 * @brief  发送一次 EtherCAT BRD 探测帧并等待匹配响应
 *
 * @param[in] probe_index   本次探测序号，从1开始，仅用于日志记录
 *
 * @return
 * 收到匹配响应帧返回WKC，等待超时返回EC_NOFRAME
 *
 * @warning
 * 仅应在 ECAT_STATE_PROBE 状态且 SOEM 上下文已打开时调用；该函数会占用端口
 * 帧缓冲并阻塞等待最多 ECAT_WAIT_TIMEOUT_US，禁止并发调用
 */
static int ECAT_RunProbe(uint32 probe_index)
{
  uint16 read_data = 0U;
  uint8 idx;
  int tx_ret;
  int wait_ret;
  uint64 start_us;
  uint64 elapsed_us;

  ECAT_Printf("[SOEM] probe %lu/%u\r\n",
                       (unsigned long)probe_index,
                       (unsigned int)ECAT_PROBE_COUNT);

  idx = ECAT_PrepareProbeFrame(&read_data);
  ECAT_PrintProbeFrame(idx);

  tx_ret = ecx_outframe(&ECAT_ctx.soem.port, idx);
  ECAT_Printf("[SOEM] ecx_outframe ret=%d %s\r\n",
                       tx_ret,
                       (tx_ret > 0) ? "PASS" : "FAIL");

  start_us = BSP_SOEM_Timebase_GetUs();
  wait_ret = ecx_waitinframe(&ECAT_ctx.soem.port,
                             idx,
                             ECAT_WAIT_TIMEOUT_US);
  elapsed_us = BSP_SOEM_Timebase_GetUs() - start_us;

  ECAT_Printf("[SOEM] wait ret=%d elapsed_us=%llu\r\n",
                       wait_ret,
                       (unsigned long long)elapsed_us);
  //ECAT_DiagPrintRx(&ECAT_ctx.log);
  //ECAT_DiagPrintEth(&ECAT_ctx.log);
  ECAT_PrintProbeResult(wait_ret);

  ecx_setbufstat(&ECAT_ctx.soem.port, idx, EC_BUF_EMPTY);
  return wait_ret;
}

/**
 * @brief  分配帧缓冲并构造读取从站类型寄存器的 BRD 探测帧
 *
 * @param[in] read_data   16位读取数据占位地址，用于确定 BRD 数据长度
 *
 * @return
 * 返回构造完成的 SOEM 帧缓冲索引
 *
 * @warning
 * read_data 必须指向有效对象，且 SOEM 上下文必须已打开；调用方使用完返回索引后
 * 必须通过 ecx_setbufstat() 将对应帧缓冲释放为 EC_BUF_EMPTY
 */
static uint8 ECAT_PrepareProbeFrame(uint16 *read_data)
{
  uint8 idx;

  idx = ecx_getindex(&ECAT_ctx.soem.port);
  ecx_setupdatagram(&ECAT_ctx.soem.port,
                    ECAT_ctx.soem.port.txbuf[idx],
                    EC_CMD_BRD,
                    idx,
                    0x0000U,
                    ECT_REG_TYPE,
                    sizeof(*read_data),
                    read_data);
  return idx;
}

/**
 * @brief  输出 EtherCAT BRD 探测帧的头部诊断信息
 *
 * @param[in] idx   SOEM 发送帧缓冲索引
 *
 * @return
 * 无
 *
 * @warning
 * idx 必须是 ECAT_PrepareProbeFrame() 返回的有效索引，且对应发送帧已构造完成；
 * 该函数不检查索引范围或帧长度，非法值会导致越界访问
 */
static void ECAT_PrintProbeFrame(uint8 idx)
{
  uint16 tx_ethertype;
  uint8 tx_cmd;
  uint8 tx_idx;

  tx_ethertype = ECAT_ReadTxEtherType(idx);
  tx_cmd =
    ECAT_ctx.soem.port.txbuf[idx][ETH_HEADERSIZE + EC_CMDOFFSET];
  tx_idx =
    ECAT_ctx.soem.port.txbuf[idx][ETH_HEADERSIZE + EC_CMDOFFSET + 1U];

  ECAT_Printf("[SOEM] idx=%u tx_len=%d cmd=0x%02X tx_idx=%u\r\n",
                       idx,
                       ECAT_ctx.soem.port.txbuflength[idx],
                       tx_cmd,
                       tx_idx);
  ECAT_Printf("[SOEM] ethertype=0x%04X %s\r\n",
                       tx_ethertype,
                       (tx_ethertype == ETH_P_ECAT) ? "PASS" : "FAIL");
  ECAT_Printf("[SOEM] tx dst=%02X:%02X:%02X:%02X:%02X:%02X\r\n",
                       (unsigned int)ECAT_ctx.soem.port.txbuf[idx][0],
                       (unsigned int)ECAT_ctx.soem.port.txbuf[idx][1],
                       (unsigned int)ECAT_ctx.soem.port.txbuf[idx][2],
                       (unsigned int)ECAT_ctx.soem.port.txbuf[idx][3],
                       (unsigned int)ECAT_ctx.soem.port.txbuf[idx][4],
                       (unsigned int)ECAT_ctx.soem.port.txbuf[idx][5]);
  ECAT_Printf("[SOEM] tx src=%02X:%02X:%02X:%02X:%02X:%02X\r\n",
                       (unsigned int)ECAT_ctx.soem.port.txbuf[idx][6],
                       (unsigned int)ECAT_ctx.soem.port.txbuf[idx][7],
                       (unsigned int)ECAT_ctx.soem.port.txbuf[idx][8],
                       (unsigned int)ECAT_ctx.soem.port.txbuf[idx][9],
                       (unsigned int)ECAT_ctx.soem.port.txbuf[idx][10],
                       (unsigned int)ECAT_ctx.soem.port.txbuf[idx][11]);
}

/**
 * @brief  根据探测等待结果输出 EtherCAT 响应诊断信息
 *
 * @param[in] wait_ret   ecx_waitinframe() 返回的WKC或错误码
 *
 * @return
 * 无
 *
 * @warning
 * 该函数仅区分无响应、非正值和正WKC，不校验WKC是否等于预期从站数量；
 * 探测是否成功必须由调用方另行判断
 */
static void ECAT_PrintProbeResult(int wait_ret)
{
  if (wait_ret == EC_NOFRAME)
  {
    ECAT_Printf("[SOEM] timeout noframe\r\n");
  }
  else if (wait_ret <= 0)
  {
    ECAT_Printf("[SOEM] non_positive WKC=%d\r\n", wait_ret);
  }
  else
  {
    ECAT_Printf("[SOEM] WKC=%d PASS\r\n", wait_ret);
  }
}

/**
 * @brief  发现并校验 EtherCAT 从站并注册 PRE-OP 至 SAFE-OP 配置回调
 *
 * @return
 * 无
 *
 * @warning
 * 仅应在 ECAT_STATE_DISCOVER 状态且 SOEM 上下文已打开时调用；ecx_config_init()
 * 会重建从站列表，发现数量不等于 ECAT_SLAVE_COUNT 时主站将进入错误状态
 */
static void ECAT_StepDiscover(void)
{
  int slave_count;
  uint16 slave;

  slave_count = ecx_config_init(&ECAT_ctx.soem);
  ECAT_Printf("[SOEM] ecx_config_init ret=%d slavecount=%d\r\n",
                       slave_count,
                       ECAT_ctx.soem.slavecount);

  if (slave_count != ECAT_EXPECTED_SLAVES)
  {
    ECAT_Printf("[SOEM] slave count=%d expected=%d\r\n",
                         slave_count,
                         ECAT_EXPECTED_SLAVES);
    ECAT_EnterError("slave discovery mismatch");
    return;
  }

  /* ecx_config_init() clears slavelist, so register the callback only now. */
  ECAT_PdoConfigRegisterPo2So(
    &ECAT_ctx.soem,
    &ECAT_ctx.log,
    ECAT_PO2SOconfig);

  for (slave = ECAT_FIRST_SLAVE; slave <= ECAT_LAST_SLAVE; slave++)
  {
    ECAT_Printf("[SOEM] slave%u state=0x%04X configadr=0x%04X\r\n",
                (unsigned int)slave,
                ECAT_ctx.soem.slavelist[slave].state,
                ECAT_ctx.soem.slavelist[slave].configadr);
    ECAT_Printf("[SOEM] slave%u vendor=0x%08lX product=0x%08lX\r\n",
                (unsigned int)slave,
                (unsigned long)ECAT_ctx.soem.slavelist[slave].eep_man,
                (unsigned long)ECAT_ctx.soem.slavelist[slave].eep_id);
    ECAT_Printf("[SOEM] slave%u name=%s\r\n",
                (unsigned int)slave,
                ECAT_ctx.soem.slavelist[slave].name);
  }

  ECAT_Transition(ECAT_STATE_EVENT_DISCOVER_OK);
}

/**
 * @brief  读取全部从站状态并广播请求切换至 PRE-OP
 *
 * @return
 * 无
 *
 * @warning
 * 仅应在 ECAT_STATE_PREOP 状态、从站发现完成且 SOEM 上下文已打开时调用；
 * 该函数会阻塞等待最多 EC_TIMEOUTSTATE，无从站或任一从站未进入 PRE-OP 时
 * 主站将进入 ECAT_STATE_ERROR 状态
 */
static void ECAT_ReadStateAndRequestPreOp(void)
{
  int write_state_ret;
  uint16 check_state_ret;

  if (ECAT_ctx.soem.slavecount <= 0)
  {
    ECAT_Printf("[SOEM] PRE-OP skip: no slave\r\n");
    ECAT_EnterError("PRE-OP has no slave");
    return;
  }

  ECAT_PrintSectionLine();
  ECAT_Printf("[SOEM] state before PRE-OP request\r\n");
  ECAT_RuntimeReadAllStates(&ECAT_ctx.soem,
                                      &ECAT_ctx.log);

  ECAT_ctx.soem.slavelist[0].state = EC_STATE_PRE_OP;
  write_state_ret = ecx_writestate(&ECAT_ctx.soem, 0);
  ECAT_Printf("[SOEM] request PRE-OP write_ret=%d\r\n",
                       write_state_ret);

  check_state_ret = ecx_statecheck(&ECAT_ctx.soem,
                                   0,
                                   EC_STATE_PRE_OP,
                                   EC_TIMEOUTSTATE);
  ECAT_Printf("[SOEM] statecheck PRE-OP ret=0x%04X %s\r\n",
                       check_state_ret,
                       (check_state_ret == EC_STATE_PRE_OP) ? "PASS" : "FAIL");

  ECAT_Printf("[SOEM] state after PRE-OP request\r\n");
  ECAT_RuntimeReadAllStates(&ECAT_ctx.soem,
                                      &ECAT_ctx.log);

  if (ECAT_RuntimeCheckPreOp(&ECAT_ctx.soem,
                                      &ECAT_ctx.log) != 0U)
  {
    ECAT_Transition(ECAT_STATE_EVENT_PREOP_OK);
  }
  else
  {
    ECAT_EnterError("PRE-OP failed");
  }
}

/**
 * @brief  读取并输出全部受管从站的邮箱及基础 CoE SDO 信息
 *
 * @return
 * 无
 *
 * @warning
 * 仅应在 ECAT_STATE_SDO 状态、受管从站均处于 PRE-OP 且支持 CoE 时调用；
 * 从站缺失或不支持 CoE 会使主站进入错误状态，但单项 SDO 读取失败仅记录日志，
 * 不会阻止状态机继续推进
 */
static void ECAT_ReadBasicSdoInfo(void)
{
  uint16 slave;

  if (ECAT_ctx.soem.slavecount < (int)ECAT_LAST_SLAVE)
  {
    ECAT_Printf("[SOEM] basic SDO skip: slavecount=%d need=%u\r\n",
                ECAT_ctx.soem.slavecount,
                (unsigned int)ECAT_LAST_SLAVE);
    ECAT_EnterError("basic SDO slave missing");
    return;
  }

  for (slave = ECAT_FIRST_SLAVE; slave <= ECAT_LAST_SLAVE; slave++)
  {
    if ((ECAT_ctx.soem.slavelist[slave].mbx_proto &
         ECT_MBXPROT_COE) == 0U)
    {
      ECAT_Printf("[SOEM] slave%u CoE support FAIL\r\n",
                  (unsigned int)slave);
      ECAT_EnterError("managed slave has no CoE");
      return;
    }
  }

  for (slave = ECAT_FIRST_SLAVE; slave <= ECAT_LAST_SLAVE; slave++)
  {
    ECAT_PrintSectionLine();
    ECAT_Printf("[SOEM] basic SDO start slave%u\r\n",
                (unsigned int)slave);
    ECAT_DiagPrintMailbox(&ECAT_ctx.log,
                          slave,
                          &ECAT_ctx.soem.slavelist[slave]);
    ECAT_Printf("[SOEM] slave%u CoE support PASS\r\n",
                (unsigned int)slave);
    ECAT_ReadDeviceTypeSdo(slave);
    ECAT_ReadIdentitySdo(slave);
  }

  ECAT_Transition(ECAT_STATE_EVENT_SDO_DONE);
}

/**
 * @brief  读取并输出指定从站的 CiA 301 设备类型
 *
 * @param[in] slave   要读取的从站索引
 *
 * @return
 * 无
 *
 * @warning
 * 仅应在 ECAT_STATE_SDO 状态下对已发现、处于 PRE-OP 且支持 CoE 的从站调用；
 * 该函数会按 EC_TIMEOUTRXM 同步读取 0x1000:00，读取失败仅记录日志且不向调用方返回
 */
static void ECAT_ReadDeviceTypeSdo(uint16 slave)
{
  uint32 device_type = 0U;

  if (ECAT_SdoReadU32(&ECAT_ctx.soem,
                                &ECAT_ctx.log,
                                slave,
                                0x1000U,
                                0x00U,
                                &device_type) != 0U)
  {
    ECAT_Printf("[SOEM] slave%u SDO 0x1000:00 device_type=0x%08lX\r\n",
                         (unsigned int)slave,
                         (unsigned long)device_type);
  }
}

/**
 * @brief  读取并输出指定从站的 CiA 301 Identity 对象信息
 *
 * @param[in] slave   要读取的从站索引
 *
 * @return
 * 无
 *
 * @warning
 * 仅应在 ECAT_STATE_SDO 状态下对已发现、处于 PRE-OP 且支持 CoE 的从站调用；
 * 该函数会分别同步读取 0x1018:00 至 0x1018:04，不依据条目数跳过后续子索引，
 * 单项读取失败仅记录日志且不会终止后续读取
 */
static void ECAT_ReadIdentitySdo(uint16 slave)
{
  uint8 identity_entries = 0U;
  uint32 vendor_id = 0U;
  uint32 product_code = 0U;
  uint32 revision = 0U;
  uint32 serial = 0U;

  if (ECAT_SdoReadU8(&ECAT_ctx.soem,
                               &ECAT_ctx.log,
                               slave,
                               0x1018U,
                               0x00U,
                               &identity_entries) != 0U)
  {
    ECAT_Printf("[SOEM] slave%u SDO 0x1018:00 entries=%u\r\n",
                         (unsigned int)slave,
                         (unsigned int)identity_entries);
  }

  if (ECAT_SdoReadU32(&ECAT_ctx.soem,
                                &ECAT_ctx.log,
                                slave,
                                0x1018U,
                                0x01U,
                                &vendor_id) != 0U)
  {
    ECAT_Printf(
      "[SOEM] slave%u SDO 0x1018:01 vendor=0x%08lX eep=0x%08lX\r\n",
      (unsigned int)slave,
      (unsigned long)vendor_id,
      (unsigned long)ECAT_ctx.soem.slavelist[slave].eep_man);
  }

  if (ECAT_SdoReadU32(&ECAT_ctx.soem,
                                &ECAT_ctx.log,
                                slave,
                                0x1018U,
                                0x02U,
                                &product_code) != 0U)
  {
    ECAT_Printf(
      "[SOEM] slave%u SDO 0x1018:02 product=0x%08lX eep=0x%08lX\r\n",
      (unsigned int)slave,
      (unsigned long)product_code,
      (unsigned long)ECAT_ctx.soem.slavelist[slave].eep_id);
  }

  if (ECAT_SdoReadU32(&ECAT_ctx.soem,
                                &ECAT_ctx.log,
                                slave,
                                0x1018U,
                                0x03U,
                                &revision) != 0U)
  {
    ECAT_Printf(
      "[SOEM] slave%u SDO 0x1018:03 revision=0x%08lX eep=0x%08lX\r\n",
      (unsigned int)slave,
      (unsigned long)revision,
      (unsigned long)ECAT_ctx.soem.slavelist[slave].eep_rev);
  }

  if (ECAT_SdoReadU32(&ECAT_ctx.soem,
                                &ECAT_ctx.log,
                                slave,
                                0x1018U,
                                0x04U,
                                &serial) != 0U)
  {
    ECAT_Printf("[SOEM] slave%u SDO 0x1018:04 serial=0x%08lX\r\n",
                         (unsigned int)slave,
                         (unsigned long)serial);
  }
}

/**
 * @brief  读取并输出显式配置前的受管从站 PDO 分配快照
 *
 * @return
 * 无
 *
 * @warning
 * 仅应在 ECAT_STATE_PDO_ASSIGN 状态、受管从站均处于 PRE-OP 且支持 CoE 时调用；
 * 从站缺失或不支持 CoE 会使主站进入错误状态，但单个 PDO 映射读取失败仅记录日志，
 * 仍会继续推进至 ECAT_STATE_MAP 状态
 */
static void ECAT_StepPdoAssignment(void)
{
  ECAT_PdoConfigResult result;

  result = ECAT_PdoConfigReadAssignment(
    &ECAT_ctx.soem,
    &ECAT_ctx.log,
    &ECAT_ctx.pdo_config);
  if (result == ECAT_PDO_CONFIG_RESULT_OK)
  {
    ECAT_Transition(ECAT_STATE_EVENT_PDO_ASSIGN_OK);
    return;
  }

  ECAT_EnterError(ECAT_PdoConfigResultReason(result));
}

/**
 * @brief  生成并校验受管从站的 PDO IOmap，随后推进主站状态机
 *
 * @return
 * 无
 *
 * @warning
 * 仅应在 ECAT_STATE_MAP 状态且从站发现、PRE-OP 和 PDO 分配读取均已完成后调用；
 * 本函数会清空并重建 ECAT_ctx.iomap，映射校验成功时进入 ECAT_STATE_SAFEOP，
 * 失败时会关闭 SOEM 上下文并进入错误状态，需重新调用 ECAT_Init() 才能恢复
 */
static void ECAT_StepPdoConfig(void)
{
  ECAT_PdoConfigResult result;

  result = ECAT_PdoConfigMapGroup(
    &ECAT_ctx.soem,
    &ECAT_ctx.log,
    &ECAT_ctx.pdo_config,
    ECAT_ctx.iomap,
    sizeof(ECAT_ctx.iomap));
  if (result == ECAT_PDO_CONFIG_RESULT_OK)
  {
    result = ECAT_PdoConfigEnableCyclicMailbox(&ECAT_ctx.soem,
                                               &ECAT_ctx.log);
  }
  if (result == ECAT_PDO_CONFIG_RESULT_OK)
  {
    ECAT_ctx.cyclic_mbx_service_ready = 1U;
    ECAT_Transition(ECAT_STATE_EVENT_MAP_OK);
    return;
  }

  ECAT_EnterError(ECAT_PdoConfigResultReason(result));
}

/**
 * @brief  请求从站进入 SAFE-OP，完成首次 PDO 交换并推进主站状态机
 *
 * @return
 * 无
 *
 * @warning
 * 仅应在 ECAT_STATE_SAFEOP 状态、PDO IOmap 已通过校验且全部从站仍处于 PRE-OP 时
 * 调用；本函数会阻塞等待状态切换和过程数据接收，并根据 TxPDO 准备安全 RxPDO。
 * 成功时进入 ECAT_STATE_OP_REQUEST，失败时关闭 SOEM 上下文并进入错误状态，
 * 需重新调用 ECAT_Init() 才能恢复
 */
static void ECAT_StepSafeOp(void)
{
  ECAT_RuntimeResult result;

  result = ECAT_RuntimeEnterSafeOp(&ECAT_ctx.soem,
                                   &ECAT_ctx.log,
                                   &ECAT_ctx.runtime);
  if (result == ECAT_RUNTIME_RESULT_OK)
  {
    ECAT_Transition(ECAT_STATE_EVENT_SAFEOP_OK);
    return;
  }

  ECAT_EnterError(ECAT_RuntimeResultReason(result));
}

/**
 * @brief  预发送安全输出并请求从站稳定进入 OPERATIONAL
 *
 * @return
 * 无
 *
 * @warning
 * 仅应在 ECAT_STATE_OP_REQUEST 状态、全部受管从站已进入 SAFE-OP 且安全 RxPDO 与
 * expected_wkc 已准备完成后调用；请求期间双轴控制字保持 0x0000，函数会阻塞执行
 * 多个过程数据周期。成功时进入 ECAT_STATE_OPERATIONAL，失败时先尝试回退 SAFE-OP，
 * 随后关闭 SOEM 上下文并进入错误状态，需重新调用 ECAT_Init() 才能恢复
 */
static void ECAT_StepOpRequest(void)
{
  ECAT_RuntimeResult result;

  result = ECAT_RuntimeRequestOp(&ECAT_ctx.soem,
                                 &ECAT_ctx.log,
                                 &ECAT_ctx.runtime);
  if (result == ECAT_RUNTIME_RESULT_OK)
  {
    ECAT_Transition(ECAT_STATE_EVENT_OP_STABLE);
    return;
  }

  ECAT_EnterError(ECAT_RuntimeResultReason(result));
}

/**
 * @brief  执行一个 OPERATIONAL PDO 周期并处理运行时监控状态机
 *
 * @return
 * 无
 *
 * @warning
 * 仅应在 ECAT_STATE_OPERATIONAL 状态、SOEM 上下文保持打开且 PDO 映射与 expected_wkc
 * 有效时周期调用，本函数自身不控制调用周期。成功交换会推进 CW06 流程，可能将全部
 * 受管轴控制字写为 0x0006，并周期性发布 TxPDO 快照；连续通信异常达到阈值或安全
 * 回退失败时会关闭 SOEM 上下文并进入错误状态，需重新调用 ECAT_Init() 才能恢复
 */
static void ECAT_StepOperational(void)
{
  ECAT_RuntimeResult result;

  result = ECAT_RuntimeExchangeOperational(
    &ECAT_ctx.soem,
    &ECAT_ctx.log,
    &ECAT_ctx.runtime);
  if (result != ECAT_RUNTIME_RESULT_OK)
  {
    ECAT_EnterError(ECAT_RuntimeResultReason(result));
    return;
  }

  /* A tolerated bad-WKC cycle has no valid current mailbox status. */
  if (ECAT_ctx.runtime.bad_wkc_continuous != 0U)
  {
    return;
  }

  /* 成功PDO周期完成后发布递增计数，供CAN反馈标识数据新鲜度。 */
  ECAT_ctx.can_feedback_cycle_count++;

  if (ECAT_ServiceCyclicMailbox() == 0U)
  {
    ECAT_EnterError("cyclic mailbox service invariant failed");
  }
}

static uint8 ECAT_ServiceCyclicMailbox(void)
{
  uint16 slave;

  if ((ECAT_ctx.context_open == 0U) ||
      (ECAT_ctx.cyclic_mbx_service_ready == 0U) ||
      (ECAT_ctx.soem.slavecount < ECAT_LAST_SLAVE))
  {
    return 0U;
  }

  for (slave = ECAT_FIRST_SLAVE; slave <= ECAT_LAST_SLAVE; slave++)
  {
    if ((ECAT_ctx.soem.slavelist[slave].mbxhandlerstate !=
         ECT_MBXH_CYCLIC) ||
        (ECAT_ctx.soem.slavelist[slave].mbxstatus == 0) ||
        (ECAT_ctx.soem.slavelist[slave].coembxin == 0))
    {
      return 0U;
    }
  }

  (void)ecx_mbxhandler(&ECAT_ctx.soem,
                       ECAT_MBX_SERVICE_GROUP,
                       ECAT_MBX_SERVICE_LIMIT);
  return 1U;
}

/**
 * @brief  在 PRE-OP 至 SAFE-OP 切换期间校验或重写从站 PDO 映射
 *
 * @param[in,out] context   SOEM 主站上下文
 * @param[in]     slave    要配置的从站索引
 *
 * @return
 * 配置及回读校验成功返回1，参数、状态或 PDO 配置异常返回0
 *
 * @warning
 * 仅可作为 ECAT_PdoConfigRegisterPo2So() 注册的回调使用；context 必须是当前主站
 * 上下文，slave 必须处于受管范围、无错误的 PRE-OP 状态并支持 CoE；SOEM 调用点
 * 不处理该返回值，配置失败时 PDO 分配可能保持禁用
 */
static int ECAT_PO2SOconfig(ecx_contextt *context, uint16 slave)
{
  return ECAT_PdoConfigConfigurePo2So(
    context,
    &ECAT_ctx.soem,
    &ECAT_ctx.log,
    &ECAT_ctx.pdo_config,
    slave);
}

static uint16 ECAT_ReadTxEtherType(uint8 idx)
{
  return (uint16)(((uint16)ECAT_ctx.soem.port.txbuf[idx][12] << 8) |
                  ECAT_ctx.soem.port.txbuf[idx][13]);
}
