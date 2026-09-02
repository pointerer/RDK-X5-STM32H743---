#include "ECAT_pdo_config.h"

#include "EtherCAT/app_ethercat_pdo.h"

#include <string.h>

#define ECAT_PDO_CONFIG_GROUP 0U
#define ECAT_CYCLIC_MBX_MIN_LENGTH 16U

typedef struct
{
  uint8 po2so_ok;
  uint8 pdo_cache_ok;
  uint8 outputs_ok;
  uint8 inputs_ok;
  uint8 obytes_ok;
  uint8 ibytes_ok;
} ECAT_PdoConfigSlaveStatus;

typedef struct
{
  int iomap_size;
  uint8 iomap_ok;
  ECAT_PdoConfigSlaveStatus slave[ECAT_SLAVE_COUNT];
} ECAT_PdoConfigMapStatus;

static void ECAT_PdoConfigRunMap(
  ecx_contextt *context,
  ECAT_LogContext *log,
  ECAT_PdoConfigContext *pdo_config,
  uint8 *iomap,
  ECAT_PdoConfigMapStatus *status);
static void ECAT_PdoConfigPrintLayout(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint32 iomap_capacity,
  const ECAT_PdoConfigMapStatus *status);
static void ECAT_PdoConfigEvaluateMap(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint32 iomap_capacity,
  ECAT_PdoConfigMapStatus *status);
static void ECAT_PdoConfigPrintResolved(
  ecx_contextt *context,
  ECAT_LogContext *log,
  ECAT_PdoConfigContext *pdo_config,
  uint8 *iomap,
  uint32 iomap_capacity,
  const ECAT_PdoConfigMapStatus *status);
static uint8 ECAT_PdoConfigMapPassed(
  const ECAT_PdoConfigMapStatus *status);
static uint8 ECAT_PdoConfigCheckCyclicMailboxPrerequisites(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint8 *iomap,
  const ECAT_PdoConfigMapStatus *status);

void ECAT_PdoConfigReset(ECAT_PdoConfigContext *pdo_config)
{
  uint32 slave_index;

  for (slave_index = 0U; slave_index < ECAT_SLAVE_COUNT; slave_index++)
  {
    ECAT_PdoMapReset(&pdo_config->pdo_map[slave_index]);
  }
}

/**
 * @brief  清除所有受管从站的 PO2SO 回调执行结果
 *
 * @param[in,out] pdo_config PDO 配置上下文
 *
 * @return
 * 无
 *
 * @warning
 * pdo_config 必须有效；本函数仅清零各从站的 po2so_attempted 和 po2so_ok 标志，
 * 不会清除 RxPDO、TxPDO 映射缓存，且不得与 PO2SO 回调或映射流程并发调用
 */
void ECAT_PdoConfigResetPo2SoResult(
  ECAT_PdoConfigContext *pdo_config)
{
  uint32 slave_index;

  for (slave_index = 0U; slave_index < ECAT_SLAVE_COUNT; slave_index++)
  {
    ECAT_PdoMapResetPo2SoResult(&pdo_config->pdo_map[slave_index]);
  }
}

/**
 * @brief  为工程配置范围内的 EtherCAT 从站注册 PRE-OP 至 SAFE-OP 配置回调
 *
 * @param[in,out] context        SOEM 主站上下文，函数会更新从站回调字段
 * @param[in,out] log            EtherCAT 日志上下文
 * @param[in]     po2so_config   PRE-OP 至 SAFE-OP 状态切换配置回调
 *
 * @return
 * 无
 *
 * @warning
 * 必须在 ecx_config_init() 完成后调用，且 log 必须有效；该函数会覆盖目标从站
 * 已有的 PO2SOconfig，参数无效或从站数量不足时仅记录日志并直接返回
 */
void ECAT_PdoConfigRegisterPo2So(
  ecx_contextt *context,
  ECAT_LogContext *log,
  int (*po2so_config)(ecx_contextt *context, uint16 slave))
{
  uint16 slave;

  if ((context == 0) || (po2so_config == 0) ||
      (context->slavecount < (int)ECAT_LAST_SLAVE))
  {
    ECAT_LogPrintf(
      log,
      "[SOEM] PO2SO callback register FAIL slavecount=%d need=%u\r\n",
      (context != 0) ? context->slavecount : 0,
      (unsigned int)ECAT_LAST_SLAVE);
    return;
  }

  for (slave = ECAT_FIRST_SLAVE; slave <= ECAT_LAST_SLAVE; slave++)
  {
    context->slavelist[slave].PO2SOconfig = po2so_config;
    ECAT_LogPrintf(
      log,
      "[SOEM] explicit PO2SO PDO callback registered slave%u\r\n",
      (unsigned int)slave);
  }
}

/**
 * @brief  读取并缓存显式 PO2SO 配置前的受管从站 PDO 分配
 *
 * @param[in,out] context      SOEM 主站上下文
 * @param[in,out] log          EtherCAT 日志上下文
 * @param[in,out] pdo_config   PDO 配置上下文，用于保存各从站映射缓存
 *
 * @return
 * 前置检查通过返回ECAT_PDO_CONFIG_RESULT_OK；从站不足或缺少CoE支持时返回对应错误码
 *
 * @warning
 * context、log 和 pdo_config 必须有效，受管从站应处于可进行 CoE 邮箱通信的状态；
 * 该函数会同步读取各从站，但忽略单次 ECAT_PdoMapLoad() 的失败，因此返回OK不表示
 * 所有 PDO 映射均已成功读取
 */
ECAT_PdoConfigResult ECAT_PdoConfigReadAssignment(
  ecx_contextt *context,
  ECAT_LogContext *log,
  ECAT_PdoConfigContext *pdo_config)
{
  uint32 slave_index;
  uint16 slave;

  if (context->slavecount < (int)ECAT_LAST_SLAVE)
  {
    ECAT_LogPrintf(
      log,
      "[SOEM] PDO assign skip: slavecount=%d need=%u\r\n",
      context->slavecount,
      (unsigned int)ECAT_LAST_SLAVE);
    return ECAT_PDO_CONFIG_RESULT_ASSIGN_SLAVE_MISSING;
  }

  for (slave = ECAT_FIRST_SLAVE; slave <= ECAT_LAST_SLAVE; slave++)
  {
    if ((context->slavelist[slave].mbx_proto & ECT_MBXPROT_COE) == 0U)
    {
      ECAT_LogPrintf(
        log,
        "[SOEM] PDO assign skip: slave%u no CoE\r\n",
        (unsigned int)slave);
      return ECAT_PDO_CONFIG_RESULT_COE_REQUIRED;
    }
  }

  ECAT_LogSectionLine(log);
  for (slave_index = 0U; slave_index < ECAT_SLAVE_COUNT; slave_index++)
  {
    slave = (uint16)(ECAT_FIRST_SLAVE + slave_index);
    ECAT_LogPrintf(
      log,
      "[SOEM] PDO assignment before explicit PO2SO config slave%u\r\n",
      (unsigned int)slave);
    (void)ECAT_PdoMapLoad(context,
                         log,
                         &pdo_config->pdo_map[slave_index],
                         slave,
                         "before PO2SO");
  }

  return ECAT_PDO_CONFIG_RESULT_OK;
}

ECAT_PdoConfigResult ECAT_PdoConfigEnableCyclicMailbox(
  ecx_contextt *context,
  ECAT_LogContext *log)
{
  ec_slavet *slave_info;
  uint16 slave;
  uint16 reset_slave;
  int enable_ret;

  if ((context == 0) || (log == 0) ||
      (context->slavecount < (int)ECAT_LAST_SLAVE))
  {
    return ECAT_PDO_CONFIG_RESULT_CYCLIC_MBX_ENABLE_FAILED;
  }

  /* A second call could overwrite a pending response pointer. */
  for (slave = ECAT_FIRST_SLAVE; slave <= ECAT_LAST_SLAVE; slave++)
  {
    slave_info = &context->slavelist[slave];
    if ((slave_info->mbxstatus == 0) ||
        (slave_info->mbxhandlerstate != ECT_MBXH_NONE) ||
        (slave_info->mbxrmpstate != 0) ||
        (slave_info->coembxin != 0) ||
        (slave_info->coembxinfull != FALSE))
    {
      ECAT_LogPrintf(
        log,
        "[SOEM] cyclic MBX enable precheck slave%u FAIL\r\n",
        (unsigned int)slave);
      return ECAT_PDO_CONFIG_RESULT_CYCLIC_MBX_ENABLE_FAILED;
    }
  }

  for (slave = ECAT_FIRST_SLAVE; slave <= ECAT_LAST_SLAVE; slave++)
  {
    enable_ret = ecx_slavembxcyclic(context, slave);
    slave_info = &context->slavelist[slave];
    if ((enable_ret != 1) ||
        (slave_info->mbxhandlerstate != ECT_MBXH_CYCLIC) ||
        (slave_info->coembxin != EC_MBXINENABLE) ||
        (slave_info->coembxinfull != FALSE))
    {
      for (reset_slave = ECAT_FIRST_SLAVE;
           reset_slave <= ECAT_LAST_SLAVE;
           reset_slave++)
      {
        context->slavelist[reset_slave].mbxhandlerstate = ECT_MBXH_NONE;
        context->slavelist[reset_slave].mbxrmpstate = 0;
        context->slavelist[reset_slave].mbxinstateex = 0U;
        context->slavelist[reset_slave].coembxin = 0;
        context->slavelist[reset_slave].coembxinfull = FALSE;
      }
      ECAT_LogPrintf(
        log,
        "[SOEM] cyclic MBX enable slave%u ret=%d FAIL, rolled back\r\n",
        (unsigned int)slave,
        enable_ret);
      return ECAT_PDO_CONFIG_RESULT_CYCLIC_MBX_ENABLE_FAILED;
    }

    ECAT_LogPrintf(log,
                   "[SOEM] cyclic MBX enable slave%u PASS\r\n",
                   (unsigned int)slave);
  }

  return ECAT_PDO_CONFIG_RESULT_OK;
}

/**
 * @brief  生成 SOEM 组 IOmap 并校验受管从站的 PDO 映射布局
 *
 * @param[in,out] context        SOEM 主站上下文，映射后会更新组及从站的 IO 指针和长度
 * @param[in,out] log            EtherCAT 日志上下文
 * @param[in,out] pdo_config     PDO 配置上下文，用于记录 PO2SO 结果并刷新映射缓存
 * @param[out]    iomap          用于保存过程数据映射的缓冲区
 * @param[in]     iomap_capacity IOmap 缓冲区容量，单位：字节
 *
 * @return
 * 成功返回 ECAT_PDO_CONFIG_RESULT_OK；受管从站数量不足返回
 * ECAT_PDO_CONFIG_RESULT_IOMAP_SLAVE_MISSING；映射或布局校验失败返回
 * ECAT_PDO_CONFIG_RESULT_IOMAP_CHECK_FAILED or
 * ECAT_PDO_CONFIG_RESULT_CYCLIC_MBX_PREREQUISITE_FAILED
 *
 * @warning
 * 所有指针参数必须有效，iomap_capacity 必须与 iomap 的实际容量一致；调用前必须完成
 * 从站发现并注册 PO2SO 回调。本函数会清空 iomap、调用 ecx_config_map_group() 修改
 * SOEM 映射信息，并在映射后通过邮箱重新读取 PDO 配置
 */
ECAT_PdoConfigResult ECAT_PdoConfigMapGroup(
  ecx_contextt *context,
  ECAT_LogContext *log,
  ECAT_PdoConfigContext *pdo_config,
  uint8 *iomap,
  uint32 iomap_capacity)
{
  ECAT_PdoConfigMapStatus status;

  if (context->slavecount < (int)ECAT_LAST_SLAVE)
  {
    ECAT_LogPrintf(
      log,
      "[SOEM] IOmap skip: slavecount=%d need=%u\r\n",
      context->slavecount,
      (unsigned int)ECAT_LAST_SLAVE);
    return ECAT_PDO_CONFIG_RESULT_IOMAP_SLAVE_MISSING;
  }

  memset(iomap, 0, iomap_capacity);
  memset(&status, 0, sizeof(status));
  ECAT_PdoConfigRunMap(context,
                                 log,
                                 pdo_config,
                                 iomap,
                                 &status);
  ECAT_PdoConfigPrintLayout(context,
                                      log,
                                      iomap_capacity,
                                      &status);
  ECAT_PdoConfigEvaluateMap(context,
                                      log,
                                      iomap_capacity,
                                      &status);
  // ECAT_PdoConfigPrintResolved(context,
  //                                       log,
  //                                       pdo_config,
  //                                       iomap,
  //                                       iomap_capacity,
  //                                       &status);

  if (ECAT_PdoConfigMapPassed(&status) == 0U)
  {
    ECAT_LogPrintf(log,
                            "[SOEM] SAFE-OP skip: IOmap check failed\r\n");
    return ECAT_PDO_CONFIG_RESULT_IOMAP_CHECK_FAILED;
  }

  if (ECAT_PdoConfigCheckCyclicMailboxPrerequisites(
        context, log, iomap, &status) == 0U)
  {
    ECAT_LogPrintf(
      log,
      "[SOEM] SAFE-OP skip: cyclic mailbox prerequisites failed\r\n");
    return ECAT_PDO_CONFIG_RESULT_CYCLIC_MBX_PREREQUISITE_FAILED;
  }

  return ECAT_PDO_CONFIG_RESULT_OK;
}

/**
 * @brief  执行 SOEM 组映射并采集各受管从站的 PO2SO 与 PDO 缓存结果
 *
 * @param[in,out] context    SOEM 主站上下文，函数会临时修改 manualstatechange 并更新映射信息
 * @param[in,out] log        EtherCAT 日志上下文
 * @param[in,out] pdo_config PDO 配置上下文，函数会复位 PO2SO 结果并刷新映射缓存
 * @param[out]    iomap      用于接收 SOEM 过程数据映射的缓冲区
 * @param[out]    status     映射状态，记录 IOmap 大小及各从站的 PO2SO、缓存读取结果
 *
 * @return
 * 无
 *
 * @warning
 * 所有指针参数必须有效，调用前必须完成从站发现并注册 PO2SO 回调；本函数不接收
 * iomap 容量，调用方必须保证缓冲区足以容纳 ecx_config_map_group() 生成的映射。
 * 映射期间会临时启用 manualstatechange，返回前恢复其原值，并通过邮箱读取 PDO 配置
 */
static void ECAT_PdoConfigRunMap(
  ecx_contextt *context,
  ECAT_LogContext *log,
  ECAT_PdoConfigContext *pdo_config,
  uint8 *iomap,
  ECAT_PdoConfigMapStatus *status)
{
  const uint8 group = ECAT_PDO_CONFIG_GROUP;
  int previous_manual_statechange;
  uint32 slave_index;
  uint16 slave;

  ECAT_LogSectionLine(log);
  ECAT_LogPrintf(log,
                          "[SOEM] config map group start group=%u\r\n",
                          (unsigned int)group);

  previous_manual_statechange = context->manualstatechange;
  context->manualstatechange = 1;
  ECAT_PdoConfigResetPo2SoResult(pdo_config);
  ECAT_LogPrintf(
    log,
    "[SOEM] config map manual state change enabled previous=%d current=%d\r\n",
    previous_manual_statechange,
    context->manualstatechange);

  status->iomap_size = ecx_config_map_group(context, iomap, group);

  context->manualstatechange = previous_manual_statechange;
  ECAT_LogPrintf(
    log,
    "[SOEM] config map manual state change restored=%d\r\n",
    context->manualstatechange);

  for (slave_index = 0U; slave_index < ECAT_SLAVE_COUNT; slave_index++)
  {
    slave = (uint16)(ECAT_FIRST_SLAVE + slave_index);
    status->slave[slave_index].po2so_ok =
      ((pdo_config->pdo_map[slave_index].po2so_attempted != 0U) &&
       (pdo_config->pdo_map[slave_index].po2so_ok != 0U)) ? 1U : 0U;
    ECAT_LogPrintf(
      log,
      "[SOEM] slave%u explicit PO2SO callback attempted=%u result=%s\r\n",
      (unsigned int)slave,
      (unsigned int)pdo_config->pdo_map[slave_index].po2so_attempted,
      (status->slave[slave_index].po2so_ok != 0U) ? "PASS" : "FAIL");

    /* The pre-map cache describes the old setup. Reload after PO2SO. */
    status->slave[slave_index].pdo_cache_ok =
      ECAT_PdoMapLoad(context,
                      log,
                      &pdo_config->pdo_map[slave_index],
                      slave,
                      "after PO2SO/map");
  }
}

/**
 * @brief  打印 SOEM 组及各受管从站的 IOmap 布局信息
 *
 * @param[in]     context        SOEM 主站上下文
 * @param[in,out] log            EtherCAT 日志上下文
 * @param[in]     iomap_capacity IOmap 缓冲区容量，单位：字节
 * @param[in]     status         映射状态，提供 SOEM 返回的实际 IOmap 大小
 *
 * @return
 * 无
 *
 * @warning
 * context、log 和 status 必须有效，且必须在 ecx_config_map_group() 完成并填充组及
 * 从站映射信息后调用；本函数仅输出布局，不校验 IOmap 容量、指针或收发数据长度
 */
static void ECAT_PdoConfigPrintLayout(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint32 iomap_capacity,
  const ECAT_PdoConfigMapStatus *status)
{
  const uint8 group = ECAT_PDO_CONFIG_GROUP;
  uint32 slave_index;
  uint16 slave;

  ECAT_LogPrintf(log,
                          "[SOEM] IOmap size=%d buffer=%u\r\n",
                          status->iomap_size,
                          (unsigned int)iomap_capacity);
  ECAT_LogPrintf(
    log,
    "[SOEM] group0 Obytes=%lu Ibytes=%lu outWKC=%u inWKC=%u\r\n",
    (unsigned long)context->grouplist[group].Obytes,
    (unsigned long)context->grouplist[group].Ibytes,
    (unsigned int)context->grouplist[group].outputsWKC,
    (unsigned int)context->grouplist[group].inputsWKC);
  ECAT_LogPrintf(log,
                          "[SOEM] group0 outputs=0x%08lX inputs=0x%08lX\r\n",
                          (unsigned long)context->grouplist[group].outputs,
                          (unsigned long)context->grouplist[group].inputs);
  for (slave_index = 0U; slave_index < ECAT_SLAVE_COUNT; slave_index++)
  {
    slave = (uint16)(ECAT_FIRST_SLAVE + slave_index);
    ECAT_LogPrintf(
      log,
      "[SOEM] slave%u outputs=0x%08lX inputs=0x%08lX\r\n",
      (unsigned int)slave,
      (unsigned long)context->slavelist[slave].outputs,
      (unsigned long)context->slavelist[slave].inputs);
    ECAT_LogPrintf(
      log,
      "[SOEM] slave%u Obits=%lu Obytes=%lu Ibits=%lu Ibytes=%lu\r\n",
      (unsigned int)slave,
      (unsigned long)context->slavelist[slave].Obits,
      (unsigned long)context->slavelist[slave].Obytes,
      (unsigned long)context->slavelist[slave].Ibits,
      (unsigned long)context->slavelist[slave].Ibytes);
  }
}

/**
 * @brief  评估 IOmap 总体大小及各受管从站的过程数据布局
 *
 * @param[in]     context        SOEM 主站上下文
 * @param[in,out] log            EtherCAT 日志上下文
 * @param[in]     iomap_capacity IOmap 缓冲区容量，单位：字节
 * @param[in,out] status         映射状态，函数会写入总体及逐从站布局检查标志
 *
 * @return
 * 无
 *
 * @warning
 * context、log 和 status 必须有效，调用前必须完成组映射，并已设置 iomap_size、
 * PO2SO 与 PDO 缓存结果；从站列表必须覆盖全部受管从站。本函数只检查输入输出指针
 * 是否非空，不验证其地址是否位于 IOmap 缓冲区范围内
 */
static void ECAT_PdoConfigEvaluateMap(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint32 iomap_capacity,
  ECAT_PdoConfigMapStatus *status)
{
  ECAT_PdoConfigSlaveStatus *slave_status;
  uint32 slave_index;
  uint16 slave;

  status->iomap_ok = ((status->iomap_size > 0) &&
                      ((uint32)status->iomap_size <= iomap_capacity)) ?
                       1U : 0U;
  ECAT_LogPrintf(
    log,
    "[SOEM] IOmap size check=%s expected per-slave Obytes=%u Ibytes=%u\r\n",
    (status->iomap_ok != 0U) ? "PASS" : "FAIL",
    (unsigned int)APP_ETHERCAT_SERVO_RXPDO_SIZE,
    (unsigned int)APP_ETHERCAT_SERVO_TXPDO_SIZE);

  for (slave_index = 0U; slave_index < ECAT_SLAVE_COUNT; slave_index++)
  {
    slave = (uint16)(ECAT_FIRST_SLAVE + slave_index);
    slave_status = &status->slave[slave_index];
    slave_status->outputs_ok =
      (context->slavelist[slave].outputs != 0) ? 1U : 0U;
    slave_status->inputs_ok =
      (context->slavelist[slave].inputs != 0) ? 1U : 0U;
    slave_status->obytes_ok =
      (context->slavelist[slave].Obytes ==
       APP_ETHERCAT_SERVO_RXPDO_SIZE) ? 1U : 0U;
    slave_status->ibytes_ok =
      (context->slavelist[slave].Ibytes ==
       APP_ETHERCAT_SERVO_TXPDO_SIZE) ? 1U : 0U;

    ECAT_LogPrintf(
      log,
      "[SOEM] slave%u IOmap check PO2SO=%s PDOcache=%s outputs=%s inputs=%s Obytes=%s Ibytes=%s\r\n",
      (unsigned int)slave,
      (slave_status->po2so_ok != 0U) ? "PASS" : "FAIL",
      (slave_status->pdo_cache_ok != 0U) ? "PASS" : "FAIL",
      (slave_status->outputs_ok != 0U) ? "PASS" : "FAIL",
      (slave_status->inputs_ok != 0U) ? "PASS" : "FAIL",
      (slave_status->obytes_ok != 0U) ? "PASS" : "FAIL",
      (slave_status->ibytes_ok != 0U) ? "PASS" : "FAIL");
  }
}

/**
 * @brief  解析并打印各受管从站在 IOmap 中的 RxPDO 和 TxPDO 条目
 *
 * @param[in]     context        SOEM 主站上下文
 * @param[in,out] log            EtherCAT 日志上下文
 * @param[in]     pdo_config     PDO 配置上下文，提供各从站的收发映射缓存
 * @param[in]     iomap          过程数据映射缓冲区
 * @param[in]     iomap_capacity IOmap 缓冲区容量，单位：字节
 * @param[in]     status         已评估的映射状态
 *
 * @return
 * 无
 *
 * @warning
 * 所有指针参数必须有效，iomap_capacity 必须与实际缓冲区容量一致，status 必须已由
 * ECAT_PdoConfigEvaluateMap() 填充；iomap_ok 为0时不输出任何解析结果，输入或输出指针
 * 无效时跳过对应方向。本函数只读取并打印 IOmap，不修改过程数据
 */
static void ECAT_PdoConfigPrintResolved(
  ecx_contextt *context,
  ECAT_LogContext *log,
  ECAT_PdoConfigContext *pdo_config,
  uint8 *iomap,
  uint32 iomap_capacity,
  const ECAT_PdoConfigMapStatus *status)
{
  const ECAT_PdoConfigSlaveStatus *slave_status;
  uint32 slave_index;
  uint16 slave;

  if (status->iomap_ok == 0U)
  {
    return;
  }

  for (slave_index = 0U; slave_index < ECAT_SLAVE_COUNT; slave_index++)
  {
    slave = (uint16)(ECAT_FIRST_SLAVE + slave_index);
    slave_status = &status->slave[slave_index];
    if (slave_status->outputs_ok != 0U)
    {
      ECAT_PdoMapPrintResolved(
        log,
        iomap,
        iomap_capacity,
        slave,
        "OUT/RxPDO",
        &pdo_config->pdo_map[slave_index].rxpdo_map,
        context->slavelist[slave].outputs,
        context->slavelist[slave].Ostartbit,
        context->slavelist[slave].Obits,
        status->iomap_size);
    }

    if (slave_status->inputs_ok != 0U)
    {
      ECAT_PdoMapPrintResolved(
        log,
        iomap,
        iomap_capacity,
        slave,
        "IN/TxPDO",
        &pdo_config->pdo_map[slave_index].txpdo_map,
        context->slavelist[slave].inputs,
        context->slavelist[slave].Istartbit,
        context->slavelist[slave].Ibits,
      status->iomap_size);
    }
  }
}

/**
 * @brief  判断 IOmap 及所有受管从站的 PDO 映射检查是否全部通过
 *
 * @param[in] status 已完成填充的映射状态
 *
 * @return
 * IOmap 有效，且每个从站的 PO2SO、PDO 缓存、输入输出指针及收发字节长度标志均
 * 通过时返回1；任一标志未通过返回0
 *
 * @warning
 * status 必须有效，并已依次经过映射执行和 ECAT_PdoConfigEvaluateMap() 评估；本函数
 * 只汇总已保存的标志，不重新校验 SOEM 上下文，也不包含解析打印阶段的地址范围结果
 */
static uint8 ECAT_PdoConfigMapPassed(
  const ECAT_PdoConfigMapStatus *status)
{
  const ECAT_PdoConfigSlaveStatus *slave_status;
  uint32 slave_index;

  if (status->iomap_ok == 0U)
  {
    return 0U;
  }

  for (slave_index = 0U; slave_index < ECAT_SLAVE_COUNT; slave_index++)
  {
    slave_status = &status->slave[slave_index];
    if ((slave_status->po2so_ok == 0U) ||
        (slave_status->pdo_cache_ok == 0U) ||
        (slave_status->outputs_ok == 0U) ||
        (slave_status->inputs_ok == 0U) ||
        (slave_status->obytes_ok == 0U) ||
        (slave_status->ibytes_ok == 0U))
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8 ECAT_PdoConfigCheckCyclicMailboxPrerequisites(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint8 *iomap,
  const ECAT_PdoConfigMapStatus *status)
{
  const uint8 group = ECAT_PDO_CONFIG_GROUP;
  ec_groupt *group_info;
  ec_slavet *slave_info;
  uintptr_t iomap_begin;
  uintptr_t iomap_end;
  uintptr_t status_begin;
  uint32 expected_size;
  uint32 fmmu_log_start;
  int32 lookup_index;
  uint16 slave;
  uint16 mapped_slave;
  uint8 fmmu_index;
  uint8 lookup_count;
  uint8 fmmu_count;
  uint8 group_ok;
  uint8 slave_ok;
  uint8 all_ok;
  int pool_count;
  int queue_count;

  if ((context == 0) || (log == 0) || (iomap == 0) || (status == 0) ||
      (status->iomap_size <= 0))
  {
    return 0U;
  }

  group_info = &context->grouplist[group];
  iomap_begin = (uintptr_t)iomap;
  iomap_end = iomap_begin + (uint32)status->iomap_size;
  status_begin = (uintptr_t)group_info->mbxstatus;

  pool_count = -1;
  if (context->mbxpool.mbxmutex != 0)
  {
    osal_mutex_lock(context->mbxpool.mbxmutex);
    pool_count = context->mbxpool.listcount;
    osal_mutex_unlock(context->mbxpool.mbxmutex);
  }

  queue_count = -1;
  if (group_info->mbxtxqueue.mbxmutex != 0)
  {
    osal_mutex_lock(group_info->mbxtxqueue.mbxmutex);
    queue_count = group_info->mbxtxqueue.listcount;
    osal_mutex_unlock(group_info->mbxtxqueue.mbxmutex);
  }

  expected_size = 0U;
  if ((group_info->mbxstatuslength > 0) &&
      (group_info->mbxstatuslength <= EC_MAXSLAVE))
  {
    expected_size = group_info->Obytes + group_info->Ibytes +
                    (uint32)group_info->mbxstatuslength;
  }
  group_ok = ((context->overlappedMode == FALSE) &&
              (group_info->blockLRW == 0U) &&
              (group_info->logstartaddr == 0U) &&
              (context->mbxpool.mbxmutex != 0) &&
              (group_info->mbxtxqueue.mbxmutex != 0) &&
              (pool_count == EC_MBXPOOLSIZE) &&
              (queue_count == 0) &&
              (group_info->outputsWKC == ECAT_SLAVE_COUNT) &&
              (group_info->inputsWKC == ECAT_SLAVE_COUNT) &&
              (group_info->mbxstatus != 0) &&
              (group_info->mbxstatuslength > 0) &&
              (group_info->mbxstatuslength <= EC_MAXSLAVE) &&
              (status_begin == iomap_begin + group_info->Obytes +
                               group_info->Ibytes) &&
              (status_begin >= iomap_begin) &&
              (status_begin <= iomap_end) &&
              ((uint32)group_info->mbxstatuslength <=
               iomap_end - status_begin) &&
              (expected_size == (uint32)status->iomap_size)) ? 1U : 0U;

  if (group_ok != 0U)
  {
    for (lookup_index = 0;
         lookup_index < group_info->mbxstatuslength;
         lookup_index++)
    {
      mapped_slave = group_info->mbxstatuslookup[lookup_index];
      if ((mapped_slave < 1U) ||
          (mapped_slave > (uint16)context->slavecount) ||
          (context->slavelist[mapped_slave].mbxstatus !=
           group_info->mbxstatus + lookup_index))
      {
        group_ok = 0U;
        break;
      }
    }
  }

  ECAT_LogPrintf(
    log,
    "[SOEM] cyclic MBX group0 size=%d status=%ld WKC=%u/%u LRW=%s pool=%d queue=%d result=%s\r\n",
    status->iomap_size,
    (long)group_info->mbxstatuslength,
    (unsigned int)group_info->outputsWKC,
    (unsigned int)group_info->inputsWKC,
    (group_info->blockLRW == 0U) ? "YES" : "NO",
    pool_count,
    queue_count,
    (group_ok != 0U) ? "PASS" : "FAIL");

  all_ok = group_ok;
  for (slave = ECAT_FIRST_SLAVE; slave <= ECAT_LAST_SLAVE; slave++)
  {
    slave_info = &context->slavelist[slave];
    lookup_count = 0U;
    fmmu_count = 0U;

    if ((group_info->mbxstatuslength > 0) &&
        (group_info->mbxstatuslength <= EC_MAXSLAVE))
    {
      for (lookup_index = 0;
           lookup_index < group_info->mbxstatuslength;
           lookup_index++)
      {
        if (group_info->mbxstatuslookup[lookup_index] == slave)
        {
          lookup_count++;
        }
      }
    }

    if (slave_info->FMMUunused <= EC_MAXFMMU)
    {
      for (fmmu_index = 0U;
           fmmu_index < slave_info->FMMUunused;
           fmmu_index++)
      {
        fmmu_log_start = etohl(slave_info->FMMU[fmmu_index].LogStart);
        if ((etohs(slave_info->FMMU[fmmu_index].LogLength) == 1U) &&
            (slave_info->FMMU[fmmu_index].LogStartbit == 0U) &&
            (slave_info->FMMU[fmmu_index].LogEndbit == 7U) &&
            (slave_info->FMMU[fmmu_index].PhysStart == ECT_REG_SM1STAT) &&
            (slave_info->FMMU[fmmu_index].PhysStartBit == 0U) &&
            (slave_info->FMMU[fmmu_index].FMMUtype == 1U) &&
            (slave_info->FMMU[fmmu_index].FMMUactive == 1U) &&
            (fmmu_log_start < (uint32)status->iomap_size) &&
            ((uintptr_t)slave_info->mbxstatus ==
             iomap_begin + fmmu_log_start))
        {
          fmmu_count++;
        }
      }
    }

    slave_ok = ((slave_info->group == group) &&
                ((slave_info->mbx_proto & ECT_MBXPROT_COE) != 0U) &&
                (slave_info->mbx_l >= ECAT_CYCLIC_MBX_MIN_LENGTH) &&
                (slave_info->mbx_l <= EC_MAXMBX) &&
                (slave_info->mbx_rl >= ECAT_CYCLIC_MBX_MIN_LENGTH) &&
                (slave_info->mbx_rl <= EC_MAXMBX) &&
                (slave_info->mbx_wo != 0U) &&
                (slave_info->mbx_ro != 0U) &&
                (slave_info->mbxhandlerstate == ECT_MBXH_NONE) &&
                (slave_info->mbxrmpstate == 0) &&
                (slave_info->coembxin == 0) &&
                (slave_info->coembxinfull == FALSE) &&
                (lookup_count == 1U) &&
                (fmmu_count == 1U)) ? 1U : 0U;
    all_ok = ((all_ok != 0U) && (slave_ok != 0U)) ? 1U : 0U;

    /* CoEdetails is diagnostic. The successful PDO configuration SDOs
       above are the effective CoE SDO capability check for this device. */
    ECAT_LogPrintf(
      log,
      "[SOEM] cyclic MBX slave%u CoE=%s SDOflag=%s tx=%u rx=%u lookup=%u FMMU=%u idle=%s result=%s\r\n",
      (unsigned int)slave,
      ((slave_info->mbx_proto & ECT_MBXPROT_COE) != 0U) ? "PASS" : "FAIL",
      ((slave_info->CoEdetails & ECT_COEDET_SDO) != 0U) ? "YES" : "NO",
      (unsigned int)slave_info->mbx_l,
      (unsigned int)slave_info->mbx_rl,
      (unsigned int)lookup_count,
      (unsigned int)fmmu_count,
      ((slave_info->mbxhandlerstate == ECT_MBXH_NONE) &&
       (slave_info->mbxrmpstate == 0) &&
       (slave_info->coembxin == 0) &&
       (slave_info->coembxinfull == FALSE)) ? "PASS" : "FAIL",
      (slave_ok != 0U) ? "PASS" : "FAIL");
  }

  return all_ok;
}

/**
 * @brief  配置受管从站在 PRE-OP 至 SAFE-OP 阶段使用的显式 PDO 映射
 *
 * @param[in,out] context            SOEM 回调传入的主站上下文
 * @param[in]     expected_context   期望的 SOEM 主站上下文地址，用于身份校验
 * @param[in,out] log                EtherCAT 日志上下文
 * @param[in,out] pdo_config         PDO 配置上下文，用于记录对应从站的配置结果
 * @param[in]     slave              要配置的从站索引
 *
 * @return
 * 配置及回读校验成功返回1，参数、从站状态或 PDO 配置异常返回0
 *
 * @warning
 * context 必须与 expected_context 相同，log 和 pdo_config 必须有效，且 slave 必须
 * 处于受管范围内、无错误的 PRE-OP 状态并支持 CoE；配置失败时 PDO 分配可能保持禁用
 */
int ECAT_PdoConfigConfigurePo2So(
  ecx_contextt *context,
  ecx_contextt *expected_context,
  ECAT_LogContext *log,
  ECAT_PdoConfigContext *pdo_config,
  uint16 slave)
{
  uint32 slave_index;

  if ((pdo_config == 0) ||
      (slave < ECAT_FIRST_SLAVE) ||
      (slave > ECAT_LAST_SLAVE))
  {
    ECAT_LogPrintf(
      log,
      "[SOEM] PO2SO unmanaged slave%u FAIL\r\n",
      (unsigned int)slave);
    return 0;
  }

  slave_index = (uint32)(slave - ECAT_FIRST_SLAVE);
  return ECAT_PdoMapConfigurePo2So(
    context,
    expected_context,
    log,
    &pdo_config->pdo_map[slave_index],
    slave,
    slave);
}

/**
 * @brief  获取 PDO 配置结果对应的失败原因文本
 *
 * @param[in] result PDO 配置结果码
 *
 * @return
 * 返回指向只读英文原因字符串的指针
 *
 * @warning
 * 本函数仅用于解释失败结果；result 为 ECAT_PDO_CONFIG_RESULT_OK 或未知枚举值时，
 * 均返回通用文本 "PDO configuration failed"，调用方不得修改返回的字符串
 */
const char *ECAT_PdoConfigResultReason(
  ECAT_PdoConfigResult result)
{
  switch (result)
  {
    case ECAT_PDO_CONFIG_RESULT_ASSIGN_SLAVE_MISSING:
      return "PDO assignment slave missing";

    case ECAT_PDO_CONFIG_RESULT_COE_REQUIRED:
      return "PDO assignment requires CoE";

    case ECAT_PDO_CONFIG_RESULT_IOMAP_SLAVE_MISSING:
      return "IOmap slave missing";

    case ECAT_PDO_CONFIG_RESULT_IOMAP_CHECK_FAILED:
      return "IOmap check failed";

    case ECAT_PDO_CONFIG_RESULT_CYCLIC_MBX_PREREQUISITE_FAILED:
      return "cyclic mailbox prerequisites failed";

    case ECAT_PDO_CONFIG_RESULT_CYCLIC_MBX_ENABLE_FAILED:
      return "cyclic mailbox enable failed";

    case ECAT_PDO_CONFIG_RESULT_OK:
    default:
      return "PDO configuration failed";
  }
}
