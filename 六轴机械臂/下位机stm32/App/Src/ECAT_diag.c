#include "ECAT_diag.h"

#include "bsp_eth_raw.h"
#include "bsp_lan8720.h"
#include "bsp_soem_timebase.h"

static const char *ECAT_DiagCiA402StateText(uint16_t statusword);
static void ECAT_DiagPrintAxis2TxPdo(
  ECAT_LogContext *log,
  uint16_t slave,
  const AppEtherCAT_ServoTxPdo *txpdo);

void ECAT_DiagReset(ECAT_DiagContext *diag)
{
  diag->last_phy_print_us = 0ULL;
}

/**
 * @brief  逐字节打印指定从站的原始输入过程数据
 *
 * @param[in,out] log          EtherCAT 日志上下文
 * @param[in]     slave        从站索引，仅用于日志标识
 * @param[in]     inputs       输入过程数据缓冲区
 * @param[in]     input_len    实际输入数据长度，单位：字节
 * @param[in]     expected_len 期望输入数据长度，单位：字节，仅用于日志对比
 *
 * @return
 * 无
 *
 * @warning
 * log 必须有效，inputs 必须指向至少 input_len 字节的可读区域；inputs 为空时仅记录
 * 跳过信息。函数不会按 expected_len 截断或判定失败，而是为每个实际字节输出一条日志，
 * 数据较长时会显著占用串口和执行时间，仅应在诊断阶段调用
 */
void ECAT_DiagPrintInputsRaw(ECAT_LogContext *log,
                                      uint16_t slave,
                                      const uint8_t *inputs,
                                      uint32_t input_len,
                                      uint32_t expected_len)
{
  uint32_t offset;

  if (inputs == 0)
  {
    ECAT_LogPrintf(log,
                            "[SOEM] slave%d input raw skip: NULL\r\n",
                            slave);
    return;
  }

  ECAT_LogPrintf(log,
                          "[SOEM] slave%d input raw len=%lu expected=%u\r\n",
                          slave,
                          (unsigned long)input_len,
                          (unsigned int)expected_len);

  for (offset = 0U; offset < input_len; offset++)
  {
    ECAT_LogPrintf(log,
                            "[SOEM] slave%d input[%02lu]=0x%02X\r\n",
                            slave,
                            (unsigned long)offset,
                            (unsigned int)inputs[offset]);
  }
}

/**
 * @brief  输出指定 EtherCAT 从站的邮箱配置及协议能力
 *
 * @param[in,out] log          EtherCAT 日志上下文
 * @param[in]     slave       从站索引，仅用于日志标识
 * @param[in]     slave_info  SOEM 从站信息结构体地址
 *
 * @return
 * 无
 *
 * @warning
 * log 必须有效且日志串口必须已初始化；slave_info 应来自从站发现后的 slavelist
 * 并与 slave 对应，slave_info 为空时函数静默返回且不输出诊断信息
 */
void ECAT_DiagPrintMailbox(ECAT_LogContext *log,
                                     uint16_t slave,
                                     const ec_slavet *slave_info)
{
  if (slave_info == 0)
  {
    return;
  }

  ECAT_LogPrintf(log,
                          "[SOEM] slave%d mbx out addr=0x%04X len=%u\r\n",
                          slave,
                          slave_info->mbx_wo,
                          slave_info->mbx_l);
  ECAT_LogPrintf(log,
                          "[SOEM] slave%d mbx in  addr=0x%04X len=%u\r\n",
                          slave,
                          slave_info->mbx_ro,
                          slave_info->mbx_rl);
  ECAT_LogPrintf(log,
                          "[SOEM] slave%d mbx_proto=0x%04X CoE=%s\r\n",
                          slave,
                          slave_info->mbx_proto,
                          ((slave_info->mbx_proto & ECT_MBXPROT_COE) != 0U) ?
                            "yes" : "no");
}

/**
 * @brief  输出最近一次 EtherCAT 响应等待过程的接收诊断快照
 *
 * @param[in,out] log   EtherCAT 日志上下文
 *
 * @return
 * 无
 *
 * @warning
 * log 必须指向有效上下文，且日志串口必须已初始化；诊断快照会在每次
 * ecx_waitinframe() 开始时被清零，禁止与接收流程并发调用
 */
void ECAT_DiagPrintRx(ECAT_LogContext *log)
{
  const ecx_rxdebugt *rx;

  rx = ecx_get_rxdebug();

  ECAT_LogPrintf(log,
                          "[SOEM] rx polls=%lu none=%lu frames=%lu matched=%lu\r\n",
                          (unsigned long)rx->polls,
                          (unsigned long)rx->no_frame,
                          (unsigned long)rx->rx_frames,
                          (unsigned long)rx->matched);
  ECAT_LogPrintf(log,
                          "[SOEM] rx short=%lu non_ecat=%lu idx_mis=%lu bad_len=%lu\r\n",
                          (unsigned long)rx->too_short,
                          (unsigned long)rx->non_ethercat,
                          (unsigned long)rx->idx_mismatch,
                          (unsigned long)rx->bad_length);
  ECAT_LogPrintf(log,
                          "[SOEM] rx last len=%u etype=0x%04X idx=%u ecat_len=%u wkc=%d\r\n",
                          (unsigned int)rx->last_rx_len,
                          (unsigned int)rx->last_ethertype,
                          (unsigned int)rx->last_rx_idx,
                          (unsigned int)rx->last_ecat_length,
                          rx->last_wkc);
  ECAT_LogPrintf(log,
                          "[SOEM] rx dst=%02X:%02X:%02X:%02X:%02X:%02X\r\n",
                          (unsigned int)rx->last_dst[0],
                          (unsigned int)rx->last_dst[1],
                          (unsigned int)rx->last_dst[2],
                          (unsigned int)rx->last_dst[3],
                          (unsigned int)rx->last_dst[4],
                          (unsigned int)rx->last_dst[5]);
  ECAT_LogPrintf(log,
                          "[SOEM] rx src=%02X:%02X:%02X:%02X:%02X:%02X\r\n",
                          (unsigned int)rx->last_src[0],
                          (unsigned int)rx->last_src[1],
                          (unsigned int)rx->last_src[2],
                          (unsigned int)rx->last_src[3],
                          (unsigned int)rx->last_src[4],
                          (unsigned int)rx->last_src[5]);
}

/**
 * @brief  输出 STM32 ETH 硬件及接收描述符诊断快照
 *
 * @param[in,out] log   EtherCAT 日志上下文
 *
 * @return
 * 无
 *
 * @warning
 * log 必须指向有效上下文；调用前必须完成 MX_ETH_Init() 及日志串口初始化，
 * 输出内容仅表示读取时刻的 MAC、DMA、MTL 和 HAL 接收描述符状态
 */
void ECAT_DiagPrintEth(ECAT_LogContext *log)
{
  ETH_RawDebugInfo info;

  ETH_Raw_GetDebugInfo(&info);

  ECAT_LogPrintf(log,
                          "[ETH] MACCR=0x%08lX MACPFR=0x%08lX MACRXTXSR=0x%08lX\r\n",
                          (unsigned long)info.maccr,
                          (unsigned long)info.macpfr,
                          (unsigned long)info.macrxtxsr);
  ECAT_LogPrintf(log,
                          "[ETH] DMACSR=0x%08lX DMACRCR=0x%08lX DMACTCR=0x%08lX\r\n",
                          (unsigned long)info.dmacsr,
                          (unsigned long)info.dmacrcr,
                          (unsigned long)info.dmactcr);
  ECAT_LogPrintf(log,
                          "[ETH] MTLRQOMR=0x%08lX rx_idx=%lu rx_build=%lu rx_len=%lu\r\n",
                          (unsigned long)info.mtlrqomr,
                          (unsigned long)info.rx_desc_idx,
                          (unsigned long)info.rx_build_desc_cnt,
                          (unsigned long)info.rx_data_length);
}

/**
 * @brief  按指定时间间隔读取并输出 LAN8720A PHY 诊断信息
 *
 * @param[in,out] diag                PHY 诊断上下文，用于记录上次打印时间
 * @param[in,out] log                 EtherCAT 日志上下文
 * @param[in]     print_interval_us   最小打印间隔，单位：us；0 表示不限制间隔
 *
 * @return
 * 无
 *
 * @warning
 * diag 和 log 必须指向有效上下文；调用前必须完成 SOEM 微秒时基、以太网外设
 * 及日志串口初始化
 */
void ECAT_DiagProcessPhyMonitor(ECAT_DiagContext *diag,
                                          ECAT_LogContext *log,
                                          uint64_t print_interval_us)
{
  uint64_t now_us;
  uint32_t bsr = 0U;
  uint8_t link_up;
  uint8_t bsr_ok;

  now_us = (uint64_t)BSP_SOEM_Timebase_GetUs();
  if ((diag->last_phy_print_us != 0ULL) &&
      ((now_us - diag->last_phy_print_us) < print_interval_us))
  {
    return;
  }

  diag->last_phy_print_us = now_us;

  link_up = LAN8720_IsLinkUp();
  bsr_ok = LAN8720_ReadBSR(&bsr);

  ECAT_LogPrintf(log,
                          "[PHY] link=%u bsr=0x%04lX read=%s\r\n",
                          link_up,
                          (unsigned long)(bsr & 0xFFFFU),
                          (bsr_ok != 0U) ? "OK" : "FAIL");
}

void ECAT_DiagPrintTxPdo(ECAT_LogContext *log,
                                   uint16_t slave,
                                   const AppEtherCAT_ServoTxPdo *txpdo)
{
  if (txpdo == 0)
  {
    return;
  }

  ECAT_LogPrintf(log,
                          "[SOEM] TxPDO decoded slave%d size=%u bytes\r\n",
                          slave,
                          (unsigned int)sizeof(*txpdo));
  ECAT_DiagPrintTxPdoAxis1(log, slave, txpdo);
  ECAT_DiagPrintAxis2TxPdo(log, slave, txpdo);
}

void ECAT_DiagPrintTxPdoAxis1(
  ECAT_LogContext *log,
  uint16_t slave,
  const AppEtherCAT_ServoTxPdo *txpdo)
{
  uint16_t statusword;

  if (txpdo == 0)
  {
    return;
  }

  statusword = txpdo->statusword;

  ECAT_LogPrintf(log,
                          "[SOEM] slave%u Axis1 0x6041 SW=0x%04X %s 0x6061 mode=%d 0x603F error=0x%04X\r\n",
                          (unsigned int)slave,
                          (unsigned int)statusword,
                          ECAT_DiagCiA402StateText(statusword),
                          (int)txpdo->modes_of_operation_display,
                          (unsigned int)txpdo->error_code);
  ECAT_LogPrintf(log,
                          "[SOEM] slave%u Axis1 0x6064 pos=%ld 0x606C vel=%ld 0x6077 torque=%d\r\n",
                          (unsigned int)slave,
                          (long)txpdo->position_actual_value,
                          (long)txpdo->velocity_actual_value,
                          (int)txpdo->torque_actual_value);
  ECAT_LogPrintf(log,
                          "[SOEM] slave%u Axis1 0x3154 accel=%d 0x6164 multi_pos=%ld\r\n",
                          (unsigned int)slave,
                          (int)txpdo->axis1_accelerometer,
                          (long)txpdo->axis1_multi_position_actual);
  ECAT_LogPrintf(log,
                          "[SOEM] slave%u Axis1 CiA402 ready=%u switched=%u op=%u fault=%u warn=%u\r\n",
                          (unsigned int)slave,
                          (unsigned int)((statusword & 0x0001U) ? 1U : 0U),
                          (unsigned int)((statusword & 0x0002U) ? 1U : 0U),
                          (unsigned int)((statusword & 0x0004U) ? 1U : 0U),
                          (unsigned int)((statusword & 0x0008U) ? 1U : 0U),
                          (unsigned int)((statusword & 0x0080U) ? 1U : 0U));
}

static void ECAT_DiagPrintAxis2TxPdo(
  ECAT_LogContext *log,
  uint16_t slave,
  const AppEtherCAT_ServoTxPdo *txpdo)
{
  const uint16_t statusword = txpdo->axis2_statusword;

  ECAT_LogPrintf(log,
                          "[SOEM] slave%u Axis2 0x6841 SW=0x%04X %s 0x6861 mode=%d 0x683F error=0x%04X\r\n",
                          (unsigned int)slave,
                          (unsigned int)statusword,
                          ECAT_DiagCiA402StateText(statusword),
                          (int)txpdo->axis2_modes_of_operation_display,
                          (unsigned int)txpdo->axis2_error_code);
  ECAT_LogPrintf(log,
                          "[SOEM] slave%u Axis2 0x6864 pos=%ld 0x686C vel=%ld 0x6877 torque=%d\r\n",
                          (unsigned int)slave,
                          (long)txpdo->axis2_position_actual_value,
                          (long)txpdo->axis2_velocity_actual_value,
                          (int)txpdo->axis2_torque_actual_value);
  ECAT_LogPrintf(log,
                          "[SOEM] slave%u Axis2 0x3954 accel=%d 0x6964 multi_pos=%ld\r\n",
                          (unsigned int)slave,
                          (int)txpdo->axis2_accelerometer,
                          (long)txpdo->axis2_multi_position_actual);
  ECAT_LogPrintf(log,
                          "[SOEM] slave%u Axis2 CiA402 ready=%u switched=%u op=%u fault=%u warn=%u\r\n",
                          (unsigned int)slave,
                          (unsigned int)((statusword & 0x0001U) ? 1U : 0U),
                          (unsigned int)((statusword & 0x0002U) ? 1U : 0U),
                          (unsigned int)((statusword & 0x0004U) ? 1U : 0U),
                          (unsigned int)((statusword & 0x0008U) ? 1U : 0U),
                          (unsigned int)((statusword & 0x0080U) ? 1U : 0U));
}

static const char *ECAT_DiagCiA402StateText(uint16_t statusword)
{
  if ((statusword & 0x004FU) == 0x0000U)
  {
    return "Not ready";
  }

  if ((statusword & 0x004FU) == 0x0040U)
  {
    return "Switch on disabled";
  }

  if ((statusword & 0x006FU) == 0x0021U)
  {
    return "Ready to switch on";
  }

  if ((statusword & 0x006FU) == 0x0023U)
  {
    return "Switched on";
  }

  if ((statusword & 0x006FU) == 0x0027U)
  {
    return "Operation enabled";
  }

  if ((statusword & 0x006FU) == 0x0007U)
  {
    return "Quick stop active";
  }

  if ((statusword & 0x004FU) == 0x000FU)
  {
    return "Fault reaction active";
  }

  if ((statusword & 0x004FU) == 0x0008U)
  {
    return "Fault";
  }

  return "Unknown";
}
