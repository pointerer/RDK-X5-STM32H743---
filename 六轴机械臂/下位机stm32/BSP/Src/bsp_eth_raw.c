/*
 * 文件作用：
 * 本文件在 STM32H743 ETH HAL 驱动之上封装裸二层以太网帧收发接口。
 * 主要负责启动 ETH MAC/DMA、发送和轮询接收完整 Ethernet frame，
 * 并提供 HAL 接收路径需要的 RX buffer 分配与链接回调。
 * RX/TX 缓冲区按编译器差异放置到指定内存区域，供 DMA 直接访问。
 */
#include "bsp_eth_raw.h"

#include "eth.h"
#include <stddef.h>
#include <string.h>

#if defined ( __ICCARM__ )
#pragma location=0x30000400
static uint8_t eth_raw_rx_buffers[ETH_RX_DESC_CNT][ETH_RAW_FRAME_MAX_LEN];
#pragma location=0x30002000
static uint8_t eth_raw_tx_buffer[ETH_RAW_FRAME_MAX_LEN];
#elif defined ( __CC_ARM )
__attribute__((at(0x30000400))) static uint8_t eth_raw_rx_buffers[ETH_RX_DESC_CNT][ETH_RAW_FRAME_MAX_LEN];
__attribute__((at(0x30002000))) static uint8_t eth_raw_tx_buffer[ETH_RAW_FRAME_MAX_LEN];
#elif defined ( __GNUC__ )
static uint8_t eth_raw_rx_buffers[ETH_RX_DESC_CNT][ETH_RAW_FRAME_MAX_LEN] __attribute__((section(".RxArraySection")));
static uint8_t eth_raw_tx_buffer[ETH_RAW_FRAME_MAX_LEN] __attribute__((section(".TxArraySection")));
#else
static uint8_t eth_raw_rx_buffers[ETH_RX_DESC_CNT][ETH_RAW_FRAME_MAX_LEN];
static uint8_t eth_raw_tx_buffer[ETH_RAW_FRAME_MAX_LEN];
#endif

static uint32_t eth_raw_rx_alloc_index = 0;

static uint8_t ETH_Raw_ConfigMacFilter(void)
{
  ETH_MACFilterConfigTypeDef filter_config;

  memset(&filter_config, 0, sizeof(filter_config));
  filter_config.PromiscuousMode = ENABLE;
  filter_config.PassAllMulticast = ENABLE;
  filter_config.BroadcastFilter = ENABLE;

  return (HAL_ETH_SetMACFilterConfig(&heth, &filter_config) == HAL_OK) ? 1U : 0U;
}

/*
 * 功能：确保 STM32 ETH MAC 和 DMA 收发流程已经启动。
 * 说明：MX_ETH_Init() 只完成外设初始化；若 ETH 已经启动，则直接返回成功，避免重复启动时报错。
 */
uint8_t ETH_Raw_Start(void)
{
  if ((heth.gState != HAL_ETH_STATE_READY) &&
      (heth.gState != HAL_ETH_STATE_STARTED))
  {
    return 0U;
  }

  if (ETH_Raw_ConfigMacFilter() == 0U)
  {
    return 0U;
  }

  if (heth.gState == HAL_ETH_STATE_STARTED)
  {
    return 1U;
  }

  return (HAL_ETH_Start(&heth) == HAL_OK) ? 1U : 0U;
}

/*
 * 功能：获取 ETH HAL 当前配置的本机 MAC 地址。
 * 说明：返回值直接来自 heth.Init.MACAddr，若 ETH 尚未配置 MAC 地址则返回 NULL。
 */
const uint8_t *ETH_Raw_GetMacAddress(void)
{
  return heth.Init.MACAddr;
}

/*
 * 功能：发送一帧完整的裸二层以太网帧。
 * 说明：frame 中应包含目的 MAC、源 MAC、EtherType 和 payload；CRC/pad 由 MAC 硬件补齐。
 */
uint8_t ETH_Raw_Send(const uint8_t *frame, uint32_t length)
{
  ETH_BufferTypeDef tx_buffer;
  ETH_TxPacketConfigTypeDef tx_config;

  if ((frame == NULL) ||
      (length < ETH_RAW_FRAME_HEADER_LEN) ||
      (length > ETH_RAW_FRAME_MAX_LEN))
  {
    return 0U;
  }

  memcpy(eth_raw_tx_buffer, frame, length);

  tx_buffer.buffer = eth_raw_tx_buffer;
  tx_buffer.len = length;
  tx_buffer.next = NULL;

  memset(&tx_config, 0, sizeof(tx_config));
  tx_config.Attributes = ETH_TX_PACKETS_FEATURES_CRCPAD;
  tx_config.Length = length;
  tx_config.TxBuffer = &tx_buffer;
  tx_config.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  tx_config.ChecksumCtrl = ETH_CHECKSUM_DISABLE;

  return (HAL_ETH_Transmit(&heth, &tx_config, 100U) == HAL_OK) ? 1U : 0U;
}

/*
 * 功能：轮询接收一帧裸二层以太网帧，并复制到调用者提供的 buffer。
 * 说明：返回值为实际复制的帧长度；返回 0 表示当前没有收到完整帧或参数无效。
 */
uint16_t ETH_Raw_Receive(uint8_t *frame, uint16_t max_length)
{
  void *rx_buffer = NULL;
  uint32_t rx_length = 0U;

  if ((frame == NULL) || (max_length == 0U))
  {
    return 0U;
  }

  if (HAL_ETH_ReadData(&heth, &rx_buffer) != HAL_OK)
  {
    return 0U;
  }

  if (rx_buffer == NULL)
  {
    return 0U;
  }

  rx_length = heth.RxDescList.RxDataLength;
  if (rx_length > max_length)
  {
    rx_length = max_length;
  }

  memcpy(frame, rx_buffer, rx_length);

  return (uint16_t)rx_length;
}

/**
 * @brief  读取 STM32 ETH 硬件及接收描述符调试状态
 *
 * @param[out] info   ETH 调试状态输出结构体地址
 *
 * @return
 * 无
 *
 * @warning
 * 调用前必须完成 MX_ETH_Init()；info 为空时函数直接返回且不提供错误状态，
 * 输出字段仅表示各寄存器及 HAL 接收描述符被读取时的瞬时状态
 */
void ETH_Raw_GetDebugInfo(ETH_RawDebugInfo *info)
{
  if (info == NULL)
  {
    return;
  }

  info->maccr = heth.Instance->MACCR;
  info->macpfr = heth.Instance->MACPFR;
  info->macrxtxsr = heth.Instance->MACRXTXSR;
  info->dmacsr = heth.Instance->DMACSR;
  info->dmacrcr = heth.Instance->DMACRCR;
  info->dmactcr = heth.Instance->DMACTCR;
  info->mtlrqomr = heth.Instance->MTLRQOMR;
  info->rx_desc_idx = heth.RxDescList.RxDescIdx;
  info->rx_build_desc_cnt = heth.RxDescList.RxBuildDescCnt;
  info->rx_data_length = heth.RxDescList.RxDataLength;
}

/*
 * 功能：为 ETH HAL 接收描述符分配 RX buffer。
 * 说明：H7 新版 HAL 在接收路径中通过该回调向应用层索要接收缓冲区。
 */
void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
  *buff = eth_raw_rx_buffers[eth_raw_rx_alloc_index];
  eth_raw_rx_alloc_index++;

  if (eth_raw_rx_alloc_index >= ETH_RX_DESC_CNT)
  {
    eth_raw_rx_alloc_index = 0U;
  }
}

/*
 * 功能：把 HAL 读到的 RX buffer 挂接成一帧数据。
 * 说明：当前 RxBuffLen 为 1524 字节，一帧普通以太网帧可以放在一个 RX buffer 中。
 */
void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t Length)
{
  (void)Length;

  if (*pStart == NULL)
  {
    *pStart = buff;
  }

  *pEnd = buff;
}
