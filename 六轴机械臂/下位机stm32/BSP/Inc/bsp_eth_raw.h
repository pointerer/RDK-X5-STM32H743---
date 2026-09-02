#ifndef __BSP_ETH_RAW_H__
#define __BSP_ETH_RAW_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ETH_RAW_FRAME_HEADER_LEN  14U    /* 以太网二层帧头长度：目的 MAC、源 MAC 和 EtherType。 */
#define ETH_RAW_FRAME_MAX_LEN     1524U  /* 不含 FCS 的普通以太网帧最大缓冲长度。 */

typedef struct
{
  uint32_t maccr;             /* MAC Configuration Register，用于观察 MAC 收发使能、速率、双工等配置。 */
  uint32_t macpfr;            /* MAC Packet Filter Register，用于观察 Promiscuous、组播、广播过滤配置。 */
  uint32_t macrxtxsr;         /* MAC Rx Tx Status Register，用于观察 MAC 层收发状态和错误标志。 */
  uint32_t dmacsr;            /* DMA Channel Status Register，用于观察 DMA 收发完成、停止、buffer 不可用等状态。 */
  uint32_t dmacrcr;           /* DMA Channel Rx Control Register，用于观察 RX DMA 是否启动以及 RX buffer 配置。 */
  uint32_t dmactcr;           /* DMA Channel Tx Control Register，用于观察 TX DMA 是否启动以及 TX buffer 配置。 */
  uint32_t mtlrqomr;          /* MTL Rx Queue Operating Mode Register，用于观察接收队列工作模式。 */
  uint32_t rx_desc_idx;       /* HAL 当前检查的 RX 描述符索引，用于判断接收描述符是否在推进。 */
  uint32_t rx_build_desc_cnt; /* HAL 待重新构建的 RX 描述符数量，正常接收启动后通常应回到 0。 */
  uint32_t rx_data_length;    /* HAL 最近一次解析到的 RX 帧长度，未收到帧时通常为 0。 */
} ETH_RawDebugInfo;

uint8_t ETH_Raw_Start(void);
const uint8_t *ETH_Raw_GetMacAddress(void);
uint8_t ETH_Raw_Send(const uint8_t *frame, uint32_t length);
uint16_t ETH_Raw_Receive(uint8_t *frame, uint16_t max_length);
void ETH_Raw_GetDebugInfo(ETH_RawDebugInfo *info);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ETH_RAW_H__ */
