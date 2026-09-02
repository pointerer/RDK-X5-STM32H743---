/*
 * 文件作用：
 * 本文件放置以太网链路调试相关的应用层逻辑。
 * 它组合 LAN8720 PHY 状态读取、裸二层测试帧收发和 UART 日志队列输出，
 * 让 Core 层只负责启动框架，BSP 层只负责底层外设访问。
 */
#include "app_eth_debug.h"

#include "app_uart_log.h"
#include "bsp_eth_raw.h"
#include "bsp_lan8720.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define APP_ETH_DEBUG_HEADER_LEN        14U      /* 以太网二层帧头长度：目的 MAC、源 MAC 和 EtherType。 */
#define APP_ETH_DEBUG_MIN_FRAME_LEN     60U      /* 不含 FCS 的最小以太网帧长度，发送短帧时按该长度补齐。 */
#define APP_ETH_DEBUG_TEST_ETHERTYPE    0x88A4U  /* STM32 主动发送的裸二层测试帧 EtherType。 */
#define APP_ETH_DEBUG_PC_ETHERTYPE      0x88B5U  /* PC 或上位机发送给 STM32 的裸二层测试帧 EtherType。 */
#define APP_ETH_DEBUG_RX_PAYLOAD_MAX    128U     /* 调试输出中保存和打印的最大 payload 字节数。 */
#define APP_ETH_DEBUG_STATUS_LINE_MAX   64U      /* PHY 状态名称、单字节值和 CRLF 的最大组合长度。 */

typedef struct
{
  uint8_t dst_mac[6];
  uint8_t src_mac[6];
  uint16_t eth_type;
  uint16_t frame_len;
  uint16_t payload_len;
  uint8_t payload[APP_ETH_DEBUG_RX_PAYLOAD_MAX + 1U];
} AppEthDebug_RawFrameInfo;

static uint8_t app_eth_debug_rx_frame[ETH_RAW_FRAME_MAX_LEN];

static void App_EthDebug_SendByteValue(const uint8_t *name, uint16_t name_len, uint8_t value);
static void App_EthDebug_SendPhyStatus(void);
static uint8_t App_EthDebug_ReceiveRawFrame(AppEthDebug_RawFrameInfo *info);

/*
 * 功能：初始化以太网调试应用逻辑。
 * 说明：配置 LAN8720A 为全部能力自动协商模式，并通过 UART 日志队列输出一次 PHY 识别和链路状态。
 */
void App_EthDebug_Init(void)
{
  (void)LAN8720_SetModeAndSoftReset(LAN8720_MODE_ALL_CAPABLE_AUTONEG);
  App_EthDebug_SendPhyStatus();
}

/*
 * 功能：周期性处理以太网调试收包逻辑。
 * 说明：收到指定 EtherType 的裸二层帧后，将帧头和 payload 格式化后作为一条完整日志入队。
 */
void App_EthDebug_Process(void)
{
  AppEthDebug_RawFrameInfo rx;
  static char raw_print_buf[320];
  int len;

  if (App_EthDebug_ReceiveRawFrame(&rx) == 0U)
  {
    return;
  }

  len = snprintf(raw_print_buf,
                 sizeof(raw_print_buf),
                 "Received raw frame\r\n"
                 "dst mac = %02X:%02X:%02X:%02X:%02X:%02X\r\n"
                 "src mac = %02X:%02X:%02X:%02X:%02X:%02X\r\n"
                 "eth type = 0x%04X\r\n"
                 "payload = %s\r\n",
                 rx.dst_mac[0], rx.dst_mac[1], rx.dst_mac[2],
                 rx.dst_mac[3], rx.dst_mac[4], rx.dst_mac[5],
                 rx.src_mac[0], rx.src_mac[1], rx.src_mac[2],
                 rx.src_mac[3], rx.src_mac[4], rx.src_mac[5],
                 rx.eth_type,
                 rx.payload);

  if (len <= 0)
  {
    return;
  }

  if ((uint32_t)len >= sizeof(raw_print_buf))
  {
    len = (int)(sizeof(raw_print_buf) - 1U);
  }

  (void)APP_UART_LOG_Write((const uint8_t *)raw_print_buf, (uint16_t)len);
}

/*
 * 功能：发送一帧测试帧并输出一次 LAN8720A 状态。
 * 说明：用于任务中按需周期性上报链路状态；默认任务当前只做收包处理。
 */
void App_EthDebug_ReportStatus(void)
{
  (void)App_EthDebug_SendRawTestFrame();
  App_EthDebug_SendPhyStatus();
}

/*
 * 功能：构造并发送一帧广播裸二层以太网测试帧。
 * 说明：使用本机 MAC 作为源 MAC，EtherType 为 APP_ETH_DEBUG_TEST_ETHERTYPE，payload 为固定测试字符串。
 */
uint8_t App_EthDebug_SendRawTestFrame(void)
{
  const uint8_t *local_mac = ETH_Raw_GetMacAddress();
  static const uint8_t dst_mac[6] = {
    0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU
  };
  static const uint8_t payload[] = "STM32 RAW ETH TEST";
  uint8_t frame[APP_ETH_DEBUG_MIN_FRAME_LEN] = {0U};
  uint32_t payload_len = (uint32_t)(sizeof(payload) - 1U);
  uint32_t frame_len = APP_ETH_DEBUG_HEADER_LEN + payload_len;

  if ((local_mac == NULL) || (frame_len > APP_ETH_DEBUG_MIN_FRAME_LEN))
  {
    return 0U;
  }

  if (ETH_Raw_Start() == 0U)
  {
    return 0U;
  }

  memcpy(&frame[0], dst_mac, sizeof(dst_mac));
  memcpy(&frame[6], local_mac, 6U);
  frame[12] = (uint8_t)((APP_ETH_DEBUG_TEST_ETHERTYPE >> 8) & 0xFFU);
  frame[13] = (uint8_t)(APP_ETH_DEBUG_TEST_ETHERTYPE & 0xFFU);
  memcpy(&frame[APP_ETH_DEBUG_HEADER_LEN], payload, payload_len);

  return ETH_Raw_Send(frame, APP_ETH_DEBUG_MIN_FRAME_LEN);
}

/*
 * 功能：把名称和值组合为一条完整调试行并写入 UART 日志队列。
 * 说明：name、字符值和 CRLF 在一次非阻塞入队操作中提交，不再拆分成多个 UART 事务。
 */
static void App_EthDebug_SendByteValue(const uint8_t *name, uint16_t name_len, uint8_t value)
{
  uint8_t line[APP_ETH_DEBUG_STATUS_LINE_MAX];
  uint16_t line_len;

  if ((name == NULL) ||
      (name_len > (APP_ETH_DEBUG_STATUS_LINE_MAX - 3U)))
  {
    return;
  }

  memcpy(line, name, name_len);
  line[name_len] = (uint8_t)('0' + (value ? 1U : 0U));
  line[name_len + 1U] = (uint8_t)'\r';
  line[name_len + 2U] = (uint8_t)'\n';
  line_len = (uint16_t)(name_len + 3U);

  (void)APP_UART_LOG_Write(line, line_len);
}

/*
 * 功能：读取并输出 LAN8720A 的基础状态。
 * 说明：通过 LAN8720_ReadID() 更新 PHY ID 有效标志，并读取链路状态后通过 UART 日志队列输出。
 */
static void App_EthDebug_SendPhyStatus(void)
{
  uint8_t link_up;

  (void)LAN8720_ReadID();
  link_up = LAN8720_IsLinkUp();

  App_EthDebug_SendByteValue((const uint8_t *)"LAN8720_IDValid=",
                             (uint16_t)(sizeof("LAN8720_IDValid=") - 1U),
                             LAN8720_IDValid);
  App_EthDebug_SendByteValue((const uint8_t *)"LAN8720_IsLinkUp=",
                             (uint16_t)(sizeof("LAN8720_IsLinkUp=") - 1U),
                             link_up);
}

/*
 * 功能：轮询接收并解析发给本机 MAC 的裸二层以太网调试帧。
 * 说明：仅接收 APP_ETH_DEBUG_PC_ETHERTYPE 或 APP_ETH_DEBUG_TEST_ETHERTYPE 类型的帧，并把帧头、帧长和 payload 填入 info。
 */
static uint8_t App_EthDebug_ReceiveRawFrame(AppEthDebug_RawFrameInfo *info)
{
  const uint8_t *local_mac = ETH_Raw_GetMacAddress();
  uint16_t frame_len;
  uint16_t payload_len;
  uint16_t payload_copy_len;
  uint16_t eth_type;

  if ((info == NULL) || (local_mac == NULL))
  {
    return 0U;
  }

  if (ETH_Raw_Start() == 0U)
  {
    return 0U;
  }

  frame_len = ETH_Raw_Receive(app_eth_debug_rx_frame, ETH_RAW_FRAME_MAX_LEN);
  if (frame_len < APP_ETH_DEBUG_HEADER_LEN)
  {
    return 0U;
  }

  if (memcmp(&app_eth_debug_rx_frame[0], local_mac, 6U) != 0)
  {
    return 0U;
  }

  eth_type = (uint16_t)(((uint16_t)app_eth_debug_rx_frame[12] << 8) |
                        app_eth_debug_rx_frame[13]);
  if ((eth_type != APP_ETH_DEBUG_PC_ETHERTYPE) &&
      (eth_type != APP_ETH_DEBUG_TEST_ETHERTYPE))
  {
    return 0U;
  }

  memset(info, 0, sizeof(*info));
  memcpy(info->dst_mac, &app_eth_debug_rx_frame[0], sizeof(info->dst_mac));
  memcpy(info->src_mac, &app_eth_debug_rx_frame[6], sizeof(info->src_mac));
  info->eth_type = eth_type;
  info->frame_len = frame_len;

  payload_len = (uint16_t)(frame_len - APP_ETH_DEBUG_HEADER_LEN);
  payload_copy_len = payload_len;
  if (payload_copy_len > APP_ETH_DEBUG_RX_PAYLOAD_MAX)
  {
    payload_copy_len = APP_ETH_DEBUG_RX_PAYLOAD_MAX;
  }

  memcpy(info->payload,
         &app_eth_debug_rx_frame[APP_ETH_DEBUG_HEADER_LEN],
         payload_copy_len);
  info->payload[payload_copy_len] = 0U;
  info->payload_len = payload_copy_len;

  return 1U;
}
