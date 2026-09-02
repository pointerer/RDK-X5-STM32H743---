#include "ECAT_log.h"

#include "app_uart_log.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#if (ECAT_LOG_BLOCK_BUFFER_SIZE > APP_UART_LOG_MESSAGE_SIZE)
#error "ECAT log block exceeds the UART log message capacity"
#endif

/**
 * @brief  格式化 EtherCAT 日志并将整条消息非阻塞写入 UART 日志队列
 *
 * @param[in]     context   日志上下文，用于保持现有调用接口
 * @param[in]     fmt       printf 风格的格式字符串
 * @param[in]     ...       与格式字符串匹配的可变参数
 *
 * @return
 * 无
 *
 * @warning
 * context 和 fmt 必须有效，且调用前必须完成 APP_UART_LOG_Init()；超出缓冲区容量的
 * 日志会被截断，队列未初始化或已满时本条日志会被丢弃，禁止在中断上下文调用
 */
void ECAT_LogPrintf(ECAT_LogContext *context,
                             const char *fmt,
                             ...)
{
  va_list args;
  char buffer[ECAT_LOG_BUFFER_SIZE];
  int len;

  if ((context == NULL) || (fmt == NULL))
  {
    return;
  }

  va_start(args, fmt);
  len = vsnprintf(buffer,
                  sizeof(buffer),
                  fmt,
                  args);
  va_end(args);

  if (len <= 0)
  {
    return;
  }

  if ((uint32_t)len >= sizeof(buffer))
  {
    len = (int)(sizeof(buffer) - 1U);
  }

  (void)APP_UART_LOG_Write((const uint8_t *)buffer,
                           (uint16_t)len);
}

uint8_t ECAT_LogWriteBlock(ECAT_LogContext *context,
                           const char *data,
                           uint16_t length)
{
  if ((context == NULL) ||
      (data == NULL) ||
      (length == 0U) ||
      (length > ECAT_LOG_BLOCK_BUFFER_SIZE))
  {
    return 0U;
  }

  return APP_UART_LOG_Write((const uint8_t *)data, length);
}

void ECAT_LogSectionLine(ECAT_LogContext *context)
{
  ECAT_LogPrintf(
    context,
    "\r\n[SOEM] ------------------------------------------------------------\r\n");
}
