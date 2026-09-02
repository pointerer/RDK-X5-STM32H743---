#include "bsp_rs485.h"

#include "usart.h"
#include <stddef.h>

/*
 * 功能：通过 UART8 发送一段 RS485 数据。
 * 说明：对输入指针和长度做基本检查后，调用 HAL_UART_Transmit() 按阻塞方式发送指定长度的数据。
 */
HAL_StatusTypeDef RS485_Send(const uint8_t *data, uint16_t len, uint32_t timeout)
{
  if ((data == NULL) || (len == 0U))
  {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit(&huart8, data, len, timeout);
}

/*
 * 功能：通过 UART8 接收指定长度的 RS485 数据。
 * 说明：该函数按固定长度接收，只有在超时时间内收到 len 个字节时才返回 HAL_OK。
 */
HAL_StatusTypeDef RS485_Receive(uint8_t *data, uint16_t len, uint32_t timeout)
{
  if ((data == NULL) || (len == 0U))
  {
    return HAL_ERROR;
  }

  return HAL_UART_Receive(&huart8, data, len, timeout);
}

/*
 * 功能：按字节间隔超时方式接收一帧 RS485 数据。
 * 说明：先用 first_timeout 等待首字节，收到后继续用 byte_timeout 接收后续字节；后续接收超时或达到 max_len 时结束，并返回实际接收长度。
 */
uint16_t RS485_ReceiveFrame(uint8_t *buf, uint16_t max_len, uint32_t first_timeout, uint32_t byte_timeout)
{
  uint16_t len = 0U;

  if ((buf == NULL) || (max_len == 0U))
  {
    return 0U;
  }

  if (HAL_UART_Receive(&huart8, &buf[len], 1U, first_timeout) != HAL_OK)
  {
    return 0U;
  }

  len++;

  while (len < max_len)
  {
    if (HAL_UART_Receive(&huart8, &buf[len], 1U, byte_timeout) != HAL_OK)
    {
      break;
    }

    len++;
  }

  return len;
}
