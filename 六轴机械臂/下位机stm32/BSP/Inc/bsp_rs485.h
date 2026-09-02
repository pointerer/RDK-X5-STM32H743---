#ifndef __BSP_RS485_H__
#define __BSP_RS485_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

HAL_StatusTypeDef RS485_Send(const uint8_t *data, uint16_t len, uint32_t timeout);
HAL_StatusTypeDef RS485_Receive(uint8_t *data, uint16_t len, uint32_t timeout);
uint16_t RS485_ReceiveFrame(uint8_t *buf, uint16_t max_len, uint32_t first_timeout, uint32_t byte_timeout);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_RS485_H__ */
