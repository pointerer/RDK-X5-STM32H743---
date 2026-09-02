#ifndef APP_UART_LOG_H
#define APP_UART_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Maximum bytes retained for one complete log message. */
#define APP_UART_LOG_MESSAGE_SIZE  384U

/* Number of complete messages retained by the future RTOS queue. */
#define APP_UART_LOG_QUEUE_DEPTH   16U

typedef struct
{
  uint16_t length;                         /* Valid bytes in data[]. */
  uint8_t data[APP_UART_LOG_MESSAGE_SIZE]; /* Complete copied message. */
} APP_UART_LOG_Message;

/* Create the log service resources. Returns 1 on success, otherwise 0. */
uint8_t APP_UART_LOG_Init(void);

/*
 * Copy one complete message without blocking the calling control task.
 * Returns 1 only when queued; invalid, oversized or full-queue writes return 0.
 */
uint8_t APP_UART_LOG_Write(const uint8_t *data, uint16_t length);

/*
 * Process one queued message from the single UART log task.
 * This function may block; it must not run in a control task or ISR.
 */
void APP_UART_LOG_Process(void);

/* Notify the log service from the UART8 HAL callbacks. */
void APP_UART_LOG_TxCompleteFromIsr(void);
void APP_UART_LOG_TxErrorFromIsr(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_UART_LOG_H */
