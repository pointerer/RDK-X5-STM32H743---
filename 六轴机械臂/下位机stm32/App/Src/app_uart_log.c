#include "app_uart_log.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "usart.h"
#include <string.h>

#define APP_UART_LOG_TX_EVENT_NONE      0U
#define APP_UART_LOG_TX_EVENT_COMPLETE  1U
#define APP_UART_LOG_TX_EVENT_ERROR     2U
#define APP_UART_LOG_TX_TIMEOUT_TICKS   pdMS_TO_TICKS(100U)

/* Consumed later by the single log-task state machine. */
static volatile uint8_t app_uart_log_tx_event = APP_UART_LOG_TX_EVENT_NONE;

static StaticQueue_t app_uart_log_queue_control_block;
static uint8_t app_uart_log_queue_storage[
  APP_UART_LOG_QUEUE_DEPTH * sizeof(APP_UART_LOG_Message)];
static const osMessageQueueAttr_t app_uart_log_queue_attributes = {
  .name = "uartLogQueue",
  .cb_mem = &app_uart_log_queue_control_block,
  .cb_size = sizeof(app_uart_log_queue_control_block),
  .mq_mem = app_uart_log_queue_storage,
  .mq_size = sizeof(app_uart_log_queue_storage)
};

static StaticSemaphore_t app_uart_log_semaphore_control_block;
static const osSemaphoreAttr_t app_uart_log_semaphore_attributes = {
  .name = "uartLogTxDone",
  .cb_mem = &app_uart_log_semaphore_control_block,
  .cb_size = sizeof(app_uart_log_semaphore_control_block)
};

static osMessageQueueId_t app_uart_log_queue;
static osSemaphoreId_t app_uart_log_tx_event_semaphore;

uint8_t APP_UART_LOG_Init(void)
{
  if ((app_uart_log_queue != NULL) &&
      (app_uart_log_tx_event_semaphore != NULL))
  {
    return 1U;
  }

  if ((app_uart_log_queue != NULL) ||
      (app_uart_log_tx_event_semaphore != NULL))
  {
    return 0U;
  }

  app_uart_log_queue = osMessageQueueNew(APP_UART_LOG_QUEUE_DEPTH,
                                         sizeof(APP_UART_LOG_Message),
                                         &app_uart_log_queue_attributes);
  if (app_uart_log_queue == NULL)
  {
    return 0U;
  }

  app_uart_log_tx_event_semaphore =
    osSemaphoreNew(1U, 0U, &app_uart_log_semaphore_attributes);
  if (app_uart_log_tx_event_semaphore == NULL)
  {
    (void)osMessageQueueDelete(app_uart_log_queue);
    app_uart_log_queue = NULL;
    return 0U;
  }

  app_uart_log_tx_event = APP_UART_LOG_TX_EVENT_NONE;
  return 1U;
}

uint8_t APP_UART_LOG_Write(const uint8_t *data, uint16_t length)
{
  APP_UART_LOG_Message message = {0};

  if ((app_uart_log_queue == NULL) ||
      (data == NULL) ||
      (length == 0U) ||
      (length > APP_UART_LOG_MESSAGE_SIZE))
  {
    return 0U;
  }

  message.length = length;
  (void)memcpy(message.data, data, length);

  if (osMessageQueuePut(app_uart_log_queue, &message, 0U, 0U) != osOK)
  {
    return 0U;
  }

  return 1U;
}

void APP_UART_LOG_Process(void)
{
  APP_UART_LOG_Message message;
  osStatus_t wait_status;

  if ((app_uart_log_queue == NULL) ||
      (app_uart_log_tx_event_semaphore == NULL))
  {
    return;
  }

  if (osMessageQueueGet(app_uart_log_queue,
                        &message,
                        NULL,
                        osWaitForever) != osOK)
  {
    return;
  }

  if ((message.length == 0U) ||
      (message.length > APP_UART_LOG_MESSAGE_SIZE))
  {
    return;
  }

  while (osSemaphoreAcquire(app_uart_log_tx_event_semaphore, 0U) == osOK)
  {
    /* Drain a stale notification before starting a new transfer. */
  }

  app_uart_log_tx_event = APP_UART_LOG_TX_EVENT_NONE;
  __DMB();

  if (HAL_UART_Transmit_DMA(&huart8, message.data, message.length) != HAL_OK)
  {
    return;
  }

  wait_status = osSemaphoreAcquire(app_uart_log_tx_event_semaphore,
                                   APP_UART_LOG_TX_TIMEOUT_TICKS);
  __DMB();

  if ((wait_status != osOK) ||
      (app_uart_log_tx_event != APP_UART_LOG_TX_EVENT_COMPLETE))
  {
    (void)HAL_UART_AbortTransmit(&huart8);
  }
}

void APP_UART_LOG_TxCompleteFromIsr(void)
{
  app_uart_log_tx_event = APP_UART_LOG_TX_EVENT_COMPLETE;
  __DMB();

  if (app_uart_log_tx_event_semaphore != NULL)
  {
    (void)osSemaphoreRelease(app_uart_log_tx_event_semaphore);
  }
}

void APP_UART_LOG_TxErrorFromIsr(void)
{
  app_uart_log_tx_event = APP_UART_LOG_TX_EVENT_ERROR;
  __DMB();

  if (app_uart_log_tx_event_semaphore != NULL)
  {
    (void)osSemaphoreRelease(app_uart_log_tx_event_semaphore);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart8)
  {
    APP_UART_LOG_TxCompleteFromIsr();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart8)
  {
    APP_UART_LOG_TxErrorFromIsr();
  }
}
