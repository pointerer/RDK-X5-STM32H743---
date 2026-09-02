/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_key.h"
#include "bsp_can.h"
#include "ECAT.h"
#include "app_can.h"
#include "app_uart_log.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_SOEM_TASK_CYCLE_TICKS  pdMS_TO_TICKS(2U)
#define APP_SOEM_TASK_STACK_BYTES  (1024U * 4U)
#define APP_SOEM_TASK_PRIORITY     ((osPriority_t)osPriorityHigh)
#define APP_UART_LOG_TASK_STACK_BYTES  (1024U * 2U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t txPdoReportTaskHandle;
const osThreadAttr_t txPdoReportTask_attributes = {
  .name = "txPdoReport",
  .stack_size = 1024 * 2,
  .priority = (osPriority_t) osPriorityLow,
};
osThreadId_t sdoWorkerTaskHandle;
const osThreadAttr_t sdoWorkerTask_attributes = {
  .name = "sdoWorker",
  .stack_size = 1024 * 2,
  .priority = (osPriority_t) osPriorityLow,
};
osThreadId_t canServiceTaskHandle;
const osThreadAttr_t canServiceTask_attributes = {
  .name = "canService",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t uartLogTaskHandle;
static StaticTask_t uartLogTaskControlBlock;
static StackType_t uartLogTaskStack[
  APP_UART_LOG_TASK_STACK_BYTES / sizeof(StackType_t)];
const osThreadAttr_t uartLogTask_attributes = {
  .name = "uartLog",
  .cb_mem = &uartLogTaskControlBlock,
  .cb_size = sizeof(uartLogTaskControlBlock),
  .stack_mem = uartLogTaskStack,
  .stack_size = sizeof(uartLogTaskStack),
  .priority = (osPriority_t) osPriorityLow,
};
volatile uint32_t ecat_dbg_delay_blocked;
volatile uint32_t ecat_dbg_delay_immediate;
volatile uint32_t ecat_dbg_can_delay_blocked;
volatile uint32_t ecat_dbg_can_delay_immediate;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartTxPdoReportTask(void *argument);
void StartSdoWorkerTask(void *argument);
void StartCanServiceTask(void *argument);
void StartUartLogTask(void *argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* Decode and print the periodic TxPDO snapshot outside the 2 ms task. */
  txPdoReportTaskHandle = osThreadNew(StartTxPdoReportTask, NULL, &txPdoReportTask_attributes);
  sdoWorkerTaskHandle = osThreadNew(StartSdoWorkerTask, NULL, &sdoWorkerTask_attributes);
  /* CAN任务处理收发、命令超时监测和Bus-Off恢复。 */
  canServiceTaskHandle = osThreadNew(StartCanServiceTask, NULL, &canServiceTask_attributes);
  if (APP_UART_LOG_Init() != 0U)
  {
    uartLogTaskHandle = osThreadNew(StartUartLogTask, NULL, &uartLogTask_attributes);
  }
  /* Queue startup diagnostics before the scheduler starts the log consumer. */
  ECAT_Init();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  TickType_t last_wake_tick = 0U;
  TickType_t wake_tick;
  TickType_t delay_entry_tick;
  TickType_t delay_exit_tick;
  uint8_t op_cycle_active = 0U;
  uint8_t enable_request_accepted = 0U;

  (void)argument;

  /* Infinite loop */
  for(;;)
  {
    BSP_Key_Process2ms();
    ECAT_Process();

    if (ECAT_GetState() == ECAT_STATE_OPERATIONAL)
    {
      if ((enable_request_accepted == 0U) &&
          (ECAT_RequestEnableOperation() != 0U))
      {
        enable_request_accepted = 1U;
      }

      if (op_cycle_active == 0U)
      {
        last_wake_tick = xTaskGetTickCount();
        op_cycle_active = 1U;
      }

      delay_entry_tick = xTaskGetTickCount();
      vTaskDelayUntil(&last_wake_tick, APP_SOEM_TASK_CYCLE_TICKS);
      delay_exit_tick = xTaskGetTickCount();

      if (delay_exit_tick == delay_entry_tick)
      {
        ecat_dbg_delay_immediate++;
      }
      else
      {
        ecat_dbg_delay_blocked++;
      }

      /* Drop an overdue release instead of sending catch-up frames. */
      wake_tick = xTaskGetTickCount();
      if (wake_tick != last_wake_tick)
      {
        last_wake_tick = wake_tick;
      }
    }
    else
    {
      op_cycle_active = 0U;
      enable_request_accepted = 0U;
      vTaskDelay(pdMS_TO_TICKS(1U));
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartTxPdoReportTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    ECAT_ProcessTxPdoReport();
    vTaskDelay(pdMS_TO_TICKS(1U));
  }
}

void StartSdoWorkerTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    ECAT_ProcessRuntimeSdoWorker();
    vTaskDelay(pdMS_TO_TICKS(1U));
  }
}

void StartCanServiceTask(void *argument)
{
  TickType_t last_wake_tick = xTaskGetTickCount();
  TickType_t delay_entry_tick;
  TickType_t delay_exit_tick;
  uint32_t current_tick_ms;

  (void)argument;

  for (;;)
  {
    /* 依次执行维护、收包解析、超时判断和非阻塞周期发送。 */
    BSP_CAN_Process();
    APP_CAN_ProcessRx();
    current_tick_ms = HAL_GetTick();
    APP_CAN_CheckTimeout(current_tick_ms);
    APP_CAN_ProcessTx(current_tick_ms);

    delay_entry_tick = xTaskGetTickCount();
    vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(1U));
    delay_exit_tick = xTaskGetTickCount();

    if (delay_exit_tick == delay_entry_tick)
    {
      ecat_dbg_can_delay_immediate++;
    }
    else
    {
      ecat_dbg_can_delay_blocked++;
    }

    /* Drop overdue CAN service releases instead of running catch-up loops. */
    if (delay_exit_tick != last_wake_tick)
    {
      last_wake_tick = delay_exit_tick;
    }
  }
}

void StartUartLogTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    APP_UART_LOG_Process();
  }
}

/* USER CODE END Application */

