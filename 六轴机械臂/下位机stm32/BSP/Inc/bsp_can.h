#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define BSP_CAN_MAX_DATA_LEN  64U

typedef enum
{
  BSP_CAN_OK = 0,
  BSP_CAN_INVALID_ARGUMENT,
  BSP_CAN_NOT_READY,
  BSP_CAN_BUSY,
  BSP_CAN_ERROR
} BSP_CAN_Result;

typedef enum
{
  BSP_CAN_ID_STANDARD = 0,
  BSP_CAN_ID_EXTENDED
} BSP_CAN_IdType;

typedef struct
{
  uint32_t id;                              /* 帧标识符 */
  uint8_t length;                           /* 有效数据长度 */
  uint8_t is_fd;                            /* 是否为 CAN FD 帧 */
  uint8_t bit_rate_switch;                  /* 是否启用位速率切换 */
  BSP_CAN_IdType id_type;                   /* 标准或扩展标识符 */
  uint8_t data[BSP_CAN_MAX_DATA_LEN];        /* 帧数据 */
} BSP_CAN_Frame;

typedef struct
{
  uint32_t standard_filter_id;
  uint32_t standard_filter_mask;
} BSP_CAN_Config;

typedef struct
{
  uint32_t rx_frames;
  uint32_t rx_bytes;
  uint32_t rx_fifo_full;
  uint32_t rx_fifo_lost;
  uint32_t rx_queue_dropped;
  uint32_t tx_requests;
  uint32_t tx_queued;
  uint32_t tx_completed;
  uint32_t tx_busy;
  uint32_t error_warning;
  uint32_t error_passive;
  uint32_t bus_off;
  uint32_t arbitration_errors;
  uint32_t data_errors;
  uint32_t hal_errors;
  uint32_t last_hal_error;
  uint32_t recovery_success;                /* Bus-Off 重启成功次数 */
  uint32_t recovery_failed;                 /* Bus-Off 重启失败次数 */
} BSP_CAN_Stats;

BSP_CAN_Result BSP_CAN_Init(const BSP_CAN_Config *config);
BSP_CAN_Result BSP_CAN_Start(void);
BSP_CAN_Result BSP_CAN_Stop(void);
BSP_CAN_Result BSP_CAN_Send(const BSP_CAN_Frame *frame);
bool BSP_CAN_TryReceive(BSP_CAN_Frame *frame);
uint32_t BSP_CAN_GetRxPending(void);
void BSP_CAN_Process(void);
void BSP_CAN_GetStats(BSP_CAN_Stats *stats);
void BSP_CAN_ResetStats(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CAN_H__ */
