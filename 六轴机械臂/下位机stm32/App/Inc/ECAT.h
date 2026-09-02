#ifndef ECAT_H
#define ECAT_H

#include <stdint.h>

#include "EtherCAT/app_ethercat_pdo.h"

#define ECAT_FIRST_SLAVE 1U
#define ECAT_SLAVE_COUNT 3U
#define ECAT_LAST_SLAVE  (ECAT_FIRST_SLAVE + ECAT_SLAVE_COUNT - 1U)

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  ECAT_STATE_IDLE = 0,
  ECAT_STATE_WAIT_LINK,
  ECAT_STATE_SOEM_INIT,
  ECAT_STATE_PROBE,
  ECAT_STATE_DISCOVER,
  ECAT_STATE_PREOP,
  ECAT_STATE_SDO,
  ECAT_STATE_PDO_ASSIGN,
  ECAT_STATE_MAP,
  ECAT_STATE_SAFEOP,
  ECAT_STATE_OP_REQUEST,
  ECAT_STATE_OPERATIONAL,
  ECAT_STATE_DONE,
  ECAT_STATE_ERROR
} ECAT_State;

typedef struct
{
  AppEtherCAT_ServoTxPdo slave[ECAT_SLAVE_COUNT]; /* 三个从站的双轴TxPDO快照 */
  ECAT_State state;                              /* 主站应用状态 */
  uint32_t cycle_counter;                        /* 成功PDO周期计数 */
  uint16_t working_counter;                      /* 当前有效WKC */
  uint8_t valid_slave_mask;                      /* 数据有效从站掩码 */
  uint8_t operational_slave_mask;                /* OP状态从站掩码 */
} ECAT_FeedbackSnapshot;

void ECAT_Init(void);
void ECAT_Process(void);
void ECAT_ProcessTxPdoReport(void);
uint8_t ECAT_IsRuntimeSdoWorkPending(void);
void ECAT_ProcessRuntimeSdoWorker(void);
uint8_t ECAT_RequestEnableOperation(void);
ECAT_State ECAT_GetState(void);
uint8_t ECAT_GetFeedbackSnapshot(ECAT_FeedbackSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* ECAT_H */
