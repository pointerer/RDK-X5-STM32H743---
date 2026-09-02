#ifndef ECAT_RUNTIME_H
#define ECAT_RUNTIME_H

#include "ECAT.h"
#include "ECAT_log.h"
#include "ECAT_runtime_sdo.h"
#include "EtherCAT/app_ethercat_pdo.h"
#include "soem/soem.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ECAT_RUNTIME_SNAPSHOT_BUFFER_COUNT 2U

typedef enum
{
  ECAT_RUNTIME_RESULT_OK = 0,
  ECAT_RUNTIME_RESULT_SAFEOP_SLAVE_MISSING,
  ECAT_RUNTIME_RESULT_MAPPING_LEFT_PREOP,
  ECAT_RUNTIME_RESULT_SAFEOP_FAILED,
  ECAT_RUNTIME_RESULT_TXPDO_DECODE_FAILED,
  ECAT_RUNTIME_RESULT_SAFE_RXPDO_PREPARE_FAILED,
  ECAT_RUNTIME_RESULT_OP_REQUEST_FAILED,
  ECAT_RUNTIME_RESULT_OP_EXCHANGE_LOST
} ECAT_RuntimeResult;

typedef enum
{
  ECAT_RUNTIME_CW06_DISABLED = 0,
  ECAT_RUNTIME_CW06_WAIT_DELAY,
  ECAT_RUNTIME_CW06_WAIT_READY,
  ECAT_RUNTIME_CW06_DONE,
  ECAT_RUNTIME_CW06_ROLLBACK_ZERO,
  ECAT_RUNTIME_CW06_FAILED,
  ECAT_RUNTIME_CW07_WAIT_SWITCHED_ON,
  ECAT_RUNTIME_CW07_DONE,
  ECAT_RUNTIME_CW0F_WAIT_TARGET_STABLE,
  ECAT_RUNTIME_CW0F_WAIT_OPERATION_ENABLED,
  ECAT_RUNTIME_CW0F_DONE,
  ECAT_RUNTIME_CW0F_ROLLBACK_CW07,
  ECAT_RUNTIME_TEST,
  ECAT_RUNTIME_AXIS1_BRAKE_RELEASE,
  ECAT_RUNTIME_AXIS2_BRAKE_RELEASE
} ECAT_RuntimeDriveState;

typedef struct
{
  uint8 step;
  uint8 retry_count;
  uint16 current_slave;
  uint32 feedback_2023;
  uint32 feedback_2024;
  uint32 wait_start_us;
} ECAT_RuntimeBrakeContext;

typedef struct
{
  int expected_wkc;
  uint32 bad_wkc_continuous;
  uint32 pdo_cycle_count;          /* 完整成功的OP PDO交换累计数 */
  uint32 successful_pdo_count;
  uint32 txpdo_report_count;
  ECAT_RuntimeDriveState drive_state;
  uint32 state_cycle_count;
  uint32 target_stable_cycle_count;
  uint32 test_position_offset;
  uint8 test_completed;
  uint8 test_keys_armed;
  ECAT_RuntimeBrakeContext brake;
  ECAT_RuntimeSdoJob sdo_job;
  volatile uint8 enable_operation_request;
  ECAT_RuntimeDriveState
    txpdo_snapshot_drive_state[ECAT_RUNTIME_SNAPSHOT_BUFFER_COUNT];
  uint32 txpdo_snapshot_report[ECAT_RUNTIME_SNAPSHOT_BUFFER_COUNT];
  volatile uint32 txpdo_snapshot_publish_sequence;
  uint32 txpdo_snapshot_consumed_sequence;
  AppEtherCAT_ServoTxPdo
    txpdo_snapshot[ECAT_RUNTIME_SNAPSHOT_BUFFER_COUNT][ECAT_SLAVE_COUNT];
} ECAT_RuntimeContext;

void ECAT_RuntimeReset(ECAT_RuntimeContext *runtime);
void ECAT_RuntimeResetAfterError(ECAT_RuntimeContext *runtime);
uint8 ECAT_RuntimeRequestEnableOperation(ECAT_RuntimeContext *runtime);
void ECAT_RuntimeReadAllStates(ecx_contextt *context,
                                         ECAT_LogContext *log);
uint8 ECAT_RuntimeCheckPreOp(ecx_contextt *context,
                                      ECAT_LogContext *log);
ECAT_RuntimeResult ECAT_RuntimeEnterSafeOp(
  ecx_contextt *context,
  ECAT_LogContext *log,
  ECAT_RuntimeContext *runtime);
ECAT_RuntimeResult ECAT_RuntimeRequestOp(
  ecx_contextt *context,
  ECAT_LogContext *log,
  ECAT_RuntimeContext *runtime);
ECAT_RuntimeResult ECAT_RuntimeExchangeOperational(
  ecx_contextt *context,
  ECAT_LogContext *log,
  ECAT_RuntimeContext *runtime);
void ECAT_RuntimeProcessTxPdoReport(
  ECAT_LogContext *log,
  ECAT_RuntimeContext *runtime);
const char *ECAT_RuntimeResultReason(
  ECAT_RuntimeResult result);

#ifdef __cplusplus
}
#endif

#endif /* ECAT_RUNTIME_H */
