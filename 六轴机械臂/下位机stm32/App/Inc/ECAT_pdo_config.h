#ifndef ECAT_PDO_CONFIG_H
#define ECAT_PDO_CONFIG_H

#include "ECAT.h"
#include "ECAT_log.h"
#include "ECAT_pdo_map.h"
#include "soem/soem.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  ECAT_PDO_CONFIG_RESULT_OK = 0,
  ECAT_PDO_CONFIG_RESULT_ASSIGN_SLAVE_MISSING,
  ECAT_PDO_CONFIG_RESULT_COE_REQUIRED,
  ECAT_PDO_CONFIG_RESULT_IOMAP_SLAVE_MISSING,
  ECAT_PDO_CONFIG_RESULT_IOMAP_CHECK_FAILED,
  ECAT_PDO_CONFIG_RESULT_CYCLIC_MBX_PREREQUISITE_FAILED,
  ECAT_PDO_CONFIG_RESULT_CYCLIC_MBX_ENABLE_FAILED
} ECAT_PdoConfigResult;

typedef struct
{
  ECAT_PdoMapContext pdo_map[ECAT_SLAVE_COUNT];
} ECAT_PdoConfigContext;

void ECAT_PdoConfigReset(ECAT_PdoConfigContext *pdo_config);
void ECAT_PdoConfigResetPo2SoResult(
  ECAT_PdoConfigContext *pdo_config);
void ECAT_PdoConfigRegisterPo2So(
  ecx_contextt *context,
  ECAT_LogContext *log,
  int (*po2so_config)(ecx_contextt *context, uint16 slave));
ECAT_PdoConfigResult ECAT_PdoConfigReadAssignment(
  ecx_contextt *context,
  ECAT_LogContext *log,
  ECAT_PdoConfigContext *pdo_config);
ECAT_PdoConfigResult ECAT_PdoConfigMapGroup(
  ecx_contextt *context,
  ECAT_LogContext *log,
  ECAT_PdoConfigContext *pdo_config,
  uint8 *iomap,
  uint32 iomap_capacity);
ECAT_PdoConfigResult ECAT_PdoConfigEnableCyclicMailbox(
  ecx_contextt *context,
  ECAT_LogContext *log);
int ECAT_PdoConfigConfigurePo2So(
  ecx_contextt *context,
  ecx_contextt *expected_context,
  ECAT_LogContext *log,
  ECAT_PdoConfigContext *pdo_config,
  uint16 slave);
const char *ECAT_PdoConfigResultReason(
  ECAT_PdoConfigResult result);

#ifdef __cplusplus
}
#endif

#endif /* ECAT_PDO_CONFIG_H */
