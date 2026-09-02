#ifndef ECAT_DIAG_H
#define ECAT_DIAG_H

#include "ECAT_log.h"
#include "EtherCAT/app_ethercat_pdo.h"
#include "soem/soem.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint64_t last_phy_print_us;
} ECAT_DiagContext;

void ECAT_DiagReset(ECAT_DiagContext *diag);
void ECAT_DiagPrintTxPdo(ECAT_LogContext *log,
                                   uint16_t slave,
                                   const AppEtherCAT_ServoTxPdo *txpdo);
void ECAT_DiagPrintTxPdoAxis1(ECAT_LogContext *log,
                              uint16_t slave,
                              const AppEtherCAT_ServoTxPdo *txpdo);
void ECAT_DiagPrintInputsRaw(ECAT_LogContext *log,
                                      uint16_t slave,
                                      const uint8_t *inputs,
                                      uint32_t input_len,
                                      uint32_t expected_len);
void ECAT_DiagPrintMailbox(ECAT_LogContext *log,
                                     uint16_t slave,
                                     const ec_slavet *slave_info);
void ECAT_DiagPrintRx(ECAT_LogContext *log);
void ECAT_DiagPrintEth(ECAT_LogContext *log);
void ECAT_DiagProcessPhyMonitor(ECAT_DiagContext *diag,
                                          ECAT_LogContext *log,
                                          uint64_t print_interval_us);

#ifdef __cplusplus
}
#endif

#endif /* ECAT_DIAG_H */
