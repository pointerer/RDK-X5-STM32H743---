#ifndef ECAT_SDO_H
#define ECAT_SDO_H

#include "ECAT_log.h"
#include "soem/soem.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8 ECAT_SdoReadU8(ecx_contextt *context,
                               ECAT_LogContext *log,
                               uint16 slave,
                               uint16 index,
                               uint8 subindex,
                               uint8 *value);
uint8 ECAT_SdoReadU16(ecx_contextt *context,
                                ECAT_LogContext *log,
                                uint16 slave,
                                uint16 index,
                                uint8 subindex,
                                uint16 *value);
uint8 ECAT_SdoReadU32(ecx_contextt *context,
                                ECAT_LogContext *log,
                                uint16 slave,
                                uint16 index,
                                uint8 subindex,
                                uint32 *value);
uint8 ECAT_SdoPdoWriteU8(ecx_contextt *context,
                                   ECAT_LogContext *log,
                                   uint16 slave,
                                   uint16 index,
                                   uint8 subindex,
                                   uint8 value);
uint8 ECAT_SdoPdoWriteU16(ecx_contextt *context,
                                    ECAT_LogContext *log,
                                    uint16 slave,
                                    uint16 index,
                                    uint8 subindex,
                                    uint16 value);
uint8 ECAT_SdoPdoWriteU32(ecx_contextt *context,
                                    ECAT_LogContext *log,
                                    uint16 slave,
                                    uint16 index,
                                    uint8 subindex,
                                    uint32 value);
uint8 ECAT_SdoPdoReadU8(ecx_contextt *context,
                                  ECAT_LogContext *log,
                                  uint16 slave,
                                  uint16 index,
                                  uint8 subindex,
                                  uint8 *value);
uint8 ECAT_SdoPdoReadU16(ecx_contextt *context,
                                   ECAT_LogContext *log,
                                   uint16 slave,
                                   uint16 index,
                                   uint8 subindex,
                                   uint16 *value);
uint8 ECAT_SdoPdoReadU32(ecx_contextt *context,
                                   ECAT_LogContext *log,
                                   uint16 slave,
                                   uint16 index,
                                   uint8 subindex,
                                   uint32 *value);

#ifdef __cplusplus
}
#endif

#endif /* ECAT_SDO_H */
