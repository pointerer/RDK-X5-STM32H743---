#ifndef ECAT_PDO_MAP_H
#define ECAT_PDO_MAP_H

#include "ECAT_log.h"
#include "soem/soem.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ECAT_PDO_MAP_MAX_ENTRIES 128U

typedef struct
{
  uint32 mapping_entry;
  uint32 direction_bit_offset;
  uint16 pdo_index;
  uint8 assign_subindex;
  uint8 mapping_subindex;
} ECAT_PdoMapEntry;

typedef struct
{
  ECAT_PdoMapEntry entries[ECAT_PDO_MAP_MAX_ENTRIES];
  uint32 total_bits;
  uint16 entry_count;
  uint8 complete;
} ECAT_PdoMapCache;

typedef struct
{
  ECAT_PdoMapCache rxpdo_map;
  ECAT_PdoMapCache txpdo_map;
  uint8 po2so_attempted;
  uint8 po2so_ok;
} ECAT_PdoMapContext;

void ECAT_PdoMapReset(ECAT_PdoMapContext *pdo_map);
void ECAT_PdoMapResetPo2SoResult(ECAT_PdoMapContext *pdo_map);
uint8 ECAT_PdoMapLoad(ecx_contextt *context,
                                ECAT_LogContext *log,
                                ECAT_PdoMapContext *pdo_map,
                                uint16 slave,
                                const char *stage);
int ECAT_PdoMapConfigurePo2So(ecx_contextt *context,
                                        ecx_contextt *expected_context,
                                        ECAT_LogContext *log,
                                        ECAT_PdoMapContext *pdo_map,
                                        uint16 config_slave,
                                        uint16 slave);
void ECAT_PdoMapPrintResolved(ECAT_LogContext *log,
                                        const uint8 *iomap,
                                        uint32 iomap_capacity,
                                        uint16 slave,
                                        const char *direction,
                                        const ECAT_PdoMapCache *cache,
                                        const uint8 *pdo_base,
                                        uint8 start_bit,
                                        uint32 mapped_bits,
                                        int iomap_size);

#ifdef __cplusplus
}
#endif

#endif /* ECAT_PDO_MAP_H */
