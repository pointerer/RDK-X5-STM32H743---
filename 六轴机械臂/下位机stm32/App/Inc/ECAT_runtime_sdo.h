#ifndef ECAT_RUNTIME_SDO_H
#define ECAT_RUNTIME_SDO_H

#include "soem/soem.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ECAT_RUNTIME_SDO_RESPONSE_TIMEOUT_US 80000U
#define ECAT_RUNTIME_SDO_DATA_SIZE           4U

typedef enum
{
  ECAT_RUNTIME_SDO_JOB_IDLE = 0,
  ECAT_RUNTIME_SDO_JOB_READ_PENDING,
  ECAT_RUNTIME_SDO_JOB_WRITE_PENDING,
  ECAT_RUNTIME_SDO_JOB_RUNNING,
  ECAT_RUNTIME_SDO_JOB_DONE
} ECAT_RuntimeSdoJobState;

typedef struct
{
  volatile ECAT_RuntimeSdoJobState state;
  uint16 slave;
  uint16 index;
  uint8 subindex;
  uint8 data[ECAT_RUNTIME_SDO_DATA_SIZE];
  int size;
  int wkc;
} ECAT_RuntimeSdoJob;

/*
 * Blocking raw SDO helpers for a future low-priority worker. Never call these
 * functions from the 2 ms task that runs ecx_mbxhandler(). Data is not endian
 * converted. For reads, *size is input capacity and output actual byte count.
 */
int ECAT_RuntimeSdoRead(
  ecx_contextt *context,
  uint16 slave,
  uint16 index,
  uint8 subindex,
  void *data,
  int *size);
int ECAT_RuntimeSdoWrite(
  ecx_contextt *context,
  uint16 slave,
  uint16 index,
  uint8 subindex,
  const void *data,
  int size);

#ifdef __cplusplus
}
#endif

#endif /* ECAT_RUNTIME_SDO_H */
