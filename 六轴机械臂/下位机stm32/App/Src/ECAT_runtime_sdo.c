#include "ECAT_runtime_sdo.h"

int ECAT_RuntimeSdoRead(
  ecx_contextt *context,
  uint16 slave,
  uint16 index,
  uint8 subindex,
  void *data,
  int *size)
{
  if ((context == 0) || (context->slavecount <= 0) || (slave == 0U) ||
      (slave >= EC_MAXSLAVE) || ((int)slave > context->slavecount) ||
      (data == 0) ||
      (size == 0) || (*size <= 0))
  {
    return 0;
  }

  return ecx_SDOread(context,
                     slave,
                     index,
                     subindex,
                     FALSE,
                     size,
                     data,
                     (int)ECAT_RUNTIME_SDO_RESPONSE_TIMEOUT_US);
}

int ECAT_RuntimeSdoWrite(
  ecx_contextt *context,
  uint16 slave,
  uint16 index,
  uint8 subindex,
  const void *data,
  int size)
{
  if ((context == 0) || (context->slavecount <= 0) || (slave == 0U) ||
      (slave >= EC_MAXSLAVE) || ((int)slave > context->slavecount) ||
      (data == 0) ||
      (size <= 0))
  {
    return 0;
  }

  return ecx_SDOwrite(context,
                      slave,
                      index,
                      subindex,
                      FALSE,
                      size,
                      data,
                      (int)ECAT_RUNTIME_SDO_RESPONSE_TIMEOUT_US);
}
