#ifndef ECAT_LOG_H
#define ECAT_LOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ECAT_LOG_BUFFER_SIZE 192U
#define ECAT_LOG_BLOCK_BUFFER_SIZE 384U

typedef struct
{
  char buffer[ECAT_LOG_BUFFER_SIZE]; /* 兼容保留；实际格式化缓冲区位于调用栈。 */
} ECAT_LogContext;

void ECAT_LogPrintf(ECAT_LogContext *context,
                             const char *fmt,
                             ...);
uint8_t ECAT_LogWriteBlock(ECAT_LogContext *context,
                           const char *data,
                           uint16_t length);
void ECAT_LogSectionLine(ECAT_LogContext *context);

#ifdef __cplusplus
}
#endif

#endif /* ECAT_LOG_H */
