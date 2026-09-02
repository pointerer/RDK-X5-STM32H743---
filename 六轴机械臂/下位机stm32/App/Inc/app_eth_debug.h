#ifndef __APP_ETH_DEBUG_H__
#define __APP_ETH_DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void App_EthDebug_Init(void);
void App_EthDebug_Process(void);
void App_EthDebug_ReportStatus(void);
uint8_t App_EthDebug_SendRawTestFrame(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ETH_DEBUG_H__ */
