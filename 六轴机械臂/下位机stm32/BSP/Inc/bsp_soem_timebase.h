#ifndef __BSP_SOEM_TIMEBASE_H__
#define __BSP_SOEM_TIMEBASE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 初始化 SOEM 专用微秒时基。
 * 当前实现使用 TIM2 配成 1MHz 自由运行计数器，必须在 SystemClock_Config() 之后调用。
 */
void BSP_SOEM_Timebase_Init(void);

/*
 * 获取系统启动以来的 64 位微秒计数。
 * SOEM 的 osal_current_time()、超时判断和微秒延时都可以基于这个函数实现。
 */
uint64_t BSP_SOEM_Timebase_GetUs(void);

/*
 * 基于 SOEM 微秒时基做阻塞式微秒延时。
 * 第一版用于裸机/移植调试阶段，后续可按 FreeRTOS 调度情况优化。
 */
void BSP_SOEM_Timebase_DelayUs(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif
