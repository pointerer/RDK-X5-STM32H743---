#include "osal.h"
#include "bsp_soem_timebase.h"
#include "FreeRTOS.h"
#include "semphr.h"

struct osal_mutex
{
   SemaphoreHandle_t handle;
};

/*
 * SOEM 微秒级阻塞延时函数。
 *
 * SOEM 的超时和短延时参数单位是 us，因此这里直接读取 TIM2 微秒时基，
 * 通过忙等方式等待指定的微秒数过去。
 *
 * 当前阶段不使用 HAL_Delay()，因为 HAL_Delay() 的粒度是 1ms；
 * 也不使用 osDelay()，因为 osDelay() 依赖 FreeRTOS tick，同样是 ms 级。
 */
void osal_usleep(uint32 usec)
{
   uint64_t start_us;

   if (usec == 0U)
   {
      return;
   }

   start_us = BSP_SOEM_Timebase_GetUs();
   while ((BSP_SOEM_Timebase_GetUs() - start_us) < (uint64_t)usec)
   {
   }
}

/*
    * SOEM 需要一个单调递增的当前时间，用于错误时间戳和超时判断。
    * 底层 TIM2 时基返回的是系统启动以来的微秒数，这里转换成
    * SOEM 使用的 ec_timet 格式：秒 + 纳秒。
    */
ec_timet osal_current_time(void)
{
   uint64_t now_us;
   ec_timet now;

   now_us = BSP_SOEM_Timebase_GetUs();

   now.tv_sec = (int64)(now_us / 1000000ULL);
   now.tv_nsec = (int32)((now_us % 1000000ULL) * 1000ULL);

   return now;
}

/*
 * 在一个 ec_timet 时间点上增加指定的微秒数。
 * SOEM 的超时参数单位是 us，而 ec_timet 使用 sec + nsec 表示时间，
 * 所以这里负责完成单位转换，并处理 nsec 超过 1 秒时的进位。
 */
static ec_timet osal_time_add_us(ec_timet time, uint32 timeout_usec)
{
   time.tv_sec += (int64)(timeout_usec / 1000000U);
   time.tv_nsec += (int32)((timeout_usec % 1000000U) * 1000U);

   if (time.tv_nsec >= 1000000000L)
   {
      time.tv_sec += 1;
      time.tv_nsec -= 1000000000L;
   }

   return time;
}

/*
 * 启动一个 OSAL 定时器。
 * 这里保存的是“截止时间 stop_time”，而不是“开始时间 + 超时时长”两个值，
 * 这样后续判断是否超时的时候，只需要比较当前时间是否已经到达 stop_time。
 */
void osal_timer_start(osal_timert *timer, uint32 timeout_usec)
{
   if (timer == 0)
   {
      return;
   }

   timer->stop_time = osal_time_add_us(osal_current_time(), timeout_usec);
}

/*
 * 判断 OSAL 定时器是否已经超时。
 * SOEM 会在等待从站响应或状态切换时反复调用该函数；
 * 当前时间大于或等于 stop_time 时，表示等待时间已经用完。
 */
boolean osal_timer_is_expired(osal_timert *timer)
{
   ec_timet now;

   if (timer == 0)
   {
      return TRUE;
   }

   now = osal_current_time();

   if (now.tv_sec > timer->stop_time.tv_sec)
   {
      return TRUE;
   }

   if (now.tv_sec < timer->stop_time.tv_sec)
   {
      return FALSE;
   }

   return (now.tv_nsec >= timer->stop_time.tv_nsec) ? TRUE : FALSE;
}

osal_mutext *osal_mutex_create(void)
{
   osal_mutext *mutex;

   mutex = (osal_mutext *)pvPortMalloc(sizeof(*mutex));
   if (mutex == 0)
   {
      return 0;
   }

   mutex->handle = xSemaphoreCreateMutex();
   if (mutex->handle == 0)
   {
      vPortFree(mutex);
      return 0;
   }

   return mutex;
}

void osal_mutex_destroy(osal_mutext *mutex)
{
   if (mutex == 0)
   {
      return;
   }

   configASSERT(xPortIsInsideInterrupt() == pdFALSE);
   if (mutex->handle != 0)
   {
      vSemaphoreDelete(mutex->handle);
      mutex->handle = 0;
   }
   vPortFree(mutex);
}

void osal_mutex_lock(osal_mutext *mutex)
{
   BaseType_t result;

   configASSERT(xPortIsInsideInterrupt() == pdFALSE);
   configASSERT(mutex != 0);
   configASSERT((mutex != 0) && (mutex->handle != 0));
   if ((mutex == 0) || (mutex->handle == 0))
   {
      return;
   }

   result = xSemaphoreTake(mutex->handle, portMAX_DELAY);
   configASSERT(result == pdTRUE);
}

void osal_mutex_unlock(osal_mutext *mutex)
{
   BaseType_t result;

   configASSERT(xPortIsInsideInterrupt() == pdFALSE);
   configASSERT(mutex != 0);
   configASSERT((mutex != 0) && (mutex->handle != 0));
   if ((mutex == 0) || (mutex->handle == 0))
   {
      return;
   }

   result = xSemaphoreGive(mutex->handle);
   configASSERT(result == pdTRUE);
}
