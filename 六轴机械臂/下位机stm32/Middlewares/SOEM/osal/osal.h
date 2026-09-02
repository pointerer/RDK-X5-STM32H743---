#ifndef OSAL_H
#define OSAL_H

#include <stdint.h>

typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef int64_t int64;

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;

typedef uint8 boolean;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef EC_PRINT
#define EC_PRINT(...) ((void)0)
#endif

typedef struct
{
   int64 tv_sec;
   int32 tv_nsec;
} ec_timet;

typedef struct osal_mutex osal_mutext;

typedef struct
{
   ec_timet stop_time;
} osal_timert;

#if defined(__CC_ARM)
/* ARMCC5 requires nested anonymous unions in a __packed struct to be
 * explicitly packed too. Use pragma packing to keep SOEM frame layouts
 * byte-aligned without editing every protocol structure.
 */
#define OSAL_PACKED_BEGIN _Pragma("pack(push, 1)")
#define OSAL_PACKED
#define OSAL_PACKED_END _Pragma("pack(pop)")
#elif defined(__GNUC__)
#define OSAL_PACKED_BEGIN
#define OSAL_PACKED __attribute__((__packed__))
#define OSAL_PACKED_END
#else
#define OSAL_PACKED_BEGIN
#define OSAL_PACKED
#define OSAL_PACKED_END
#endif

#ifdef __cplusplus
extern "C" {
#endif

ec_timet osal_current_time(void);
void osal_usleep(uint32 usec);

void osal_timer_start(osal_timert *timer, uint32 timeout_usec);
boolean osal_timer_is_expired(osal_timert *timer);

osal_mutext *osal_mutex_create(void);
void osal_mutex_destroy(osal_mutext *mutex);
void osal_mutex_lock(osal_mutext *mutex);
void osal_mutex_unlock(osal_mutext *mutex);

#ifdef __cplusplus
}
#endif

#endif
