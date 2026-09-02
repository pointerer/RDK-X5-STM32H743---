#ifndef BSP_KEY_H
#define BSP_KEY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  BSP_KEY_1 = 0,
  BSP_KEY_2,
  BSP_KEY_COUNT
} BSP_KeyId;

typedef enum
{
  BSP_KEY_RELEASED = 0,
  BSP_KEY_PRESSED = 1
} BSP_KeyState;

void BSP_Key_Init(void);
void BSP_Key_Process2ms(void);
BSP_KeyState BSP_Key_GetState(BSP_KeyId key);
uint8_t BSP_Key_TakePressEvent(BSP_KeyId key);

#ifdef __cplusplus
}
#endif

#endif /* BSP_KEY_H */
