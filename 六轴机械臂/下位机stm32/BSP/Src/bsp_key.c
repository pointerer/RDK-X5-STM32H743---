#include "bsp_key.h"

#include "stm32h7xx_hal.h"

#define BSP_KEY_SCAN_INTERVAL_MS  2U
#define BSP_KEY_DEBOUNCE_SAMPLES  10U

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
} BSP_KeyHardware;

typedef struct
{
  volatile uint8_t stable_state;
  uint8_t sample_state;
  uint8_t debounce_count;
  volatile uint8_t press_event;
} BSP_KeyContext;

/* KEY1=PI8, KEY2=PC13. Both inputs are active-low. */
static const BSP_KeyHardware BSP_KeyHardwareTable[BSP_KEY_COUNT] =
{
  {GPIOI, GPIO_PIN_8},
  {GPIOC, GPIO_PIN_13}
};

/* Each key owns an independent debounce state and press event. */
static BSP_KeyContext BSP_KeyContexts[BSP_KEY_COUNT];
static uint32_t BSP_KeyLastProcessTick;

static uint8_t BSP_KeyReadPressed(BSP_KeyId key)
{
  return (HAL_GPIO_ReadPin(BSP_KeyHardwareTable[key].port,
                           BSP_KeyHardwareTable[key].pin) == GPIO_PIN_RESET) ?
         1U : 0U;
}

void BSP_Key_Init(void)
{
  uint32_t key;
  uint8_t pressed;

  for (key = 0U; key < (uint32_t)BSP_KEY_COUNT; ++key)
  {
    pressed = BSP_KeyReadPressed((BSP_KeyId)key);
    BSP_KeyContexts[key].stable_state = pressed;
    BSP_KeyContexts[key].sample_state = pressed;
    BSP_KeyContexts[key].debounce_count = 0U;
    BSP_KeyContexts[key].press_event = 0U;
  }

  BSP_KeyLastProcessTick = HAL_GetTick();
}

void BSP_Key_Process2ms(void)
{
  uint32_t now;
  uint32_t key;
  uint8_t pressed;
  BSP_KeyContext *context;

  now = HAL_GetTick();
  if ((uint32_t)(now - BSP_KeyLastProcessTick) < BSP_KEY_SCAN_INTERVAL_MS)
  {
    return;
  }
  BSP_KeyLastProcessTick = now;

  for (key = 0U; key < (uint32_t)BSP_KEY_COUNT; ++key)
  {
    context = &BSP_KeyContexts[key];
    pressed = BSP_KeyReadPressed((BSP_KeyId)key);

    if (pressed != context->sample_state)
    {
      context->sample_state = pressed;
      context->debounce_count = 1U;
    }
    else if (context->debounce_count < BSP_KEY_DEBOUNCE_SAMPLES)
    {
      ++context->debounce_count;
    }
    else
    {
      /* The current level has already passed debounce processing. */
    }

    if ((context->debounce_count >= BSP_KEY_DEBOUNCE_SAMPLES) &&
        (context->stable_state != context->sample_state))
    {
      context->stable_state = context->sample_state;
      if (context->stable_state == (uint8_t)BSP_KEY_PRESSED)
      {
        context->press_event = 1U;
      }
    }
  }
}

BSP_KeyState BSP_Key_GetState(BSP_KeyId key)
{
  if ((uint32_t)key >= (uint32_t)BSP_KEY_COUNT)
  {
    return BSP_KEY_RELEASED;
  }

  return (BSP_KeyContexts[key].stable_state != 0U) ?
         BSP_KEY_PRESSED : BSP_KEY_RELEASED;
}

uint8_t BSP_Key_TakePressEvent(BSP_KeyId key)
{
  uint32_t primask;
  uint8_t event;

  if ((uint32_t)key >= (uint32_t)BSP_KEY_COUNT)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  event = BSP_KeyContexts[key].press_event;
  BSP_KeyContexts[key].press_event = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }

  return event;
}
