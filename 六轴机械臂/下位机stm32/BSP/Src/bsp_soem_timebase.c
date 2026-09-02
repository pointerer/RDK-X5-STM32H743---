#include "bsp_soem_timebase.h"

#include "stm32h7xx_hal.h"

#define SOEM_TIMEBASE_TIMER         TIM2
#define SOEM_TIMEBASE_IRQn          TIM2_IRQn
#define SOEM_TIMEBASE_IRQ_PRIORITY  15U
#define SOEM_TIMEBASE_HZ            1000000U

static volatile uint32_t soem_timebase_overflow = 0U;

/*
 * 获取 TIM2 的真实输入时钟频率。
 * STM32H7 中当 APB1 分频不为 1 时，挂在 APB1 上的定时器时钟等于 PCLK1 的 2 倍。
 */
static uint32_t BSP_SOEM_Timebase_GetTim2ClockHz(void)
{
   RCC_ClkInitTypeDef clkconfig;
   uint32_t flash_latency;
   uint32_t pclk1;

   HAL_RCC_GetClockConfig(&clkconfig, &flash_latency);

   pclk1 = HAL_RCC_GetPCLK1Freq();
   if (clkconfig.APB1CLKDivider == RCC_HCLK_DIV1)
   {
      return pclk1;
   }

   return pclk1 * 2U;//如果 APB1 分频不为 1，则定时器时钟是 PCLK1 的 2 倍
}

/*
 * 初始化 SOEM 专用微秒时基。
 * TIM2 被配置成 1MHz 递增计数，即 CNT 每增加 1 表示过去 1us。
 */
void BSP_SOEM_Timebase_Init(void)
{
   uint32_t tim_clock_hz;
   uint32_t prescaler;

   __HAL_RCC_TIM2_CLK_ENABLE();

   HAL_NVIC_DisableIRQ(SOEM_TIMEBASE_IRQn);
   SOEM_TIMEBASE_TIMER->CR1 = 0U;//清空控制寄存器
   SOEM_TIMEBASE_TIMER->DIER = 0U;//清空中断使能寄存器

   tim_clock_hz = BSP_SOEM_Timebase_GetTim2ClockHz();//获取 TIM2 输入时钟频率
   if (tim_clock_hz < SOEM_TIMEBASE_HZ)
   {
      tim_clock_hz = SOEM_TIMEBASE_HZ;
   }

   prescaler = (tim_clock_hz / SOEM_TIMEBASE_HZ) - 1U;

   soem_timebase_overflow = 0U;//清零高位溢出计数

   SOEM_TIMEBASE_TIMER->PSC = prescaler;
   SOEM_TIMEBASE_TIMER->ARR = 0xFFFFFFFFUL;
   SOEM_TIMEBASE_TIMER->CNT = 0U;
   SOEM_TIMEBASE_TIMER->EGR = TIM_EGR_UG;//生成更新事件，让刚写入的 PSC/ARR 立即装载生效
   SOEM_TIMEBASE_TIMER->SR = 0U;//清空状态寄存器，避免误触发溢出中断
   SOEM_TIMEBASE_TIMER->DIER = TIM_DIER_UIE;//使能更新中断，即溢出中断

   HAL_NVIC_SetPriority(SOEM_TIMEBASE_IRQn, SOEM_TIMEBASE_IRQ_PRIORITY, 0U);
   HAL_NVIC_EnableIRQ(SOEM_TIMEBASE_IRQn);

   SOEM_TIMEBASE_TIMER->CR1 = TIM_CR1_CEN;//使能定时器开始计数
}

/**
 * @brief  获取 SOEM 时基初始化以来的64位单调微秒计数
 *
 * @return
 * 返回 BSP_SOEM_Timebase_Init() 执行后经过的时间，单位：us
 *
 * @warning
 * 调用前必须完成 BSP_SOEM_Timebase_Init()；TIM2 及其更新中断由该时基专用，
 * 运行期间不得由其他模块停止或重新配置
 */
uint64_t BSP_SOEM_Timebase_GetUs(void)
{
   uint32_t high_a;
   uint32_t high_b;
   uint32_t low;

   do
   {
      high_a = soem_timebase_overflow;
      low = SOEM_TIMEBASE_TIMER->CNT;
      high_b = soem_timebase_overflow;
   } while (high_a != high_b);//如果读到的高位前后不一致，说明在读低位过程中发生了一次溢出，重试一次

   if ((SOEM_TIMEBASE_TIMER->SR & TIM_SR_UIF) != 0U)
   {
      low = SOEM_TIMEBASE_TIMER->CNT;
      if (low < 0x80000000UL)
      {
         high_a++;
      }
   }

   return (((uint64_t)high_a) << 32) | (uint64_t)low;
}

/**
 * @brief  基于 SOEM 64 位微秒时基执行阻塞延时
 *
 * @param[in] us 期望延时时间，单位：us
 *
 * @return
 * 无
 *
 * @warning
 * 调用前必须完成 BSP_SOEM_Timebase_Init()，且 TIM2 及其溢出中断必须持续运行；
 * 本函数采用忙等待且不会让出 CPU，时基停止时将永久阻塞。中断抢占可能使实际延时
 * 长于 us，不应使用该函数实现较长延时或在严格实时路径中频繁调用
 */
void BSP_SOEM_Timebase_DelayUs(uint32_t us)
{
   uint64_t start_us;

   start_us = BSP_SOEM_Timebase_GetUs();
   while ((BSP_SOEM_Timebase_GetUs() - start_us) < (uint64_t)us)
   {
      __NOP();
   }
}

/*
 * TIM2 溢出中断。
 * 每溢出一次表示低 32 位微秒计数回绕一次，这里只负责累加高位计数。
 */
void TIM2_IRQHandler(void)
{
   if ((SOEM_TIMEBASE_TIMER->SR & TIM_SR_UIF) != 0U)
   {
      SOEM_TIMEBASE_TIMER->SR &= ~TIM_SR_UIF;//清除溢出中断标志
      soem_timebase_overflow++;
   }
}
