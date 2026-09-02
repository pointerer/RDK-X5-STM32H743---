/*
 * 文件作用：
 * 本文件封装 LAN8720A 以太网 PHY 的基础控制接口。
 * 主要负责通过 ETH_RST 引脚复位 PHY、通过 MDIO/MDC 读取 PHY ID
 * 和链路状态、配置 PHY 工作模式并执行软复位。
 */
#include "bsp_lan8720.h"

#include "eth.h"
#include "main.h"

uint32_t LAN8720_ID1 = 0U;
uint32_t LAN8720_ID2 = 0U;
uint8_t LAN8720_IDValid = 0U;
uint32_t LAN8720_Reg18Before = 0U;
uint32_t LAN8720_Reg18Written = 0U;
uint32_t LAN8720_Reg18AfterReset = 0U;
uint32_t LAN8720_BMCRAfterReset = 0U;

/*
 * 功能：通过 ETH_RST 引脚对 LAN8720A 执行一次硬件复位。
 * 说明：LAN8720A 的 nRST 为低电平有效，先拉低再拉高，使 PHY 重新采样硬件配置脚并重新输出 RMII REF_CLK。
 */
void LAN8720_Reset(void)
{
  HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(10U);

  HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(1000U);
}

/*
 * 功能：读取 LAN8720A 的 PHY ID1/ID2 寄存器，确认 STM32 能通过 MDIO/MDC 访问 PHY。
 * 说明：读到的 ID 会保存到 LAN8720_ID1 和 LAN8720_ID2，LAN8720_IDValid 表示 ID 是否有效。
 */
uint8_t LAN8720_ReadID(void)
{
  uint32_t id1 = 0U;
  uint32_t id2 = 0U;

  if (HAL_ETH_ReadPHYRegister(&heth, LAN8720_PHY_ADDRESS, LAN8720_PHY_ID1_REG, &id1) != HAL_OK)
  {
    LAN8720_IDValid = 0U;
    return 0U;
  }

  if (HAL_ETH_ReadPHYRegister(&heth, LAN8720_PHY_ADDRESS, LAN8720_PHY_ID2_REG, &id2) != HAL_OK)
  {
    LAN8720_IDValid = 0U;
    return 0U;
  }

  LAN8720_ID1 = id1;
  LAN8720_ID2 = id2;

  if (((id1 == 0x0000U) && (id2 == 0x0000U)) ||
      ((id1 == 0xFFFFU) && (id2 == 0xFFFFU)))
  {
    LAN8720_IDValid = 0U;
    return 0U;
  }

  LAN8720_IDValid = 1U;
  return 1U;
}

/**
 * @brief  读取 LAN8720A 基本模式状态寄存器（BSR）
 *
 * @param[out] bsr   BSR 寄存器值输出地址
 *
 * @return
 * 成功返回1，输出参数为空或 PHY 寄存器读取失败返回0
 *
 * @warning
 * 调用该函数前必须完成 MX_ETH_Init()；读取失败时不会更新 *bsr
 */
uint8_t LAN8720_ReadBSR(uint32_t *bsr)
{
  uint32_t value = 0U;

  if (bsr == 0)
  {
    return 0U;
  }

  if (HAL_ETH_ReadPHYRegister(&heth, LAN8720_PHY_ADDRESS, LAN8720_BSR_REG, &value) != HAL_OK)
  {
    return 0U;
  }

  *bsr = value;
  return 1U;
}

/**
 * @brief  检查 LAN8720A PHY 链路是否已建立
 *
 * @return
 * 链路已建立返回1，链路断开或 PHY 寄存器读取失败返回0
 *
 * @warning
 * 调用该函数前必须完成 MX_ETH_Init()；返回0时无法区分链路断开与 MDIO 读取失败
 */
uint8_t LAN8720_IsLinkUp(void)
{
  uint32_t bsr = 0U;

  if (HAL_ETH_ReadPHYRegister(&heth, LAN8720_PHY_ADDRESS, LAN8720_BSR_REG, &bsr) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_ETH_ReadPHYRegister(&heth, LAN8720_PHY_ADDRESS, LAN8720_BSR_REG, &bsr) != HAL_OK)
  {
    return 0U;
  }

  return ((bsr & LAN8720_BSR_LINK) != 0U) ? 1U : 0U;
}

/*
 * 功能：配置 LAN8720A Special Modes 寄存器并执行 PHY 软复位。
 * 说明：写入新的工作模式时保留 PHY 地址位，随后置位 BMCR Reset 并等待复位完成，同时记录关键寄存器值用于调试观察。
 */
uint8_t LAN8720_SetModeAndSoftReset(uint8_t mode)
{
  uint32_t bmcr = 0U;
  uint32_t reg18 = 0U;
  uint32_t reg18_write;
  uint32_t tickstart;

  if (HAL_ETH_ReadPHYRegister(&heth,
                              LAN8720_PHY_ADDRESS,
                              LAN8720_SPECIAL_MODES_REG,
                              &reg18) != HAL_OK)
  {
    return 0U;
  }

  LAN8720_Reg18Before = reg18;

  reg18_write = LAN8720_SPECIAL_MODES_WRITE_1 |
                (reg18 & LAN8720_SPECIAL_MODES_PHYAD_MASK) |
                ((((uint32_t)mode) << LAN8720_SPECIAL_MODES_MODE_SHIFT) &
                 LAN8720_SPECIAL_MODES_MODE_MASK);
  LAN8720_Reg18Written = reg18_write;

  if (HAL_ETH_WritePHYRegister(&heth,
                               LAN8720_PHY_ADDRESS,
                               LAN8720_SPECIAL_MODES_REG,
                               reg18_write) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_ETH_ReadPHYRegister(&heth,
                              LAN8720_PHY_ADDRESS,
                              LAN8720_BMCR_REG,
                              &bmcr) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_ETH_WritePHYRegister(&heth,
                               LAN8720_PHY_ADDRESS,
                               LAN8720_BMCR_REG,
                               bmcr | LAN8720_BMCR_RESET) != HAL_OK)
  {
    return 0U;
  }

  tickstart = HAL_GetTick();
  do
  {
    if (HAL_ETH_ReadPHYRegister(&heth,
                                LAN8720_PHY_ADDRESS,
                                LAN8720_BMCR_REG,
                                &bmcr) != HAL_OK)
    {
      return 0U;
    }

    LAN8720_BMCRAfterReset = bmcr;
    if ((bmcr & LAN8720_BMCR_RESET) == 0U)
    {
      (void)HAL_ETH_ReadPHYRegister(&heth,
                                    LAN8720_PHY_ADDRESS,
                                    LAN8720_SPECIAL_MODES_REG,
                                    &LAN8720_Reg18AfterReset);
      return 1U;
    }
  } while ((HAL_GetTick() - tickstart) <= LAN8720_SOFT_RESET_TIMEOUT_MS);

  return 0U;
}
