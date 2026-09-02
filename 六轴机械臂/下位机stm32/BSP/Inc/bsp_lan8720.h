#ifndef __BSP_LAN8720_H__
#define __BSP_LAN8720_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define LAN8720_PHY_ADDRESS                 0x00U    /* LAN8720A 在 MDIO 总线上的 PHY 地址。 */

#define LAN8720_BMCR_REG                    0x00U    /* PHY Basic Mode Control Register 寄存器地址。 */
#define LAN8720_BMCR_RESET                  0x8000U  /* BMCR Reset 位，置 1 触发 PHY 软复位。 */

#define LAN8720_BSR_REG                     0x01U    /* PHY Basic Mode Status Register 寄存器地址。 */
#define LAN8720_BSR_LINK                    0x0004U  /* BSR Link Status 位，表示当前链路是否建立。 */

#define LAN8720_PHY_ID1_REG                 0x02U    /* PHY Identifier 1 寄存器地址。 */
#define LAN8720_PHY_ID2_REG                 0x03U    /* PHY Identifier 2 寄存器地址。 */

#define LAN8720_SPECIAL_MODES_REG           0x12U    /* PHY Special Modes 寄存器地址，用于配置 PHY 模式和地址。 */

#define LAN8720_SPECIAL_MODES_MODE_MASK     0x00E0U  /* Special Modes 寄存器中模式选择位掩码。 */
#define LAN8720_SPECIAL_MODES_MODE_SHIFT    5U       /* 模式选择字段在 Special Modes 寄存器中的位移。 */
#define LAN8720_SPECIAL_MODES_PHYAD_MASK    0x001FU  /* Special Modes 寄存器中 PHY 地址字段掩码。 */
#define LAN8720_SPECIAL_MODES_WRITE_1       0x4000U  /* 写 Special Modes 寄存器时需要置 1 的保留控制位。 */
#define LAN8720_MODE_ALL_CAPABLE_AUTONEG    0x07U    /* 使能全部能力并开启自动协商的 LAN8720A 模式值。 */
#define LAN8720_SOFT_RESET_TIMEOUT_MS       500U     /* 等待 PHY 软复位完成的超时时间，单位 ms。 */

extern uint32_t LAN8720_ID1;
extern uint32_t LAN8720_ID2;
extern uint8_t LAN8720_IDValid;
extern uint32_t LAN8720_Reg18Before;
extern uint32_t LAN8720_Reg18Written;
extern uint32_t LAN8720_Reg18AfterReset;
extern uint32_t LAN8720_BMCRAfterReset;

void LAN8720_Reset(void);
uint8_t LAN8720_ReadID(void);
uint8_t LAN8720_ReadBSR(uint32_t *bsr);
uint8_t LAN8720_IsLinkUp(void);
uint8_t LAN8720_SetModeAndSoftReset(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_LAN8720_H__ */
