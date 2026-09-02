#include "oshw.h"
#include "soem/ec_main.h"
#include "soem/ec_options.h"

/*
 * STM32 平台没有 PC 系统里的网卡枚举机制。
 * 这里用一个静态对象表示当前板子上的唯一 ETH 网口，
 * 后续 oshw_find_adapters() 会直接返回该对象地址。
 */
static ec_adaptert stm32_adapter =
{
   "stm32_eth",
   "STM32H743 ETH",
   0
};

ec_adaptert *oshw_find_adapters(void)
{
   stm32_adapter.next = 0;
   return &stm32_adapter;
}

void oshw_free_adapters(ec_adaptert *adapter)
{
   (void)adapter;
}

/* SOEM 主网口 EtherCAT 帧内部使用的 MAC 字段，不是 STM32 板卡真实 MAC 地址。 */
const uint16 priMAC[3] = EC_PRIMARY_MAC_ARRAY;
/* SOEM 冗余网口逻辑使用的备用 MAC 字段；当前单网口阶段仅用于满足核心代码依赖。 */
const uint16 secMAC[3] = EC_SECONDARY_MAC_ARRAY;


/* 主机字节序转网络字节序。STM32H743 是小端，网络字节序是大端，所以 16 位数据需要高低字节交换。 */
uint16 oshw_htons(uint16 host)
{
   return (uint16)(((host & 0x00ffU) << 8) |
                   ((host & 0xff00U) >> 8));
}

/* 网络字节序转主机字节序。16 位字节交换是可逆操作，所以直接复用 oshw_htons()。 */
uint16 oshw_ntohs(uint16 network)
{
   return oshw_htons(network);
}

/* 主机字节序转网络字节序。32 位数据需要按字节反转顺序。 */
uint32 oshw_htonl(uint32 host)
{
   return (((host & 0x000000ffUL) << 24) |
           ((host & 0x0000ff00UL) << 8) |
           ((host & 0x00ff0000UL) >> 8) |
           ((host & 0xff000000UL) >> 24));
}

/* 网络字节序转主机字节序。32 位字节交换同样可逆，所以直接复用 oshw_htonl()。 */
uint32 oshw_ntohl(uint32 network)
{
   return oshw_htonl(network);
}
