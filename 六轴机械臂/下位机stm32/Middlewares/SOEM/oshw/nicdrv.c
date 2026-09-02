#include "nicdrv.h"

#include <string.h>

#include "bsp_eth_raw.h"

static ecx_rxdebugt ecx_rxdebug;

void ecx_clear_rxdebug(void)
{
   memset(&ecx_rxdebug, 0, sizeof(ecx_rxdebug));
}

const ecx_rxdebugt *ecx_get_rxdebug(void)
{
   return &ecx_rxdebug;
}

/*
 * 初始化 SOEM 使用的网口对象。
 *
 * 当前阶段只支持 STM32 的单 ETH 网口，因此 secondary 为 TRUE 时直接返回失败。
 * 本函数负责把 ecx_portt 中的发送长度、接收缓冲状态、帧索引等软件状态恢复到初始值，
 * 然后调用已经验证过的 ETH_Raw_Start() 启动 STM32 ETH MAC/DMA。
 *
 * 注意：这里还不负责真正发送或接收 EtherCAT 帧。
 */
int ecx_setupnic(ecx_portt *port, const char *ifname, boolean secondary)
{
   int idx;
   ecx_redportt *redport;  /* 暂存冗余端口指针 */
   const uint8_t *local_mac;  /* 指向 STM32 ETH 外设本机 MAC 地址。 */
   /* EtherCAT 主站默认使用 01:01:01:01:01:01 作为二层目的 MAC。 */
   static const uint8_t ethercat_dst_mac[6] = {
      0x01U, 0x01U, 0x01U, 0x01U, 0x01U, 0x01U
   };

   /* STM32 只有一个固定 ETH 外设，不需要按接口名称选择网卡。 */
   (void)ifname;

   /* port空指针无法初始化。 */
   if (port == 0)
   {
      return 0;
   }

   /* 当前移植不支持 SOEM 的第二冗余网口 */
   if (secondary != FALSE)
   {
      return 0;
   }

   /* 从底层 ETH 驱动取得 MX_ETH_Init() 配置的本机 MAC 地址。 */
   local_mac = ETH_Raw_GetMacAddress();
   if (local_mac == 0)
   {
      return 0;
   }

   /* ecx_init_redundant() 可能已在 port 中设置 redport，先保存该指针。 */
   redport = port->redport;
   /* 清除上一次运行遗留的帧长度、缓冲内容、状态和索引。 */
   memset(port, 0, sizeof(*port));
   port->redport = redport;

   /* 为 SOEM 的每一个并行收发槽位预先构造以太网帧头并复位状态。 */
   for (idx = 0; idx < EC_MAXBUF; idx++)
   {
      /* 以太网头字节 0~5：写入 EtherCAT 默认目的 MAC。 */
      memcpy(&port->txbuf[idx][0], ethercat_dst_mac, sizeof(ethercat_dst_mac));
      /* 以太网头字节 6~11：写入 STM32 本机源 MAC。 */
      memcpy(&port->txbuf[idx][6], local_mac, 6U);
      /* 以太网头字节 12：EtherType 0x88A4 的高字节，按网络字节序保存。 */
      port->txbuf[idx][12] = (uint8_t)((ETH_P_ECAT >> 8) & 0xFFU);
      /* 以太网头字节 13：EtherType 0x88A4 的低字节。 */
      port->txbuf[idx][13] = (uint8_t)(ETH_P_ECAT & 0xFFU);

      /* 当前只有预填帧头，还没有有效 EtherCAT 数据报，因此长度清零。 */
      port->txbuflength[idx] = 0;
      /* 标记对应接收槽为空闲，后续 ecx_getindex() 才能分配该槽位。 */
      port->rxbufstat[idx] = EC_BUF_EMPTY;
   }

   /* 从索引 0 作为后续循环分配收发槽位的起点。 */
   port->lastidx = 0;
   /* 当前不使用冗余网口 有效长度清零。 */
   port->txbuflength2 = 0;

   /* 启动 STM32 ETH MAC/DMA */
   return (ETH_Raw_Start() != 0U) ? 1 : 0;
}

/*
 * 关闭 SOEM 网口对象。
 *
 * 当前 BSP 还没有提供 ETH_Raw_Stop()，并且 ETH 外设可能仍被调试收发流程共用，
 * 所以这里不停止 STM32 ETH MAC/DMA，只清理 SOEM port 内部的软件缓冲状态。
 * 这样后续重新 ecx_setupnic() 时，idx 分配和 rxbufstat 状态不会继承上一次运行的残留值。
 */
void ecx_closenic(ecx_portt *port)
{
   int idx;

   if (port == 0)
   {
      return;
   }

   for (idx = 0; idx < EC_MAXBUF; idx++)
   {
      port->txbuflength[idx] = 0;
      port->rxbufstat[idx] = EC_BUF_EMPTY;
   }

   port->lastidx = 0;
   port->txbuflength2 = 0;
}

/*
 * 为一次 EtherCAT 报文收发分配帧索引 idx。
 * SOEM 会把该 idx 写入 EtherCAT datagram header，后续收到响应帧时再用 idx 找回对应缓冲区。
 */
uint8 ecx_getindex(ecx_portt *port)
{
   uint8 idx;
   uint8 cnt;

   /* port 是 SOEM 主站网口对象；为空时没有可操作的缓冲区，只能返回 0 作为保护。 */
   if (port == 0)
   {
      return 0;
   }

   /* 从上一次分配的 idx 开始，下面会先加 1，所以实际搜索从下一个槽位开始。 */
   idx = port->lastidx;

   /* rxbufstat[] 记录每个收发缓冲区的状态；这里只寻找空闲槽位。 */
   for (cnt = 0; cnt < EC_MAXBUF; cnt++)
   {
      idx++;
      if (idx >= EC_MAXBUF)
      {
         /* idx 是环形使用的，到达缓冲区末尾后回到 0。 */
         idx = 0;
      }

      if (port->rxbufstat[idx] == EC_BUF_EMPTY)
      {
         /* 找到空闲槽后立即标记为已分配，避免后续再次分到同一个 idx。 */
         port->rxbufstat[idx] = EC_BUF_ALLOC;
         port->lastidx = idx;
         return idx;
      }
   }

   /* 所有槽位都忙时，SOEM 没有为该函数设计错误返回值；先返回最近一次 idx。 */
   return port->lastidx;
}

/*
* 设置指定 idx 对应的接收缓冲区状态。
*
* SOEM 每发送一帧 EtherCAT 报文，都会通过 idx 记录该帧使用的缓冲区。
* 当该帧处理完成后，协议层会调用本函数把缓冲区重新标记为空闲，
* 这样后续 ecx_getindex() 才能再次分配这个 idx。
*
* port    : SOEM 主站网口对象。
* idx     : 缓冲区索引，范围应为 0 ~ EC_MAXBUF - 1。
* bufstat : 要写入的新状态，例如 EC_BUF_EMPTY、EC_BUF_ALLOC 等。
*/
void ecx_setbufstat(ecx_portt *port, int idx, ec_bufstate bufstat)
{
   if (port == 0)
   {
      return;
   }

   if ((idx < 0) || (idx >= EC_MAXBUF))
   {
      return;
   }

   port->rxbufstat[idx] = bufstat;

   if (port->redport != 0)
   {
      port->redport->rxbufstat[idx] = bufstat;
   }
}

/**
 * @brief  发送指定缓冲索引对应的 EtherCAT 以太网帧
 *
 * @param[in,out] port   SOEM 主站网口对象
 * @param[in]     idx    发送帧缓冲索引，由 ecx_getindex() 分配
 *
 * @return
 * 成功返回实际发送的帧长度，参数非法或底层发送失败返回0
 *
 * @warning
 * port 必须已完成网口初始化，且 idx 对应的发送帧及帧长度必须有效；该函数只负责
 * 发送，不等待从站响应，调用方需继续调用 ecx_waitinframe() 或 ecx_srconfirm()
 */
int ecx_outframe(ecx_portt *port, uint8 idx)
{
   int frame_length;

   if (port == 0)
   {
      return 0;
   }

   if (idx >= EC_MAXBUF)
   {
      return 0;
   }

   frame_length = port->txbuflength[idx];
   if ((frame_length <= 0) || (frame_length > EC_BUFSIZE))
   {
      return 0;
   }

   if (ETH_Raw_Send(port->txbuf[idx], (uint32_t)frame_length) == 0U)
   {
      return 0;
   }

   port->rxbufstat[idx] = EC_BUF_TX;

   return frame_length;
}

/**
 * @brief  阻塞等待并解析与指定索引匹配的 EtherCAT 响应帧
 *
 * @param[in,out] port      SOEM 主站网口对象
 * @param[in]     idx       期望匹配的 EtherCAT 数据报索引
 * @param[in]     timeout   最大等待时间，单位：us；小于等于0时按0处理
 *
 * @return
 * 收到匹配响应帧返回WKC，参数无效或等待超时返回EC_NOFRAME
 *
 * @warning
 * port 必须已完成网口初始化，且 idx 必须对应已发送的请求；该函数会阻塞轮询、
 * 消耗不匹配帧并覆盖对应接收缓冲，禁止在同一 port 上并发接收
 */
int ecx_waitinframe(ecx_portt *port, uint8 idx, int timeout)
{
   osal_timert timer;
   uint32 timeout_usec;
   uint16 rx_length;
   uint16 stripped_length;
   uint16 ecat_length;
   uint16 wkc_offset;
   uint8 rx_index;
   int wkc;

   if (port == 0)
   {
      return EC_NOFRAME;
   }

   if (idx >= EC_MAXBUF)
   {
      return EC_NOFRAME;
   }

   /* SOEM 的 timeout 单位是 us；负数没有实际意义，这里按 0us 处理。 */
   timeout_usec = (timeout > 0) ? (uint32)timeout : 0U;
   ecx_clear_rxdebug();
   osal_timer_start(&timer, timeout_usec);

   do
   {
      ecx_rxdebug.polls++;

      /*
       * 从 STM32 ETH 裸帧接收接口取一帧。
       * 这里临时借用 rxbuf[idx] 保存完整 Ethernet frame，后面匹配成功后再剥掉以太网头。
       */
      rx_length = ETH_Raw_Receive(port->rxbuf[idx], EC_BUFSIZE);

      if (rx_length == 0U)
      {
         ecx_rxdebug.no_frame++;
         continue;
      }

      ecx_rxdebug.rx_frames++;
      ecx_rxdebug.last_rx_len = rx_length;

      /* 最短有效 EtherCAT 响应帧至少包含 Ethernet header + EtherCAT header + WKC。 */
      if (rx_length < (ETH_HEADERSIZE + EC_HEADERSIZE + EC_WKCSIZE))
      {
         ecx_rxdebug.too_short++;
         continue;
      }

      memcpy(ecx_rxdebug.last_dst, &port->rxbuf[idx][0], sizeof(ecx_rxdebug.last_dst));
      memcpy(ecx_rxdebug.last_src, &port->rxbuf[idx][6], sizeof(ecx_rxdebug.last_src));
      ecx_rxdebug.last_ethertype = (uint16)(((uint16)port->rxbuf[idx][12] << 8) |
                                            port->rxbuf[idx][13]);

      /*
       * 过滤非 EtherCAT 帧。
       * Ethernet header 中 byte[12..13] 是 EtherType，EtherCAT 的 EtherType 为 0x88A4。
       */
      if ((port->rxbuf[idx][12] != 0x88U) || (port->rxbuf[idx][13] != 0xA4U))
      {
         ecx_rxdebug.non_ethercat++;
         continue;
      }

      /*
       * EtherCAT datagram header 中 command 后面的 1 字节就是 index。
       * 只有 index 与本次等待的 idx 相同，才认为这是当前请求的响应帧。
       */
      rx_index = port->rxbuf[idx][ETH_HEADERSIZE + EC_CMDOFFSET + 1U];
      ecx_rxdebug.last_rx_idx = rx_index;
      if (rx_index != idx)
      {
         ecx_rxdebug.idx_mismatch++;
         continue;
      }

      stripped_length = (uint16)(rx_length - ETH_HEADERSIZE); /* 去掉 Ethernet header 后的长度
                                                               EtherCAT Header + Datagram Header + Data + WKC + Padding */
      ecat_length = (uint16)(port->rxbuf[idx][ETH_HEADERSIZE] |
                            ((uint16)port->rxbuf[idx][ETH_HEADERSIZE + 1U] << 8)); /* 读取EtherCAT Header 的 2 字节 elength 字段（小端格式）。 */
      /* elength 字段结构
      * bit 0  ~ bit 10   EtherCAT 数据长度
      * bit 11             reserved
      * bit 12 ~ bit 15   EtherCAT Type 
      */
      ecat_length &= 0x0fffU; /* 真实长度取低 12 bit。该长度为EtherCAT Header之后的长度。
                              Datagram Header + Data + WKC + Padding*/
      ecx_rxdebug.last_ecat_length = ecat_length;
      wkc_offset = ecat_length - 2; /* 基于Datagram Header起点位置的偏移。 */

      /* 防止异常帧导致后续读取 WKC 时越界。 */
      if ((ecat_length < EC_DATAGRAM_HEADERSIZE + EC_WKCSIZE) ||
          ((ecat_length + EC_ELENGTHSIZE) > stripped_length))
      {
         ecx_rxdebug.bad_length++;
         continue;
      }

      /* WKC 是小端 16 bit，位置在 EtherCAT header 起点 + wkc_offset。 */
      wkc = (int)(port->rxbuf[idx][ETH_HEADERSIZE +EC_ELENGTHSIZE + wkc_offset] |
                  ((uint16)port->rxbuf[idx][ETH_HEADERSIZE +EC_ELENGTHSIZE + wkc_offset + 1U] << 8));
      ecx_rxdebug.last_wkc = wkc;
      ecx_rxdebug.matched++;

      /*
       * SOEM 上层读取 rxbuf[idx] 时默认它从 EtherCAT header 开始，
       * 所以这里把 14 字节 Ethernet header 剥掉，只保留 EtherCAT 数据区和 WKC。
       */
      memmove(port->rxbuf[idx],
              &port->rxbuf[idx][ETH_HEADERSIZE],
              (size_t)(EC_ELENGTHSIZE + ecat_length));

      /* 标记该 idx 的一次收发已经完成，后续 ec_base.c 可以读取 rxbuf[idx]。 */
      port->rxbufstat[idx] = EC_BUF_COMPLETE;

      return wkc;
   } while (osal_timer_is_expired(&timer) == FALSE);

   /* 超时时仍未收到 idx 匹配的 EtherCAT 响应帧。 */
   return EC_NOFRAME;
}

/*
 * 发送指定 idx 的 EtherCAT 帧，并等待对应响应帧返回。
 *
 * SOEM 基础读写函数会先调用 ecx_getindex() 分配 idx，再调用 ecx_setupdatagram()
 * 把请求帧构造到 port->txbuf[idx] 中，最后通过本函数完成一次阻塞式收发。
 *
 * port    : SOEM 主站网口对象。
 * idx     : 本次请求使用的帧索引，同时也是等待响应时用于匹配的 datagram index。
 * timeout : 等待响应的超时时间，单位 us。
 *
 * 返回值：
 *   >= 0 表示收到匹配响应帧，返回该帧 WKC；
 *   EC_NOFRAME 表示发送失败或等待超时。
 *
 * 注意：
 *   当前阶段只实现单网口收发流程；在总超时内按 EC_TIMEOUTRET 分段重发。
 */
int ecx_srconfirm(ecx_portt *port, uint8 idx, int timeout)
{
   int wkc = EC_NOFRAME;
   int receive_timeout;
   osal_timert total_timer;

   /*
    * SOEM management datagrams use timeout as the total transaction window.
    * Split that window into EC_TIMEOUTRET receive attempts and retransmit the
    * same indexed frame while no matching response has been received.
    */
   receive_timeout = (timeout < EC_TIMEOUTRET) ? timeout : EC_TIMEOUTRET;
   osal_timer_start(&total_timer, (timeout > 0) ? (uint32)timeout : 0U);

   do
   {
      (void)ecx_outframe(port, idx);
      wkc = ecx_waitinframe(port, idx, receive_timeout);
   } while ((wkc <= EC_NOFRAME) &&
            (osal_timer_is_expired(&total_timer) == FALSE));

   return wkc;
}

/*
 * 发送冗余模式下指定 idx 的 EtherCAT 帧。
 *
 * SOEM 核心代码在部分过程数据收发路径中会调用 ecx_outframe_red()，
 * 用于同时处理主网口和冗余网口发送。
 *
 * 当前 STM32 移植阶段只支持单 ETH 网口，不启用冗余网口。
 * 因此本函数暂时作为兼容入口，内部直接复用单网口发送函数 ecx_outframe()。
 *
 * port : SOEM 主站网口对象。
 * idx  : 本次发送使用的缓冲区索引。
 *
 * 返回值：
 *   > 0 表示发送成功，返回实际发送帧长度；
 *   0   表示参数非法或底层发送失败。
 */
int ecx_outframe_red(ecx_portt *port, uint8 idx)
{
   return ecx_outframe(port, idx);
}
