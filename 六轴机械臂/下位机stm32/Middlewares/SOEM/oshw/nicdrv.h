#ifndef NICDRV_H
#define NICDRV_H

#include "soem/ec_type.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 冗余收发栈描述；当前单网口阶段暂不使用，先用于完整定义 ecx_redportt。 */
typedef struct
{
   int *sock;
   ec_bufT (*txbuf)[EC_MAXBUF];
   int (*txbuflength)[EC_MAXBUF];
   ec_bufT *tempbuf;
   ec_bufT (*rxbuf)[EC_MAXBUF];
   ec_bufstate (*rxbufstat)[EC_MAXBUF];
   int (*rxsa)[EC_MAXBUF];
   uint64 rxcnt;
} ec_stackT;

/* 冗余网口对象；单网口移植阶段只保留结构定义，不启用冗余逻辑。 */
typedef struct ecx_redport
{
   ec_stackT stack;
   int sockhandle;
   ec_bufT rxbuf[EC_MAXBUF];
   ec_bufstate rxbufstat[EC_MAXBUF];
   int rxsa[EC_MAXBUF];
   ec_bufT tempinbuf;
} ecx_redportt;

/* SOEM 主站网口对象，协议层通过该对象管理以太网帧的发送和接收。 */
typedef struct ecx_port
/* 发送帧缓冲区数组；等价于 uint8 txbuf[EC_MAXBUF][EC_BUFSIZE]。 */
{
   ec_bufT txbuf[EC_MAXBUF];
   /* 每个发送缓冲区中实际有效的帧长度，单位为字节。 */
   int txbuflength[EC_MAXBUF];

   /* 接收帧缓冲区数组；从站返回的 EtherCAT 帧会暂存在这里。 */
   ec_bufT rxbuf[EC_MAXBUF];
   /* 每个接收缓冲区的状态，用于标记空闲、已接收、已完成等状态。 */
   ec_bufstate rxbufstat[EC_MAXBUF];

   /* 最近一次分配的帧索引 idx，用于匹配发送帧和返回帧。 */
   uint8 lastidx;

   /* 冗余网口指针；单网口模式下暂时不用，保留是为了兼容 SOEM 核心代码。 */
   ecx_redportt *redport;
   /* 冗余网口使用的第二发送缓冲区；当前单网口阶段暂不使用。 */
   ec_bufT txbuf2;
   /* 第二发送缓冲区中的实际有效帧长度，单位为字节。 */
   int txbuflength2;
} ecx_portt;

/*
 * SOEM 单次接收等待过程的诊断快照。
 *
 * ecx_waitinframe() 每次开始等待响应帧时都会先清零本结构体，因此下面的计数值只覆盖
 * 最近一次等待过程，并不是系统启动以来的累计值。计数器用于判断帧在哪一级被过滤；
 * last_* 字段用于保存最近一帧已经解析到的关键内容。
 */
typedef struct
{
   /* 调用 ETH_Raw_Receive() 轮询底层接收队列的总次数。 */
   uint32 polls;
   /* ETH_Raw_Receive() 返回 0 的次数；表示本次轮询暂时没有完整帧，不等同于丢包数。 */
   uint32 no_frame;
   /* 底层返回非零长度的完整以太网帧数量，尚未经过长度、EtherType 和 idx 过滤。 */
   uint32 rx_frames;
   /* 长度不足 Ethernet header + EtherCAT header + WKC 的帧数量。 */
   uint32 too_short;
   /* EtherType 不是 0x88A4、因而被判定为非 EtherCAT 的帧数量。 */
   uint32 non_ethercat;
   /* EtherCAT datagram 的 index 与当前等待 idx 不一致的帧数量。 */
   uint32 idx_mismatch;
   /* EtherCAT elength 太小或超过实际接收长度、无法安全读取 WKC 的帧数量。 */
   uint32 bad_length;
   /* 通过长度、EtherType、idx 和 elength 检查并成功匹配当前请求的帧数量。 */
   uint32 matched;

   /* 最近一帧非空接收帧的总长度，包含 14 字节 Ethernet header，单位为字节。 */
   uint16 last_rx_len;
   /* 最近一帧达到最小检查长度的以太网 EtherType，主机字节序，例如 EtherCAT 为 0x88A4。 */
   uint16 last_ethertype;
   /* 最近一帧通过 idx 检查后的 EtherCAT elength 长度部分，不包含 2 字节 elength 字段。 */
   uint16 last_ecat_length;
   /* 最近一帧 EtherCAT datagram header 中携带的接收帧 index。 */
   uint8 last_rx_idx;
   /* 最近一帧成功匹配且长度合法的 Working Counter；未匹配时保持清零后的 0。 */
   int last_wkc;
   /* 最近一帧达到最小检查长度的 6 字节以太网目的 MAC 地址。 */
   uint8 last_dst[6];
   /* 最近一帧达到最小检查长度的 6 字节以太网源 MAC 地址。 */
   uint8 last_src[6];
} ecx_rxdebugt;

int ecx_setupnic(ecx_portt *port, const char *ifname, boolean secondary);
void ecx_closenic(ecx_portt *port);
uint8 ecx_getindex(ecx_portt *port);
void ecx_setbufstat(ecx_portt *port, int idx, ec_bufstate bufstat);
int ecx_outframe(ecx_portt *port, uint8 idx);
int ecx_srconfirm(ecx_portt *port, uint8 idx, int timeout);
int ecx_waitinframe(ecx_portt *port, uint8 idx, int timeout);
int ecx_outframe_red(ecx_portt *port, uint8 idx);
void ecx_clear_rxdebug(void);
const ecx_rxdebugt *ecx_get_rxdebug(void);

#ifdef __cplusplus
}
#endif

#endif
