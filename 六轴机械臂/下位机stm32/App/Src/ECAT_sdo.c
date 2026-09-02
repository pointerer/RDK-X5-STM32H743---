#include "ECAT_sdo.h"

/**
 * @brief  读取指定从站对象字典中的8位 CoE SDO 数据
 *
 * @param[in,out] context    SOEM 主站上下文
 * @param[in,out] log        EtherCAT 日志上下文
 * @param[in]     slave      从站索引
 * @param[in]     index      对象字典索引
 * @param[in]     subindex   对象字典子索引
 * @param[out]    value      8位读取结果输出地址
 *
 * @return
 * WKC大于0且读取长度为1字节时返回1，读取失败或长度不符返回0
 *
 * @warning
 * context、log 和 value 必须有效，从站必须支持 CoE 并处于可进行邮箱通信的状态；
 * 该函数会按 EC_TIMEOUTRXM 阻塞等待，value 仅在返回1时有效
 */
uint8 ECAT_SdoReadU8(ecx_contextt *context,
                               ECAT_LogContext *log,
                               uint16 slave,
                               uint16 index,
                               uint8 subindex,
                               uint8 *value)
{
  int size = sizeof(*value);
  int wkc;

  *value = 0U;
  wkc = ecx_SDOread(context,
                    slave,
                    index,
                    subindex,
                    FALSE,
                    &size,
                    value,
                    EC_TIMEOUTRXM);

  ECAT_LogPrintf(log,
                          "[SOEM] SDO 0x%04X:%02X wkc=%d size=%d\r\n",
                          index,
                          subindex,
                          wkc,
                          size);

  if ((wkc <= 0) || (size != (int)sizeof(*value)))
  {
    ECAT_LogPrintf(log,
                            "[SOEM] SDO 0x%04X:%02X FAIL\r\n",
                            index,
                            subindex);
    return 0U;
  }

  return 1U;
}

uint8 ECAT_SdoReadU16(ecx_contextt *context,
                                ECAT_LogContext *log,
                                uint16 slave,
                                uint16 index,
                                uint8 subindex,
                                uint16 *value)
{
  int size = sizeof(*value);
  int wkc;

  *value = 0U;
  wkc = ecx_SDOread(context,
                    slave,
                    index,
                    subindex,
                    FALSE,
                    &size,
                    value,
                    EC_TIMEOUTRXM);

  ECAT_LogPrintf(log,
                          "[SOEM] SDO 0x%04X:%02X wkc=%d size=%d\r\n",
                          index,
                          subindex,
                          wkc,
                          size);

  if ((wkc <= 0) || (size != (int)sizeof(*value)))
  {
    ECAT_LogPrintf(log,
                            "[SOEM] SDO 0x%04X:%02X FAIL\r\n",
                            index,
                            subindex);
    return 0U;
  }

  return 1U;
}

/**
 * @brief  读取指定从站对象字典中的32位 CoE SDO 数据
 *
 * @param[in,out] context    SOEM 主站上下文
 * @param[in,out] log        EtherCAT 日志上下文
 * @param[in]     slave      从站索引
 * @param[in]     index      对象字典索引
 * @param[in]     subindex   对象字典子索引
 * @param[out]    value      32位读取结果输出地址
 *
 * @return
 * WKC大于0且读取长度为4字节时返回1，读取失败或长度不符返回0
 *
 * @warning
 * context、log 和 value 必须有效，从站必须支持 CoE 并处于可进行邮箱通信的状态；
 * 该函数会按 EC_TIMEOUTRXM 阻塞等待，value 仅在返回1时有效
 */
uint8 ECAT_SdoReadU32(ecx_contextt *context,
                                ECAT_LogContext *log,
                                uint16 slave,
                                uint16 index,
                                uint8 subindex,
                                uint32 *value)
{
  int size = sizeof(*value);
  int wkc;

  *value = 0U;
  wkc = ecx_SDOread(context,
                    slave,
                    index,
                    subindex,
                    FALSE,
                    &size,
                    value,
                    EC_TIMEOUTRXM);

  ECAT_LogPrintf(log,
                          "[SOEM] SDO 0x%04X:%02X wkc=%d size=%d\r\n",
                          index,
                          subindex,
                          wkc,
                          size);

  if ((wkc <= 0) || (size != (int)sizeof(*value)))
  {
    ECAT_LogPrintf(log,
                            "[SOEM] SDO 0x%04X:%02X FAIL\r\n",
                            index,
                            subindex);
    return 0U;
  }

  return 1U;
}

/**
 * @brief  通过 SDO 向从站对象字典写入一个 8 位 PDO 配置值
 *
 * @param[in,out] context  SOEM 主站上下文
 * @param[in,out] log      EtherCAT 日志上下文
 * @param[in]     slave    目标从站索引
 * @param[in]     index    对象字典索引
 * @param[in]     subindex 对象字典子索引
 * @param[in]     value    要写入的 8 位值
 *
 * @return
 * SDO 写入的工作计数大于0返回1，否则记录失败日志并返回0
 *
 * @warning
 * context 和 log 必须有效，从站必须支持 CoE、处于允许修改目标对象的状态且邮箱通信
 * 可用；本函数会按 EC_TIMEOUTRXM 阻塞等待，只判断写入工作计数，不执行回读校验
 */
uint8 ECAT_SdoPdoWriteU8(ecx_contextt *context,
                                   ECAT_LogContext *log,
                                   uint16 slave,
                                   uint16 index,
                                   uint8 subindex,
                                   uint8 value)
{
  int wkc;

  wkc = ecx_SDOwrite(context,
                     slave,
                     index,
                     subindex,
                     FALSE,
                     (int)sizeof(value),
                     &value,
                     EC_TIMEOUTRXM);
  if (wkc <= 0)
  {
    ECAT_LogPrintf(log,
                            "[SOEM] PO2SO write 0x%04X:%02X U8=0x%02X FAIL wkc=%d\r\n",
                            index,
                            subindex,
                            (unsigned int)value,
                            wkc);
    return 0U;
  }
  return 1U;
}

/**
 * @brief  通过 SDO 向从站对象字典写入一个 16 位 PDO 配置值
 *
 * @param[in,out] context  SOEM 主站上下文
 * @param[in,out] log      EtherCAT 日志上下文
 * @param[in]     slave    目标从站索引
 * @param[in]     index    对象字典索引
 * @param[in]     subindex 对象字典子索引
 * @param[in]     value    要写入的主机字节序 16 位值
 *
 * @return
 * SDO 写入的工作计数大于0返回1，否则记录失败日志并返回0
 *
 * @warning
 * context 和 log 必须有效，从站必须支持 CoE、处于允许修改目标对象的状态且邮箱通信
 * 可用；函数会使用 htoes() 转换为 EtherCAT 小端格式并按 EC_TIMEOUTRXM 阻塞等待，
 * 只判断写入工作计数，不执行回读校验
 */
uint8 ECAT_SdoPdoWriteU16(ecx_contextt *context,
                                    ECAT_LogContext *log,
                                    uint16 slave,
                                    uint16 index,
                                    uint8 subindex,
                                    uint16 value)
{
  uint16 wire_value = htoes(value);
  int wkc;

  wkc = ecx_SDOwrite(context,
                     slave,
                     index,
                     subindex,
                     FALSE,
                     (int)sizeof(wire_value),
                     &wire_value,
                     EC_TIMEOUTRXM);
  if (wkc <= 0)
  {
    ECAT_LogPrintf(log,
                            "[SOEM] PO2SO write 0x%04X:%02X U16=0x%04X FAIL wkc=%d\r\n",
                            index,
                            subindex,
                            value,
                            wkc);
    return 0U;
  }
  return 1U;
}

/**
 * @brief  通过 SDO 向从站对象字典写入一个 32 位 PDO 配置值
 *
 * @param[in,out] context  SOEM 主站上下文
 * @param[in,out] log      EtherCAT 日志上下文
 * @param[in]     slave    目标从站索引
 * @param[in]     index    对象字典索引
 * @param[in]     subindex 对象字典子索引
 * @param[in]     value    要写入的主机字节序 32 位值
 *
 * @return
 * SDO 写入的工作计数大于0返回1，否则记录失败日志并返回0
 *
 * @warning
 * context 和 log 必须有效，从站必须支持 CoE、处于允许修改目标对象的状态且邮箱通信
 * 可用；函数会使用 htoel() 转换为 EtherCAT 小端格式并按 EC_TIMEOUTRXM 阻塞等待，
 * 只判断写入工作计数，不执行回读校验
 */
uint8 ECAT_SdoPdoWriteU32(ecx_contextt *context,
                                    ECAT_LogContext *log,
                                    uint16 slave,
                                    uint16 index,
                                    uint8 subindex,
                                    uint32 value)
{
  uint32 wire_value = htoel(value);
  int wkc;

  wkc = ecx_SDOwrite(context,
                     slave,
                     index,
                     subindex,
                     FALSE,
                     (int)sizeof(wire_value),
                     &wire_value,
                     EC_TIMEOUTRXM);
  if (wkc <= 0)
  {
    ECAT_LogPrintf(log,
                            "[SOEM] PO2SO write 0x%04X:%02X U32=0x%08lX FAIL wkc=%d\r\n",
                            index,
                            subindex,
                            (unsigned long)value,
                            wkc);
    return 0U;
  }
  return 1U;
}

uint8 ECAT_SdoPdoReadU8(ecx_contextt *context,
                                  ECAT_LogContext *log,
                                  uint16 slave,
                                  uint16 index,
                                  uint8 subindex,
                                  uint8 *value)
{
  int size;
  int wkc;

  if (value == 0)
  {
    return 0U;
  }

  *value = 0U;
  size = (int)sizeof(*value);
  wkc = ecx_SDOread(context,
                    slave,
                    index,
                    subindex,
                    FALSE,
                    &size,
                    value,
                    EC_TIMEOUTRXM);
  if ((wkc <= 0) || (size != (int)sizeof(*value)))
  {
    ECAT_LogPrintf(log,
                            "[SOEM] PO2SO read 0x%04X:%02X U8 FAIL wkc=%d size=%d\r\n",
                            index,
                            subindex,
                            wkc,
                            size);
    return 0U;
  }
  return 1U;
}

uint8 ECAT_SdoPdoReadU16(ecx_contextt *context,
                                   ECAT_LogContext *log,
                                   uint16 slave,
                                   uint16 index,
                                   uint8 subindex,
                                   uint16 *value)
{
  uint16 wire_value = 0U;
  int size = (int)sizeof(wire_value);
  int wkc;

  if (value == 0)
  {
    return 0U;
  }

  *value = 0U;
  wkc = ecx_SDOread(context,
                    slave,
                    index,
                    subindex,
                    FALSE,
                    &size,
                    &wire_value,
                    EC_TIMEOUTRXM);
  if ((wkc <= 0) || (size != (int)sizeof(wire_value)))
  {
    ECAT_LogPrintf(log,
                            "[SOEM] PO2SO read 0x%04X:%02X U16 FAIL wkc=%d size=%d\r\n",
                            index,
                            subindex,
                            wkc,
                            size);
    return 0U;
  }
  *value = etohs(wire_value);
  return 1U;
}

uint8 ECAT_SdoPdoReadU32(ecx_contextt *context,
                                   ECAT_LogContext *log,
                                   uint16 slave,
                                   uint16 index,
                                   uint8 subindex,
                                   uint32 *value)
{
  uint32 wire_value = 0U;
  int size = (int)sizeof(wire_value);
  int wkc;

  if (value == 0)
  {
    return 0U;
  }

  *value = 0U;
  wkc = ecx_SDOread(context,
                    slave,
                    index,
                    subindex,
                    FALSE,
                    &size,
                    &wire_value,
                    EC_TIMEOUTRXM);
  if ((wkc <= 0) || (size != (int)sizeof(wire_value)))
  {
    ECAT_LogPrintf(log,
                            "[SOEM] PO2SO read 0x%04X:%02X U32 FAIL wkc=%d size=%d\r\n",
                            index,
                            subindex,
                            wkc,
                            size);
    return 0U;
  }
  *value = etohl(wire_value);
  return 1U;
}
