#include "ECAT_pdo_map.h"

#include "ECAT_sdo.h"
#include "EtherCAT/app_ethercat_pdo.h"

#include <stdint.h>
#include <string.h>

#define ECAT_PDO_MAP_MAX_ASSIGN  32U
#define ECAT_PDO_MAP_MAX_MAPPING 64U
#define ECAT_PDO_MAP_RX_BITS     (APP_ETHERCAT_SERVO_RXPDO_SIZE * 8U)
#define ECAT_PDO_MAP_TX_BITS     (APP_ETHERCAT_SERVO_TXPDO_SIZE * 8U)
#define ECAT_PDO_MAP_ARRAY_COUNT(array) \
  (sizeof(array) / sizeof((array)[0]))

typedef struct
{
  uint16 index;
  const uint32 *entries;
  uint8 entry_count;
} ECAT_ExplicitPdoMap;

typedef struct
{
  uint32 iomap_base_offset;
  uint32 iomap_size_bits;
  uint32 pdo_start_bit;
  uint16 current_pdo;
  uint8 current_assign_subindex;
  uint8 range_ok;
} ECAT_PdoMapResolveState;

static const uint32 ECAT_rxpdo_1600[] =
{
  0x60400010UL, 0x60600008UL, 0x607A0020UL,
  0x60710010UL, 0x30970010UL
};

static const uint32 ECAT_rxpdo_1610[] =
{
  0x68400010UL, 0x68600008UL, 0x687A0020UL,
  0x68710010UL, 0x38970010UL
};

static const uint32 ECAT_txpdo_1a00[] =
{
  0x60410010UL, 0x60770010UL, 0x60640020UL, 0x606C0020UL,
  0x603F0010UL, 0x60610008UL, 0x31540010UL, 0x61640020UL
};

static const uint32 ECAT_txpdo_1a10[] =
{
  0x68410010UL, 0x68770010UL, 0x68640020UL, 0x686C0020UL,
  0x683F0010UL, 0x68610008UL, 0x39540010UL, 0x69640020UL
};

static const ECAT_ExplicitPdoMap ECAT_explicit_pdo_maps[] =
{
  {0x1600U, ECAT_rxpdo_1600,
   (uint8)ECAT_PDO_MAP_ARRAY_COUNT(ECAT_rxpdo_1600)},
  {0x1610U, ECAT_rxpdo_1610,
   (uint8)ECAT_PDO_MAP_ARRAY_COUNT(ECAT_rxpdo_1610)},
  {0x1A00U, ECAT_txpdo_1a00,
   (uint8)ECAT_PDO_MAP_ARRAY_COUNT(ECAT_txpdo_1a00)},
  {0x1A10U, ECAT_txpdo_1a10,
   (uint8)ECAT_PDO_MAP_ARRAY_COUNT(ECAT_txpdo_1a10)}
};

static const uint16 ECAT_rxpdo_assign[] = {0x1600U, 0x1610U};
static const uint16 ECAT_txpdo_assign[] = {0x1A00U, 0x1A10U};

static void ECAT_PdoMapReadAssignObject(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave,
  uint16 assign_index,
  const char *assign_name,
  ECAT_PdoMapCache *cache);
static void ECAT_PdoMapReadMapping(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave,
  uint16 pdo_index,
  const char *assign_name,
  uint8 assign_subindex,
  ECAT_PdoMapCache *cache);
static void ECAT_PdoMapPrintEntry(
  ECAT_LogContext *log,
  uint16 pdo_index,
  uint8 entry_subindex,
  uint32 mapping_entry);
static void ECAT_PdoMapCacheMappingEntry(
  ECAT_PdoMapCache *cache,
  uint16 pdo_index,
  uint8 assign_subindex,
  uint8 mapping_subindex,
  uint32 mapping_entry);
static uint8 ECAT_PdoMapWriteExplicit(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave,
  const ECAT_ExplicitPdoMap *map);
static uint8 ECAT_PdoMapVerifyExplicit(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave,
  const ECAT_ExplicitPdoMap *map);
static uint8 ECAT_PdoMapWriteAssignment(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave,
  uint16 assign_index,
  const uint16 *pdo_indices,
  uint8 pdo_count);
static uint8 ECAT_PdoMapVerifyAssignment(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave,
  uint16 assign_index,
  const uint16 *pdo_indices,
  uint8 pdo_count);
static uint8 ECAT_PdoMapVerifyExplicitConfiguration(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave);
static uint8 ECAT_PdoMapPrepareResolved(
  ECAT_LogContext *log,
  const uint8 *iomap,
  uint32 iomap_capacity,
  uint16 slave,
  const char *direction,
  const ECAT_PdoMapCache *cache,
  const uint8 *pdo_base,
  uint8 start_bit,
  uint32 mapped_bits,
  int iomap_size,
  ECAT_PdoMapResolveState *state);
static void ECAT_PdoMapPrintResolvedPdo(
  ECAT_LogContext *log,
  const char *direction,
  uint32 iomap_base_offset,
  uint8 start_bit,
  uint16 pdo_index,
  uint8 assign_subindex,
  uint32 pdo_start_bit,
  uint32 pdo_end_bit);
static uint8 ECAT_PdoMapPrintResolvedEntry(
  ECAT_LogContext *log,
  const char *direction,
  const ECAT_PdoMapEntry *entry,
  uint8 start_bit,
  const ECAT_PdoMapResolveState *state);
static void ECAT_PdoMapPrintResolvedSummary(
  ECAT_LogContext *log,
  uint16 slave,
  const char *direction,
  const ECAT_PdoMapCache *cache,
  uint8 start_bit,
  uint32 mapped_bits,
  ECAT_PdoMapResolveState *state);
static uint8 ECAT_PdoMapRewriteExplicitConfiguration(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave);
static void ECAT_PdoMapDisableAssignments(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave);

void ECAT_PdoMapReset(ECAT_PdoMapContext *pdo_map)
{
  memset(pdo_map, 0, sizeof(*pdo_map));
}

/**
 * @brief  清除指定 PDO 映射上下文的 PO2SO 回调执行结果
 *
 * @param[in,out] pdo_map PDO 映射上下文
 *
 * @return
 * 无
 *
 * @warning
 * pdo_map 必须有效；本函数仅清零 po2so_attempted 和 po2so_ok 标志，不会修改
 * RxPDO、TxPDO 映射缓存，且不得与使用同一上下文的 PO2SO 回调并发调用
 */
void ECAT_PdoMapResetPo2SoResult(ECAT_PdoMapContext *pdo_map)
{
  pdo_map->po2so_attempted = 0U;
  pdo_map->po2so_ok = 0U;
}

/**
 * @brief  从对象字典加载指定从站的 RxPDO 和 TxPDO 映射缓存
 *
 * @param[in,out] context   SOEM 主站上下文
 * @param[in,out] log       EtherCAT 日志上下文
 * @param[in,out] pdo_map   PDO 映射上下文，函数会重建其中的收发映射缓存
 * @param[in]     slave     要读取的从站索引
 * @param[in]     stage     诊断阶段标签，可为空
 *
 * @return
 * 收发映射均读取完整且总位数符合工程预期返回1，否则返回0
 *
 * @warning
 * context、log 和 pdo_map 必须有效，从站必须支持 CoE 并处于可进行邮箱通信的状态；
 * 该函数会同步执行多次 SDO 读取并先清空原映射缓存，失败时缓存中可能保留部分结果
 */
uint8 ECAT_PdoMapLoad(ecx_contextt *context,
                                ECAT_LogContext *log,
                                ECAT_PdoMapContext *pdo_map,
                                uint16 slave,
                                const char *stage)
{
  uint8 cache_ok;

  memset(&pdo_map->rxpdo_map, 0, sizeof(pdo_map->rxpdo_map));
  memset(&pdo_map->txpdo_map, 0, sizeof(pdo_map->txpdo_map));
  pdo_map->rxpdo_map.complete = 1U;
  pdo_map->txpdo_map.complete = 1U;
  ECAT_LogPrintf(log,
                          "[SOEM] PDO cache load slave%u stage=%s\r\n",
                          (unsigned int)slave,
                          (stage != 0) ? stage : "unknown");
  ECAT_PdoMapReadAssignObject(context,
                                        log,
                                        slave,
                                        ECT_SDO_RXPDOASSIGN,
                                        "RxPDO",
                                        &pdo_map->rxpdo_map);
  ECAT_PdoMapReadAssignObject(context,
                                        log,
                                        slave,
                                        ECT_SDO_TXPDOASSIGN,
                                        "TxPDO",
                                        &pdo_map->txpdo_map);

  cache_ok = ((pdo_map->rxpdo_map.complete != 0U) &&
              (pdo_map->txpdo_map.complete != 0U) &&
              (pdo_map->rxpdo_map.total_bits == ECAT_PDO_MAP_RX_BITS) &&
              (pdo_map->txpdo_map.total_bits == ECAT_PDO_MAP_TX_BITS)) ? 1U : 0U;
  ECAT_LogPrintf(log,
                          "[SOEM] PDO cache stage=%s RxBits=%lu/%u TxBits=%lu/%u complete=%s\r\n",
                          (stage != 0) ? stage : "unknown",
                          (unsigned long)pdo_map->rxpdo_map.total_bits,
                          (unsigned int)ECAT_PDO_MAP_RX_BITS,
                          (unsigned long)pdo_map->txpdo_map.total_bits,
                          (unsigned int)ECAT_PDO_MAP_TX_BITS,
                          (cache_ok != 0U) ? "PASS" : "FAIL");
  return cache_ok;
}

/**
 * @brief  读取一个 PDO 分配对象及其引用的全部映射对象
 *
 * @param[in,out] context       SOEM 主站上下文
 * @param[in,out] log           EtherCAT 日志上下文
 * @param[in]     slave        要读取的从站索引
 * @param[in]     assign_index 分配对象索引
 * @param[in]     assign_name  分配方向名称，用于日志标识
 * @param[in,out] cache         PDO 映射缓存，函数会追加读取到的映射项
 *
 * @return
 * 无
 *
 * @warning
 * 所有指针参数必须有效，从站必须支持 CoE 并处于可进行邮箱通信的状态；分配数量
 * 超过 ECAT_PDO_MAP_MAX_ASSIGN 时会被截断，读取失败通过 cache->complete 标记，
 * 调用方必须在函数返回后检查该标志
 */
static void ECAT_PdoMapReadAssignObject(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave,
  uint16 assign_index,
  const char *assign_name,
  ECAT_PdoMapCache *cache)
{
  uint8 assign_count = 0U;
  uint8 assign_subindex;
  uint8 read_count;
  uint16 pdo_index;

  if (ECAT_SdoReadU8(context,
                               log,
                               slave,
                               assign_index,
                               0x00U,
                               &assign_count) == 0U)
  {
    ECAT_LogPrintf(log,
                            "[SOEM] %s assign 0x%04X read count FAIL\r\n",
                            assign_name,
                            assign_index);
    if (cache != 0)
    {
      cache->complete = 0U;
    }
    return;
  }

  ECAT_LogPrintf(log,
                          "[SOEM] %s assign 0x%04X count=%u\r\n",
                          assign_name,
                          assign_index,
                          (unsigned int)assign_count);

  read_count = assign_count;
  if (read_count > ECAT_PDO_MAP_MAX_ASSIGN)
  {
    ECAT_LogPrintf(log,
                            "[SOEM] %s assign count capped %u -> %u\r\n",
                            assign_name,
                            (unsigned int)assign_count,
                            (unsigned int)ECAT_PDO_MAP_MAX_ASSIGN);
    read_count = ECAT_PDO_MAP_MAX_ASSIGN;
    if (cache != 0)
    {
      cache->complete = 0U;
    }
  }

  for (assign_subindex = 1U; assign_subindex <= read_count; assign_subindex++)
  {
    if (ECAT_SdoReadU16(context,
                                  log,
                                  slave,
                                  assign_index,
                                  assign_subindex,
                                  &pdo_index) == 0U)
    {
      ECAT_LogPrintf(log,
                              "[SOEM] %s assign 0x%04X:%02X read PDO index FAIL\r\n",
                              assign_name,
                              assign_index,
                              assign_subindex);
      if (cache != 0)
      {
        cache->complete = 0U;
      }
      continue;
    }

    ECAT_LogPrintf(log,
                            "[SOEM] %s assign 0x%04X:%02X PDO=0x%04X\r\n",
                            assign_name,
                            assign_index,
                            assign_subindex,
                            pdo_index);

    if (pdo_index != 0U)
    {
      ECAT_PdoMapReadMapping(context,
                                      log,
                                      slave,
                                      pdo_index,
                                      assign_name,
                                      assign_subindex,
                                      cache);
    }
  }
}

/**
 * @brief  读取一个 PDO 映射对象的全部条目并追加到映射缓存
 *
 * @param[in,out] context          SOEM 主站上下文
 * @param[in,out] log              EtherCAT 日志上下文
 * @param[in]     slave           要读取的从站索引
 * @param[in]     pdo_index       PDO 映射对象索引
 * @param[in]     assign_name     分配方向名称，用于日志标识
 * @param[in]     assign_subindex 该 PDO 在分配对象中的子索引
 * @param[in,out] cache            PDO 映射缓存，函数会追加条目并累计总位数
 *
 * @return
 * 无
 *
 * @warning
 * 所有指针参数必须有效，从站必须支持 CoE 并处于可进行邮箱通信的状态；映射数量
 * 超过 ECAT_PDO_MAP_MAX_MAPPING、缓存容量不足或条目读取失败时会将 cache->complete
 * 置零，调用方必须在函数返回后检查该标志
 */
static void ECAT_PdoMapReadMapping(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave,
  uint16 pdo_index,
  const char *assign_name,
  uint8 assign_subindex,
  ECAT_PdoMapCache *cache)
{
  uint8 mapping_count = 0U;
  uint8 mapping_subindex;
  uint8 read_count;
  uint8 bit_length;
  uint32 mapping_entry;
  uint32 total_bits = 0U;

  if (ECAT_SdoReadU8(context,
                               log,
                               slave,
                               pdo_index,
                               0x00U,
                               &mapping_count) == 0U)
  {
    ECAT_LogPrintf(log,
                            "[SOEM] %s assign%u PDO 0x%04X read mapping count FAIL\r\n",
                            assign_name,
                            (unsigned int)assign_subindex,
                            pdo_index);
    if (cache != 0)
    {
      cache->complete = 0U;
    }
    return;
  }

  ECAT_LogPrintf(log,
                          "[SOEM] %s assign%u PDO 0x%04X mapping_count=%u\r\n",
                          assign_name,
                          (unsigned int)assign_subindex,
                          pdo_index,
                          (unsigned int)mapping_count);

  read_count = mapping_count;
  if (read_count > ECAT_PDO_MAP_MAX_MAPPING)
  {
    ECAT_LogPrintf(log,
                            "[SOEM] PDO 0x%04X mapping count capped %u -> %u\r\n",
                            pdo_index,
                            (unsigned int)mapping_count,
                            (unsigned int)ECAT_PDO_MAP_MAX_MAPPING);
    read_count = ECAT_PDO_MAP_MAX_MAPPING;
    if (cache != 0)
    {
      cache->complete = 0U;
    }
  }

  for (mapping_subindex = 1U; mapping_subindex <= read_count; mapping_subindex++)
  {
    if (ECAT_SdoReadU32(context,
                                  log,
                                  slave,
                                  pdo_index,
                                  mapping_subindex,
                                  &mapping_entry) == 0U)
    {
      ECAT_LogPrintf(log,
                              "[SOEM] PDO 0x%04X:%02X mapping entry FAIL\r\n",
                              pdo_index,
                              mapping_subindex);
      if (cache != 0)
      {
        cache->complete = 0U;
      }
      continue;
    }

    bit_length = (uint8)(mapping_entry & 0xFFU);
    ECAT_PdoMapCacheMappingEntry(cache,
                                           pdo_index,
                                           assign_subindex,
                                           mapping_subindex,
                                           mapping_entry);

    total_bits += bit_length;
    ECAT_PdoMapPrintEntry(log,
                                    pdo_index,
                                    mapping_subindex,
                                    mapping_entry);
  }

  ECAT_LogPrintf(log,
                          "[SOEM] PDO 0x%04X total_bits=%lu total_bytes=%lu\r\n",
                          pdo_index,
                          (unsigned long)total_bits,
                          (unsigned long)((total_bits + 7U) / 8U));
}

/**
 * @brief  将 PDO 映射条目写入缓存并累计映射总位数
 *
 * @param[in,out] cache            PDO 映射缓存
 * @param[in]     pdo_index        条目所属的 PDO 映射对象索引
 * @param[in]     assign_subindex  该 PDO 在分配对象中的子索引
 * @param[in]     mapping_subindex 条目在 PDO 映射对象中的子索引
 * @param[in]     mapping_entry    PDO 映射条目，低 8 位为条目位长度
 *
 * @return
 * 无
 *
 * @warning
 * cache 必须已完成初始化；cache 为空时函数不执行任何操作，缓存已满时不会保存
 * 当前条目，并会将 complete 置零，但仍会把该条目的位长度累加到 total_bits
 */
static void ECAT_PdoMapCacheMappingEntry(
  ECAT_PdoMapCache *cache,
  uint16 pdo_index,
  uint8 assign_subindex,
  uint8 mapping_subindex,
  uint32 mapping_entry)
{
  ECAT_PdoMapEntry *cached_entry;

  if (cache == 0)
  {
    return;
  }

  if (cache->entry_count < ECAT_PDO_MAP_MAX_ENTRIES)
  {
    cached_entry = &cache->entries[cache->entry_count];
    cached_entry->mapping_entry = mapping_entry;
    cached_entry->direction_bit_offset = cache->total_bits;
    cached_entry->pdo_index = pdo_index;
    cached_entry->assign_subindex = assign_subindex;
    cached_entry->mapping_subindex = mapping_subindex;
    cache->entry_count++;
  }
  else
  {
    cache->complete = 0U;
  }

  cache->total_bits += (uint8)(mapping_entry & 0xFFU);
}

/**
 * @brief  解码并打印一个 PDO 映射条目
 *
 * @param[in,out] log            EtherCAT 日志上下文
 * @param[in]     pdo_index      条目所属的 PDO 映射对象索引
 * @param[in]     entry_subindex 条目在 PDO 映射对象中的子索引
 * @param[in]     mapping_entry  PDO 映射条目编码值
 *
 * @return
 * 无
 *
 * @warning
 * log 必须有效且日志输出接口已完成初始化；mapping_entry 应按 PDO 映射格式编码，
 * 其高 16 位为对象索引、中间 8 位为对象子索引、低 8 位为映射位长度，本函数不校验其合法性
 */
static void ECAT_PdoMapPrintEntry(
  ECAT_LogContext *log,
  uint16 pdo_index,
  uint8 entry_subindex,
  uint32 mapping_entry)
{
  uint16 object_index;
  uint8 object_subindex;
  uint8 bit_length;

  object_index = (uint16)((mapping_entry >> 16) & 0xFFFFU);
  object_subindex = (uint8)((mapping_entry >> 8) & 0xFFU);
  bit_length = (uint8)(mapping_entry & 0xFFU);

  ECAT_LogPrintf(log,
                          "[SOEM] PDO 0x%04X:%02X map=0x%08lX -> 0x%04X:%02X %u bit\r\n",
                          pdo_index,
                          entry_subindex,
                          (unsigned long)mapping_entry,
                          object_index,
                          object_subindex,
                          (unsigned int)bit_length);
}

void ECAT_PdoMapPrintResolved(ECAT_LogContext *log,
                                        const uint8 *iomap,
                                        uint32 iomap_capacity,
                                        uint16 slave,
                                        const char *direction,
                                        const ECAT_PdoMapCache *cache,
                                        const uint8 *pdo_base,
                                        uint8 start_bit,
                                        uint32 mapped_bits,
                                        int iomap_size)
{
  uint16 entry_index;
  const ECAT_PdoMapEntry *entry;
  ECAT_PdoMapResolveState state;

  if (ECAT_PdoMapPrepareResolved(log,
                                           iomap,
                                           iomap_capacity,
                                           slave,
                                           direction,
                                           cache,
                                           pdo_base,
                                           start_bit,
                                           mapped_bits,
                                           iomap_size,
                                           &state) == 0U)
  {
    return;
  }

  for (entry_index = 0U; entry_index < cache->entry_count; entry_index++)
  {
    entry = &cache->entries[entry_index];

    if ((entry_index == 0U) ||
        (entry->pdo_index != state.current_pdo) ||
        (entry->assign_subindex != state.current_assign_subindex))
    {
      if (entry_index != 0U)
      {
        ECAT_PdoMapPrintResolvedPdo(
          log,
          direction,
          state.iomap_base_offset,
          start_bit,
          state.current_pdo,
          state.current_assign_subindex,
          state.pdo_start_bit,
          entry->direction_bit_offset);
      }

      state.current_pdo = entry->pdo_index;
      state.current_assign_subindex = entry->assign_subindex;
      state.pdo_start_bit = entry->direction_bit_offset;
    }

    if (ECAT_PdoMapPrintResolvedEntry(log,
                                                direction,
                                                entry,
                                                start_bit,
                                                &state) == 0U)
    {
      state.range_ok = 0U;
    }
  }

  if (cache->entry_count != 0U)
  {
    ECAT_PdoMapPrintResolvedPdo(log,
                                          direction,
                                          state.iomap_base_offset,
                                          start_bit,
                                          state.current_pdo,
                                          state.current_assign_subindex,
                                          state.pdo_start_bit,
                                          cache->total_bits);
  }

  ECAT_PdoMapPrintResolvedSummary(log,
                                            slave,
                                            direction,
                                            cache,
                                            start_bit,
                                            mapped_bits,
                                            &state);
}

static uint8 ECAT_PdoMapPrepareResolved(
  ECAT_LogContext *log,
  const uint8 *iomap,
  uint32 iomap_capacity,
  uint16 slave,
  const char *direction,
  const ECAT_PdoMapCache *cache,
  const uint8 *pdo_base,
  uint8 start_bit,
  uint32 mapped_bits,
  int iomap_size,
  ECAT_PdoMapResolveState *state)
{
  uintptr_t iomap_begin;
  uintptr_t iomap_end;
  uintptr_t pdo_address;

  ECAT_LogSectionLine(log);
  if ((iomap == 0) || (cache == 0) || (pdo_base == 0) ||
      (iomap_size <= 0) || ((uint32)iomap_size > iomap_capacity))
  {
    ECAT_LogPrintf(log,
                            "[SOEM] IOmap %s slave%d resolve FAIL: invalid argument\r\n",
                            direction,
                            slave);
    return 0U;
  }

  if (cache->complete == 0U)
  {
    ECAT_LogPrintf(log,
                            "[SOEM] IOmap %s slave%d resolve FAIL: PDO mapping cache incomplete entries=%u bits=%lu\r\n",
                            direction,
                            slave,
                            (unsigned int)cache->entry_count,
                            (unsigned long)cache->total_bits);
    return 0U;
  }

  iomap_begin = (uintptr_t)&iomap[0];
  iomap_end = iomap_begin + (uint32)iomap_size;
  pdo_address = (uintptr_t)pdo_base;
  if ((pdo_address < iomap_begin) || (pdo_address >= iomap_end))
  {
    ECAT_LogPrintf(log,
                            "[SOEM] IOmap %s slave%d resolve FAIL: base=0x%08lX outside IOmap\r\n",
                            direction,
                            slave,
                            (unsigned long)pdo_address);
    return 0U;
  }

  state->iomap_base_offset = (uint32)(pdo_address - iomap_begin);
  state->iomap_size_bits = (uint32)iomap_size * 8U;
  state->pdo_start_bit = 0U;
  state->current_pdo = 0U;
  state->current_assign_subindex = 0U;
  state->range_ok = 1U;
  ECAT_LogPrintf(log,
                          "[SOEM] IOmap %s slave%d base=+0x%04lX startbit=%u mapped_bits=%lu mapped_bytes=%lu entries=%u\r\n",
                          direction,
                          slave,
                          (unsigned long)state->iomap_base_offset,
                          (unsigned int)start_bit,
                          (unsigned long)mapped_bits,
                          (unsigned long)((mapped_bits + 7U) / 8U),
                          (unsigned int)cache->entry_count);
  return 1U;
}

static void ECAT_PdoMapPrintResolvedPdo(
  ECAT_LogContext *log,
  const char *direction,
  uint32 iomap_base_offset,
  uint8 start_bit,
  uint16 pdo_index,
  uint8 assign_subindex,
  uint32 pdo_start_bit,
  uint32 pdo_end_bit)
{
  uint32 pdo_bits;
  uint32 pdo_start_relative_bit;
  uint32 pdo_start_byte_offset;

  pdo_bits = pdo_end_bit - pdo_start_bit;
  pdo_start_relative_bit = (uint32)start_bit + pdo_start_bit;
  pdo_start_byte_offset = iomap_base_offset +
                          (pdo_start_relative_bit / 8U);
  ECAT_LogPrintf(log,
                          "[SOEM] IOmap %s PDO=0x%04X assign%u start=+0x%04lX.%u bits=%lu bytes=%lu\r\n",
                          direction,
                          pdo_index,
                          (unsigned int)assign_subindex,
                          (unsigned long)pdo_start_byte_offset,
                          (unsigned int)(pdo_start_relative_bit % 8U),
                          (unsigned long)pdo_bits,
                          (unsigned long)((pdo_bits + 7U) / 8U));
}

static uint8 ECAT_PdoMapPrintResolvedEntry(
  ECAT_LogContext *log,
  const char *direction,
  const ECAT_PdoMapEntry *entry,
  uint8 start_bit,
  const ECAT_PdoMapResolveState *state)
{
  uint32 relative_bit;
  uint32 iomap_byte_offset;
  uint32 iomap_end_bit;
  uint16 object_index;
  uint8 object_subindex;
  uint8 bit_length;
  uint8 entry_ok;

  object_index = (uint16)((entry->mapping_entry >> 16) & 0xFFFFU);
  object_subindex = (uint8)((entry->mapping_entry >> 8) & 0xFFU);
  bit_length = (uint8)(entry->mapping_entry & 0xFFU);
  relative_bit = (uint32)start_bit + entry->direction_bit_offset;
  iomap_byte_offset = state->iomap_base_offset + (relative_bit / 8U);
  iomap_end_bit = (state->iomap_base_offset * 8U) +
                  relative_bit + bit_length;
  entry_ok = ((bit_length != 0U) &&
              (iomap_end_bit <= state->iomap_size_bits)) ? 1U : 0U;
  ECAT_LogPrintf(log,
                          "[SOEM] IOmap %s +0x%04lX.%u assign%u PDO=0x%04X:%02X map=0x%08lX -> 0x%04X:%02X %u bit %s\r\n",
                          direction,
                          (unsigned long)iomap_byte_offset,
                          (unsigned int)(relative_bit % 8U),
                          (unsigned int)entry->assign_subindex,
                          entry->pdo_index,
                          (unsigned int)entry->mapping_subindex,
                          (unsigned long)entry->mapping_entry,
                          object_index,
                          object_subindex,
                          (unsigned int)bit_length,
                          (entry_ok != 0U) ? "PASS" : "FAIL");
  return entry_ok;
}

static void ECAT_PdoMapPrintResolvedSummary(
  ECAT_LogContext *log,
  uint16 slave,
  const char *direction,
  const ECAT_PdoMapCache *cache,
  uint8 start_bit,
  uint32 mapped_bits,
  ECAT_PdoMapResolveState *state)
{
  uint32 iomap_end_bit;
  uint8 size_ok;

  iomap_end_bit = (state->iomap_base_offset * 8U) +
                  (uint32)start_bit + cache->total_bits;
  if (iomap_end_bit > state->iomap_size_bits)
  {
    state->range_ok = 0U;
  }
  size_ok = (cache->total_bits == mapped_bits) ? 1U : 0U;
  ECAT_LogPrintf(log,
                          "[SOEM] IOmap %s slave%d summary entries=%u mapping_bits=%lu slave_bits=%lu size=%s range=%s result=%s\r\n",
                          direction,
                          slave,
                          (unsigned int)cache->entry_count,
                          (unsigned long)cache->total_bits,
                          (unsigned long)mapped_bits,
                          (size_ok != 0U) ? "PASS" : "FAIL",
                          (state->range_ok != 0U) ? "PASS" : "FAIL",
                          ((size_ok != 0U) && (state->range_ok != 0U)) ?
                            "PASS" : "FAIL");
}

/**
 * @brief  将一组显式 PDO 映射条目写入指定从站的映射对象
 *
 * @param[in,out] context SOEM 主站上下文
 * @param[in,out] log     EtherCAT 日志上下文
 * @param[in]     slave   目标从站索引
 * @param[in]     map     显式 PDO 映射描述，包含对象索引、条目数组及条目数量
 *
 * @return
 * 映射对象禁用、全部条目写入及条目数量恢复均成功返回1；参数无效或任一 SDO 写入
 * 失败返回0
 *
 * @warning
 * context、log、map 及 map->entries 必须有效，map->entry_count 必须大于0；目标从站
 * 必须处于允许修改 PDO 的状态，且相关 PDO 分配应已禁用。本函数不执行回读或回滚，
 * 失败时映射对象可能保持禁用或包含部分新条目
 */
static uint8 ECAT_PdoMapWriteExplicit(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave,
  const ECAT_ExplicitPdoMap *map)
{
  uint8 entry_index;

  if ((map == 0) || (map->entries == 0) || (map->entry_count == 0U))
  {
    return 0U;
  }

  ECAT_LogPrintf(log,
                          "[SOEM] PO2SO configure PDO 0x%04X entries=%u\r\n",
                          map->index,
                          (unsigned int)map->entry_count);
  if (ECAT_SdoPdoWriteU8(context,
                                   log,
                                   slave,
                                   map->index,
                                   0x00U,
                                   0U) == 0U)
  {
    return 0U;
  }

  for (entry_index = 0U; entry_index < map->entry_count; entry_index++)
  {
    if (ECAT_SdoPdoWriteU32(context,
                                     log,
                                     slave,
                                     map->index,
                                     (uint8)(entry_index + 1U),
                                     map->entries[entry_index]) == 0U)
    {
      return 0U;
    }
  }

  if (ECAT_SdoPdoWriteU8(context,
                                   log,
                                   slave,
                                   map->index,
                                   0x00U,
                                   map->entry_count) == 0U)
  {
    return 0U;
  }
  return 1U;
}

/**
 * @brief  校验单个 PDO 映射对象是否与显式映射描述完全一致
 *
 * @param[in,out] context SOEM 主站上下文
 * @param[in,out] log     EtherCAT 日志上下文
 * @param[in]     slave   要校验的从站索引
 * @param[in]     map     期望的显式 PDO 映射描述
 *
 * @return
 * 实际条目数量及所有映射条目均与 map 一致返回1；参数无效、SDO 读取失败或任一
 * 内容不匹配时返回0
 *
 * @warning
 * context、log、map 及 map->entries 必须有效，map->entry_count 必须大于0；从站
 * 必须支持 CoE 并处于可进行邮箱通信的状态。本函数同步执行多次 SDO 读取，
 * 仅按顺序比较条目，不会修改映射对象
 */
static uint8 ECAT_PdoMapVerifyExplicit(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave,
  const ECAT_ExplicitPdoMap *map)
{
  uint8 actual_count;
  uint8 entry_index;
  uint32 actual_entry;

  if ((map == 0) || (map->entries == 0) || (map->entry_count == 0U))
  {
    return 0U;
  }

  if (ECAT_SdoPdoReadU8(context,
                                  log,
                                  slave,
                                  map->index,
                                  0x00U,
                                  &actual_count) == 0U)
  {
    return 0U;
  }
  if (actual_count != map->entry_count)
  {
    ECAT_LogPrintf(log,
                            "[SOEM] PO2SO verify PDO 0x%04X count=%u expected=%u FAIL\r\n",
                            map->index,
                            (unsigned int)actual_count,
                            (unsigned int)map->entry_count);
    return 0U;
  }

  for (entry_index = 0U; entry_index < map->entry_count; entry_index++)
  {
    if (ECAT_SdoPdoReadU32(context,
                                    log,
                                    slave,
                                    map->index,
                                    (uint8)(entry_index + 1U),
                                    &actual_entry) == 0U)
    {
      return 0U;
    }
    if (actual_entry != map->entries[entry_index])
    {
      ECAT_LogPrintf(log,
                              "[SOEM] PO2SO verify PDO 0x%04X:%02X value=0x%08lX expected=0x%08lX FAIL\r\n",
                              map->index,
                              (unsigned int)(entry_index + 1U),
                              (unsigned long)actual_entry,
                              (unsigned long)map->entries[entry_index]);
      return 0U;
    }
  }
  return 1U;
}

/**
 * @brief  按指定顺序写入从站的 PDO 分配对象
 *
 * @param[in,out] context      SOEM 主站上下文
 * @param[in,out] log          EtherCAT 日志上下文
 * @param[in]     slave        目标从站索引
 * @param[in]     assign_index PDO 分配对象索引
 * @param[in]     pdo_indices  按分配顺序排列的 PDO 映射对象索引数组
 * @param[in]     pdo_count    要分配的 PDO 数量
 *
 * @return
 * 分配对象禁用、全部 PDO 索引写入及数量恢复均成功返回1；参数无效或任一 SDO 写入
 * 失败返回0
 *
 * @warning
 * context、log 和 pdo_indices 必须有效，pdo_count 必须大于0；目标从站必须处于允许
 * 修改 PDO 分配的状态。本函数会先将子索引 0 写为0，且不执行回读或回滚，失败时
 * 分配对象可能保持禁用或包含部分新索引
 */
static uint8 ECAT_PdoMapWriteAssignment(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave,
  uint16 assign_index,
  const uint16 *pdo_indices,
  uint8 pdo_count)
{
  uint8 pdo_number;

  if ((pdo_indices == 0) || (pdo_count == 0U))
  {
    return 0U;
  }

  if (ECAT_SdoPdoWriteU8(context,
                                   log,
                                   slave,
                                   assign_index,
                                   0x00U,
                                   0U) == 0U)
  {
    return 0U;
  }
  for (pdo_number = 0U; pdo_number < pdo_count; pdo_number++)
  {
    if (ECAT_SdoPdoWriteU16(context,
                                     log,
                                     slave,
                                     assign_index,
                                     (uint8)(pdo_number + 1U),
                                     pdo_indices[pdo_number]) == 0U)
    {
      return 0U;
    }
  }
  if (ECAT_SdoPdoWriteU8(context,
                                   log,
                                   slave,
                                   assign_index,
                                   0x00U,
                                   pdo_count) == 0U)
  {
    return 0U;
  }
  return 1U;
}

/**
 * @brief  校验 PDO 分配对象中的数量及索引顺序是否符合预期
 *
 * @param[in,out] context      SOEM 主站上下文
 * @param[in,out] log          EtherCAT 日志上下文
 * @param[in]     slave        要校验的从站索引
 * @param[in]     assign_index PDO 分配对象索引
 * @param[in]     pdo_indices  按期望顺序排列的 PDO 映射对象索引数组
 * @param[in]     pdo_count    期望的 PDO 数量
 *
 * @return
 * 实际数量及每个 PDO 索引均与预期一致返回1；参数无效、SDO 读取失败、数量或顺序
 * 不匹配时返回0
 *
 * @warning
 * context、log 和 pdo_indices 必须有效，pdo_count 必须大于0；从站必须支持 CoE
 * 并处于可进行邮箱通信的状态。本函数会同步执行多次 SDO 读取，不修改分配对象
 */
static uint8 ECAT_PdoMapVerifyAssignment(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave,
  uint16 assign_index,
  const uint16 *pdo_indices,
  uint8 pdo_count)
{
  uint8 actual_count;
  uint8 pdo_number;
  uint16 actual_index;

  if ((pdo_indices == 0) || (pdo_count == 0U))
  {
    return 0U;
  }

  if (ECAT_SdoPdoReadU8(context,
                                  log,
                                  slave,
                                  assign_index,
                                  0x00U,
                                  &actual_count) == 0U)
  {
    return 0U;
  }
  if (actual_count != pdo_count)
  {
    ECAT_LogPrintf(log,
                            "[SOEM] PO2SO verify assign 0x%04X count=%u expected=%u FAIL\r\n",
                            assign_index,
                            (unsigned int)actual_count,
                            (unsigned int)pdo_count);
    return 0U;
  }

  for (pdo_number = 0U; pdo_number < pdo_count; pdo_number++)
  {
    if (ECAT_SdoPdoReadU16(context,
                                    log,
                                    slave,
                                    assign_index,
                                    (uint8)(pdo_number + 1U),
                                    &actual_index) == 0U)
    {
      return 0U;
    }
    if (actual_index != pdo_indices[pdo_number])
    {
      ECAT_LogPrintf(log,
                              "[SOEM] PO2SO verify assign 0x%04X:%02X value=0x%04X expected=0x%04X FAIL\r\n",
                              assign_index,
                              (unsigned int)(pdo_number + 1U),
                              actual_index,
                              pdo_indices[pdo_number]);
      return 0U;
    }
  }
  return 1U;
}

/**
 * @brief  校验指定从站的完整显式 PDO 分配与映射配置
 *
 * @param[in,out] context SOEM 主站上下文
 * @param[in,out] log     EtherCAT 日志上下文
 * @param[in]     slave   要校验的从站索引
 *
 * @return
 * 0x1C12、0x1C13 分配对象、全部显式 PDO 映射条目及 Rx/Tx 总位数均符合预期返回1；
 * 任一 SDO 读取失败或配置不匹配返回0
 *
 * @warning
 * context 和 log 必须有效，从站必须支持 CoE 并处于可进行邮箱通信的状态；本函数会
 * 同步执行多次 SDO 读取，仅验证当前配置，不会修改从站对象字典
 */
static uint8 ECAT_PdoMapVerifyExplicitConfiguration(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave)
{
  uint32 map_index;
  uint32 entry_index;
  uint32 rx_bits = 0U;
  uint32 tx_bits = 0U;
  const ECAT_ExplicitPdoMap *map;

  if (ECAT_PdoMapVerifyAssignment(
        context,
        log,
        slave,
        ECT_SDO_RXPDOASSIGN,
        ECAT_rxpdo_assign,
        (uint8)ECAT_PDO_MAP_ARRAY_COUNT(ECAT_rxpdo_assign)) == 0U)
  {
    return 0U;
  }
  if (ECAT_PdoMapVerifyAssignment(
        context,
        log,
        slave,
        ECT_SDO_TXPDOASSIGN,
        ECAT_txpdo_assign,
        (uint8)ECAT_PDO_MAP_ARRAY_COUNT(ECAT_txpdo_assign)) == 0U)
  {
    return 0U;
  }

  for (map_index = 0U;
       map_index < ECAT_PDO_MAP_ARRAY_COUNT(ECAT_explicit_pdo_maps);
       map_index++)
  {
    map = &ECAT_explicit_pdo_maps[map_index];
    if (ECAT_PdoMapVerifyExplicit(context, log, slave, map) == 0U)
    {
      return 0U;
    }
    for (entry_index = 0U; entry_index < map->entry_count; entry_index++)
    {
      if ((map->index == 0x1600U) || (map->index == 0x1610U))
      {
        rx_bits += map->entries[entry_index] & 0xFFU;
      }
      else
      {
        tx_bits += map->entries[entry_index] & 0xFFU;
      }
    }
  }

  if ((rx_bits != ECAT_PDO_MAP_RX_BITS) ||
      (tx_bits != ECAT_PDO_MAP_TX_BITS))
  {
    ECAT_LogPrintf(log,
                            "[SOEM] PO2SO verify bits Rx=%lu/%u Tx=%lu/%u FAIL\r\n",
                            (unsigned long)rx_bits,
                            (unsigned int)ECAT_PDO_MAP_RX_BITS,
                            (unsigned long)tx_bits,
                            (unsigned int)ECAT_PDO_MAP_TX_BITS);
    return 0U;
  }

  return 1U;
}

/**
 * @brief  在 PO2SO 阶段校验并按需重写指定从站的显式 PDO 配置
 *
 * @param[in,out] context          SOEM 回调传入的主站上下文
 * @param[in]     expected_context 期望的 SOEM 主站上下文地址，用于身份校验
 * @param[in,out] log              EtherCAT 日志上下文
 * @param[in,out] pdo_map          PDO 映射上下文，用于记录本次配置尝试及结果
 * @param[in]     config_slave     pdo_map 对应且允许配置的从站索引
 * @param[in]     slave            SOEM 回调传入的实际从站索引
 *
 * @return
 * 配置原本已匹配或重写并回读校验成功返回1；上下文、从站、状态或 CoE 能力无效，
 * 以及配置读写失败时返回0
 *
 * @warning
 * log 和 pdo_map 必须有效，context 必须与 expected_context 相同；目标从站必须处于
 * 无错误的 PRE-OP 状态并支持 CoE。本函数会同步读写从站对象字典，若确定性重写失败，
 * 将禁用 RxPDO 和 TxPDO 分配并保持禁用状态
 */
int ECAT_PdoMapConfigurePo2So(ecx_contextt *context,
                                        ecx_contextt *expected_context,
                                        ECAT_LogContext *log,
                                        ECAT_PdoMapContext *pdo_map,
                                        uint16 config_slave,
                                        uint16 slave)
{
  const ec_slavet *slave_info;

  pdo_map->po2so_attempted = 1U;
  pdo_map->po2so_ok = 0U;

  if ((context == 0) ||
      (context != expected_context) ||
      (slave != config_slave) ||
      (context->slavecount < (int)slave))
  {
    ECAT_LogPrintf(log,
                            "[SOEM] PO2SO invalid context/slave=%u FAIL\r\n",
                            (unsigned int)slave);
    return 0;
  }

  slave_info = &context->slavelist[slave];
  ECAT_LogSectionLine(log);
  ECAT_LogPrintf(log,
                          "[SOEM] PO2SO explicit PDO start slave%u vendor=0x%08lX product=0x%08lX state=0x%04X\r\n",
                          (unsigned int)slave,
                          (unsigned long)slave_info->eep_man,
                          (unsigned long)slave_info->eep_id,
                          (unsigned int)slave_info->state);

  if (((slave_info->state & 0x000FU) != EC_STATE_PRE_OP) ||
      ((slave_info->state & EC_STATE_ERROR) != 0U) ||
      ((slave_info->mbx_proto & ECT_MBXPROT_COE) == 0U))
  {
    ECAT_LogPrintf(log,
                            "[SOEM] PO2SO requires PRE-OP without error and CoE FAIL\r\n");
    return 0;
  }

  if (ECAT_PdoMapVerifyExplicitConfiguration(context,
                                                        log,
                                                        slave) != 0U)
  {
    pdo_map->po2so_ok = 1U;
    ECAT_LogPrintf(log,
                            "[SOEM] PO2SO explicit PDO already matches, no write needed PASS\r\n");
    return 1;
  }

  ECAT_LogPrintf(log,
                          "[SOEM] PO2SO explicit PDO mismatch, start deterministic rewrite\r\n");
  if (ECAT_PdoMapRewriteExplicitConfiguration(context,
                                                        log,
                                                        slave) == 0U)
  {
    ECAT_PdoMapDisableAssignments(context, log, slave);
    pdo_map->po2so_ok = 0U;
    ECAT_LogPrintf(
      log,
      "[SOEM] PO2SO explicit PDO FAIL, assignments left disabled\r\n");
    return 0;
  }

  pdo_map->po2so_ok = 1U;
  ECAT_LogPrintf(
    log,
    "[SOEM] PO2SO explicit PDO write and readback PASS Rx=%u Tx=%u bytes\r\n",
    (unsigned int)APP_ETHERCAT_SERVO_RXPDO_SIZE,
    (unsigned int)APP_ETHERCAT_SERVO_TXPDO_SIZE);
  return 1;
}

/**
 * @brief  确定性重写指定从站的显式 PDO 配置并进行回读校验
 *
 * @param[in,out] context SOEM 主站上下文
 * @param[in,out] log     EtherCAT 日志上下文
 * @param[in]     slave   要重写配置的从站索引
 *
 * @return
 * 全部映射对象及 RxPDO、TxPDO 分配写入成功且回读校验通过返回1；任一步骤失败返回0
 *
 * @warning
 * context 和 log 必须有效，目标从站必须处于无错误的 PRE-OP 状态、支持 CoE 且允许
 * 修改 PDO 对象。本函数会先禁用 0x1C12 和 0x1C13 分配，再重写映射并恢复分配；
 * 失败时不执行回滚，从站可能保留分配禁用或部分改写状态，调用方必须进行故障收尾
 */
static uint8 ECAT_PdoMapRewriteExplicitConfiguration(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave)
{
  uint32 map_index;

  /* 重写 PDO 映射前先禁用 RxPDO 分配。 */
  if (ECAT_SdoPdoWriteU8(context,
                                   log,
                                   slave,
                                   ECT_SDO_RXPDOASSIGN,
                                   0x00U,
                                   0U) == 0U)
  {
    return 0U;
  }
  /* 重写 PDO 映射前先禁用 TxPDO 分配。 */
  if (ECAT_SdoPdoWriteU8(context,
                                   log,
                                   slave,
                                   ECT_SDO_TXPDOASSIGN,
                                   0x00U,
                                   0U) == 0U)
  {
    return 0U;
  }

  for (map_index = 0U;
       map_index < ECAT_PDO_MAP_ARRAY_COUNT(ECAT_explicit_pdo_maps);
       map_index++)
  {
    if (ECAT_PdoMapWriteExplicit(
          context,
          log,
          slave,
          &ECAT_explicit_pdo_maps[map_index]) == 0U)
    {
      return 0U;
    }
  }

  if (ECAT_PdoMapWriteAssignment(
        context,
        log,
        slave,
        ECT_SDO_RXPDOASSIGN,
        ECAT_rxpdo_assign,
        (uint8)ECAT_PDO_MAP_ARRAY_COUNT(ECAT_rxpdo_assign)) == 0U)
  {
    return 0U;
  }
  if (ECAT_PdoMapWriteAssignment(
        context,
        log,
        slave,
        ECT_SDO_TXPDOASSIGN,
        ECAT_txpdo_assign,
        (uint8)ECAT_PDO_MAP_ARRAY_COUNT(ECAT_txpdo_assign)) == 0U)
  {
    return 0U;
  }

  return ECAT_PdoMapVerifyExplicitConfiguration(context,
                                                          log,
                                                          slave);
}

/**
 * @brief  尝试禁用指定从站的 RxPDO 和 TxPDO 分配
 *
 * @param[in,out] context SOEM 主站上下文
 * @param[in,out] log     EtherCAT 日志上下文
 * @param[in]     slave   目标从站索引
 *
 * @return
 * 无
 *
 * @warning
 * context 和 log 必须有效，目标从站必须支持 CoE、邮箱通信可用且处于允许修改 PDO
 * 分配的状态；本函数忽略两次 SDO 写入的返回值且不回读校验，失败仅由底层记录日志，
 * 返回后一个或两个分配对象仍可能处于启用状态
 */
static void ECAT_PdoMapDisableAssignments(
  ecx_contextt *context,
  ECAT_LogContext *log,
  uint16 slave)
{
  (void)ECAT_SdoPdoWriteU8(context,
                                     log,
                                     slave,
                                     ECT_SDO_RXPDOASSIGN,
                                     0x00U,
                                     0U);
  (void)ECAT_SdoPdoWriteU8(context,
                                     log,
                                     slave,
                                     ECT_SDO_TXPDOASSIGN,
                                     0x00U,
                                     0U);
}
