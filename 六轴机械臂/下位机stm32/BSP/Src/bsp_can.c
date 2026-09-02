#include "bsp_can.h"

#include "fdcan.h"

#define BSP_CAN_STANDARD_ID_MAX  0x7FFU
#define BSP_CAN_EXTENDED_ID_MAX  0x1FFFFFFFU
#define BSP_CAN_TDC_VALUE_MAX    0x7FU
#define BSP_CAN_TDC_FILTER       0U
#define BSP_CAN_RX_QUEUE_DEPTH   16U
#define BSP_CAN_RX_QUEUE_MASK    (BSP_CAN_RX_QUEUE_DEPTH - 1U)
#define BSP_CAN_RECOVERY_DELAY_MS  100U

#define BSP_CAN_NOTIFICATION_MASK  (FDCAN_IT_RX_FIFO0_NEW_MESSAGE  | /* FIFO0 写入新报文 */ \
                                    FDCAN_IT_RX_FIFO0_FULL         | /* FIFO0 已满 */ \
                                    FDCAN_IT_RX_FIFO0_MESSAGE_LOST | /* FIFO0 报文丢失 */ \
                                    FDCAN_IT_TX_COMPLETE           | /* 发送完成 */ \
                                    FDCAN_IT_ERROR_WARNING         | /* 错误警告状态变化 */ \
                                    FDCAN_IT_ERROR_PASSIVE         | /* 错误被动状态变化 */ \
                                    FDCAN_IT_BUS_OFF               | /* Bus-Off 状态变化 */ \
                                    FDCAN_IT_ARB_PROTOCOL_ERROR    | /* 仲裁阶段协议错误 */ \
                                    FDCAN_IT_DATA_PROTOCOL_ERROR   | /* 数据阶段协议错误 */ \
                                    FDCAN_IT_RAM_ACCESS_FAILURE)     /* 消息 RAM 访问失败 */

#define BSP_CAN_TX_BUFFER_MASK  (FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | \
                                 FDCAN_TX_BUFFER2 | FDCAN_TX_BUFFER3 | \
                                 FDCAN_TX_BUFFER4 | FDCAN_TX_BUFFER5 | \
                                 FDCAN_TX_BUFFER6 | FDCAN_TX_BUFFER7)

static bool bsp_can_initialized = false;
static BSP_CAN_Frame bsp_can_rx_queue[BSP_CAN_RX_QUEUE_DEPTH]; /* 接收中断生产、任务侧消费的软件环形队列 */
static volatile uint32_t bsp_can_rx_write_index = 0U;
static volatile uint32_t bsp_can_rx_read_index = 0U;
static BSP_CAN_Stats bsp_can_stats;
static uint32_t bsp_can_handled_hal_errors = 0U;
static volatile bool bsp_can_bus_off_pending = false;
static volatile uint32_t bsp_can_bus_off_tick = 0U;

static const uint8_t bsp_can_dlc_to_length[16] =
{
  0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
  8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U
};

static const uint32_t bsp_can_dlc_values[16] =
{
  FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1,
  FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
  FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
  FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
  FDCAN_DLC_BYTES_8, FDCAN_DLC_BYTES_12,
  FDCAN_DLC_BYTES_16, FDCAN_DLC_BYTES_20,
  FDCAN_DLC_BYTES_24, FDCAN_DLC_BYTES_32,
  FDCAN_DLC_BYTES_48, FDCAN_DLC_BYTES_64
};

/**
 * @brief  将 CAN/CAN FD 有效数据长度转换为 HAL FDCAN DLC 编码
 *
 * @param[in]  length 待转换的数据长度，单位：字节
 * @param[out] dlc    转换成功后写入对应的 FDCAN_DLC_BYTES_x 编码
 *
 * @return
 * length 属于标准 DLC 长度集合时写入编码并返回true，否则返回false
 *
 * @warning
 * dlc 必须指向有效且可写的 uint32_t 对象；支持的长度为 0～8、12、16、20、24、32、
 * 48 和 64 字节，转换失败时不会修改 dlc 指向的原值
 */
static bool BSP_CAN_LengthToDlc(uint8_t length, uint32_t *dlc)
{
  uint32_t index;

  /* 仅接受 CAN/CAN FD 标准 DLC 对应的数据长度。 */
  for (index = 0U; index < 16U; index++)
  {
    if (bsp_can_dlc_to_length[index] == length)
    {
      *dlc = bsp_can_dlc_values[index];
      return true;
    }
  }

  return false;
}

static void BSP_CAN_RecordHalError(FDCAN_HandleTypeDef *hfdcan)
{
  uint32_t error_code = HAL_FDCAN_GetError(hfdcan);
  uint32_t new_errors = error_code & ~bsp_can_handled_hal_errors;

  /* 同一轮运行中每个 HAL 错误位只统计一次。 */
  if (new_errors == HAL_FDCAN_ERROR_NONE)
  {
    return;
  }

  bsp_can_stats.hal_errors++;
  bsp_can_stats.last_hal_error = error_code;

  if ((new_errors & HAL_FDCAN_ERROR_PROTOCOL_ARBT) != 0U)
  {
    bsp_can_stats.arbitration_errors++;
  }

  if ((new_errors & HAL_FDCAN_ERROR_PROTOCOL_DATA) != 0U)
  {
    bsp_can_stats.data_errors++;
  }

  bsp_can_handled_hal_errors |= new_errors;
}

/**
 * @brief  从接收中断向 CAN 软件环形队列压入一帧
 *
 * @param[in] frame 待复制入队的 CAN 帧
 *
 * @return
 * 成功复制帧并发布写索引时返回true；队列已满时丢弃新帧、累计丢帧计数并返回false
 *
 * @warning
 * 调用前必须完成 BSP_CAN_Init()，frame 必须有效且内容完整；本函数按单生产者、
 * 单消费者模型实现，只允许 FDCAN 接收中断写入，并由一个执行上下文通过
 * BSP_CAN_TryReceive() 读取，禁止其他任务或中断并发入队。函数不会唤醒接收任务，
 * 队列满时也不会覆盖已经排队的数据
 */
static bool BSP_CAN_RxQueuePushFromISR(const BSP_CAN_Frame *frame)
{
  uint32_t write_index = bsp_can_rx_write_index;

  /* 队列满时丢弃新帧，保留已经排队的数据。 */
  if ((write_index - bsp_can_rx_read_index) >= BSP_CAN_RX_QUEUE_DEPTH)
  {
    bsp_can_stats.rx_queue_dropped++;//软件队列溢出
    return false;
  }

  /* 先写入完整帧，再发布新的写索引。 */
  bsp_can_rx_queue[write_index & BSP_CAN_RX_QUEUE_MASK] = *frame; //访问数组时write_index通过掩码转换为物理下标
  __DMB();
  bsp_can_rx_write_index = write_index + 1U;

  return true;
}

/**
 * @brief  配置 CAN BSP 层接收过滤器并初始化软件运行状态
 *
 * @param[in] config 标准帧掩码过滤配置，ID 和掩码均限于 11 位范围
 *
 * @return
 * 配置成功返回 BSP_CAN_OK；config 无效或过滤值越界返回 BSP_CAN_INVALID_ARGUMENT；
 * FDCAN1 尚未就绪返回 BSP_CAN_NOT_READY；HAL 过滤器配置失败返回 BSP_CAN_ERROR
 *
 * @warning
 * 调用前必须由 CubeMX 底层初始化使 hfdcan1 处于 HAL_FDCAN_STATE_READY 状态。
 * 本函数不会启动 FDCAN，成功后还需调用 BSP_CAN_Start()；成功初始化会清空软件接收队列、
 * BSP 层统计、HAL 错误去重记录及 Bus-Off 待恢复状态，调用方如需保留数据应提前获取快照
 */
BSP_CAN_Result BSP_CAN_Init(const BSP_CAN_Config *config)
{
  FDCAN_FilterTypeDef filter = {0};

  /* 校验配置指针及 11 位标准 ID/掩码范围。 */
  if (config == NULL)
  {
    return BSP_CAN_INVALID_ARGUMENT;
  }

  if ((config->standard_filter_id > BSP_CAN_STANDARD_ID_MAX) ||
      (config->standard_filter_mask > BSP_CAN_STANDARD_ID_MAX))
  {
    return BSP_CAN_INVALID_ARGUMENT;
  }

  /* 确认 CubeMX 已完成 FDCAN 底层初始化。 */
  if (hfdcan1.State != HAL_FDCAN_STATE_READY)
  {
    return BSP_CAN_NOT_READY;
  }

  bsp_can_initialized = false;

  /* 配置标准 ID 掩码过滤器 0，匹配帧进入 Rx FIFO0。 */
  filter.IdType = FDCAN_STANDARD_ID;  //匹配标准 CAN ID 0x000 ～ 0x7FF
  filter.FilterIndex = 0U;
  filter.FilterType = FDCAN_FILTER_MASK;  //(接收ID & FilterID2) == (FilterID1 & FilterID2)
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = config->standard_filter_id;
  filter.FilterID2 = config->standard_filter_mask;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
  {
    return BSP_CAN_ERROR;
  }

  /* 拒绝未匹配帧、扩展帧及所有远程帧。 */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_REJECT,  // 未匹配标准帧：拒绝
                                   FDCAN_REJECT,  // 未匹配扩展帧：拒绝
                                   FDCAN_REJECT_REMOTE, // 标准远程帧：拒绝
                                   FDCAN_REJECT_REMOTE) != HAL_OK)  // 扩展远程帧：拒绝
  {
    return BSP_CAN_ERROR;
  }

  /* 初始化静态接收队列。 */
  bsp_can_rx_write_index = 0U;
  bsp_can_rx_read_index = 0U;
  BSP_CAN_ResetStats();
  bsp_can_handled_hal_errors = 0U;
  bsp_can_bus_off_pending = false;
  bsp_can_bus_off_tick = 0U;
  bsp_can_initialized = true;

  return BSP_CAN_OK;
}

/**
 * @brief  尝试从 CAN 软件接收队列取出最早到达的一帧
 *
 * @param[out] frame 成功取帧后写入完整的 CAN 帧
 *
 * @return
 * 成功复制并移除队首帧时返回true；frame 为空或接收队列为空时返回false
 *
 * @warning
 * 调用前必须完成 BSP_CAN_Init()，frame 必须指向有效且可写的 BSP_CAN_Frame 对象。
 * 本函数为非阻塞接口，返回false时不会修改 frame 指向的内容；软件队列按单生产者、
 * 单消费者模型实现，只允许一个任务或执行上下文调用本函数，禁止多个消费者并发取帧
 */
bool BSP_CAN_TryReceive(BSP_CAN_Frame *frame)
{
  uint32_t read_index;

  /* 校验输出指针并检查软件接收队列是否为空。 */
  if (frame == NULL)
  {
    return false;
  }

  read_index = bsp_can_rx_read_index;
  if (read_index == bsp_can_rx_write_index)
  {
    return false;
  }

  /* 取出一帧后再释放对应的队列槽位。 */
  __DMB();
  *frame = bsp_can_rx_queue[read_index & BSP_CAN_RX_QUEUE_MASK];
  __DMB();
  bsp_can_rx_read_index = read_index + 1U;

  return true;
}

uint32_t BSP_CAN_GetRxPending(void)
{
  /* 返回软件接收队列中等待处理的帧数。 */
  return bsp_can_rx_write_index - bsp_can_rx_read_index;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rx_header;

  /* 仅处理 FDCAN1 的 FIFO0 接收相关通知。 */
  if ((hfdcan != &hfdcan1) ||
      ((RxFifo0ITs & (FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                      FDCAN_IT_RX_FIFO0_FULL |
                      FDCAN_IT_RX_FIFO0_MESSAGE_LOST)) == 0U))
  {
    return;
  }

  /* 记录硬件 FIFO0 满和报文丢失通知。 */
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL) != 0U)
  {
    bsp_can_stats.rx_fifo_full++;
  }

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
  {
    bsp_can_stats.rx_fifo_lost++;
  }

  /* 循环排空硬件 FIFO0，避免高负载下残留报文。 */
  /* HAL_FDCAN_GetRxFifoFillLevel() 用于查询 FDCAN 接收 FIFO 中当前存放了多少条报文 */
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    BSP_CAN_Frame frame = {0};

    if (HAL_FDCAN_GetRxMessage(hfdcan,
                               FDCAN_RX_FIFO0,
                               &rx_header,
                               frame.data) != HAL_OK)
    {
      BSP_CAN_RecordHalError(hfdcan);
      break;
    }

    /* 将 HAL 接收头转换为 BSP 统一帧格式。 */
    frame.id = rx_header.Identifier;
    frame.length = bsp_can_dlc_to_length[rx_header.DataLength & 0x0FU]; //取得 DLC 编码并转化为实机字节数
    frame.is_fd = (rx_header.FDFormat == FDCAN_FD_CAN) ? 1U : 0U;
    frame.bit_rate_switch = (rx_header.BitRateSwitch == FDCAN_BRS_ON) ? 1U : 0U;
    frame.id_type = (rx_header.IdType == FDCAN_EXTENDED_ID) ?
                    BSP_CAN_ID_EXTENDED : BSP_CAN_ID_STANDARD;

    bsp_can_stats.rx_frames++;
    bsp_can_stats.rx_bytes += frame.length;
    (void)BSP_CAN_RxQueuePushFromISR(&frame);
  }
}

/**
 * @brief  配置 FDCAN 运行通知和发送延迟补偿并启动 CAN 控制器
 *
 * @return
 * 启动成功返回 BSP_CAN_OK；BSP 层未初始化或 FDCAN1 不处于 READY 状态时返回
 * BSP_CAN_NOT_READY；TDC 偏移超限或任一 HAL 配置、通知激活及启动操作失败时返回
 * BSP_CAN_ERROR
 *
 * @warning
 * 调用前必须成功完成 BSP_CAN_Init()，且 FDCAN1 必须处于停止就绪状态；本函数不支持
 * 对已运行控制器重复启动。启动成功后会清除 HAL 错误去重记录和 Bus-Off 待恢复标志，
 * 运行期间应周期调用 BSP_CAN_Process() 以执行 Bus-Off 退避恢复
 */
BSP_CAN_Result BSP_CAN_Start(void)
{
  uint32_t tdc_offset;

  /* 确认过滤配置已完成且 FDCAN 处于可启动状态。 */
  if ((!bsp_can_initialized) || (hfdcan1.State != HAL_FDCAN_STATE_READY))
  {
    return BSP_CAN_NOT_READY;
  }

  /* 按当前数据段时序计算 TDC 初始偏移。 */
  tdc_offset = hfdcan1.Init.DataPrescaler * hfdcan1.Init.DataTimeSeg1;
  if (tdc_offset > BSP_CAN_TDC_VALUE_MAX)
  {
    return BSP_CAN_ERROR;
  }

  /* 配置并开启发送延迟补偿。 */
  /* TDC 的作用是补偿 MCU、CAN 收发器和线路产生的发送环回延迟，
  调整 CAN FD 数据阶段的次级采样点，避免在较高数据速率下采样过早。它主要用于启用了 BRS 的 CAN FD */
  if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1,
                                          tdc_offset,
                                          BSP_CAN_TDC_FILTER) != HAL_OK)
  {
    return BSP_CAN_ERROR;
  }

  if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
  {
    return BSP_CAN_ERROR;
  }

  /* 开启接收、发送完成和错误状态等中断通知。 */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                     BSP_CAN_NOTIFICATION_MASK,
                                     BSP_CAN_TX_BUFFER_MASK) != HAL_OK)
  {
    return BSP_CAN_ERROR;
  }

  /* 所有运行参数配置完成后再启动 FDCAN。 */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    return BSP_CAN_ERROR;
  }

  bsp_can_handled_hal_errors = 0U;
  bsp_can_bus_off_pending = false;

  return BSP_CAN_OK;
}

BSP_CAN_Result BSP_CAN_Stop(void)
{
  /* 已停止时直接返回成功。 */
  if (hfdcan1.State == HAL_FDCAN_STATE_READY)
  {
    bsp_can_bus_off_pending = false;
    return BSP_CAN_OK;
  }

  /* 仅允许停止正在运行的 FDCAN。 */
  if (hfdcan1.State != HAL_FDCAN_STATE_BUSY)
  {
    return BSP_CAN_NOT_READY;
  }

  if (HAL_FDCAN_Stop(&hfdcan1) != HAL_OK)
  {
    BSP_CAN_RecordHalError(&hfdcan1);
    return BSP_CAN_ERROR;
  }

  bsp_can_bus_off_pending = false;

  return BSP_CAN_OK;
}

BSP_CAN_Result BSP_CAN_Send(const BSP_CAN_Frame *frame)
{
  FDCAN_TxHeaderTypeDef tx_header = {0};
  uint32_t dlc;

  bsp_can_stats.tx_requests++;

  /* 校验帧指针、格式标志和标准 DLC 长度。 */
  if ((frame == NULL) ||
      (frame->is_fd > 1U) ||
      (frame->bit_rate_switch > 1U) ||
      (!BSP_CAN_LengthToDlc(frame->length, &dlc)))
  {
    return BSP_CAN_INVALID_ARGUMENT;
  }

  /* 经典 CAN 帧最多 8 字节且不能启用位速率切换。 */
  if ((frame->is_fd == 0U) &&
      ((frame->length > 8U) || (frame->bit_rate_switch != 0U)))
  {
    return BSP_CAN_INVALID_ARGUMENT;
  }

  /* 校验标准或扩展标识符范围。 */
  if (((frame->id_type == BSP_CAN_ID_STANDARD) &&
       (frame->id > BSP_CAN_STANDARD_ID_MAX)) ||
      ((frame->id_type == BSP_CAN_ID_EXTENDED) &&
       (frame->id > BSP_CAN_EXTENDED_ID_MAX)) ||
      ((frame->id_type != BSP_CAN_ID_STANDARD) &&
       (frame->id_type != BSP_CAN_ID_EXTENDED)))
  {
    return BSP_CAN_INVALID_ARGUMENT;
  }

  /* 仅在 FDCAN 已启动且未处于 Bus-Off 时允许提交发送请求。 */
  if ((!bsp_can_initialized) ||
      bsp_can_bus_off_pending ||
      (hfdcan1.State != HAL_FDCAN_STATE_BUSY))
  {
    return BSP_CAN_NOT_READY;
  }

  /* 硬件发送 FIFO 满时立即返回，不进行阻塞等待。 */
  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
  {
    bsp_can_stats.tx_busy++;
    return BSP_CAN_BUSY;
  }

  /* 将 BSP 帧属性转换为 HAL 发送头。 */
  tx_header.Identifier = frame->id;
  tx_header.IdType = (frame->id_type == BSP_CAN_ID_EXTENDED) ?
                     FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = dlc;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = (frame->bit_rate_switch != 0U) ?
                            FDCAN_BRS_ON : FDCAN_BRS_OFF;
  tx_header.FDFormat = (frame->is_fd != 0U) ?
                       FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0U;

  /* 非阻塞提交到硬件 Tx FIFO。 */
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,
                                    &tx_header,
                                    frame->data) != HAL_OK)
  {
    BSP_CAN_RecordHalError(&hfdcan1);
    return BSP_CAN_ERROR;
  }

  bsp_can_stats.tx_queued++;

  return BSP_CAN_OK;
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan,
                                        uint32_t BufferIndexes)
{
  uint32_t completed_count = 0U;

  /* 仅统计 FDCAN1 已完成发送的缓冲区数量。 */
  if (hfdcan != &hfdcan1)
  {
    return;
  }

  while (BufferIndexes != 0U)
  {
    completed_count += BufferIndexes & 1U;
    BufferIndexes >>= 1U;
  }

  bsp_can_stats.tx_completed += completed_count;
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs)
{
  FDCAN_ProtocolStatusTypeDef protocol_status;

  /* 仅记录 FDCAN1 的错误状态变化，恢复动作留给任务处理。 */
  if (hfdcan != &hfdcan1)
  {
    return;
  }

  if ((ErrorStatusITs & FDCAN_IT_ERROR_WARNING) != 0U)
  {
    bsp_can_stats.error_warning++;
  }

  if ((ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE) != 0U)
  {
    bsp_can_stats.error_passive++;
  }

  if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U)
  {
    /* 仅在进入 Bus-Off 状态时启动任务侧退避恢复。 */
    if ((HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol_status) == HAL_OK) &&
        (protocol_status.BusOff != 0U) &&
        (!bsp_can_bus_off_pending))
    {
      bsp_can_stats.bus_off++;
      bsp_can_bus_off_tick = HAL_GetTick();
      bsp_can_bus_off_pending = true;
    }
  }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  /* 保存 FDCAN1 本轮运行中新出现的 HAL 错误位。 */
  if (hfdcan == &hfdcan1)
  {
    BSP_CAN_RecordHalError(hfdcan);
  }
}

void BSP_CAN_GetStats(BSP_CAN_Stats *stats)
{
  uint32_t primask;

  if (stats == NULL)
  {
    return;
  }

  /* 在短临界区内获取一致的统计快照。 */
  primask = __get_PRIMASK();
  __disable_irq();
  __DMB();
  *stats = bsp_can_stats;
  __DMB();
  __set_PRIMASK(primask);
}

/**
 * @brief  清零全部 CAN BSP 层运行统计
 *
 * @return
 * 无
 *
 * @warning
 * 本函数会在短临界区内不可恢复地清除 bsp_can_stats，可在 BSP_CAN_Init() 前后调用；
 * 它不会清空软件接收队列，也不会改变初始化状态、Bus-Off 待恢复状态及 FDCAN 硬件状态，
 * 同时不会清除 HAL 错误去重掩码 bsp_can_handled_hal_errors，因此本轮运行中已处理的
 * HAL 错误位不会因统计清零而再次计数。调用方如需保留当前统计，应先通过
 * BSP_CAN_GetStats() 获取快照
 */
void BSP_CAN_ResetStats(void)
{
  BSP_CAN_Stats empty_stats = {0};
  uint32_t primask;

  /* 在短临界区内清零全部统计计数。 */
  primask = __get_PRIMASK();
  __disable_irq();
  __DMB();
  bsp_can_stats = empty_stats;
  __DMB();
  __set_PRIMASK(primask);
}

/**
 * @brief  在任务上下文处理 Bus-Off 挂起事件并退避重启 FDCAN
 *
 * @return
 * 无；无待恢复事件或退避时间未到时立即返回，恢复结果通过 BSP CAN 统计体现
 *
 * @warning
 * 调用前必须完成 BSP_CAN_Init() 和 BSP_CAN_Start()，并应由单一 CAN 服务任务周期调用，
 * 禁止在中断中执行或与 BSP_CAN_Init()/BSP_CAN_Start()/BSP_CAN_Stop() 及发送操作并发。
 * Bus-Off 挂起期间 BSP_CAN_Send() 会拒绝发送；每次停止或重启失败都会累计
 * recovery_failed、重新开始 BSP_CAN_RECOVERY_DELAY_MS 退避并在后续调用继续重试，当前
 * 实现没有最大重试次数。重启成功后累计 recovery_success 并清除 Bus-Off 挂起标志
 */
void BSP_CAN_Process(void)
{
  uint32_t current_tick;

  /* 无 Bus-Off 待恢复事件时立即返回。 */
  if (!bsp_can_bus_off_pending)
  {
    return;
  }

  /* 等待退避时间，避免故障总线上频繁重启。 */
  current_tick = HAL_GetTick();
  if ((current_tick - bsp_can_bus_off_tick) < BSP_CAN_RECOVERY_DELAY_MS)
  {
    return;
  }

  /* 在任务上下文停止控制器并重新启动。 */
  if ((hfdcan1.State == HAL_FDCAN_STATE_BUSY) &&
      (HAL_FDCAN_Stop(&hfdcan1) != HAL_OK))
  {
    BSP_CAN_RecordHalError(&hfdcan1);
    bsp_can_stats.recovery_failed++;
    bsp_can_bus_off_tick = current_tick;
    return;
  }

  if ((hfdcan1.State != HAL_FDCAN_STATE_READY) ||
      (BSP_CAN_Start() != BSP_CAN_OK))
  {
    bsp_can_stats.recovery_failed++;
    bsp_can_bus_off_tick = current_tick;
    return;
  }

  bsp_can_stats.recovery_success++;
  bsp_can_bus_off_pending = false;
}
