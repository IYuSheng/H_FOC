#include "bsp_uart.h"

static char d_buffer[256];  // 调试信息缓冲区

// 全局变量
static uint8_t dma_tx_buffer[UART_TX_BUFFER_SIZE];  // DMA发送缓冲区
static SemaphoreHandle_t uart_tx_semaphore = NULL;  // UART发送完成信号量

/**
 * @brief 初始化UART3 DMA
 */
void bsp_uart_dma_init(void)
{
  DMA_InitTypeDef DMA_InitStructure;

  // 使能DMA1时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  // 配置DMA1_Stream3用于USART3_TX
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;                      // USART3_TX使用DMA通道4
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR); // 外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dma_tx_buffer;    // 内存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;             // 内存到外设
  DMA_InitStructure.DMA_BufferSize = 0;                               // 数据传输量
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    // 外设地址不递增
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;             // 内存地址递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据宽度为字节
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;     // 内存数据宽度为字节
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                       // 正常模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;               // 中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;              // 不使用FIFO
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream3, &DMA_InitStructure);

  // 配置DMA中断
  NVIC->IP[DMA1_Stream3_IRQn] = 6 << 4;  // 抢占优先级 6（4 位），子优先级 0（0 位）
  NVIC_EnableIRQ(DMA1_Stream3_IRQn);

  // 使能DMA传输完成中断
  DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);

  // 创建信号量，用于同步DMA发送完成
  if (uart_tx_semaphore == NULL)
    {
      uart_tx_semaphore = xSemaphoreCreateBinary();
      // 给信号量赋初值，表示DMA可以使用
      if (uart_tx_semaphore != NULL)
        {
          xSemaphoreGive(uart_tx_semaphore);
        }
    }

  // 使能USART3的DMA发送功能
  USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
}

/**
 * @brief 初始化UART3，使用PB10(TX)和PB11(RX)
 */
void bsp_uart_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // 使能GPIOB和UART3时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  // 配置PB10为复用推挽输出(UART3_TX)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;         // 复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    // 速度50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       // 推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;         // 上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // 配置PB11为复用浮空输入(UART3_RX)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;         // 复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    // 速度50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;       // 推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;         // 上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // PB10和PB11复用为UART3
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

  // UART3初始化配置
  USART_InitStructure.USART_BaudRate = 115200;         // 波特率设置为115200
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;   // 字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;        // 一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;           // 无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
  USART_Init(USART3, &USART_InitStructure);            // 初始化串口3

  // 配置UART3中断
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;    // UART3中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; // 抢占优先级5
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;   // 子优先级1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;      // IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);                      // 根据指定的参数初始化NVIC

  // 配置Uart中断
  NVIC->IP[USART3_IRQn] = 5 << 4;  // 抢占优先级 5（4 位），子优先级 0（0 位）
  NVIC_EnableIRQ(USART3_IRQn);

  // 使能UART3接收中断
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

  // 初始化DMA
  bsp_uart_dma_init();

  // 使能UART3
  USART_Cmd(USART3, ENABLE);
}

/**
 * @brief UART3发送一个字节（阻塞方式）
 * @param data 要发送的数据
 */
void bsp_uart_send_byte(uint8_t data)
{
  // 带超时的等待（避免硬件异常导致永久阻塞）
  uint32_t timeout = 0xFFFF;
  while ((USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET) && (timeout-- > 0));
  if (timeout > 0)
    {
      USART_SendData(USART3, data);
    }
}

/**
 * @brief UART3发送字符串（阻塞方式）
 * @param str 要发送的字符串
 */
void bsp_uart_send_string(char *str)
{
  while (*str)
    {
      bsp_uart_send_byte(*str++);
    }
}

/**
 * @brief UART3发送缓冲区数据（阻塞方式，不使用DMA）
 * @param buffer 数据缓冲区
 * @param len 数据长度
 */
void bsp_uart_send(uint8_t *buffer, uint16_t len)
{
  uint16_t i;
  // 等待可以发送（即上一次传输已完成）
  for (i = 0; i < len; i++)
    {
      bsp_uart_send_byte(buffer[i]);
    }
}

/**
 * @brief 格式化打印调试信息（不使用DMA）
 * @param format 格式化字符串
 * @param ... 可变参数
 */
void debug_log(const char *format, ...)
{
  va_list args;
  int len;

  // 格式化字符串
  va_start(args, format);
  len = vsnprintf(d_buffer, sizeof(d_buffer), format, args);
  va_end(args);

  // 确保不会超出缓冲区范围
  if (len > sizeof(d_buffer) - 3)
    {
      len = sizeof(d_buffer) - 3;
    }

  // 添加\r\n确保换行
  if (len > 0)
    {
      d_buffer[len++] = '\r';
      d_buffer[len++] = '\n';
      d_buffer[len] = '\0';
    }

  if (len > 0)
    {
      bsp_uart_send((uint8_t*)d_buffer, (uint16_t)len);
    }
}

/**
 * @brief UART3通过DMA发送缓冲区数据
 * @param buffer 数据缓冲区
 * @param len 数据长度
 */
void bsp_uart_send_buffer(uint8_t *buffer, uint16_t len)
{
  if (len > UART_TX_BUFFER_SIZE)
    {
      len = UART_TX_BUFFER_SIZE; // 限制最大长度
    }
  // 等待可以发送（即上一次传输已完成）
  if (uart_tx_semaphore != NULL)
    {
      xSemaphoreTake(uart_tx_semaphore, portMAX_DELAY);
    }
  // 复制数据到DMA缓冲区
  memcpy(dma_tx_buffer, buffer, len);

  // 配置DMA传输
  DMA_Cmd(DMA1_Stream3, DISABLE);                     // 先关闭DMA
  while (DMA_GetCmdStatus(DMA1_Stream3) == ENABLE);   // 确保DMA已完全停止
  DMA1_Stream3->NDTR = len;                           // 数据传输数量
  DMA1_Stream3->M0AR = (uint32_t)dma_tx_buffer;       // 设置内存地址
  DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);        // 清除传输完成标志

  // 启动DMA传输
  DMA_Cmd(DMA1_Stream3, ENABLE);
}

/**
 * @brief DMA传输完成中断处理函数
 */
void DMA1_Stream3_IRQHandler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // 检查是否为传输完成中断
  if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) != RESET)
    {
      // 清除中断标志
      DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);

      // 给信号量，表示传输完成
      if (uart_tx_semaphore != NULL)
        {
          xSemaphoreGiveFromISR(uart_tx_semaphore, &xHigherPriorityTaskWoken);
        }

      // 停止当前DMA传输
      DMA_Cmd(DMA1_Stream3, DISABLE);

    }
  // 确保高优先级任务先执行
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief UART3中断服务函数
 */
void USART3_IRQHandler(void)
{
  // 判断是否为接收中断
  if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
      // 读取接收到的数据
      uint8_t res = USART_ReceiveData(USART3);

      // 将接收到的数据回传（回显）
      bsp_uart_send_byte(res);
    }
}
