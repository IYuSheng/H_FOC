#include "foc_debug.h"

static char debug_buffer[512];  // 调试信息缓冲区

/**
 * @brief 初始化调试功能
 */
void debug_init(void)
{
  // 调试功能初始化在bsp_uart_init中已完成
  // 这里可以添加其他调试相关的初始化
}

/**
 * @brief 格式化打印调试信息
 * @param format 格式化字符串
 * @param ... 可变参数
 */
void debug_printf(const char *format, ...)
{
  va_list args;
  int len;

  // 格式化字符串
  va_start(args, format);
  len = vsnprintf(debug_buffer, sizeof(debug_buffer), format, args);
  va_end(args);

  // 确保不会超出缓冲区范围
  if (len > sizeof(debug_buffer) - 3)
    {
      len = sizeof(debug_buffer) - 3;
    }

  // 添加\r\n确保换行
  if (len > 0)
    {
      debug_buffer[len++] = '\r';
      debug_buffer[len++] = '\n';
      debug_buffer[len] = '\0';
    }

  // 通过DMA发送
  bsp_uart_send_buffer((uint8_t*)debug_buffer, (uint16_t)len);
}

void vDebugProcessTask(void *pvParameters)
{
  while (1)
    {

      vTaskDelay(1000);
    }
}

