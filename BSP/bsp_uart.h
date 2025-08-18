#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "stm32f4xx.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// 发送缓冲区大小
#define UART_TX_BUFFER_SIZE 1024

// 串口调试相关函数
void bsp_uart_init(void);
void bsp_uart_send_byte(uint8_t data);
void bsp_uart_send_string(char *str);
void bsp_uart_send_buffer(uint8_t *buffer, uint16_t len);
void debug_log(const char *format, ...);

// DMA相关函数
void bsp_uart_dma_init(void);
void DMA1_Stream3_IRQHandler(void);
void USART3_IRQHandler(void);

#endif /* __BSP_UART_H */
