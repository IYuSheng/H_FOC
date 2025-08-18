#include "led.h"
#include "bsp_uart.h"
void led_init(void)
{
  // 初始化先点亮两个LED灯
  GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);  // 点亮
}

void vLEDProcessTask(void *pvParameters)
{
  while (1)
    {
      // 闪烁两个LED灯
      GPIO_SetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);  // 点亮
      vTaskDelay(500);                                 // 亮500ms
      GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1); // 熄灭
      vTaskDelay(1000);
    }
}
