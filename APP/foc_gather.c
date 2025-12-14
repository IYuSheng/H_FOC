#include "foc_gather.h"

// 全局变量，用于在任务间共享位置和速度信息
float32_t g_total_angle = 0.0f;
float32_t bemf_angle = 0.0f;
float32_t g_speed = 0.0f;
float32_t bemf_speed = 0.0f;

void vGatherProcessTask(void *pvParameters)
{
  portTickType xLastWakeTime;
  
  // 初始化xLastWakeTime变量
  xLastWakeTime = xTaskGetTickCount();
  
  while (1)
    {

     // 处理新的ADC数据（电压）
     if (bsp_adc_process_data())
     {
       // 获取校准后的电压ADC数据
       foc_datav = bsp_adc_get_calib_data();
     }

      g_speed = calculate_motor_speed_rpm();

      // 精确延时1ms
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
