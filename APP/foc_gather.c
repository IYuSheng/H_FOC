#include "foc_gather.h"

extern foc_data_i foc_datai;
extern foc_data_v foc_datav;
void foc_gather_init(void)
{

}


void vGatherProcessTask(void *pvParameters)
{
  float speed_rpm;
  float total_angle;
  float total_rotations;
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

      // debug_printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
      //                foc_datav.vbus,
      //                foc_datav.va, foc_datav.vb, foc_datav.vc,
      //                foc_datai.ia, foc_datai.ib, foc_datai.ic);
      }

// --------------------------------------------------------------------------------------------------------------//
      
      // 更新位置和速度计算，使用FreeRTOS时间戳
      hall_update_position_and_speed(xTaskGetTickCount());
      
      // 获取速度和位置信息
      speed_rpm = hall_get_speed_rpm();
      total_angle = hall_get_total_mechanical_angle();
      total_rotations = hall_get_total_rotations();
      
      // 打印HALL状态、电角度、速度和位置信息
      debug_printf("%.2f,%.2f,%.4f", 
                  speed_rpm, total_angle, total_rotations);

      // 精确延时1ms
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
