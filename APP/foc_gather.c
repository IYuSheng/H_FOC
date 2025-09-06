#include "foc_gather.h"

extern foc_data_t foc_raw_data;
extern foc_data_i foc_datai;
extern foc_data_v foc_datav;

// 全局变量，用于在任务间共享位置和速度信息
float g_speed_rpm = 0.0f;
float g_total_angle = 0.0f;
float g_total_rotations = 0.0f;
float g_angle = 0.0f;
void foc_gather_init(void)
{

}


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

      // debug_printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
      //                foc_datav.vbus,
      //                foc_datav.va, foc_datav.vb, foc_datav.vc,
      //                foc_datai.ia, foc_datai.ib, foc_datai.ic);
      
      // debug_printf("%.4f,%.4f,%.4f",foc_datai.ia, foc_datai.ib, foc_datai.ic);

      }

// --------------------------------------------------------------------------------------------------------------//
      
      // 更新位置和速度计算，使用100us时间戳
      g_angle = hall_update_position_and_speed(bsp_get_micros());
      
      // 获取速度和位置信息
      g_speed_rpm  = hall_get_speed_rpm();
      g_total_angle  = hall_get_total_mechanical_angle();
      g_total_rotations  = hall_get_total_rotations();
      
      // 打印HALL状态、电角度、速度和位置信息
      // debug_printf("%.6f,%.6f,%.2f", 
      //             g_speed_rpm , g_angle , g_total_rotations);

      // 精确延时1ms
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    }
}
