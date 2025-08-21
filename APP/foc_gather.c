#include "foc_gather.h"

extern foc_data_i foc_datai;
extern foc_data_v foc_datav;
void foc_gather_init(void)
{

}


void vGatherProcessTask(void *pvParameters)
{
  
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

      vTaskDelay(1000);
    }
}
