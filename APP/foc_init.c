#include "foc_init.h"

BaseType_t xReturn;  /* 保存xTaskCreate返回值 */

void LED_task_create(void);
void Debug_task_create(void);
void Gather_task_create(void);
void FOC_Control_task_create(void);


void foc_bsp_init(void)
{
  NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);

  bsp_gpio_init();
  bsp_uart_init();
  bsp_adc_init();
  bsp_timer_init();

  debug_log("\nfoc_bsp_init");
}

void foc_app_init(void)
{
  led_init();
  debug_init();
  foc_gather_init();
  foc_control_init();

  debug_log("foc_app_init");
}

void foc_task_init(void)
{
  LED_task_create();
  Debug_task_create();
  Gather_task_create();
  FOC_Control_task_create();

  debug_log("foc_task_init");
}

/**
  * @brief  初始化LED指示灯任务
  */
void LED_task_create(void)
{
  xReturn = xTaskCreate(vLEDProcessTask, "LedProc", 64,
                        NULL, TASK_PRIO_LED, NULL);
  if (xReturn != pdPASS)
    {
      debug_log("LED task create Failed");
    }
}

/**
  * @brief  Debug任务
  */
void Debug_task_create(void)
{
  xReturn = xTaskCreate(vDebugProcessTask, "DebugProc", 1024,
                        NULL, TASK_PRIO_Debug, NULL);
  if (xReturn != pdPASS)
    {
      debug_log("Debug task create Failed");
    }
}

/**
  * @brief  采集电压电流数据任务
  */
void Gather_task_create(void)
{
  xReturn = xTaskCreate(vGatherProcessTask, "GatherProc", 1024,
                        NULL, TASK_PRIO_Gather, NULL);
  if (xReturn != pdPASS)
    {
      debug_log("Debug task create Failed");
    }
}

/**
  * @brief  FOC控制任务
  */
void FOC_Control_task_create(void)
{
  xReturn = xTaskCreate(vFOCControlTask, "FOCControl", 1024,
                        NULL, TASK_PRIO_FOC_Control, NULL);
  if (xReturn != pdPASS)
    {
      debug_log("FOC Control task create Failed");
    }
}
