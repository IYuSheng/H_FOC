#include "stm32f4xx.h"
#include "foc_init.h"

int main(void)
{
	// bsp硬件层初始化
	foc_bsp_init();
	// app接口程序初始化
	foc_app_init();
	// task任务初始化
	foc_task_init();

	/* ------------------------------ 启动调度器 ---------------------------------- */
	
  vTaskStartScheduler();
	
    while(1)
    {
			
    }
}

/* FreeRTOS回调函数 */
void vApplicationMallocFailedHook(void)
{
  debug_log("Application malloc failed");
  for(;;) { /* 内存分配失败时系统挂起 */ }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  /* 堆栈溢出处理 */
  (void)xTask;
  (void)pcTaskName;
  debug_log("\r\n!!! Stack Overflow in %s !!!\r\n", pcTaskName);
  while (1);
}
