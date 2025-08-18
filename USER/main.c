#include "stm32f4xx.h"
#include "foc_init.h"

int main(void)
{
	// bspӲ�����ʼ��
	foc_bsp_init();
	// app�ӿڳ����ʼ��
	foc_app_init();
	// task�����ʼ��
	foc_task_init();

	/* ------------------------------ ���������� ---------------------------------- */
	
  vTaskStartScheduler();
	
    while(1)
    {
			
    }
}

/* FreeRTOS�ص����� */
void vApplicationMallocFailedHook(void)
{
  debug_log("Application malloc failed");
  for(;;) { /* �ڴ����ʧ��ʱϵͳ���� */ }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  /* ��ջ������� */
  (void)xTask;
  (void)pcTaskName;
  debug_log("\r\n!!! Stack Overflow in %s !!!\r\n", pcTaskName);
  while (1);
}
