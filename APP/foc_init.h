#ifndef __FOC_INIT_H
#define __FOC_INIT_H

#include "stm32f4xx.h"
#include "freertos.h"
#include "task.h"

#include "Config.h"
#include "led.h"
#include "foc_conversion.h"
#include "foc_debug.h"
#include "foc_encoder.h"
#include "foc_gather.h"

#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "bsp_adc.h"
#include "bsp_timer.h"


#define TASK_PRIO_LED           2
#define TASK_PRIO_Debug         1
#define TASK_PRIO_Gather        3

void foc_bsp_init(void);
void foc_app_init(void);
void foc_task_init(void);


#endif /* __FOC_INIT_H */
