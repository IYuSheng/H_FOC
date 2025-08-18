#ifndef __LED_H
#define __LED_H

#include "stm32f4xx.h"
#include "freertos.h"
#include "task.h"


void led_init(void);

void vLEDProcessTask(void *pvParameters);

#endif /* __LED_H */
