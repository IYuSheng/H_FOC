#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#if defined(__CC_ARM)
	#include <stdint.h>
	extern uint32_t SystemCoreClock;
#endif

//#define configSYSTICK_CLOCK_HZ SystemCoreClock
#define configUSE_PREEMPTION			1	//抢占式调度
#define configUSE_TIME_SLICING    		1	// 启用时间片轮转
#define configUSE_IDLE_HOOK				0	//空闲任务钩子，允许定义 vApplicationIdleHook() 函数，在空闲任务中执行后台操作（如低功耗模式）
#define configUSE_TICK_HOOK				0	//允许定义 vApplicationTickHook() 函数，在系统节拍中断中执行周期性操作
#define configCPU_CLOCK_HZ				( SystemCoreClock )
#define configTICK_RATE_HZ				( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES			( 5 )	//定义系统支持的任务优先级数量。
#define configMINIMAL_STACK_SIZE		( ( unsigned short ) 128 )	//空闲任务的最小堆栈大小
#define configTOTAL_HEAP_SIZE			( ( size_t ) ( 26 * 1024 ) )	//FreeRTOS 动态内存池的总大小
#define configMAX_TASK_NAME_LEN			( 16 )
#define configUSE_TRACE_FACILITY		1
#define configUSE_16_BIT_TICKS			0
#define configIDLE_SHOULD_YIELD			1
#define configUSE_MUTEXES				1
#define configQUEUE_REGISTRY_SIZE		8
#define configCHECK_FOR_STACK_OVERFLOW	2
#define configUSE_RECURSIVE_MUTEXES		1
#define configUSE_MALLOC_FAILED_HOOK	1
#define configUSE_APPLICATION_TASK_TAG	0
#define configUSE_COUNTING_SEMAPHORES	1

#define configGENERATE_RUN_TIME_STATS	0	//启用任务 CPU 使用率统计
#if configGENERATE_RUN_TIME_STATS
extern void configureTimerForRuntimeStats(void);
extern uint32_t getRuntimeCounterValue(void);
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() configureTimerForRuntimeStats()
#define portGET_RUN_TIME_COUNTER_VALUE() getRuntimeCounterValue()
#define configUSE_STATS_FORMATTING_FUNCTIONS 1
#endif


/* Software timer definitions. */
#define configUSE_TIMERS				1		//启用软件定时器功能
#define configTIMER_TASK_PRIORITY		( 2 )		//定时器服务任务的优先级
#define configTIMER_QUEUE_LENGTH		10		//定时器命令队列长度
#define configTIMER_TASK_STACK_DEPTH	( configMINIMAL_STACK_SIZE * 2 )	//定时器任务的堆栈大小

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		1
#define INCLUDE_vTaskDelete				1
#define INCLUDE_vTaskCleanUpResources	1
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay				1

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
	/* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
	#define configPRIO_BITS       		__NVIC_PRIO_BITS
#else
	#define configPRIO_BITS       		4        /* 15 priority levels */	//指定 NVIC 优先级寄存器位数（通常与芯片相关）。对于 Cortex-M3/M4，通常为 4
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY			0xf		//定义最低中断优先级

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	5		//允许调用 FreeRTOS API 的最高中断优先级。优先级高于此值的中断不能调用 FreeRTOS API

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#endif /* FREERTOS_CONFIG_H */

