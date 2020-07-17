/*
 * ARDUINO HAL special config file, overriden default define 
 *   config arduino/hal/os/priority/......
 *  
 *    for FreeRTOS search FreeRTOSConfig.h:
 *      1 this sketch project path if has
 *      2 {variant}/config/   if has
 *      3 FreeRTOS/default
 *      This file defined can overload above.
 *   
 *    2017.12 by huaweiwx@sina.com
 */

#ifndef __HALSPECELCONFIG_H__
#define __HALSPECELCONFIG_H__

#define FREERTOS    1

#define  USE_ARDUINOSTREAMING 1

/*select Release or Release(exceptions) from menu should be closed the DEBUG auto*/
#ifdef USE_FULL_ASSERT
#define portINFO 1		/* Displaying port information at compiling */

/* ------------------------------------------------------------------
 * set USE_ERRORBLINK 1  for _Error_Handler/AssertError output information redirect to led blinking the err code 
 * blink err code:
 *   HardFault       31
 *   MemManage fault 32
 *   BusFault        33
 *   UsageFault      34
 *   MallocFailed    22 (freertos if set configUSE_MALLOC_FAILED_HOOK 1)
 *   StackOverflow   23 (freertos if configCHECK_FOR_STACK_OVERFLOW 1)
 *   others          __LINE__   (err line from assert_failed or _Error_Handler )
 */
#define USE_ERRORBLINK 1

#define configUSE_MALLOC_FAILED_HOOK   1
#define configCHECK_FOR_STACK_OVERFLOW 1

#endif				/* USE_FULL_ASSERT */

#undef  configUSE_COUNTING_SEMAPHORES
#define configUSE_COUNTING_SEMAPHORES   1

#undef    configUSE_IDLE_HOOK
#define   configUSE_IDLE_HOOK           1

#if 0
#undef  INCLUDE_vTaskDelayUntil
#define INCLUDE_vTaskDelayUntil 1

#undef  configUSE_TICK_HOOK
#define configUSE_TICK_HOOK  1
#endif

#endif	/*__HALSPECELCONFIG_H__*/
