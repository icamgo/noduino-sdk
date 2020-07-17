#ifndef __HALSPECELCONFIG_H__
#define __HALSPECELCONFIG_H__

#define FREERTOS    1

#define  USE_ARDUINOSTREAMING 0

/*select Release or Release(exceptions) from menu should be closed the DEBUG auto*/
#ifdef USE_FULL_ASSERT

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
#define portINFO 1		/* Displaying port information at compiling */

#define configUSE_MALLOC_FAILED_HOOK   1
#define configCHECK_FOR_STACK_OVERFLOW 1

#endif				/* USE_FULL_ASSERT */

#if 0

#undef  configUSE_COUNTING_SEMAPHORES
#define configUSE_COUNTING_SEMAPHORES   1

#undef  INCLUDE_vTaskDelayUntil
#define INCLUDE_vTaskDelayUntil 1

#undef  configUSE_IDLE_HOOK
#define configUSE_IDLE_HOOK 1

#undef  configUSE_TICK_HOOK
#define configUSE_TICK_HOOK  1

#endif				/* 0 */
#endif	/*__HALSPECELCONFIG_H__*/
