/*
 *  FreeRTOS V7.4.2 - Copyright (C) 2013 Real Time Engineers Ltd.
 *
 *  FEATURES AND PORTS ARE ADDED TO FREERTOS ALL THE TIME.  PLEASE VISIT
 *  http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.
 *
 ***************************************************************************
 *                                                                       *
 *    FreeRTOS tutorial books are available in pdf and paperback.        *
 *    Complete, revised, and edited pdf reference manuals are also       *
 *    available.                                                         *
 *                                                                       *
 *    Purchasing FreeRTOS documentation will not only help you, by       *
 *    ensuring you get running as quickly as possible and with an        *
 *    in-depth knowledge of how to use FreeRTOS, it will also help       *
 *    the FreeRTOS project to continue with its mission of providing     *
 *    professional grade, cross platform, de facto standard solutions    *
 *    for microcontrollers - completely free of charge!                  *
 *                                                                       *
 *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
 *                                                                       *
 *    Thank you for using FreeRTOS, and thank you for your support!      *
 *                                                                       *
 ***************************************************************************
 *
 *
 *  This file is part of the FreeRTOS distribution.
 *
 *  FreeRTOS is free software; you can redistribute it and/or modify it under
 *  the terms of the GNU General Public License (version 2) as published by the
 *  Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
 *
 *  >>>>>>NOTE<<<<<< The modification to the GPL is included to allow you to
 *  distribute a combined work that includes FreeRTOS without being obliged to
 *  provide the source code for proprietary components outside of the FreeRTOS
 *  kernel.
 *
 *  FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
 *  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 *  FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 *  details. You should have received a copy of the GNU General Public License
 *  and the FreeRTOS license exception along with FreeRTOS; if not it can be
 *  viewed here: http://www.freertos.org/a00114.html and also obtained by
 *  writing to Real Time Engineers Ltd., contact details for whom are available
 *  on the FreeRTOS WEB site.
 *
 *  1 tab == 4 spaces!
 *
 ***************************************************************************
 *                                                                       *
 *    Having a problem?  Start by reading the FAQ "My application does   *
 *    not run, what could be wrong?"                                     *
 *                                                                       *
 *    http://www.FreeRTOS.org/FAQHelp.html                               *
 *                                                                       *
 ***************************************************************************
 *
 *
 *  http://www.FreeRTOS.org - Documentation, books, training, latest versions,
 *  license and Real Time Engineers Ltd. contact details.
 *
 *  http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
 *  including FreeRTOS+Trace - an indispensable productivity tool, and our new
 *  fully thread aware and reentrant UDP/IP stack.
 *
 *  http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
 *  Integrity Systems, who sell the code with commercial support,
 *  indemnification and middleware, under the OpenRTOS brand.
 *
 *  http://www.SafeRTOS.com - High Integrity Systems also provide a safety
 *  engineered and independently SIL3 certified version for use in safety and
 *  mission critical applications that require provable dependability.
 */

#ifndef _FREERTOS_DEFAULT_CONFIG_H_
#define _FREERTOS_DEFAULT_CONFIG_H_
#include "em_assert.h"

#ifdef __cplusplus
extern "C" {
#endif

/********************** Configuration of FreeRTOS ****************************/
  
/* Implement FreeRTOS configASSERT as emlib assert */
#ifndef configASSERT
#define configASSERT( x )       EFM_ASSERT( x )
#endif

/* Modes of operations of operation system*/
#ifndef configUSE_PREEMPTION
#define configUSE_PREEMPTION       ( 1 )
#endif

/* Energy saving modes */
#ifndef configUSE_TICKLESS_IDLE
#define configUSE_TICKLESS_IDLE    ( 0 )
#endif
/* Available options when configUSE_TICKLESS_IDLE set to 1 
 * or configUSE_SLEEP_MODE_IN_IDLE set to 1 :
 * 1 - EM1, 2 - EM2, 3 - EM3 is not available on this CPU, because
 * timer doesn't work in EM3 mode */
#ifndef configSLEEP_MODE
#define configSLEEP_MODE           ( 1 )
#endif

/* Definition used only if configUSE_TICKLESS_IDLE == 0 */
#ifndef configUSE_SLEEP_MODE_IN_IDLE
#define configUSE_SLEEP_MODE_IN_IDLE       ( 1 )
#endif
 
/* EM1 use systick as system clock*/
/* EM2 use crystal 32768Hz and RTC Component as system clock
 * We use 2 times divider of this clock to reduce energy consumtion
 * You can also in this mode choose crystal oscillator to get more preccision in
 * time measurement or RC oscillator for more energy reduction.*/
/* EM3 use 2kHz RC and BURTC Component as system clock*/ 
#if ( ( configSLEEP_MODE == 2 ) && ( configUSE_TICKLESS_IDLE == 1 || configUSE_SLEEP_MODE_IN_IDLE == 1 ) )
/* Choose source of clock for RTC (system tick) 
 * if configCRYSTAL_IN_EM2 set to 1 then Crystal oscillator is used,
 * when 0 RC oscillator */
#define configCRYSTAL_IN_EM2    ( 1 )
#endif

#if (  (configSLEEP_MODE == 2 ) && ( configUSE_TICKLESS_IDLE == 1 || configUSE_SLEEP_MODE_IN_IDLE == 1 ) )
/* When we use EM2 or EM3 System clock has got low frequency,
 * so we reduce Tick rate to 128 Hz and 40 Hz, which give more clock cycles between ticks*/
#define configTICK_RATE_HZ    ( 128 )
#elif (  ( configSLEEP_MODE == 3 ) && ( configUSE_TICKLESS_IDLE == 1 || configUSE_SLEEP_MODE_IN_IDLE == 1 ) )
#define configTICK_RATE_HZ    ( 40 )
#else
#define configTICK_RATE_HZ    ( 1000 )
#endif

/* Definition used by Keil to replace default system clock source when we use EM2 or EM3 mode. */
#if ( ( configSLEEP_MODE == 2 || configSLEEP_MODE == 3 ) && ( configUSE_TICKLESS_IDLE == 1 || configUSE_SLEEP_MODE_IN_IDLE == 1 ) )
#define configOVERRIDE_DEFAULT_TICK_CONFIGURATION ( 1 )
#endif

/* Main functions*/
#ifndef configMAX_PRIORITIES
#define configMAX_PRIORITIES                      ( 3 )
#endif

#ifndef configMINIMAL_STACK_SIZE
#define configMINIMAL_STACK_SIZE                  (( unsigned short ) 140)
#endif


#ifndef portUSE_HEAP
#define portUSE_HEAP 0  /* use heap_useNewlib */
#endif

//#define configTOTAL_HEAP_SIZE                   (( size_t )((unsigned)SRAM_SIZE)/4)
#ifndef configTOTAL_HEAP_SIZE
#define configTOTAL_HEAP_SIZE                     (SRAM_SIZE/4)
#endif

#ifndef configMAX_TASK_NAME_LEN
#define configMAX_TASK_NAME_LEN                   ( 10 )
#endif

#ifndef configUSE_TRACE_FACILITY
#define configUSE_TRACE_FACILITY                  ( 0 )
#endif

#ifndef configUSE_16_BIT_TICKS
#define configUSE_16_BIT_TICKS                    ( 0 )
#endif

#ifndef configIDLE_SHOULD_YIELD
#define configIDLE_SHOULD_YIELD                   ( 0 )
#endif

#ifndef configUSE_MUTEXES
#define configUSE_MUTEXES                         ( 1 )
#endif

#ifndef configUSE_RECURSIVE_MUTEXES
#define configUSE_RECURSIVE_MUTEXES               ( 0 )
#endif
#ifndef configUSE_COUNTING_SEMAPHORES
#define configUSE_COUNTING_SEMAPHORES             ( 0 )
#endif
#ifndef configUSE_ALTERNATIVE_API
#define configUSE_ALTERNATIVE_API                 ( 0 )/* Deprecated! */
#endif
#ifndef configQUEUE_REGISTRY_SIZE
#define configQUEUE_REGISTRY_SIZE                 ( 10 )
#endif
#ifndef configUSE_QUEUE_SETS
#define configUSE_QUEUE_SETS                      ( 0 )
#endif

/* Hook function related definitions. */
#ifndef configUSE_TICK_HOOK
 #define configUSE_TICK_HOOK                       ( 0 )
#endif
#ifndef configCHECK_FOR_STACK_OVERFLOW
 #if defined(DEBUG_EFM)
  #define configCHECK_FOR_STACK_OVERFLOW            ( 1 )
 #else
  #define configCHECK_FOR_STACK_OVERFLOW            ( 0 )
 #endif 
#endif
#ifndef configUSE_MALLOC_FAILED_HOOK
 #if defined(DEBUG_EFM)
   #define configUSE_MALLOC_FAILED_HOOK              ( 1 )
 #else
   #define configUSE_MALLOC_FAILED_HOOK              ( 0 )
 #endif 
#endif

/* Run time stats gathering related definitions. */
#ifndef configGENERATE_RUN_TIME_STATS
 #define configGENERATE_RUN_TIME_STATS             ( 0 )
#endif

/* Co-routine related definitions. */
#ifndef configUSE_CO_ROUTINES
#define configUSE_CO_ROUTINES                     ( 0 )
#endif
#ifndef configMAX_CO_ROUTINE_PRIORITIES
#define configMAX_CO_ROUTINE_PRIORITIES           ( 1 )
#endif

/* Software timer related definitions. */
#ifndef configUSE_TIMERS
#define configUSE_TIMERS                          ( 1 )
#endif
#ifndef configTIMER_TASK_PRIORITY
#define configTIMER_TASK_PRIORITY                 ( configMAX_PRIORITIES - 1 ) /* Highest priority */
#endif
#ifndef configTIMER_QUEUE_LENGTH
#define configTIMER_QUEUE_LENGTH                  ( 10 )
#endif
#ifndef configTIMER_TASK_STACK_DEPTH
#define configTIMER_TASK_STACK_DEPTH              ( configMINIMAL_STACK_SIZE )
#endif

/* Interrupt nesting behaviour configuration. */
#ifndef configKERNEL_INTERRUPT_PRIORITY
#define configKERNEL_INTERRUPT_PRIORITY           ( 255 ) 
#endif
#ifndef configMAX_SYSCALL_INTERRUPT_PRIORITY
#define configMAX_SYSCALL_INTERRUPT_PRIORITY      ( 191 ) /* equivalent to 0xa0, or priority 5. */
#endif

/* Optional functions - most linkers will remove unused functions anyway. */
#ifndef INCLUDE_vTaskPrioritySet
#define INCLUDE_vTaskPrioritySet                  ( 1 )
#endif
#ifndef INCLUDE_uxTaskPriorityGet
#define INCLUDE_uxTaskPriorityGet                 ( 1 )
#endif
#ifndef INCLUDE_vTaskDelete
#define INCLUDE_vTaskDelete                       ( 1 )
#endif
#ifndef INCLUDE_vTaskSuspend
#define INCLUDE_vTaskSuspend                      ( 1 )
#endif
#ifndef INCLUDE_xResumeFromISR
#define INCLUDE_xResumeFromISR                    ( 1 )
#endif
#ifndef INCLUDE_vTaskDelayUntil
#define INCLUDE_vTaskDelayUntil                   ( 0 )
#endif
#ifndef INCLUDE_vTaskDelay
#define INCLUDE_vTaskDelay                        ( 1 )
#endif
#ifndef INCLUDE_xTaskGetSchedulerState
#define INCLUDE_xTaskGetSchedulerState            ( 1 )
#endif
#ifndef INCLUDE_xTaskGetCurrentTaskHandle
#define INCLUDE_xTaskGetCurrentTaskHandle         ( 1 )
#endif
#ifndef INCLUDE_uxTaskGetStackHighWaterMark
#define INCLUDE_uxTaskGetStackHighWaterMark       ( 0 )
#endif
#ifndef INCLUDE_xTaskGetIdleTaskHandle
#define INCLUDE_xTaskGetIdleTaskHandle            ( 0 )
#endif
#ifndef INCLUDE_xTimerGetTimerDaemonTaskHandle
#define INCLUDE_xTimerGetTimerDaemonTaskHandle    ( 0 )
#endif
#ifndef INCLUDE_pcTaskGetTaskName
#define INCLUDE_pcTaskGetTaskName                 ( 0 )
#endif
#ifndef INCLUDE_eTaskGetState
#define INCLUDE_eTaskGetState                     ( 0 )
#endif

/* Default value of CPU clock (RC)*/
#ifndef configCPU_CLOCK_HZ
#define configCPU_CLOCK_HZ                        (( unsigned long ) 14000000)
#endif

/* Defines used in energy modes */
#if ( ( configSLEEP_MODE == 2 )  && ( ( configUSE_SLEEP_MODE_IN_IDLE == 1 ) || ( configUSE_TICKLESS_IDLE == 1 ) ) )
        #define configSYSTICK_CLOCK_HZ    ( 16384 )
#endif

#if ( ( configSLEEP_MODE == 3 )  && ( ( configUSE_SLEEP_MODE_IN_IDLE == 1 ) || ( configUSE_TICKLESS_IDLE == 1 ) ) )
       #define configSYSTICK_CLOCK_HZ    ( 2000 )
#endif
  
#if ( ( configUSE_TICKLESS_IDLE == 0 ) && ( configUSE_SLEEP_MODE_IN_IDLE == 1 ) )
#define configUSE_IDLE_HOOK  ( 1 ) 
#else
#define configUSE_IDLE_HOOK  ( 0 ) 
#endif

/*-----------------------------------------------------------*/


/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
 * standard names. */
#ifndef vPortSVCHandler
#define vPortSVCHandler        SVC_Handler
#endif
#ifndef xPortPendSVHandler
#define xPortPendSVHandler     PendSV_Handler
#endif
#ifndef xPortSysTickHandler
#define xPortSysTickHandler    SysTick_Handler
#endif

#ifdef __cplusplus
}
#endif
#endif /* _FREERTOS_DEFAULT_CONFIG_H_ */

