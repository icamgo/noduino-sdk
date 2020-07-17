#ifndef  _FREERTOSARM_H_
#define  _FREERTOSARM_H_
#include "Source/FreeRTOS.h"
#include "Source/task.h"
#include "Source/queue.h"
#include "Source/semphr.h"
#include "basic_io_arm.h"

#ifdef __cplusplus
extern "C" {
#endif

 void vApplicationIdleHook( void );
 void vApplicationTickHook( void );
#ifdef __cplusplus
} //extern "C" {
#endif

#endif //_FREERTOSSTM_H_