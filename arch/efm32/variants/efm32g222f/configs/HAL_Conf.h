/*
   EFM32GENERIC default config file
   2018 huaweiwx@sina.com
*/

#ifndef _HAL_DEFAULT_CONFIG_H_
#define _HAL_DEFAULT_CONFIG_H_

/***************  for freeRtos ****************/
#ifndef portUSE_HEAP
	#define  portUSE_HEAP          3  /*default is 0 use heap_useNewlib.c*/
#endif

#ifndef configTOTAL_HEAP_SIZE
	#define configTOTAL_HEAP_SIZE  (SRAM_SIZE/4)  /*default is SRAM_SIZE/4*/
#endif


#endif /* _HAL_DEFAULT_CONFIG_H_ */
