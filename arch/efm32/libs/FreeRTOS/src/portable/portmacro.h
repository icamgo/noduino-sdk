#include "efm32.h"

#if __CORTEX_M == 0x04U  /*m4*/
  #if (portUSING_MPU_WRAPPERS >0)
	 #include "ARM_CM4_MPU/portmacro.h"
  #else
	 #include "ARM_CM4F/portmacro.h"
  #endif
#elif __CORTEX_M == 0x03U
  #if (portUSING_MPU_WRAPPERS >0)
	#include "ARM_CM3_MPU/portmacro.h"
  #else
	#include "ARM_CM3/portmacro.h"
  #endif
#elif __CORTEX_M == 0x00U
	#include "ARM_CM0/portmacro.h"
#else
	#error "Unknown ARM Cortex version"
#endif
