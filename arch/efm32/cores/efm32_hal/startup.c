#include "efm32_build_defines.h"

#ifdef  CUSTOM_STARTUP_C
#  pragma message "used custom CMSIS startup file"
#  include CUSTOM_STARTUP_C
#else
#  if  defined(UNUSED_CMSIS_STARTUP) && (UNUSED_CMSIS_STARTUP > 0)  /* has .c or .S file auto select in project or variant path */
#     pragma message "unused CMSIS startup assembly file"
#  else	
#     include CMSIS_STARTUP_C
#  endif
#endif
