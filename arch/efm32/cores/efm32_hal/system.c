#include "efm32_build_defines.h"

#ifdef  CUSTOM_SYSTEM_C
#  include CUSTOM_SYSTEM_C
#  pragma message "used custom CMSIS system file"
#else
#  if defined(UNUSED_CMSIS_SYSTEM)&& (UNUSED_CMSIS_SYSTEM > 0)  /* has .c or .S file auto select in project or variant path */
#     pragma message "unused CMSIS system file"
#  else	
#     include CMSIS_SYSTEM_C
#  endif
#endif
