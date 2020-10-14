#include "efm32.h"

void SystemClock_Config(void) __attribute__ ((weak));
void SystemClock_Config(void) {
#ifdef USE_HFXO
    HFXO_enter_DefaultMode_from_RESET();   //2    select  extern high Frequency osc Clock
#elif defined(USE_LFXO)
    LFXO_enter_DefaultMode_from_RESET();   //2 or select internal LOW Frequency RC Clock
#elif defined(USE_HFRCO) /*as default*/
	/* Using HFRCO at 14MHz as high frequency clock, HFCLK */
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
# if    F_CPU > 21000000L	
	CMU_HFRCOBandSet(cmuHFRCOBand_28MHz);
# elif  F_CPU > 14000000L
	CMU_HFRCOBandSet(cmuHFRCOBand_21MHz);
# elif  F_CPU < 14000000L
	CMU_HFRCOBandSet(cmuHFRCOBand_11MHz);
# else	
//	CMU_HFRCOBandSet(cmuHFRCOBand_14MHz); default 14M
# endif
	/* Enable peripheral clock */
	CMU_ClockEnable(cmuClock_HFPER, true);  
#else/*default mode*/
    enter_DefaultMode_from_RESET();        //2 or select internal High Frequency RC Clock
#endif
}
