#include "efm32.h"

void SystemClock_Config(void) __attribute__ ((weak));

void SystemClock_Config(void) {

#ifdef USE_HFXO
    HFXO_enter_DefaultMode_from_RESET();   // select external high Frequency osc Clock
#elif defined(USE_LFXO)
    LFXO_enter_DefaultMode_from_RESET();   // select external LOW Frequency osc Clock
#elif defined(USE_HFRCO) /*as default*/
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);

#if F_CPU == 28000000L	
	CMU_HFRCOBandSet(cmuHFRCOBand_28MHz);
#elif F_CPU == 21000000L
	CMU_HFRCOBandSet(cmuHFRCOBand_21MHz);
#elif F_CPU == 14000000L
	CMU_HFRCOBandSet(cmuHFRCOBand_14MHz);
#elif F_CPU == 11000000L
	CMU_HFRCOBandSet(cmuHFRCOBand_11MHz);
#elif F_CPU == 7000000L
	CMU_HFRCOBandSet(cmuHFRCOBand_7MHz);
#elif F_CPU == 1000000L
	CMU_HFRCOBandSet(cmuHFRCOBand_1MHz);
#else	
	CMU_HFRCOBandSet(cmuHFRCOBand_14MHz);	//default 14M
#endif

	/* Enable peripheral clock */
	CMU_ClockEnable(cmuClock_HFPER, true);
#else

	/*default mode*/
    enter_DefaultMode_from_RESET();        // select internal High Frequency RC Clock

#endif
}
