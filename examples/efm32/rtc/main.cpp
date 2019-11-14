#include "Arduino.h"
#include "rtcdriver.h"
#include "em_cmu.h"
#include "em_rtc.h"
#include "em_emu.h"

RTCDRV_TimerID_t id;

static void disableHFClocks(void)
{
	// Disable High Frequency Peripheral Clocks
	CMU_ClockEnable(cmuClock_HFPER, false);
#if defined( CMU_HFPERCLKEN0_USART0 )
	CMU_ClockEnable(cmuClock_USART0, false);
#endif
	CMU_ClockEnable(cmuClock_USART1, false);
	CMU_ClockEnable(cmuClock_TIMER0, false);
	CMU_ClockEnable(cmuClock_TIMER1, false);
#if defined( CMU_HFPERCLKEN0_TIMER2 )
	CMU_ClockEnable(cmuClock_TIMER2, false);
#endif
	CMU_ClockEnable(cmuClock_ACMP0, false);
	CMU_ClockEnable(cmuClock_PRS, false);
	CMU_ClockEnable(cmuClock_IDAC0, false);
	CMU_ClockEnable(cmuClock_GPIO, false);
	CMU_ClockEnable(cmuClock_VCMP, false);
	CMU_ClockEnable(cmuClock_ADC0, false);
	CMU_ClockEnable(cmuClock_I2C0, false);

	// Disable High Frequency Core/Bus Clocks
	CMU_ClockEnable(cmuClock_AES, false);
	CMU_ClockEnable(cmuClock_DMA, false);
	CMU_ClockEnable(cmuClock_HFLE, false);
#if defined( CMU_HFCORECLKEN0_USB )
	CMU_ClockEnable(cmuClock_USB, false);
#endif
#if defined( CMU_HFCORECLKEN0_USBC )
	// Disable USBC clock by choosing unused oscillator (LFXO)
	CMU_ClockEnable(cmuClock_USBC, true);
	CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);
	CMU_ClockSelectSet(cmuClock_USBC, cmuSelect_LFRCO);
	CMU_ClockEnable(cmuClock_USBC, false);
#endif
}

static void disableLFClocks(void)
{
	// Enable LFXO for Low Frequency Clock Disables
	CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);

	// Disable Low Frequency A Peripheral Clocks
	// Note: LFA clock must be sourced before modifying peripheral clock enables
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
	CMU_ClockEnable(cmuClock_RTC, false);
	CMU_ClockEnable(cmuClock_PCNT0, false);
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_Disabled);

	// Disable Low Frequency B Peripheral Clocks
	// Note: LFB clock must be sourced before modifying peripheral clock enables
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFRCO);
	CMU_ClockEnable(cmuClock_LEUART0, false);
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_Disabled);

#if defined( _CMU_LFCCLKEN0_MASK )
	// Disable Low Frequency C Peripheral Clocks
	// Note: LFC clock must be sourced before modifying peripheral clock enables
	CMU_ClockSelectSet(cmuClock_LFC, cmuSelect_LFRCO);
	CMU_ClockEnable(cmuClock_USBLE, false);
	CMU_ClockSelectSet(cmuClock_LFC, cmuSelect_Disabled);
#endif

	// Disable Low Frequency Oscillator
	CMU_OscillatorEnable(cmuOsc_LFXO, false, true);
}

static void disableClocks(void)
{
	// Disable High Frequency Clocks
	disableHFClocks();

	// Disable Low Frequency Clocks
	disableLFClocks();
}

void em_EM2_LfrcoRTC(void)
{
	// Make sure clocks are disabled.
	disableClocks();

#if 0
	// Route the LFRCO clock to RTC.
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
	CMU_ClockEnable(cmuClock_RTC, true);

	// Configure RTC to 1Hz.
	CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_32768);

	// Enable clock to the interface with low energy modules.
	CMU_ClockEnable(cmuClock_CORELE, true);

	RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;

	// Initialize RTC.
	RTC_Init(&rtcInit);
#else
	RTCDRV_Init();
#endif

	// Make sure unwanted oscillators are disabled specifically for EM2 and LFRCO.
	CMU_OscillatorEnable(cmuOsc_LFXO, false, true);

	// Enter EM2.
	//EMU_EnterEM2(false);
}

void myCallback(RTCDRV_TimerID_t id, void *user)
{
	(void)id;
	(void)user;

	Serial.println("Check sensor data....");

	// Restart timer
	RTCDRV_StartTimer(id, rtcdrvTimerTypeOneshot, 6000, myCallback, NULL);

	//EMU_EnterEM2(true);
}

void setup()
{
	/* Use default settings for EM23 */
	EMU_EM23Init_TypeDef em23Init = EMU_EM23INIT_DEFAULT;

	/* Initialize EM23 with default parameters */
	EMU_EM23Init(&em23Init);

	// Initialization of RTCDRV driver
	//RTCDRV_Init();
	em_EM2_LfrcoRTC();

	// Reserve a timer
	RTCDRV_AllocateTimer(&id);

	Serial.setRouteLoc(1);
	Serial.begin(115200);

	// Start a oneshot timer with 5000 ms timeout
	RTCDRV_StartTimer(id, rtcdrvTimerTypeOneshot, 6100, myCallback, NULL);
}

void loop()
{
	Serial.println("Check sensor data .... @loop()");

	delay(5);

	EMU_EnterEM2(true);
}
