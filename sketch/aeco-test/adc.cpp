#include "vbat.h"

#if 0
static void adcInit( void )
{
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

	CMU_ClockEnable( cmuClock_ADC0, true );
	CMU_ClockEnable( cmuClock_PRS, true );

	/* Only configure the ADC if it is not already running */
	if( ADC0 ->CTRL == _ADC_CTRL_RESETVALUE ) {

	   init.timebase = ADC_TimebaseCalc( 0 );
	   init.prescale = ADC_PrescaleCalc( 1000000, 0 );
	   ADC_Init( ADC0, &init );
	}

	initSingle.acqTime = adcAcqTime16;
	initSingle.reference = adcRef5VDIFF;
	initSingle.posSel = adcPosSelAVDD;
	initSingle.negSel = adcNegSelVSS;
	initSingle.prsEnable = true;
	initSingle.prsSel = adcPRSSELCh4;

	ADC_InitSingle( ADC0, &initSingle );

	return;
}

static void adcDeInit( void )
{

PRS_SourceAsyncSignalSet( 4, 0, 0 );
CMU->CTRL &= ~_CMU_CTRL_CLKOUTSEL1_MASK;

return;

}

static uint16_t getAdcSample( void )
{
ADC_Start( ADC0, adcStartSingle );
while( !( ADC_IntGet( ADC0 ) & ADC_IF_SINGLE ) )
   ;

return ADC_DataSingleGet( ADC0 );
}
 
int get_vbat(int avg)
{
	uint16_t adcData;
	float supplyVoltage;
	int i;

	adcInit();

	supplyVoltage = 0;

	for( i = 0; i < avg; i++ ) {
	   adcData = getAdcSample();
	   supplyVoltage += (float) adcData * 5.0 / 4095.0;
	}

	adcDeInit();

	supplyVoltage = supplyVoltage / (float) avg;

	return supplyVoltage * 100;
}
#endif

char *ftoa(char *a, double f, int precision)
{
	long p[] =
	    { 0, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000 };

	char *ret = a;
	long heiltal = (long)f;
	itoa(heiltal, a, 10);
	while (*a != '\0')
		a++;
	*a++ = '.';
	long desimal = abs((long)((f - heiltal) * p[precision]));
	if (desimal < p[precision - 1]) {
		*a++ = '0';
	}
	itoa(desimal, a, 10);
	return ret;
}

void setup() {

	/* setup alternate location default is 0# */  
	/* set to 1#. the serial port's Alternate LOCATION(see datasheet)*/
	Serial.setRouteLoc(1);
	Serial.begin(115200);

	vbat_adc_init();
  	Serial.print("EFM32 Tesing...");
}

void loop() {

	float vbat = 0.0;
	char vbat_s[10];

	vbat = get_vbat();

	ftoa(vbat_s, vbat, 2);

  	Serial.print("Vbat = ");
	Serial.println(vbat_s);

	delay(3000);

	/* On every wakeup enter EM2 again */
	//EMU_EnterEM2(true);
}
