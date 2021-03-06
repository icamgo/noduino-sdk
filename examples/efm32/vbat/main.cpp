/*
 *  Copyright (c) 2019 - 2029 MaiKe Labs
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
*/

#include "Arduino.h"

#include "em_adc.h"
#include "em_wdog.h"

/* Flag used to indicate ADC is finished */
static volatile bool adc_conv_ok = false;

void vbat_adc_init()
{
   ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
   ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

   /* Enable ADC clock */
   CMU_ClockEnable(cmuClock_ADC0, true);

   /* Initiate ADC peripheral */
   ADC_Init(ADC0, &init);

   /* Setup single conversions for internal VDD/3 */
   initSingle.acqTime = adcAcqTime16;
   initSingle.input   = adcSingleInpVDDDiv3;
   ADC_InitSingle(ADC0, &initSingle);

   /* Manually set some calibration values */
   ADC0->CAL = (0x7C << _ADC_CAL_SINGLEOFFSET_SHIFT) | (0x1F << _ADC_CAL_SINGLEGAIN_SHIFT);

   /* Enable interrupt on completed conversion */
   ADC_IntEnable(ADC0, ADC_IEN_SINGLE);
   NVIC_ClearPendingIRQ(ADC0_IRQn);
   NVIC_EnableIRQ(ADC0_IRQn);
}

void ADC0_IRQHandler()
{
	uint32_t flags;

	/* Clear interrupt flags */
	flags = ADC_IntGet(ADC0);
	ADC_IntClear(ADC0, flags);

	adc_conv_ok = true;
}

float get_vbat()
{
	uint32_t vv;

	vbat_adc_init();

	/* Sample ADC */
	adc_conv_ok = false;
	ADC_Start(ADC0, adcStartSingle);

	while (!adc_conv_ok) EMU_EnterEM1();

	vv = ADC_DataSingleGet(ADC0);
	
	return vv / 1000.0 - 0.18;
}

void setup()
{
	WDOG_Init_TypeDef wInit = WDOG_INIT_DEFAULT;

	/* Watchdog setup - Use defaults, excepts for these : */
	wInit.em2Run = true;
	wInit.em3Run = true;
	wInit.perSel = wdogPeriod_4k;	/* 4k 1kHz periods should give ~4 seconds */

	// wdogPeriod_256k ---> 262145 clock periods, 256s

	Serial.setRouteLoc(1);
	Serial.begin(115200);

	/* Start watchdog */
	WDOG_Init(&wInit);
}

void loop()
{
	Serial.print("Read Vbat: ");

	float temp = adc.readVbat();
	Serial.print(temp, 3);
	Serial.println(" V");

	Serial.print("get_vbat(): ");
	Serial.print(get_vbat(), 3);
	Serial.println(" V");

	delay(5000);

	//EMU_EnterEM2(true);
}
