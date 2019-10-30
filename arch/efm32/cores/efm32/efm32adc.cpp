/*
  EMF32ARDUINO efm32adc.c

  Copyright (c) 2018 huaweiwx@sina.com 2018.7.1

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include <Arduino.h>

#ifndef adcFreq
#define adcFreq   400000
#endif

//ADC_Res_TypeDef:
//  adcRes12Bit = _ADC_SINGLECTRL_RES_12BIT, /**< 12 bit sampling. */
//  adcRes8Bit  = _ADC_SINGLECTRL_RES_8BIT,  /**< 8 bit sampling. */
//  adcRes6Bit  = _ADC_SINGLECTRL_RES_6BIT,  /**< 6 bit sampling. */
//  adcResOVS   = _ADC_SINGLECTRL_RES_OVS    /**< Oversampling. */
static ADC_Res_TypeDef readResolution = adcRes12Bit;

//ADC_Ref_TypeDef readReference
//  adcRef1V25      = _ADC_SINGLECTRL_REF_1V25 /** Internal 1.25V reference. */
//  adcRef2V5       = _ADC_SINGLECTRL_REF_2V5, /** Internal 2.5V reference. */
//  adcRefVDD       = _ADC_SINGLECTRL_REF_VDD, /** Buffered VDD. */
static ADC_Ref_TypeDef readReference  = adcRef1V25;

extern "C"
void analogReadResolution(ADC_Res_TypeDef resolution) {
  readResolution = resolution;
}

extern "C"
int analogGetResolution(void) {
  return readResolution;
}


extern "C"
void   analogReference(int ref) {
  readReference = (ADC_Ref_TypeDef)ref;
}

extern "C"
int analogGetReference(void) {
  return readReference;
}


//ADC_SingleInput_TypeDef  adcSingleInputChx:
//  adcSingleInputCh0      = _ADC_SINGLECTRL_INPUTSEL_CH0,      /**< Channel 0. */
//  adcSingleInputCh1      = _ADC_SINGLECTRL_INPUTSEL_CH1,      /**< Channel 1. */
//  adcSingleInputCh2      = _ADC_SINGLECTRL_INPUTSEL_CH2,      /**< Channel 2. */
//  adcSingleInputCh3      = _ADC_SINGLECTRL_INPUTSEL_CH3,      /**< Channel 3. */
//  adcSingleInputCh4      = _ADC_SINGLECTRL_INPUTSEL_CH4,      /**< Channel 4. */
//  adcSingleInputCh5      = _ADC_SINGLECTRL_INPUTSEL_CH5,      /**< Channel 5. */
//  adcSingleInputCh6      = _ADC_SINGLECTRL_INPUTSEL_CH6,      /**< Channel 6. */
//  adcSingleInputCh7      = _ADC_SINGLECTRL_INPUTSEL_CH7,      /**< Channel 7. */
//  adcSingleInputTemp     = _ADC_SINGLECTRL_INPUTSEL_TEMP,     /**< Temperature reference. */
//  adcSingleInputVDDDiv3  = _ADC_SINGLECTRL_INPUTSEL_VDDDIV3,  /**< VDD divided by 3. */
//  adcSingleInputVDD      = _ADC_SINGLECTRL_INPUTSEL_VDD,      /**< VDD. */
//  adcSingleInputVSS      = _ADC_SINGLECTRL_INPUTSEL_VSS,      /**< VSS. */
//  adcSingleInputVrefDiv2 = _ADC_SINGLECTRL_INPUTSEL_VREFDIV2, /**< Vref divided by 2. */
//  adcSingleInputDACOut0  = _ADC_SINGLECTRL_INPUTSEL_DAC0OUT0, /**< DAC output 0. */
//  adcSingleInputDACOut1  = _ADC_SINGLECTRL_INPUTSEL_DAC0OUT1, /**< DAC output 1. */
//  adcSingleInputATEST    = 15,                                /**< ATEST. */
//
//  /* Differential mode enabled */
//  adcSingleInputCh0Ch1   = _ADC_SINGLECTRL_INPUTSEL_CH0CH1,   /**< Positive Ch0, negative Ch1. */
//  adcSingleInputCh2Ch3   = _ADC_SINGLECTRL_INPUTSEL_CH2CH3,   /**< Positive Ch2, negative Ch3. */
//  adcSingleInputCh4Ch5   = _ADC_SINGLECTRL_INPUTSEL_CH4CH5,   /**< Positive Ch4, negative Ch5. */
//  adcSingleInputCh6Ch7   = _ADC_SINGLECTRL_INPUTSEL_CH6CH7,   /**< Positive Ch6, negative Ch7. */
//  adcSingleInputDiff0    = 4                                  /**< Differential 0. */
static uint8_t adcinited = 0;

extern "C"
int analogReadChannel(ADC_SingleInput_TypeDef adcSingleInputChx, uint8_t diff) {

  if (adcinited == 0) {
    adcinited = 1;
    ADC_Init_TypeDef init = ADC_INIT_DEFAULT;  // Declare init structs
    CMU_ClockEnable(cmuClock_ADC0, true);  // Enable ADC0 clock

    // Modify init structs and initialize
    init.timebase = ADC_TimebaseCalc(0);
    init.prescale = ADC_PrescaleCalc(adcFreq, 0); // Init to max ADC clock for Series 0
    ADC_Init(ADC0, &init);
  }

  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;
  initSingle.reference  = readReference;   // internal 2.5V reference
  initSingle.diff       = diff;            // false  single ended /true differential
  initSingle.resolution = readResolution;  // 12-bit resolution

  // Select ADC input. See README for corresponding EXP header pin.
  initSingle.input = adcSingleInputChx;

  ADC_InitSingle(ADC0, &initSingle);
  ADC_Start(ADC0, adcStartSingle); // Start ADC conversion

  while (!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK)); // Wait for conversion to be complete

  return  ADC_DataSingleGet(ADC0);// Get ADC result
}

extern "C"
int analogRead(uint8_t ucPin) {
  uint32_t ch = g_Pin2PortMapArray[ucPin].adc_channel;
  if (ch == NO_ADC) return 0;
  return analogReadChannel((ADC_SingleInput_TypeDef)ch, false);
}

extern "C"
float convertToCelsius(int32_t adcSample) {
  float temp;
  /* Factory calibration temperature from device information page. */
  float cal_temp_0 = (float)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)
                             >> _DEVINFO_CAL_TEMP_SHIFT);

  float cal_value_0 = (float)((DEVINFO->ADC0CAL2
                               & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)
                              >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);

  /* Temperature gradient (from datasheet) */
  float t_grad = -6.27;

  temp = (cal_temp_0 - ((cal_value_0 - adcSample)  / t_grad));

  return temp;
}

/**************************************************************************//**
  @brief Convert ADC sample values to fahrenheit
  @param adcSample Raw value from ADC to be converted to fahrenheit
  @return The temperature in degrees Fahrenheit
*****************************************************************************/
extern "C"
float convertToFahrenheit(uint32_t adcSample)
{
  float celsius;
  float fahrenheit;
  celsius = convertToCelsius(adcSample);

  fahrenheit =  (celsius * (9.0 / 5.0)) + 32.0;

  return fahrenheit;
}

ADC adc;
