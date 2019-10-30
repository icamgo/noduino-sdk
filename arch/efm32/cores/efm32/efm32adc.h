/*
  efm32 arduino efm32adc.h
  Copyright (c) 2018 huaweiwx@sina.com 2018.7.1

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#ifndef _EFMM32_ADC_H_
#define _EFMM32_ADC_H_

#define INTERNAL1V25 0
#define DEFAULT  0
#define INTERNAL2V5  1
#define INTERNAL     1
#define INTERNAL3V3  2
#define INTERNALVDD  2

#if defined( _ADC_SINGLECTRL_REF_5VDIFF )
/** Internal differential 5V reference. */
#define INTERNAL5VDIFF  adcRef5VDIFF
#endif

#if defined( _ADC_SINGLECTRL_REF_5V )
/** Internal 5V reference. */
#define INTERNAL5V      adcRef5V
#endif

#define EXTERNAL          adcRefExtSingle
#define EXTERNAL2xDIFF    adcRef2xExtDiff
#define INTERNAL2xVDD     adcRef2xVDD

#ifdef __cplusplus
extern "C" {
#endif

void analogReadResolution(ADC_Res_TypeDef resolution);
int analogGetResolution(void);

void analogReference(int ref);
int analogGetReference(void);

int analogReadChannel(ADC_SingleInput_TypeDef adcSingleInputChx, uint8_t diff);
int analogRead(uint8_t pin);
float convertToCelsius(int32_t adcSample);
float convertToFahrenheit(uint32_t adcSample);

#ifdef __cplusplus
}

class ADC {
  public:
    inline void resolution(ADC_Res_TypeDef resolution) {
      analogReadResolution(resolution);
    }

    inline int getResolution(void) {
      return analogGetResolution();
    }

    inline void reference(int ref) {
      analogReference(ref);
    }
    inline int getReference(void) {
      return analogGetReference();
    }

    inline int read(uint8_t pin1, uint8_t pin2 = 0xff) {
      if (pin2 == 0xff)
        return analogRead(pin1);
      uint8_t ch1 = Pin2AdcChannel(pin1);
      uint8_t ch2 = Pin2AdcChannel(pin2);
      uint8_t chs = (ch1 << 4) + ch2;
      ADC_SingleInput_TypeDef dif = adcSingleInputDiff0;
      switch (chs) {
        case 0x01:
          dif = adcSingleInputCh0Ch1;
          break;
        case 0x23:
          dif = adcSingleInputCh2Ch3;
          break;
        case 0x45:
          dif = adcSingleInputCh4Ch5;
          break;
        case 0x67:
          dif = adcSingleInputCh6Ch7;
          break;
        default:
          return 0;
      }
      return analogReadChannel(dif, true);
    }

    int readTemp(void) {
      int ref = getReference();
      int res = getResolution();
      reference(adcRef1V25);   /*must be 1.25V*/
      resolution(adcRes12Bit); /*must be 12bit*/
      int rtn = analogReadChannel(adcSingleInputTemp, false);
      reference(ref);     /*resave*/
      resolution((ADC_Res_TypeDef)res);
      return rtn;
    }

    float temperatureCelsius(void) {
      int temp = readTemp();
      return  convertToCelsius(temp);
    }
    float temperatureFahrenheit(void) {
      int temp = readTemp();
      return  convertToFahrenheit(temp);
    }


};

extern ADC adc;

#endif //__cplusplus

#endif //_EFMM32_PWM_H_
