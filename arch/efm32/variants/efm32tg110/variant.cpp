/*
  Copyright (c) 2011 Arduino.  All right reserved.

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
  
  2016.9.18 for efm32tg110f32/16 board, Modified by huawei<huaweiwx@sina.com>
*/

#include "Arduino.h"


#ifdef __cplusplus
extern "C" {
#endif

/*
typedef struct _Pin2PortMapArray
{
  	GPIO_TypeDef GPIOx_Port; 
  	uint8_t  pin;   
    uint32_t  adc_channel;
    uint32_t  timerNumber;    //Timer1 to Timer4.
    uint32_t  timerChannelLoc;   //Timer channel (1-4).  
} Pin2PortMapArray ;
*/

extern const Pin2PortMapArray g_Pin2PortMapArray[]=
{
#if USE_TIMER0_PWM>0    
    {gpioPortA, GPIO_PIN_0,  NO_ADC,                     TIMER0,0x00  },  /* D00/PA0, TIM0_CC0 #0/1, I2C0_SDA #0*/
#else
    {gpioPortA, GPIO_PIN_0,  NO_ADC,                     NO_PWM,NO_PWM},  /* D00/PA0, TIM0_CC0 #0/1, I2C0_SDA #0*/
#endif

#if USE_TIMER1_PWM>0    
    {gpioPortB, GPIO_PIN_7,  NO_ADC,                     TIMER1,0x30},  /* D01/PB07,LFXTAL_P,US1_CLK #0*/ 
    {gpioPortB, GPIO_PIN_8,  NO_ADC,                     TIMER1,0x31},  /* D02/PB08,LFXTAL_N,US1_CS #0*/ 
    {gpioPortB, GPIO_PIN_11, NO_ADC,                     TIMER0,0x10},  /* D03/PB11,DAC0_OUT0,LETIM0_OUT0 #1*/
#else
    {gpioPortB, GPIO_PIN_7,  NO_ADC,                     NO_PWM,NO_PWM},  /* D01/PB07,LFXTAL_P,US1_CLK #0*/ 
    {gpioPortB, GPIO_PIN_8,  NO_ADC,                     NO_PWM,NO_PWM},  /* D02/PB08,LFXTAL_N,US1_CS #0*/ 
    {gpioPortB, GPIO_PIN_11, NO_ADC,                     NO_PWM,NO_PWM},  /* D03/PB11,DAC0_OUT0,LETIM0_OUT0 #1*/
#endif

    {gpioPortB, GPIO_PIN_13, NO_ADC,                     NO_PWM,NO_PWM},  /* D04/PB13,HFXTAL_P LEU0_TX #1*/
    {gpioPortB, GPIO_PIN_14, NO_ADC,                     NO_PWM,NO_PWM},  /* D05/PB14,HFXTAL_N LEU0_RX #1*/

#if USE_TIMER0_PWM>0    
    {gpioPortC, GPIO_PIN_0,  NO_ADC,                     TIMER0,0x41  },  /* D06/PC00,ACMP0_CH0 PCNT0_S0IN #2 US1_TX #0*/
    {gpioPortC, GPIO_PIN_1,  NO_ADC,                     TIMER0,0x42  },  /* D07/PC01,ACMP0_CH1 PCNT0_S1IN #2 US1_RX #0*/
#else
    {gpioPortC, GPIO_PIN_0,  NO_ADC,                     NO_PWM,NO_PWM},  /* D06/PC00,ACMP0_CH0 PCNT0_S0IN #2 US1_TX #0*/
    {gpioPortC, GPIO_PIN_1,  NO_ADC,                     NO_PWM,NO_PWM},  /* D07/PC01,ACMP0_CH1 PCNT0_S1IN #2 US1_RX #0*/
#endif
                                                                         
                                                                         
#if USE_TIMER1_PWM>0    
    {gpioPortC, GPIO_PIN_14, NO_ADC,                     TIMER1,0x01     },/*D08/PC14,ACMP1_CH6,TIM0_CDTI1 #1/3/TIM1_CC1 #0/PCNT0_S1IN #0*/
    {gpioPortC, GPIO_PIN_15, NO_ADC,                     TIMER1,0x02     },/*D08/PC15,ACMP1_CH7,TIM0_CDTI2 #1/3/TIM1_CC2 #0/DBG_SWO #1*/
    {gpioPortD, GPIO_PIN_6, _ADC_SINGLECTRL_INPUTSEL_CH6,TIMER1,0x40},  /* D09/PD6,ADC0_CH6 LETIM0_OUT0 #0 I2C0_SDA #1,*/
    {gpioPortD, GPIO_PIN_7, _ADC_SINGLECTRL_INPUTSEL_CH7,TIMER1,0x41},  /* D09/PD7,ADC0_CH7 LETIM0_OUT1 #0 I2C0_SCL #1*/
#else
    {gpioPortC, GPIO_PIN_14, NO_ADC,                     NO_PWM,NO_PWM},  /* D08/PC14,ACMP1_CH6,TIM0_CDTI1 #1/3/TIM1_CC1 #0/PCNT0_S1IN #0*/
    {gpioPortC, GPIO_PIN_15, NO_ADC,                     NO_PWM,NO_PWM},  /* D08/PC15,ACMP1_CH7,TIM0_CDTI2 #1/3/TIM1_CC2 #0/DBG_SWO #1*/
    {gpioPortD, GPIO_PIN_6, _ADC_SINGLECTRL_INPUTSEL_CH6,NO_PWM,NO_PWM},  /* D09/PD6,ADC0_CH6 LETIM0_OUT0 #0 I2C0_SDA #1,*/
    {gpioPortD, GPIO_PIN_7, _ADC_SINGLECTRL_INPUTSEL_CH7,NO_PWM,NO_PWM},  /* D09/PD7,ADC0_CH7 LETIM0_OUT1 #0 I2C0_SCL #1*/
#endif

#if USE_TIMER1_PWM>0    
    {gpioPortE, GPIO_PIN_12, NO_ADC,                     TIMER1,0x12},    /* D10/PE12,TIM1_CC2 #1 US0_CLK #0 BO*/
#else
    {gpioPortE, GPIO_PIN_12, NO_ADC,                     NO_PWM,NO_PWM},  /* D10/PE12,TIM1_CC2 #1 US0_CLK #0 BO*/
#endif
    {gpioPortE, GPIO_PIN_13, NO_ADC,                     NO_PWM,NO_PWM},  /* D11/PE13,ACMP0_O  #0  US0_CS #0 */

#if USE_TIMER1_PWM>0    
    {gpioPortF, GPIO_PIN_0,  NO_ADC,                     TIMER0,0x50  },  /* D12/PF0,LETIM0_OUT0 #2 DBG_SWCLK #0/1*/
    {gpioPortF, GPIO_PIN_1,  NO_ADC,                     TIMER0,0x51  },  /* D13/PF1,LETIM0_OUT1 #2 DBG_SWDIO #0/1*/
    {gpioPortF, GPIO_PIN_2,  NO_ADC,                     TIMER0,0x52  },  /* D14/PF2,ACMP1_O #0     DBG_SWO #0*/
#else
    {gpioPortF, GPIO_PIN_0,  NO_ADC,                     NO_PWM,NO_PWM},  /* D12/PF0,LETIM0_OUT0 #2 DBG_SWCLK #0/1*/
    {gpioPortF, GPIO_PIN_1,  NO_ADC,                     NO_PWM,NO_PWM},  /* D13/PF1,LETIM0_OUT1 #2 DBG_SWDIO #0/1*/
    {gpioPortF, GPIO_PIN_2,  NO_ADC,                     NO_PWM,NO_PWM},  /* D14/PF2,ACMP1_O #0     DBG_SWO #0*/
#endif
};

#ifdef __cplusplus
}
#endif



/* Private variables ---------------------------------------------------------*/

