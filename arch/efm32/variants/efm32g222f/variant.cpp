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
  
  2016.9.18 for efm32g222f128/64/32 board, Modified by huawei<huaweiwx@sina.com>
*/

#include "Arduino.h"


#ifdef __cplusplus
extern "C" {
#endif

/*
typedef struct _Pin2PortMapArray
{
  	GPIO_TypeDef GPIOx_Port; 
  	uint8_t      pin;   
    uint32_t  	 adc_channel;
    uint32_t  	 timerNumber;    //Timer1 to Timer4.
    uint32_t     timerChannelLoc;   //Timer channel (1-4).  
} Pin2PortMapArray ;
*/

extern const Pin2PortMapArray g_Pin2PortMapArray[]=
{
#if USE_TIMER0_PWM>0    
    {gpioPortA, GPIO_PIN_0,  NO_ADC,                     TIMER0,0x00  },  /* D00/PA0, TIM0_CC0 #0/1, I2C0_SDA #0*/
    {gpioPortA, GPIO_PIN_1,  NO_ADC,                     TIMER0,0x01  },  /* D01/PA1, TIM0_CC1 #0/1, I2C0_SCL #0,CMU_CLK1 #0*/
    {gpioPortA, GPIO_PIN_2,  NO_ADC,                     TIMER0,0x02  },  /* D02/PA2, TIM0_CC2 #0/1,             CMU_CLK0 #0*/
#else
    {gpioPortA, GPIO_PIN_0,  NO_ADC,                     NO_PWM,NO_PWM},  /* D00/PA0, TIM0_CC0 #0/1, I2C0_SDA #0*/
    {gpioPortA, GPIO_PIN_1,  NO_ADC,                     NO_PWM,NO_PWM},  /* D01/PA1, TIM0_CC1 #0/1, I2C0_SCL #0,CMU_CLK1 #0*/
    {gpioPortA, GPIO_PIN_2,  NO_ADC,                     NO_PWM,NO_PWM},  /* D02/PA2, TIM0_CC2 #0/1,             CMU_CLK0 #0*/	
#endif
#if USE_TIMER2_PWM>0    
    {gpioPortA, GPIO_PIN_8,  NO_ADC,                     TIMER2,0x00  },  /* D03/PA8, TIM2_CC0 #0*/
    {gpioPortA, GPIO_PIN_9,  NO_ADC,                     TIMER2,0x01  },  /* D04/PA9, TIM2_CC1 #0*/
    {gpioPortA, GPIO_PIN_10, NO_ADC,                     TIMER2,0x02  },  /* D05/PA10,TIM2_CC2 #0*/ 
#else
    {gpioPortA, GPIO_PIN_8,  NO_ADC,                     NO_PWM,NO_PWM},  /* D03/PA8, TIM2_CC0 #0*/
    {gpioPortA, GPIO_PIN_9,  NO_ADC,                     NO_PWM,NO_PWM},  /* D04/PA9, TIM2_CC1 #0*/
    {gpioPortA, GPIO_PIN_10, NO_ADC,                     NO_PWM,NO_PWM},  /* D05/PA10,TIM2_CC2 #0*/ 
#endif	
    {gpioPortB, GPIO_PIN_7,  NO_ADC,                     NO_PWM,NO_PWM},  /* D06/PB07,LFXTAL_P,US1_CLK #0*/ 
    {gpioPortB, GPIO_PIN_8,  NO_ADC,                     NO_PWM,NO_PWM},  /* D07/PB08,LFXTAL_N,US1_CS #0*/ 

    {gpioPortB, GPIO_PIN_11, NO_ADC,                     NO_PWM,NO_PWM},  /* D08/PB11,DAC0_OUT0,LETIM0_OUT0 #1*/

    {gpioPortB, GPIO_PIN_13, NO_ADC,                     NO_PWM,NO_PWM},  /* D09/PB13,HFXTAL_P LEU0_TX #1*/
    {gpioPortB, GPIO_PIN_14, NO_ADC,                     NO_PWM,NO_PWM},  /* D10/PB14,HFXTAL_N LEU0_RX #1*/

    {gpioPortC, GPIO_PIN_0,  NO_ADC,                     NO_PWM,NO_PWM},  /* D11/PC00,ACMP0_CH0 PCNT0_S0IN #2 US1_TX #0*/
    {gpioPortC, GPIO_PIN_1,  NO_ADC,                     NO_PWM,NO_PWM},  /* D12/PC01,ACMP0_CH1 PCNT0_S1IN #2 US1_RX #0*/
    {gpioPortC, GPIO_PIN_2,  NO_ADC,                     NO_PWM,NO_PWM},  /* D13/PC02,ACMP0_CH2*/
    {gpioPortC, GPIO_PIN_3,  NO_ADC,                     NO_PWM,NO_PWM},  /* D14/PC03,ACMP0_CH3*/
    {gpioPortC, GPIO_PIN_4,  NO_ADC,                     NO_PWM,NO_PWM},  /* D15/PC04,ACMP0_CH4,LETIM0_OUT0 #3/PCNT1_S0IN #0*/
                                                                         
#if USE_TIMER2_PWM>0    
    {gpioPortC, GPIO_PIN_8,  NO_ADC,                     TIMER2,0x20},  /* D16/PC08,ACMP1_CH0,TIM2_CC0 #2,US0_CS  #2 */
    {gpioPortC, GPIO_PIN_9,  NO_ADC,                     TIMER2,0x21},  /* D17/PC09,ACMP1_CH1,TIM2_CC1 #2,US0_CLK #2*/
    {gpioPortC, GPIO_PIN_10, NO_ADC,                     TIMER2,0x22},  /* D18/PC10,ACMP1_CH2,TIM2_CC2 #2,US0_RX  #2*/
#else
    {gpioPortC, GPIO_PIN_8,  NO_ADC,                     NO_PWM,NO_PWM},  /* D16/PC08,ACMP1_CH0,TIM2_CC0 #2,US0_CS  #2 */
    {gpioPortC, GPIO_PIN_9,  NO_ADC,                     NO_PWM,NO_PWM},  /* D17/PC09,ACMP1_CH1,TIM2_CC1 #2,US0_CLK #2*/
    {gpioPortC, GPIO_PIN_10, NO_ADC,                     NO_PWM,NO_PWM},  /* D18/PC10,ACMP1_CH2,TIM2_CC2 #2,US0_RX  #2*/
#endif	
    {gpioPortC, GPIO_PIN_11, NO_ADC,                     NO_PWM,NO_PWM},  /* D19/PC11,ACMP1_CH3,            US0_TX  #2*/
                                                                         
#if USE_TIMER1_PWM>0    
    {gpioPortC, GPIO_PIN_13, NO_ADC,                     TIMER1,0x00     },  /* D20/PC13,ACMP1_CH5,TIM0_CDTI0 #1/3/TIM1_CC0 #0/PCNT0_S0IN #0*/
    {gpioPortC, GPIO_PIN_14, NO_ADC,                     TIMER1,0x01     },  /* D21/PC14,ACMP1_CH6,TIM0_CDTI1 #1/3/TIM1_CC1 #0/PCNT0_S1IN #0*/
    {gpioPortC, GPIO_PIN_15, NO_ADC,                     TIMER1,0x02     },  /* D22/PC15,ACMP1_CH7,TIM0_CDTI2 #1/3/TIM1_CC2 #0/DBG_SWO #1*/
#else
    {gpioPortC, GPIO_PIN_13, NO_ADC,                     NO_PWM,NO_PWM},  /* D20/PC13,ACMP1_CH5,TIM0_CDTI0 #1/3/TIM1_CC0 #0/PCNT0_S0IN #0*/
    {gpioPortC, GPIO_PIN_14, NO_ADC,                     NO_PWM,NO_PWM},  /* D21/PC14,ACMP1_CH6,TIM0_CDTI1 #1/3/TIM1_CC1 #0/PCNT0_S1IN #0*/
    {gpioPortC, GPIO_PIN_15, NO_ADC,                     NO_PWM,NO_PWM},  /* D22/PC15,ACMP1_CH7,TIM0_CDTI2 #1/3/TIM1_CC2 #0/DBG_SWO #1*/
#endif
    {gpioPortD, GPIO_PIN_4, _ADC_SINGLECTRL_INPUTSEL_CH4,NO_PWM,NO_PWM},  /* D23/PD4,ADC0_CH4,LEU0_TX #0*/ /**/
    {gpioPortD, GPIO_PIN_5, _ADC_SINGLECTRL_INPUTSEL_CH5,NO_PWM,NO_PWM},  /* D24/PD5,ADC0_CH5 LEU0_RX #0*/
    {gpioPortD, GPIO_PIN_6, _ADC_SINGLECTRL_INPUTSEL_CH6,NO_PWM,NO_PWM},  /* D25/PD6,ADC0_CH6 LETIM0_OUT0 #0 I2C0_SDA #1,*/
    {gpioPortD, GPIO_PIN_7, _ADC_SINGLECTRL_INPUTSEL_CH7,NO_PWM,NO_PWM},  /* D26/PD7,ADC0_CH7 LETIM0_OUT1 #0 I2C0_SCL #1*/

#if USE_TIMER1_PWM>0    
    {gpioPortE, GPIO_PIN_10, NO_ADC,                     TIMER1,0x10},  /* D27/PE10,TIM1_CC0 #1 US0_TX  #0 BOOT_TX*/
    {gpioPortE, GPIO_PIN_11, NO_ADC,                     TIMER1,0x11},  /* D28/PE11,TIM1_CC1 #1 US0_RX  #0 BOOT_RX*/
    {gpioPortE, GPIO_PIN_12, NO_ADC,                     TIMER1,0x12},  /* D29/PE12,TIM1_CC2 #1 US0_CLK #0 BO*/
#else
    {gpioPortE, GPIO_PIN_10, NO_ADC,                     NO_PWM,NO_PWM},  /* D27/PE10,TIM1_CC0 #1 US0_TX  #0 BOOT_TX*/
    {gpioPortE, GPIO_PIN_11, NO_ADC,                     NO_PWM,NO_PWM},  /* D28/PE11,TIM1_CC1 #1 US0_RX  #0 BOOT_RX*/
    {gpioPortE, GPIO_PIN_12, NO_ADC,                     NO_PWM,NO_PWM},  /* D29/PE12,TIM1_CC2 #1 US0_CLK #0 BO*/
#endif
    {gpioPortE, GPIO_PIN_13, NO_ADC,                     NO_PWM,NO_PWM},  /* D30/PE13,ACMP0_O  #0  US0_CS #0 */

    {gpioPortF, GPIO_PIN_0,  NO_ADC,                     NO_PWM,NO_PWM},  /* D31/PF0,LETIM0_OUT0 #2 DBG_SWCLK #0/1*/
    {gpioPortF, GPIO_PIN_1,  NO_ADC,                     NO_PWM,NO_PWM},  /* D32/PF1,LETIM0_OUT1 #2 DBG_SWDIO #0/1*/
    {gpioPortF, GPIO_PIN_2,  NO_ADC,                     NO_PWM,NO_PWM},  /* D33/PF2,ACMP1_O #0     DBG_SWO #0*/
    {gpioPortF, GPIO_PIN_3,  NO_ADC,                     NO_PWM,NO_PWM},  /* D34/PF3,TIM0_CDTI0  #2*/
    {gpioPortF, GPIO_PIN_4,  NO_ADC,                     NO_PWM,NO_PWM},  /* D35/PF4,TIM0_CDTI1  #2*/
    {gpioPortF, GPIO_PIN_5,  NO_ADC,                     NO_PWM,NO_PWM},  /* D36/PF5,TIM0_CDTI2  #2*/
};

#ifdef __cplusplus
}
#endif



/* Private variables ---------------------------------------------------------*/

