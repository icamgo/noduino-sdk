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
    {gpioPortA, GPIO_PIN_0,  NO_ADC,                     TIMER0,0x00  },  /* D00/PA0/PIN1, TIM0_CC0 #0/1, I2C0_SDA #0*/
    {gpioPortA, GPIO_PIN_1,  NO_ADC,                     TIMER0,0x01  },  /* D01/PA1/PIN2, TIM0_CC1 #0/1, I2C0_SCL #0,CMU_CLK1 #0*/
    {gpioPortA, GPIO_PIN_2,  NO_ADC,                     TIMER0,0x02  },  /* D02/PA2/PIN3, TIM0_CC2 #0/1,             CMU_CLK0 #0*/
#else
    {gpioPortA, GPIO_PIN_0,  NO_ADC,                     NO_PWM,NO_PWM},  /* D00/PA0/PIN1, TIM0_CC0 #0/1, I2C0_SDA #0*/
    {gpioPortA, GPIO_PIN_1,  NO_ADC,                     NO_PWM,NO_PWM},  /* D01/PA1/PIN2, TIM0_CC1 #0/1, I2C0_SCL #0,CMU_CLK1 #0*/
    {gpioPortA, GPIO_PIN_2,  NO_ADC,                     NO_PWM,NO_PWM},  /* D02/PA2/PIN3, TIM0_CC2 #0/1,             CMU_CLK0 #0*/	
#endif
    {gpioPortA, GPIO_PIN_3,  NO_ADC,                     NO_PWM,NO_PWM},  /* D03/PA3/PIN4 */
    {gpioPortA, GPIO_PIN_4,  NO_ADC,                     NO_PWM,NO_PWM},  /* D04/PA4/PIN5 */
    {gpioPortA, GPIO_PIN_5,  NO_ADC,                     NO_PWM,NO_PWM},  /* D05/PA5/PIN6 */ 
    {gpioPortA, GPIO_PIN_6,  NO_ADC,                     NO_PWM,NO_PWM},  /* D06/PA6/PIN7 */ 

#if USE_TIMER2_PWM>0    
    {gpioPortA, GPIO_PIN_8,  NO_ADC,                     TIMER2,0x00  },  /* D03/PA8/PIN17, TIM2_CC0 #0*/
    {gpioPortA, GPIO_PIN_9,  NO_ADC,                     TIMER2,0x01  },  /* D04/PA9/PIN18, TIM2_CC1 #0*/
    {gpioPortA, GPIO_PIN_10, NO_ADC,                     TIMER2,0x02  },  /* D05/PA10/PIN19,TIM2_CC2 #0*/ 
#else
    {gpioPortA, GPIO_PIN_8,  NO_ADC,                     NO_PWM,NO_PWM},  /* D07/PA8/PIN17,   TIM2_CC0 #0*/ 
    {gpioPortA, GPIO_PIN_9,  NO_ADC,                     NO_PWM,NO_PWM},  /* D08/PA9/PIN18,   TIM2_CC1 #0*/ 
    {gpioPortA, GPIO_PIN_10,  NO_ADC,                     NO_PWM,NO_PWM},  /* D09/PA10/PIN19, TIM2_CC2 #0*/ 
#endif	

#if USE_TIMER3_PWM>0    
    {gpioPortA, GPIO_PIN_15,  NO_ADC,                     TIMER3,0x02},  /* D10/PA15/PIN64, TIM3_CC2 #0*/ 
#else
    {gpioPortA, GPIO_PIN_15,  NO_ADC,                     NO_PWM,NO_PWM},  /* D10/PA15/PIN64, TIM3_CC2 #0*/ 
#endif

	/* Port B */
#if USE_TIMER1_PWM>0    
    {gpioPortB, GPIO_PIN_7,  NO_ADC,                     TIMER1,0x30},  /* D11/PB07/PIN15, LFXTAL_P, TIM1_CC0 #3 */ 
    {gpioPortB, GPIO_PIN_8,  NO_ADC,                     TIMER1,0x31},  /* D12/PB08/PIN16, LFXTAL_N, TIM1_CC1 #3 */ 
#else
    {gpioPortB, GPIO_PIN_7,  NO_ADC,                     NO_PWM,NO_PWM},  /* D11/PB07/PIN15, LFXTAL_P, TIM1_CC0 #3 */ 
    {gpioPortB, GPIO_PIN_8,  NO_ADC,                     NO_PWM,NO_PWM},  /* D12/PB08/PIN16, LFXTAL_N, TIM1_CC1 #3 */ 
#endif

#if USE_TIMER1_PWM>0    
    {gpioPortB, GPIO_PIN_11, NO_ADC,                     TIMER1,0x32},  /* D13/PB11/PIN21, TIM1_CC2 #3, I2C1_SDA #1 */
#else
    {gpioPortB, GPIO_PIN_11, NO_ADC,                     NO_PWM,NO_PWM},  /* D13/PB11/PIN21, TIM1_CC2 #3, I2C1_SDA #1 */
#endif
    {gpioPortB, GPIO_PIN_12, NO_ADC,                     NO_PWM,NO_PWM},  /* D14/PB12/PIN22,              I2C1_SCL #1 */

    {gpioPortB, GPIO_PIN_13, NO_ADC,                     NO_PWM,NO_PWM},  /* D15/PB13/PIN24, HFXTAL_P, LEU0_TX #1*/
    {gpioPortB, GPIO_PIN_14, NO_ADC,                     NO_PWM,NO_PWM},  /* D16/PB14/PIN25, HFXTAL_N, LEU0_RX #1*/

	/* Port C */
    {gpioPortC, GPIO_PIN_0,  NO_ADC,                     NO_PWM,NO_PWM},  /* D17/PC00/PIN09, ACMP0_CH0, TIM0_CC1 #4, I2C0_SDA #4 */
    {gpioPortC, GPIO_PIN_1,  NO_ADC,                     NO_PWM,NO_PWM},  /* D18/PC01/PIN10, ACMP0_CH1, TIM0_CC2 #4, I2C0_SDA #4 */
    {gpioPortC, GPIO_PIN_2,  NO_ADC,                     NO_PWM,NO_PWM},  /* D19/PC02/PIN11, ACMP0_CH2, */
    {gpioPortC, GPIO_PIN_3,  NO_ADC,                     NO_PWM,NO_PWM},  /* D20/PC03/PIN12, ACMP0_CH3, */
    {gpioPortC, GPIO_PIN_4,  NO_ADC,                     NO_PWM,NO_PWM},  /* D21/PC04/PIN13, ACMP0_CH4, I2C1_SDA #0 */
    {gpioPortC, GPIO_PIN_5,  NO_ADC,                     NO_PWM,NO_PWM},  /* D22/PC05/PIN14, ACMP0_CH5, I2C1_SCL #0 */

    {gpioPortC, GPIO_PIN_6,  NO_ADC,                     NO_PWM,NO_PWM},  /* D23/PC06/PIN37, ACMP0_CH6, I2C0_SCL #2 */
    {gpioPortC, GPIO_PIN_7,  NO_ADC,                     NO_PWM,NO_PWM},  /* D24/PC07/PIN38, ACMP0_CH7, I2C0_SCL #2 */
#if USE_TIMER2_PWM>0    
    {gpioPortC, GPIO_PIN_8,  NO_ADC,                     TIMER2,0x20},  /* D25/PC08/PIN41, ACMP1_CH0, TIM2_CC0 #2 */
    {gpioPortC, GPIO_PIN_9,  NO_ADC,                     TIMER2,0x21},  /* D26/PC09/PIN42, ACMP1_CH1, TIM2_CC1 #2 */
    {gpioPortC, GPIO_PIN_10, NO_ADC,                     TIMER2,0x22},  /* D27/PC10/PIN43, ACMP1_CH2, TIM2_CC2 #2 */
#else
    {gpioPortC, GPIO_PIN_8,  NO_ADC,                     NO_PWM,NO_PWM},  /* D25/PC08/PIN41, ACMP1_CH0, TIM2_CC0 #2 */
    {gpioPortC, GPIO_PIN_9,  NO_ADC,                     NO_PWM,NO_PWM},  /* D26/PC09/PIN42, ACMP1_CH1, TIM2_CC1 #2 */
    {gpioPortC, GPIO_PIN_10, NO_ADC,                     NO_PWM,NO_PWM},  /* D27/PC10/PIN43, ACMP1_CH2, TIM2_CC2 #2 */
#endif	
    {gpioPortC, GPIO_PIN_11, NO_ADC,                     NO_PWM,NO_PWM},  /* D28/PC11/PIN44, ACMP1_CH3 */
    {gpioPortC, GPIO_PIN_12, NO_ADC,                     NO_PWM,NO_PWM},  /* D29/PC12/PIN45, ACMP1_CH4 */
                                                                         
#if USE_TIMER1_PWM>0    
    {gpioPortC, GPIO_PIN_13, NO_ADC,                     TIMER1,0x00     },  /* D30/PC13/PIN46, ACMP1_CH5, TIM1_CC0 #0 */
    {gpioPortC, GPIO_PIN_14, NO_ADC,                     TIMER1,0x01     },  /* D31/PC14/PIN47, ACMP1_CH6, TIM1_CC1 #0 */
    {gpioPortC, GPIO_PIN_15, NO_ADC,                     TIMER1,0x02     },  /* D32/PC15/PIN48, ACMP1_CH7, TIM1_CC2 #0, DBG_SWO #1*/
#else
    {gpioPortC, GPIO_PIN_13, NO_ADC,                     NO_PWM,NO_PWM},  /* D30/PC13/PIN46, ACMP1_CH5, TIM1_CC0 #0 */
    {gpioPortC, GPIO_PIN_14, NO_ADC,                     NO_PWM,NO_PWM},  /* D31/PC14/PIN47, ACMP1_CH6, TIM1_CC1 #0 */
    {gpioPortC, GPIO_PIN_15, NO_ADC,                     NO_PWM,NO_PWM},  /* D32/PC15/PIN48, ACMP1_CH7, TIM1_CC2 #0, DBG_SWO #1*/
#endif

	/* Port D */
    {gpioPortD, GPIO_PIN_0, _ADC_SINGLECTRL_INPUTSEL_CH0,NO_PWM,NO_PWM},  /* D33/PD0/PIN28, ADC0_CH0 */
    {gpioPortD, GPIO_PIN_1, _ADC_SINGLECTRL_INPUTSEL_CH1,NO_PWM,NO_PWM},  /* D34/PD1/PIN29, ADC0_CH1, TIM0_CC0 #3, DBG_SWO #2 */
    {gpioPortD, GPIO_PIN_2, _ADC_SINGLECTRL_INPUTSEL_CH2,NO_PWM,NO_PWM},  /* D35/PD2/PIN30, ADC0_CH2, TIM0_CC1 #3, DBG_SWO #3 */
    {gpioPortD, GPIO_PIN_3, _ADC_SINGLECTRL_INPUTSEL_CH3,NO_PWM,NO_PWM},  /* D36/PD3/PIN31, ADC0_CH3, TIM0_CC2 #3 */

    {gpioPortD, GPIO_PIN_4, _ADC_SINGLECTRL_INPUTSEL_CH4,NO_PWM,NO_PWM},  /* D37/PD4/PIN32, ADC0_CH4 */
    {gpioPortD, GPIO_PIN_5, _ADC_SINGLECTRL_INPUTSEL_CH5,NO_PWM,NO_PWM},  /* D38/PD5/PIN33, ADC0_CH5 */
    {gpioPortD, GPIO_PIN_6, _ADC_SINGLECTRL_INPUTSEL_CH6,NO_PWM,NO_PWM},  /* D39/PD6/PIN34, ADC0_CH6, I2C0_SDA #1 */
    {gpioPortD, GPIO_PIN_7, _ADC_SINGLECTRL_INPUTSEL_CH7,NO_PWM,NO_PWM},  /* D40/PD7/PIN35, ADC0_CH7, I2C0_SCL #1 */
    {gpioPortD, GPIO_PIN_8, NO_ADC,                      NO_PWM,NO_PWM},  /* D41/PD8/PIN36 */

	/* Port E */
    {gpioPortE, GPIO_PIN_8, NO_ADC,                     NO_PWM,NO_PWM},  /* D42/PE8/PIN56 */
    {gpioPortE, GPIO_PIN_9, NO_ADC,                     NO_PWM,NO_PWM},  /* D43/PE9/PIN57 */
#if USE_TIMER1_PWM>0    
    {gpioPortE, GPIO_PIN_10, NO_ADC,                     TIMER1,0x10},  /* D44/PE10/PIN58, TIM1_CC0 #1, BOOT_TX */
    {gpioPortE, GPIO_PIN_11, NO_ADC,                     TIMER1,0x11},  /* D45/PE11/PIN59, TIM1_CC1 #1, BOOT_RX */
    {gpioPortE, GPIO_PIN_12, NO_ADC,                     TIMER1,0x12},  /* D46/PE12/PIN60, TIM1_CC2 #1, I2C0_SDA #6 */
#else
    {gpioPortE, GPIO_PIN_10, NO_ADC,                     NO_PWM,NO_PWM},  /* D44/PE10/PIN58, TIM1_CC0 #1, BOOT_TX */
    {gpioPortE, GPIO_PIN_11, NO_ADC,                     NO_PWM,NO_PWM},  /* D45/PE11/PIN59, TIM1_CC1 #1, BOOT_RX */
    {gpioPortE, GPIO_PIN_12, NO_ADC,                     NO_PWM,NO_PWM},  /* D46/PE12/PIN60, TIM1_CC2 #1, I2C0_SDA #6 */
#endif
    {gpioPortE, GPIO_PIN_13, NO_ADC,                     NO_PWM,NO_PWM},  /* D47/PE13/PIN61,              I2C0_SCL #6 */

#if USE_TIMER3_PWM>0    
    {gpioPortE, GPIO_PIN_14, NO_ADC,                     TIMER3,0x00},  /* D48/PE14/PIN62, TIM3_CC0 #0, LEU0_TX #2 */
    {gpioPortE, GPIO_PIN_15, NO_ADC,                     TIMER3,0x01},  /* D49/PE15/PIN63, TIM3_CC1 #0, LEU0_RX #2 */
#else
    {gpioPortE, GPIO_PIN_14, NO_ADC,                     NO_PWM,NO_PWM},  /* D48/PE14/PIN62, TIM3_CC0 #0, LEU0_TX #2 */
    {gpioPortE, GPIO_PIN_15, NO_ADC,                     NO_PWM,NO_PWM},  /* D49/PE15/PIN63, TIM3_CC1 #0, LEU0_RX #2 */
#endif

	/* Port F */
#if USE_TIMER0_PWM>0
    {gpioPortF, GPIO_PIN_0,  NO_ADC,                     TIMER0,0x50},  /* D50/PF0/PIN49, TIM0_CC0 #5, I2C0_SDA #5, DBG_SWCLK #0/1/2/3 */
    {gpioPortF, GPIO_PIN_1,  NO_ADC,                     TIMER0,0x51},  /* D51/PF1/PIN50, TIM0_CC1 #5, I2C0_SCL #5, DBG_SWDIO #0/1/2/3 */
    {gpioPortF, GPIO_PIN_2,  NO_ADC,                     TIMER0,0x52},  /* D52/PF2/PIN51, TIM0_CC2 #5, DBG_SWO #0 */
#else
    {gpioPortF, GPIO_PIN_0,  NO_ADC,                     NO_PWM,NO_PWM},  /* D50/PF0/PIN49, TIM0_CC0 #5, I2C0_SDA #5, DBG_SWCLK #0/1/2/3 */
    {gpioPortF, GPIO_PIN_1,  NO_ADC,                     NO_PWM,NO_PWM},  /* D51/PF1/PIN50, TIM0_CC1 #5, I2C0_SCL #5, DBG_SWDIO #0/1/2/3 */
    {gpioPortF, GPIO_PIN_2,  NO_ADC,                     NO_PWM,NO_PWM},  /* D52/PF2/PIN51, TIM0_CC2 #5, DBG_SWO #0 */
#endif

    {gpioPortF, GPIO_PIN_3,  NO_ADC,                     NO_PWM,NO_PWM},  /* D53/PF3/PIN52 */
    {gpioPortF, GPIO_PIN_4,  NO_ADC,                     NO_PWM,NO_PWM},  /* D54/PF4/PIN53 */
    {gpioPortF, GPIO_PIN_5,  NO_ADC,                     NO_PWM,NO_PWM},  /* D55/PF5/PIN54 */
};

#ifdef __cplusplus
}
#endif
