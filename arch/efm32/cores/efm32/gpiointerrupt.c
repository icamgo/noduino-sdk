/***************************************************************************//**
 * @file gpiointerrupt.c
 * @brief GPIOINT API implementation
 * @version 5.1.2
 *
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2015 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#include "arduino.h"

/*******************************************************************************
 ********************************   MACROS   ***********************************
 ******************************************************************************/

/*******************************************************************************
 *******************************   STRUCTS   ***********************************
 ******************************************************************************/

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

typedef struct
{
  /* Pin number in range of 0 to 15 */
  uint8_t ucPin;

  /* Pointer to the callback function */
  GPIOINT_IrqCallbackPtr_t callback;

} GPIOINT_CallbackDesc_t;


/*******************************************************************************
 ********************************   GLOBALS   **********************************
 ******************************************************************************/

/* Array of user callbacks. One for each pin. */
static GPIOINT_IrqCallbackPtr_t gpioCallbacks[16] = {0};

/*******************************************************************************
 ******************************   PROTOTYPES   *********************************
 ******************************************************************************/
static void GPIOINT_IRQDispatcher(uint32_t iflags);

/** @endcond */

/*******************************************************************************
 ***************************   GLOBAL FUNCTIONS   ******************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *   Initialization of GPIOINT module.
 *
 ******************************************************************************/
void GPIOINT_Init(void)
{
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}


/***************************************************************************//**
 * @brief
 *   Registers user callback for given pin number.
 *
 * @details
 *   Use this function to register a callback which shall be called upon
 *   interrupt generated from given pin number (port is irrelevant). Interrupt
 *   itself must be configured externally. Function overwrites previously
 *   registered callback.
 *
 * @param[in] pin
 *   Pin number for the callback.
 * @param[in] callbackPtr
 *   A pointer to callback function.
 ******************************************************************************/
void GPIOINT_CallbackRegister(uint8_t pin, GPIOINT_IrqCallbackPtr_t callbackPtr)
{
  CORE_ATOMIC_SECTION(
    /* Dispatcher is used */
    gpioCallbacks[pin] = callbackPtr;
  )
}

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

/***************************************************************************//**
 * @brief
 *   Function calls users callback for registered pin interrupts.
 *
 * @details
 *   This function is called when GPIO interrupts are handled by the dispatcher.
 *   Function gets even or odd interrupt flags and calls user callback
 *   registered for that pin. Function iterates on flags starting from MSB.
 *
 * @param iflags
 *  Interrupt flags which shall be handled by the dispatcher.
 *
 ******************************************************************************/
static void GPIOINT_IRQDispatcher(uint32_t iflags)
{
  uint32_t irqIdx;
  GPIOINT_IrqCallbackPtr_t callback;

  /* check for all flags set in IF register */
  while(iflags)
  {
    irqIdx = SL_CTZ(iflags);

    /* clear flag*/
    iflags &= ~(1 << irqIdx);

    callback = gpioCallbacks[irqIdx];
    if (callback)
    {
      /* call user callback */
      callback();
    }
  }
}

/***************************************************************************//**
 * @brief
 *   GPIO EVEN interrupt handler. Interrupt handler clears all IF even flags and
 *   call the dispatcher passing the flags which triggered the interrupt.
 *
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  uint32_t iflags;

  /* Get all even interrupts. */
  iflags = GPIO_IntGetEnabled() & 0x00005555;

  /* Clean only even interrupts. */
  GPIO_IntClear(iflags);

  GPIOINT_IRQDispatcher(iflags);
}


/***************************************************************************//**
 * @brief
 *   GPIO ODD interrupt handler. Interrupt handler clears all IF odd flags and
 *   call the dispatcher passing the flags which triggered the interrupt.
 *
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  uint32_t iflags;

  /* Get all odd interrupts. */
  iflags = GPIO_IntGetEnabled() & 0x0000AAAA;

  /* Clean only odd interrupts. */
  GPIO_IntClear(iflags);

  GPIOINT_IRQDispatcher(iflags);
}

void attachInterrupt(uint8_t ucPin, GPIOINT_IrqCallbackPtr_t callback, int mode){
	uint8_t pin = g_Pin2PortMapArray[ucPin].Pin_abstraction;
	GPIOINT_CallbackRegister(pin, callback);
	GPIOINT_Init();
	switch (mode){
		case RISING:
		  GPIO_IntConfig(g_Pin2PortMapArray[ucPin].GPIOx_Port, pin, true, 0, true);
		  break;
		case FALLING:  
		  GPIO_IntConfig(g_Pin2PortMapArray[ucPin].GPIOx_Port, pin, 0, true, true);
		  break;
		case CHANGE:
		  GPIO_IntConfig(g_Pin2PortMapArray[ucPin].GPIOx_Port, pin, true, true,true);
		  break;
	}
	GPIO_IntEnable(1<<pin);
}

void detachInterrupt(uint8_t ucPin) {
	GPIOINT_CallbackRegister(g_Pin2PortMapArray[ucPin].Pin_abstraction,0);
}
