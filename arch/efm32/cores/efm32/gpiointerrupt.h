/***************************************************************************//**
 * @file gpiointerrupt.h
 * @brief GPIOINT API definition
 * @version 5.1.2
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#ifndef GPIOINTERRUPT_H
#define GPIOINTERRUPT_H

#include "em_device.h"


//#define CHANGE  1
//#define FALLING 2
//#define RISING  3

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*GPIOINT_IrqCallbackPtr_t)(void);

void attachInterrupt(uint8_t ucPin, GPIOINT_IrqCallbackPtr_t callback, int mode);
void detachInterrupt(uint8_t ucPin);

/** @} (end addtogroup GPIOINT */
/** @} (end addtogroup emdrv) */
#ifdef __cplusplus
}
#endif

#endif /* GPIOINTERRUPT_H */
