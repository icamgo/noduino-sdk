/**
 * Copyright (c) 20011-2017 Bill Greiman
 * This file is part of the SdFat library for SD memory cards.
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
/**
 * @file
 * @brief Fast Digital Pin functions
 *
 * @defgroup digitalPin Fast Pin I/O
 * @details  Fast Digital I/O functions and template class.
 * @{
 */
#ifndef DigitalPin_h
#define DigitalPin_h
#if defined(__AVR__) || defined(DOXYGEN)
#include <avr/io.h>
/** GpioPinMap type */
struct GpioPinMap_t {
  volatile uint8_t* pin;   /**< address of PIN for this pin */
  volatile uint8_t* ddr;   /**< address of DDR for this pin */
  volatile uint8_t* port;  /**< address of PORT for this pin */
  uint8_t mask;            /**< bit mask for this pin */
};

/** Initializer macro. */
#define GPIO_PIN(reg, bit) {&PIN##reg, &DDR##reg, &PORT##reg, 1 << bit}

// Include pin map for current board.
#include "boards/GpioPinMap.h"
//------------------------------------------------------------------------------
/** generate bad pin number error */
void badPinNumber(void)
  __attribute__((error("Pin number is too large or not a constant")));
//------------------------------------------------------------------------------
/** Check for valid pin number
 * @param[in] pin Number of pin to be checked.
 */
static inline __attribute__((always_inline))
void badPinCheck(uint8_t ucPin) {
  if (!__builtin_constant_p(ucPin) || pin >= NUM_DIGITAL_PINS) {
     badPinNumber();
  }
}
//------------------------------------------------------------------------------
/** DDR register address
 * @param[in] pin Arduino pin number
 * @return register address
 */
static inline __attribute__((always_inline))
volatile uint8_t* ddrReg(uint8_t ucPin) {
  badPinCheck(ucPin);
  return GpioPinMap[ucPin].ddr;
}
//------------------------------------------------------------------------------
/** Bit mask for pin
 * @param[in] pin Arduino pin number
 * @return mask
 */
static inline __attribute__((always_inline))
uint8_t pinMask(uint8_t ucPin) {
  badPinCheck(ucPin);
  return GpioPinMap[ucPin].mask;
}
//------------------------------------------------------------------------------
/** PIN register address
 * @param[in] pin Arduino pin number
 * @return register address
 */
static inline __attribute__((always_inline))
volatile uint8_t* pinReg(uint8_t ucPin) {
  badPinCheck(ucPin);
  return GpioPinMap[ucPin].pin;
}
//------------------------------------------------------------------------------
/** PORT register address
 * @param[in] pin Arduino pin number
 * @return register address
 */
static inline __attribute__((always_inline))
volatile uint8_t* portReg(uint8_t ucPin) {
  badPinCheck(ucPin);
  return GpioPinMap[ucPin].port;
}
//------------------------------------------------------------------------------
/** Fast write helper.
 * @param[in] address I/O register address
 * @param[in] mask bit mask for pin
 * @param[in] level value for bit
 */
static inline __attribute__((always_inline))
void fastBitWriteSafe(volatile uint8_t* address, uint8_t mask, bool level) {
  uint8_t s;
  if (address > reinterpret_cast<uint8_t*>(0X3F)) {
    s = SREG;
    cli();
  }
  if (level) {
    *address |= mask;
  } else {
    *address &= ~mask;
  }
  if (address > reinterpret_cast<uint8_t*>(0X3F)) {
    SREG = s;
  }
}
//------------------------------------------------------------------------------
/** Read pin value.
 * @param[in] pin Arduino pin number
 * @return value read
 */
static inline __attribute__((always_inline))
bool fastDigitalRead(uint8_t ucPin) {
  return *pinReg(ucPin) & pinMask(ucPin);
}
//------------------------------------------------------------------------------
/** Toggle a pin.
 * @param[in] pin Arduino pin number
 *
 * If the pin is in output mode toggle the pin level.
 * If the pin is in input mode toggle the state of the 20K pullup.
 */
static inline __attribute__((always_inline))
void fastDigitalToggle(uint8_t ucPin) {
    if (pinReg(ucPin) > reinterpret_cast<uint8_t*>(0X3F)) {
      // must write bit to high address port
      *pinReg(ucPin) = pinMask(ucPin);
    } else {
      // will compile to sbi and PIN register will not be read.
      *pinReg(ucPin) |= pinMask(ucPin);
    }
}
//------------------------------------------------------------------------------
/** Set pin value.
 * @param[in] pin Arduino pin number
 * @param[in] level value to write
 */
static inline __attribute__((always_inline))
void fastDigitalWrite(uint8_t ucPin, bool level) {
  fastBitWriteSafe(portReg(ucPin), pinMask(ucPin), level);
}
//------------------------------------------------------------------------------
/** Write the DDR register.
 * @param[in] pin Arduino pin number
 * @param[in] level value to write
 */
static inline __attribute__((always_inline))
void fastDdrWrite(uint8_t ucPin, bool level) {
  fastBitWriteSafe(ddrReg(ucPin), pinMask(ucPin), level);
}
//------------------------------------------------------------------------------
/** Set pin mode.
 * @param[in] pin Arduino pin number
 * @param[in] mode INPUT, OUTPUT, or INPUT_PULLUP.
 *
 * The internal pullup resistors will be enabled if mode is INPUT_PULLUP
 * and disabled if the mode is INPUT.
 */
static inline __attribute__((always_inline))
void fastPinMode(uint8_t ucPin, uint8_t mode) {
  fastDdrWrite(ucPin, mode == OUTPUT);
  if (mode != OUTPUT) {
    fastDigitalWrite(ucPin, mode == INPUT_PULLUP);
  }
}
#else

#if	defined(CORE_TEENSY)
//------------------------------------------------------------------------------
/** read pin value
 * @param[in] pin Arduino pin number
 * @return value read
 */
static inline __attribute__((always_inline))
bool fastDigitalRead(uint8_t ucPin) {
  return *portInputRegister(ucPin);
}
//------------------------------------------------------------------------------
/** Set pin value
 * @param[in] pin Arduino pin number
 * @param[in] level value to write
 */
static inline __attribute__((always_inline))
void fastDigitalWrite(uint8_t ucPin, bool value) {
  if (value) {
    *portSetRegister(ucPin) = 1;
  } else {
    *portClearRegister(ucPin) = 1;
  }
}

#elif defined(__SAM3X8E__) || defined(__SAM3X8H__)
//------------------------------------------------------------------------------
/** read pin value
 * @param[in] pin Arduino pin number
 * @return value read
 */
static inline __attribute__((always_inline))
bool fastDigitalRead(uint8_t ucPin) {
  return g_APinDescription[ucPin].pPort->PIO_PDSR & g_APinDescription[ucPin].ucPin;
}
//------------------------------------------------------------------------------
/** Set pin value
 * @param[in] pin Arduino pin number
 * @param[in] level value to write
 */
static inline __attribute__((always_inline))
void fastDigitalWrite(uint8_t ucPin, bool value) {
  if (value) {
    g_APinDescription[ucPin].pPort->PIO_SODR = g_APinDescription[ucPin].ucPin;
  } else {
    g_APinDescription[ucPin].pPort->PIO_CODR = g_APinDescription[ucPin].ucPin;
  }
}
#elif defined(ESP8266)
//------------------------------------------------------------------------------
/** Set pin value
 * @param[in] pin Arduino pin number
 * @param[in] val value to write
 */
static inline __attribute__((always_inline))
void fastDigitalWrite(uint8_t ucPin, uint8_t val) {
  if (ucPin < 16) {
    if (val) {
      GPOS = (1 << ucPin);
    } else {
      GPOC = (1 << ucPin);
    }
  } else if (ucPin == 16) {
    if (val) {
      GP16O |= 1;
    } else {
      GP16O &= ~1;
    }
  }
}
//------------------------------------------------------------------------------
/** Read pin value
 * @param[in] pin Arduino pin number
 * @return value read
 */
static inline __attribute__((always_inline))
bool fastDigitalRead(uint8_t ucPin) {
  if (ucPin < 16) {
    return GPIP(ucPin);
  } else if (ucPin == 16) {
    return GP16I & 0x01;
  }
  return 0;
}

#else  // CORE_TEENSY & STM/EFM32GENERIC
//------------------------------------------------------------------------------
inline void fastDigitalWrite(uint8_t ucPin, bool value) {
  digitalWrite(ucPin, value);
}
//------------------------------------------------------------------------------
inline bool fastDigitalRead(uint8_t ucPin) {
  return digitalRead(ucPin);
}
//------------------------------------------------------------------------------
inline void fastDigitalToggle(uint8_t ucPin) {
#if defined(STM32GENERIC)||defined(EFM32GENERIC)  //add by huaweiwx@sina.com 2018.8.28
	digitalToggle(ucPin);
#else
	fastDigitalWrite(ucPin, !fastDigitalRead(ucPin));
#endif
}  

#if defined(STM32GENERIC)||defined(EFM32GENERIC)  //add by huaweiwx@sina.com 2018.8.28
#ifdef  __cplusplus
inline void fastDigitalWrite(ARDUINOPIN_TypeDef CPin, bool value) {
  digitalWrite(CPin, value);
}
//------------------------------------------------------------------------------
inline bool fastDigitalRead(ARDUINOPIN_TypeDef CPin) {
  return digitalRead(CPin);
}
//------------------------------------------------------------------------------
inline void fastDigitalToggle(ARDUINOPIN_TypeDef CPin) {
  digitalToggle(CPin);
}
#endif //__cplusplus
#endif //STM/EFM32GENERIC

#endif 
//------------------------------------------------------------------------------
inline void fastPinMode(uint8_t ucPin, uint8_t mode) {
  pinMode(ucPin, mode);
}
#endif  // __AVR__
//------------------------------------------------------------------------------
/** set pin configuration
 * @param[in] pin Arduino pin number
 * @param[in] mode mode INPUT or OUTPUT.
 * @param[in] level If mode is output, set level high/low.
 *                  If mode is input, enable or disable the pin's 20K pullup.
 */
#define fastPinConfig(pin, mode, level)\
  {fastPinMode(pin, mode); fastDigitalWrite(pin, level);}

//==============================================================================
#endif  // DigitalPin_h

