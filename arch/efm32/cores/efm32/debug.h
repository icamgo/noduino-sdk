/*
  Copyright (c) 2017 Daniel Fekete
                2018 huaweiwx@sina.com for efm32

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

#ifndef EFM32_DEBUG_H
#define EFM32_DEBUG_H

#include "Arduino.h"

#include <stdio.h>
#include <stdarg.h>

#include "efm32.h"
#include "cmsis_gcc.h"


# ifndef  ASSERT_PARAM
#   define ASSERT_PARAM  EFM_ASSERT
# endif
#define assert_param(expr)  EFM_ASSERT(expr)   /*compatible with stm32*/

// Returns if we are in an interrupt
inline bool isInterrupt() {
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0 ;
}



#ifndef __LOG_LEVEL
  #define __LOG_LEVEL    0
#endif

#define __LOG_FATAL   __LOG_LEVEL >= 1
#define __LOG_ERROR   __LOG_LEVEL >= 2
#define __LOG_WARNING __LOG_LEVEL >= 3
#define __LOG_INFO    __LOG_LEVEL >= 4
#define __LOG_DEBUG   __LOG_LEVEL >= 5
#define __LOG_TRACE   __LOG_LEVEL >= 6

#ifdef __cplusplus
extern "C" {
#endif


void errorLedBlink(char* file, uint32_t n);
void _Error_Handler(char* file, uint32_t line);
// Internal: use PRINT_XXX instead
void print_log(const char *level, const char *format, const char *file, const int line, ...);

//use stderr out debug info
void debug(const char *format, ...);
void debug_if(int condition, const char *format, ...);

#ifdef __cplusplus
}
#endif

#if __unix__
#define __FILENAME__ (char *)(__builtin_strrchr(__FILE__, '/') ? __builtin_strrchr(__FILE__, '/') + 1 : __FILE__)
#else
#define __FILENAME__ (char *)(__builtin_strrchr(__FILE__, '\\') ? __builtin_strrchr(__FILE__, '\\') + 1 : __FILE__)	
#endif

#define PRINT_LOG(level, format, ...) print_log(level, format, __FILENAME__, __LINE__, ##__VA_ARGS__)


#if __LOG_FATAL
#define PRINT_FATAL(...) PRINT_LOG("FATAL", __VA_ARGS__)
#else
#define PRINT_FATAL(...)
#endif

#if __LOG_ERROR
#define PRINT_ERROR(...) PRINT_LOG("ERROR", __VA_ARGS__)
#else
#define PRINT_ERROR(...)
#endif

#if __LOG_WARNING
#define PRINT_WARNING(...) PRINT_LOG("WARNING", __VA_ARGS__)
#else
#define PRINT_WARNING(...)
#endif

#if __LOG_INFO
#define PRINT_INFO(...) PRINT_LOG("INFO", __VA_ARGS__)
#else
#define PRINT_INFO(...)
#endif

#if __LOG_DEBUG
#define PRINT_DEBUG(...) PRINT_LOG("DEBUG", __VA_ARGS__)
#else
#define PRINT_DEBUG(...)
#endif

#if __LOG_TRACE
#define PRINT_TRACE(...) PRINT_LOG("TRACE", __VA_ARGS__)
#else
#define PRINT_TRACE(...)
#endif

#endif

