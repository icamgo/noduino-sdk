/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/* form mbed by huaweiwx@sina.com 2017.12.5*/

#ifndef _TOOLSCHAIN_H_
#define _TOOLSCHAIN_H_

// Warning for unsupported compilers
#if !defined(__GNUC__)   /* GCC        */ \
 && !defined(__CC_ARM)   /* ARMCC      */ \
 && !defined(__ICCARM__) /* IAR        */
#warning "This compiler is not yet supported."
#endif

//for toolschain
 #define MBED_STRINGIFY(a) MBED_STRINGIFY_(a)
 #define MBED_STRINGIFY_(a) #a
 
#if defined(__GNUC__)
// #define __alignas(n)    __attribute__ ((aligned (n)))
// #define __unused __attribute__((unused))
//  #define __packed  __attribute__((__packed__)) 
  #define __weak __attribute__((weak))
// #define __inline static inline __attribute__((always_inline))
// #define __section(name) __attribute__ ((section (name)))
// #define __noreturn __attribute__((__noreturn__))
 #define __call_addr() __builtin_extract_return_addr(__builtin_return_address(0))
 #define __deprecated(x) __attribute__((deprecated(x)))
 #define __atadr(adr) __attribute__((at(adr)))
 
#elif defined(__CC_ARM)
 #define __alignas(n)    __align(n)
 #define __deprecated(x) __attribute__((deprecated))
 #define __unused __attribute__((unused))
 #ifndef __packed
  #define __packed __attribute__((packed)) 
 #endif
 #ifndef __weak 
  #define __weak __attribute__((weak))
 #endif 
 #define __inline static inline __attribute__((always_inline))
 #define __call_addr()  __builtin_return_address(0)
 #define __section(name) __attribute__ ((section (name)))
 #define __noreturn __attribute__((noreturn))

#elif defined(__ICCARM__)

 #define __alignas(n)  _Pragma(MBED_STRINGIFY(data_alignment=n))
 #define __deprecated(x)
 #define __unused
 #define __inline _Pragma("inline=forced") static
 #define __call_addr() (NULL)
 #define __section(name) _Pragma(MBED_STRINGIFY(location=name))
 #define __noreturn

#endif


#define __deprecated_since(D, M) __deprecated(M " [since " D "]")
#define __unusedparm(x) __unused x
/**
 * Macro expanding to a string literal of the enclosing function name.
 *
 * The string returned takes into account language specificity and yield human
 * readable content.
 *
 * As an example, if the macro is used within a C++ function then the string
 * literal containing the function name will contain the complete signature of
 * the function - including template parameters - and namespace qualifications.
 */
#ifndef PRETTY_FUNCTION
#define PRETTY_FUNCTION __PRETTY_FUNCTION__
#endif

#ifndef PRINTF
#if defined(__GNUC__) || defined(__CC_ARM)
#define PRINTF(format_idx, first_param_idx) __attribute__ ((__format__(__printf__, format_idx, first_param_idx)))
#else
#define PRINTF(format_idx, first_param_idx)
#endif
#endif

#ifndef PRINTF_METHOD
#if defined(__GNUC__) || defined(__CC_ARM)
#define PRINTF_METHOD(format_idx, first_param_idx) __attribute__ ((__format__(__printf__, format_idx+1, first_param_idx+1)))
#else
#define PRINTF_METHOD(format_idx, first_param_idx)
#endif
#endif

#ifndef SCANF
#if defined(__GNUC__) || defined(__CC_ARM)
#define SCANF(format_idx, first_param_idx) __attribute__ ((__format__(__scanf__, format_idx, first_param_idx)))
#else
#define SCANF(format_idx, first_param_idx)
#endif
#endif

#ifndef SCANF_METHOD
#if defined(__GNUC__) || defined(__CC_ARM)
#define SCANF_METHOD(format_idx, first_param_idx) __attribute__ ((__format__(__scanf__, format_idx+1, first_param_idx+1)))
#else
#define SCANF_METHOD(format_idx, first_param_idx)
#endif
#endif


// FILEHANDLE declaration
//#if defined(TOOLCHAIN_ARM)
//#include <rt_sys.h>
//#endif

#ifndef FILEHANDLE
typedef int FILEHANDLE;
#endif

typedef void (*voidFuncPtr)(void);
typedef void (*voidArgumentFuncPtr)(void *);

#endif //_TOOLSCHAIN_H_

