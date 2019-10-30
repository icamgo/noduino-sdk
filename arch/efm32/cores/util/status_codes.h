/**
 * \file
 *
 * \brief Status code definitions.
 *
 * This file defines various status codes returned by functions,
 * indicating success or failure as well as what kind of failure.
 *
 * Copyright (c) 2011-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef STATUS_CODES_H_INCLUDED
#define STATUS_CODES_H_INCLUDED

/* Note: this is a local workaround to avoid a pre-processor clash due to the
 * lwIP macro ERR_TIMEOUT. */
#if defined(__LWIP_ERR_H__) && defined(ERR_TIMEOUT)
#if (ERR_TIMEOUT != -3)

/* Internal check to make sure that the later restore of lwIP's ERR_TIMEOUT
 * macro is set to the correct value. Note that it is highly improbable that
 * this value ever changes in lwIP. */
#error ASF developers: check lwip err.h new value for ERR_TIMEOUT
#endif

#undef ERR_TIMEOUT
#endif

/**
 * Status code that may be returned by shell commands and protocol
 * implementations.
 *
 * \note Any change to these status codes and the corresponding
 * message strings is strictly forbidden. New codes can be added,
 * however, but make sure that any message string tables are updated
 * at the same time.
 */
enum status_code {
	STATUS_OK               =  0, //!< Success
	STATUS_ERR_TIMEOUT      =  0x12,
	STATUS_ERR_BUSY         =  0x19,
	STATUS_ERR_DENIED       =  0x1C,

//  STATUS_USERDEF
    STATUS_USER21           =  0x21,
    STATUS_USER22           =  0x22,
    STATUS_USER23           =  0x23,
    STATUS_USER24           =  0x24,
    STATUS_USER25           =  0x25,
    STATUS_USER26           =  0x26,
    STATUS_USER27           =  0x27,
	
//  ERR_USERDEF
    ERR_Handler             =  0X31,
	ERR_USER32              =  0x32,
	
	ERR_HardFault           =  0x51,
	ERR_USER52              =  0x52,
	ERR_INIT          		=  0x55,
//	
	
	ERR_IO_ERROR            =  0x80, //!< I/O error
	ERR_FLUSHED             =  0x81, //!< Request flushed from queue
	ERR_TIMEOUT             =  0x82, //!< Operation timed out
	ERR_BAD_DATA            =  0x83, //!< Data integrity check failed
	ERR_PROTOCOL            =  0x84, //!< Protocol error
	ERR_UNSUPPORTED_DEV     =  0x85, //!< Unsupported device
	ERR_NO_MEMORY           =  0x86, //!< Insufficient memory
	ERR_INVALID_ARG         =  0x87, //!< Invalid argument
	ERR_BAD_ADDRESS         =  0x88, //!< Bad address
	ERR_BUSY                =  0x89, //!< Resource is busy
	ERR_BAD_FORMAT          =  0x8a, //!< Data format not recognized
	ERR_NO_TIMER            =  0x8b, //!< No timer available
	ERR_TIMER_ALREADY_RUNNING   =  0x8c, //!< Timer already running
	ERR_TIMER_NOT_RUNNING   =  0x8d, //!< Timer not running
	ERR_ABORTED             =  0x8e, //!< Operation aborted by user
	/**
	 * \brief Operation in progress
	 *
	 * This status code is for driver-internal use when an operation
	 * is currently being performed.
	 *
	 * \note Drivers should never return this status code to any
	 * callers. It is strictly for internal use.
	 */
	OPERATION_IN_PROGRESS	= 0xfe,
};

typedef enum status_code status_code_t;

#if defined(__LWIP_ERR_H__)
#define ERR_TIMEOUT -3
#endif

#endif /* STATUS_CODES_H_INCLUDED */
