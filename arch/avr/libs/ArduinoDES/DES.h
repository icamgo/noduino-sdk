/*
	DES.h - Library for DES and 3DES encryption and decryption
	Ported to the Arduino platform 2013 by Tim Riemann
   
	Original version taken from the AVR-Crypto-Lib
	(http://www.das-labor.org/wiki/AVR-Crypto-Lib)
	Copyright (C) 2006-2010  Daniel Otte (daniel.otte@rub.de)

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/**
 * \file     des.c
 * \author   Daniel Otte
 * \email    daniel.otte@rub.de
 * \date     2007-06-16
 * \brief    DES and EDE-DES implementation
 * \license      GPLv3 or later
 */

#ifndef DES_h
#define DES_h

#if  (defined(__linux) || defined(linux)) && !defined(__ARDUINO_X86__)

  #define RF24_LINUX
  
  #include <stdint.h>
  #include <stdio.h>
  #include <stdlib.h>
  #include <string.h>
  #include <sys/time.h>
  #include <unistd.h> 
#else
  #include <Arduino.h>
#endif
#include "DES.h"

#include <stdint.h>
#include <string.h>
#if defined(__ARDUINO_X86__) || (defined (__linux) || defined (linux))
	#undef PROGMEM
	#define PROGMEM __attribute__(( section(".progmem.data") ))
	#define pgm_read_byte(p) (*(p))
	typedef unsigned char byte;
#else
	#include <avr/pgmspace.h>
#endif

/* the FIPS 46-3 (1999-10-25) name for triple DES is triple data encryption algorithm so TDEA.
 * Also we only implement the three key mode  */

/** \def tdea_enc
 * \brief defining an alias for void tdes_enc(void* out, const void* in, const void* key)
 */

/** \def tdea_dec
 * \brief defining an alias for void tdes_dec(void* out, const void* in, const void* key)
 */
#define tdea_enc tdes_enc
#define tdea_dec tdes_dec

class DES
{
	public:
		/** \fn void decrypt(void* out, const void* in, const void* key)
		* \brief encrypt a block with DES
		* 
		* This function encrypts a block of 64 bits (8 bytes) with the DES algorithm.
		* Key expansion is done automatically. The key is 64 bits long, but note that
		* only 56 bits are used (the LSB of each byte is dropped). The input and output
		* blocks may overlap.
		* 
		* \param out pointer to the block (64 bit = 8 byte) where the ciphertext is written to
		* \param in  pointer to the block (64 bit = 8 byte) where the plaintext is read from
		* \param key pointer to the key (64 bit = 8 byte)
		*/
		void encrypt(void* out, const void* in, const void* key);
		
		/** \fn void encrypt(void* out, const void* in, const void* key)
		* \brief decrypt a block with DES
		* 
		* This function decrypts a block of 64 bits (8 bytes) with the DES algorithm.
		* Key expansion is done automatically. The key is 64 bits long, but note that
		* only 56 bits are used (the LSB of each byte is dropped). The input and output
		* blocks may overlap.
		* 
		* \param out pointer to the block (64 bit = 8 byte) where the plaintext is written to
		* \param in  pointer to the block (64 bit = 8 byte) where the ciphertext is read from
		* \param key pointer to the key (64 bit = 8 byte)
		*/
		void decrypt(void* out, const void* in, const uint8_t* key);
		
		/** \fn void tripleEncrypt(void* out, const void* in, const void* key)
		* \brief encrypt a block with Tripple-DES
		* 
		* This function encrypts a block of 64 bits (8 bytes) with the Tripple-DES (EDE)
		* algorithm. Key expansion is done automatically. The key is 192 bits long, but
		* note that only 178 bits are used (the LSB of each byte is dropped). The input
		* and output blocks may overlap.
		* 
		* \param out pointer to the block (64 bit = 8 byte) where the ciphertext is written to
		* \param in  pointer to the block (64 bit = 8 byte) where the plaintext is read from
		* \param key pointer to the key (192 bit = 24 byte)
		*/
		void tripleEncrypt(void* out, void* in, const void* key);

		/** \fn void tripleDecrypt(void* out, const void* in, const void* key)
		* \brief decrypt a block with Tripple-DES
		* 
		* This function decrypts a block of 64 bits (8 bytes) with the Tripple-DES (EDE)
		* algorithm. Key expansion is done automatically. The key is 192 bits long, but
		* note that only 178 bits are used (the LSB of each byte is dropped). The input
		* and output blocks may overlap.
		* 
		* \param out pointer to the block (64 bit = 8 byte) where the plaintext is written to
		* \param in  pointer to the block (64 bit = 8 byte) where the ciphertext is read from
		* \param key pointer to the key (192 bit = 24 byte)
		*/
		void tripleDecrypt(void* out, void* in, const uint8_t* key);
		
	private:
		void permute(const uint8_t *ptable, const uint8_t *in, uint8_t *out);
		void changeendian32(uint32_t * a);
		inline void shiftkey(uint8_t *key);
		inline void shiftkey_inv(uint8_t *key);
		inline uint64_t splitin6bitwords(uint64_t a);
		inline uint8_t substitute(uint8_t a, uint8_t * sbp);
		uint32_t des_f(uint32_t r, uint8_t* kr);
		
};

#endif