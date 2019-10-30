/*
  base.h - Type definitions for Direct IO and other libraries.
  Copyright (c) 2015 Michael Marchetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _DirectIO_Base_
#define _DirectIO_Base_

#ifdef __cplusplus

// bits_type(N) gives the smallest type that will hold N bits (0 <= N <= 32)
#define bits_type(N) typename _nbits_t<N>::bits_t

// A little ugly infrastructure to make it work
template <int N> class _nbits_t {};

#define _nbits(N, T) template<> class _nbits_t<N> { public: typedef T bits_t; };

_nbits(0, uint8_t);
_nbits(1, uint8_t);
_nbits(2, uint8_t);
_nbits(3, uint8_t);
_nbits(4, uint8_t);
_nbits(5, uint8_t);
_nbits(6, uint8_t);
_nbits(7, uint8_t);
_nbits(8, uint8_t);
_nbits(9, uint16_t);
_nbits(10, uint16_t);
_nbits(11, uint16_t);
_nbits(12, uint16_t);
_nbits(13, uint16_t);
_nbits(14, uint16_t);
_nbits(15, uint16_t);
_nbits(16, uint16_t);
_nbits(17, uint32_t);
_nbits(18, uint32_t);
_nbits(19, uint32_t);
_nbits(20, uint32_t);
_nbits(21, uint32_t);
_nbits(22, uint32_t);
_nbits(23, uint32_t);
_nbits(24, uint32_t);
_nbits(25, uint32_t);
_nbits(26, uint32_t);
_nbits(27, uint32_t);
_nbits(28, uint32_t);
_nbits(29, uint32_t);
_nbits(30, uint32_t);
_nbits(31, uint32_t);
_nbits(32, uint32_t);

#undef _nbits

#endif
#endif // _DirectIO_Base_
