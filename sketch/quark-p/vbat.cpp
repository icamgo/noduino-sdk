/*
 *  Copyright (c) 2019 - 2029 MaiKe Labs
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
*/

#include "vbat.h"
 
float get_vbat(void)
{
	// Read 1.1V reference against AVcc
	// set the reference to Vcc and the measurement to the internal 1.1V reference
	if (ADMUX != ADMUX_VCCWRT1V1) {
		ADMUX = ADMUX_VCCWRT1V1;

		// Bandgap reference start-up time: max 70us
		// Wait for Vref to settle.
		delayMicroseconds(350); 
	}

	// Start conversion and wait for it to finish.
	ADCSRA |= _BV(ADSC);
	while (bit_is_set(ADCSRA,ADSC)) {};
	 
	// Result is now stored in ADC.

	// Calculate Vcc (in V)
	float vcc = 1.1*1024.0 / ADC;

	return vcc;
}
