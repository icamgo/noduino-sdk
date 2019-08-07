/*
 *  Copyright (c) 2017 - 2025 MaiKe Labs
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
#ifndef __RADIO_H__
#define __RADIO_H__

#include "Arduino.h"
#include "SX1272.h"

#define MAX_DBM				20
#define TXRX_CH				CH_00_433		// 433.0MHz
//#define TXRX_CH				CH_00_470		// 470.0MHz

#define LORA_ADDR		1
#define LORA_MODE		11	//Default LoRa mode BW=125KHz, CR=4/5, SF=12

//#define GW_RELAY
//#define RECEIVE_ALL 

#define INFO_S(fmt,param)			Serial.print(F(param))
#define INFO_HEX(fmt,param)			Serial.print(param,HEX)
#define INFO(fmt,param)				Serial.print(param)
#define INFOLN(fmt,param)			Serial.println(param)
#define FLUSHOUTPUT					Serial.flush();

void radio_setup();
int radio_available(char *cmd);
#endif
