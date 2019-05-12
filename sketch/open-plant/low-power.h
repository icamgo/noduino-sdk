/*
 *  Copyright (c) 2019 - 2025 MaiKe Labs
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
#ifndef __LOW_POWER_H__
#define __LOW_POWER_H__

#define	INIT_MAGIC			0x7E7E55AA

#define	MQTT_RATE			1			// 1s
#define	HTTP_RATE			60	 		// 60s
#define	SLEEP_TIME			30000000	// 30s
#define	MAX_DP_NUM			5
#define RTC_MEM_START		(64+6)		// (256 ~ 256+24, 64~64+6) used by rtc time

#define	set_deepsleep_wakeup_no_rf()	system_deep_sleep_set_option(4)
#define	set_deepsleep_wakeup_normal()	system_deep_sleep_set_option(1)

struct datapoint {
	float vbat;
	float temp;
	float humi;
	uint32_t light;
#ifdef CONFIG_CO2_SENSOR
	float co2;
#endif
#ifdef CONFIG_PRESSURE_SENSOR
	float pressure;
#endif
	uint32_t timestamp;
} __attribute__((aligned(4), packed));

struct hotbuf {
	uint32_t bootflag;
	uint16_t realtime;		/* 1: mqtt enable, 0: mqtt disable */
	uint8_t airkiss_nff_on;

	uint8_t cnt;			/* max = (512-24-8) / sizeof(data_point) */
	struct datapoint datapoints[MAX_DP_NUM];
} __attribute__((aligned(4), packed));

#endif
