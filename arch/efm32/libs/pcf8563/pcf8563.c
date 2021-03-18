/*
 *  Copyright (c) 2016 - 2026 MaiKe Labs
 *
 *  Library for PCF8563 RTC Chip
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
#include "pcf8563.h"

#define PCF8563_R					0xa3
#define PCF8563_W					0xa2
#define PCF8563_ADDR				(PCF8563_R >> 1)

/* register addresses in the rtc */
#define PCF8563_STAT1_ADDR			0x0
#define PCF8563_STAT2_ADDR			0x01
#define PCF8563_SEC_ADDR  			0x02
#define PCF8563_MIN_ADDR 			0x03
#define PCF8563_HR_ADDR 			0x04
#define PCF8563_DAY_ADDR 			0x05
#define PCF8563_WEEKDAY_ADDR		0x06
#define PCF8563_MONTH_ADDR			0x07
#define PCF8563_YEAR_ADDR 			0x08
#define PCF8563_ALRM_MIN_ADDR 	    0x09
#define PCF8563_SQW_ADDR 	        0x0D

#define SECONDS_PER_DAY				86400L
#define SECONDS_FROM_1970_TO_2000	946684800UL

static DateTime_t dt;

const uint8_t daysInMonth[] = {
	31, 28, 31, 30, 31, 30,
	31, 31, 30, 31, 30, 31
};

/* number of days since 2000/01/01, valid for 2001..2099 */
uint16_t date2days(uint16_t y, uint8_t m, uint8_t d)
{
	if (y >= 2000)
		y -= 2000;
	uint16_t days = d;
	uint8_t i;
	for (i = 1; i < m; ++i)
		days += daysInMonth[i - 1];
	if (m > 2 && y % 4 == 0)
		++days;
	return days + 365 * y + (y + 3) / 4 - 1;
}

static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s)
{
	return ((days * 24L + h) * 60 + m) * 60 + s;
}

static uint8_t conv2d(const char *p)
{
	uint8_t v = 0;
	if ('0' <= *p && *p <= '9')
		v = *p - '0';
	return 10 * v + *++p - '0';
}

static void set_dt_from_int (uint16_t y, uint8_t m, uint8_t d,
		uint8_t hh, uint8_t mm, uint8_t ss)
{
	if (y >= 2000)
		y -= 2000;
	dt.yOff = y;
	dt.m = m;
	dt.d = d;
	dt.hh = hh;
	dt.mm = mm;
	dt.ss = ss;
}

/*
 * DateTime implementation - ignores time zones and DST changes
 * NOTE: also ignores leap seconds, see http://en.wikipedia.org/wiki/Leap_second
 */
static void set_dt_from_seconds(uint32_t t)
{
	t -= SECONDS_FROM_1970_TO_2000;	// bring to 2000 timestamp from 1970

	dt.ss = t % 60;
	t /= 60;
	dt.mm = t % 60;
	t /= 60;
	dt.hh = t % 24;

	uint16_t days = t / 24;
	uint8_t leap;

	for (dt.yOff = 0; ; ++(dt.yOff)) {
		leap = dt.yOff % 4 == 0;
		if (days < 365 + leap)
			break;
		days -= 365 + leap;
	}

	for (dt.m = 1;; ++(dt.m)) {
		uint8_t daysPerMonth = daysInMonth[dt.m - 1];
		if (leap && dt.m == 2)
			++daysPerMonth;
		if (days < daysPerMonth)
			break;
		days -= daysPerMonth;
	}
	dt.d = days + 1;
}

/* sample input: date = "Dec 26 2009", time = "12:34:56" */
static void set_dt_from_str(const char *date, const char *time)
{
	dt.yOff = conv2d(date + 9);

	/* Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec */
	switch (date[0]) {
	case 'J':
		dt.m = date[1] == 'a' ? 1 : (dt.m = date[2] == 'n' ? 6 : 7);
		break;
	case 'F':
		dt.m = 2;
		break;
	case 'A':
		dt.m = date[2] == 'r' ? 4 : 8;
		break;
	case 'M':
		dt.m = date[2] == 'r' ? 3 : 5;
		break;
	case 'S':
		dt.m = 9;
		break;
	case 'O':
		dt.m = 10;
		break;
	case 'N':
		dt.m = 11;
		break;
	case 'D':
		dt.m = 12;
		break;
	}
	dt.d = conv2d(date + 4);
	dt.hh = conv2d(time);
	dt.mm = conv2d(time + 3);
	dt.ss = conv2d(time + 6);
}

uint8_t day_of_week(const DateTime_t *pdt)
{
	uint16_t day = date2days((uint16_t)pdt->yOff, pdt->m, pdt->d);

	/* Jan 1, 2000 is a Saturday, i.e. returns 6 */
	return (day + 6) % 7;
}

uint32_t unixtime(const DateTime_t *pdt)
{
	uint32_t t;
	uint16_t days = date2days((uint16_t)pdt->yOff, pdt->m, pdt->d);
	t = time2long(days, pdt->hh, pdt->mm, pdt->ss);

	/* seconds from 1970 to 2000 */
	t += SECONDS_FROM_1970_TO_2000;

	return t;
}

static uint8_t bcd2bin(uint8_t val)
{
	return val - 6 * (val >> 4);
}

static uint8_t bin2bcd(uint8_t val)
{
	return val + 6 * (val / 10);
}

void pcf8563_init(uint8_t scl, uint8_t sda)
{
	wire_begin(scl, sda);
}

void pcf8563_set(const DateTime_t *pdt)
{
	if(pdt == NULL)
		pdt = &dt;
	wire_beginTransmission(PCF8563_ADDR);
	wire_write(PCF8563_SEC_ADDR);
	wire_write(bin2bcd(pdt->ss));
	wire_write(bin2bcd(pdt->mm));
	wire_write(bin2bcd(pdt->hh));
	wire_write(bin2bcd(pdt->d));
	wire_write(bin2bcd(day_of_week(pdt)));
	wire_write(bin2bcd(pdt->m));
	wire_write(bin2bcd(pdt->yOff >= 2000 ? pdt->yOff - 2000 : pdt->yOff));
	wire_endTransmission();
}

void pcf8563_set_from_int(uint16_t y, uint8_t m, uint8_t d,
							uint8_t hh, uint8_t mm, uint8_t ss)
{
	set_dt_from_int(y, m, d, hh, mm, ss);
	pcf8563_set(NULL);
}

/* sample input: date = "Dec 26 2009", time = "12:34:56" */
void pcf8563_set_from_str(const char *date, const char *time)
{
	set_dt_from_str(date, time);
	pcf8563_set(NULL);
}

void pcf8563_set_from_seconds(uint32_t t)
{
	set_dt_from_seconds(t);
	pcf8563_set(NULL);
}

void pcf8563_print()
{
	char msg[40];
	sprintf(msg, "PCF8563 Time: %04d-%02d-%02d %02d:%02d:%02d",
			dt.yOff+2000, dt.m, dt.d, dt.hh, dt.mm, dt.ss);

	Serial.println(msg);
}

uint32_t pcf8563_now()
{
	wire_beginTransmission(PCF8563_ADDR);
	wire_write(PCF8563_SEC_ADDR);
	wire_endTransmission();

	wire_requestFrom(PCF8563_ADDR, 7);
	uint8_t ss = bcd2bin(wire_read() & 0x7F);
	uint8_t mm = bcd2bin(wire_read() & 0x7f);
	uint8_t hh = bcd2bin(wire_read() & 0x3f);
	uint8_t d = bcd2bin(wire_read() & 0x3f);
	wire_read();
	uint8_t m = bcd2bin(wire_read() & 0x1f);
	uint16_t y = bcd2bin(wire_read()) + 2000;

	set_dt_from_int(y, m, d, hh, mm, ss);
	return unixtime(&dt);
}
