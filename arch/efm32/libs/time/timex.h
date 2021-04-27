/*
 @file lib/time.h

 @brief Common Linux/POSIX time functions

 @par Copyright &copy; 2015 Mike Gore, GPL License

 @par You are free to use this code under the terms of GPL
   please retain a copy of this notice in any code you use it in.

This is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option)
any later version.

This software is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _TIME_H_
#define _TIME_H_

#include "string.h"
#include "stdint.h"

struct tm {
	int tm_sec;		/*<  Seconds.     [0-60] (1 leap second) */
	int tm_min;		/*<  Minutes.     [0-59] */
	int tm_hour;		/*<  Hours.       [0-23] */
	int tm_mday;		/*<  Day.         [1-31] */
	int tm_mon;		/*<  Month.       [0-11] */
	int tm_year;		/*<  Year - 1900. */
	int tm_wday;		/*<  Day of week. [0-6] */
	int tm_yday;		/*<  Days in year.[0-365] */
	int tm_isdst;		/*<  DST.         [-1/0/1] */
	int32_t tm_gmtoff;	/*<  GMT offset in seconds [+/- 86400] */
};
typedef struct tm tm_t;

char *tm_wday_to_ascii(int i);
char *tm_mon_to_ascii(int i);

tm_t *localtime(uint32_t * tp);
tm_t *gmtime(uint32_t * tp);

char *ctime_r(uint32_t * t, char *buf);
char *ctime(uint32_t * tp);
char *ctime_gm(uint32_t * tp);
uint32_t time(uint32_t * t);
void setclock(uint32_t seconds, uint32_t us);

uint32_t str2seconds(char *buf);

#endif
