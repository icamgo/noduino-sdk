/**
 @file lib/time.c

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

#include "timex.h"

typedef uint16_t clockid_t;

struct timezone {
	int tz_minuteswest;	/*< minutes west of Greenwich */
	int tz_dsttime;		/*< type of DST correction */
};
typedef struct timezone tz_t;

struct timeval {
	uint32_t tv_sec;	/*< seconds */
	uint32_t tv_usec;	/*< microseconds */
};

struct timespec {
	uint32_t tv_sec;	/*< seconds */
	long tv_nsec;		/*< nanoseconds */
};

typedef struct timeval tv_t;
typedef struct timespec ts_t;

extern tz_t __tzone;

#define EPOCH_YEAR       1970	/*< Thursday Jan 1 1970 */
#define EPOCH_WDAY       4	/*< Jan 1, 1970 was thursday, Sunday = 0 ... Saturday = 6 */
#define EPOCH            0	/*< Zero seconds */
#define EPOCH_2000       946684800	/*< Sat, 01 Jan 2000 00:00:00 GMT */

#define TM_YEAR_BASE     1900	/*< TM year base is 1900 */
#define FIRST_GOOD_YEAR  ((uint32_t) -1 < (uint32_t) 1 ? EPOCH_YEAR-68 : EPOCH_YEAR)
#define LAST_GOOD_YEAR   (EPOCH_YEAR + ((uint32_t) -1 < (uint32_t) 1 ? 67 : 135))

#define YDAYS(month, year) yr_days[leap(year)][month]

/* Nonzero if `y' is a leap year, else zero. */
#define leap(y)  (((y) % 4 == 0 && (y) % 100 != 0) || (y) % 400 == 0)

/* Number of leap years from EPOCH_YEAR  to `y' (not including `y' itself). */
#define _P4      ((EPOCH_YEAR / 4) * 4 + 1)
#define _P100    ((EPOCH_YEAR / 100) * 100 + 1)
#define _P400    ((EPOCH_YEAR / 400) * 400 + 1)
#define nleap(y) (((y) - _P4) / 4 - ((y) - _P100) / 100 + ((y) - _P400) / 400)

/* Length of month `m' (0 .. 11) */
#define monthlen(m, y) (yr_days[0][(m)+1] - yr_days[0][m] + \
                        ((m) == 1 && leap(y)))

/// @brief  System Time Zone
tz_t __tzone;

///@brief accumulated days to the start of a month in a year.
///
/// - without Leap days.
/// - Index: Month 00 .. 11  (12 is a year).
/// @see timegm().
static const uint16_t __days_sum[] = {
	0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365
};

///@brief days in each month.
///  - without leap days.
///  - Index: Month 00 .. 11.
/// @see timegm().
static const uint16_t __days[] = {
	31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

///@brief Short Name of each Day in a week.
///
/// - Day 0 .. 6 to string.
///
///@see asctime_r()
const char *__WDay[] =
    { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "BAD" };

/// @brief Convert POSIX epoch uint32_t *tp into POSIX tm_t *result.
///
/// @param[in] t: uint32_t * epoch time input.
/// @param[out] result: tm_t *result.
///
/// @return result.
tm_t *localtime_r(uint32_t * t, tm_t * result)
{
	tz_t tz;
	long int offset;
	uint32_t epoch = *t;

	gettimezone(&tz);
	offset = 60L * tz.tz_minuteswest;

	uint32_to_tm(epoch, offset, result);

	return (result);
}

/// @brief Convert POSIX epoch uint32_t *tp into POSIX tm_t *result.
///
/// @param[in] tp: uint32_t * epoch time input.
///
/// @return struct tm result.
/// @warning result is overwritten on each call.
tm_t *localtime(uint32_t * tp)
{
	static struct tm t;
	return (localtime_r(tp, &t));
}

/// @brief Get string Short name of day from day number.
///
///@param[in] i: Day 0 .. 6.
///
/// @return string pointer to day.
/// @return "BAD" on error.
char *tm_wday_to_ascii(int i)
{
	if (i >= 0 && i <= 6)
		return ((char *)__WDay[i]);
	else
		return ((char *)__WDay[7]);
}

/// @brief Short Name or each Month in a year.
///
/// - Month 0 .. 11 to string.
///
const char *__Month[] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul",
	"Aug", "Sep", "Oct", "Nov", "Dec", "BAD"
};

/// @brief Get string Short name of Month from month number.
///
/// @param[in] i: Month 0 .. 11 to string.
///
/// @return string pointer to month.
/// @return "BAD" on error.
/// @see asctime_r()
char *tm_mon_to_ascii(int i)
{
	if (i >= 0 && i <= 11)
		return ((char *)__Month[i]);
	else
		return ((char *)__Month[12]);
}

/// @brief Check if a year is a leap year.
///
/// @param[in] year, no limit.
///
/// @warning:   No limit checking.
///
/// @return     none zero (leap year), 0 (not a leap year)
/// @see uint32_to_tm() 
/// @see ctime_gm() 
/// @see gmtime_r() 
/// @see localtime_r()
static int IS_Leap(int year)
{
/// @return return

	// if(year & 3)
	//     return(0);

	// if(year == 1900 || year == 2100)
	//     return(0);

	// return(1);
	return leap(year);
}

/// @brief  Number of leap days since 1900 to the BEGINNING of the year.
///
/// @param[in] year: valid over 1900..2199.
///
/// @warning No limit checking
///
/// @return days.
///
/// @see uint32_to_tm() 
/// @see ctime_gm() 
/// @see gmtime_r() 
/// @see localtime_r()
static int Leap_Days_Since_1900(int year)
{
	// int sum;

	// year -= 1900;

	// if(year>0)
	//     --year;

	// sum = (year >> 2);

	// if(year >= 200 )
	//     --sum;

	// return(sum);
	return (nleap(year) + 17);
}

/// @brief  Find number of days in a given year.
///
/// @param[in] year: valid over 1900..2199.
///
/// @warning No limit checking
///
/// @return days in a given year.
///
/// @see uint32_to_tm() 
/// @see ctime_gm() 
/// @see gmtime_r() 
/// @see localtime_r()
static int Days_Per_Year(int year)
{
	return (IS_Leap(year) ? 366 : 365);
}

/// @brief  Converts epoch ( seconds from 1 Jan EPOCH_YEAR UTC), offset seconds, to UNIX tm *t.
///  @param[in] epoch:  Seconds elapsed since January 1, EPOCH_YEAR.
///      - unsigned long,       range limited to: 0 .. 0xFFFFFFFFFFFD5D00>
///  - The range 0xFFFFFFFFFFFEAE80 .. 0xFFFFFFFFFFFFFFFF is reserverd for Dec 31, 1969.
///  - The range 0xFFFFFFFFFFFD5D00 .. 0xFFFFFFFFFFFEAE7F is reserverd of offset overflow.
///  @param[in] offset:         Offset in seconds to localtime.
///  - long int, range limited to +/- 86400.
///  - (Number of seconds that we add to UTC to get local time).
///  @param[out] t: Unix tm struct pointer output.

/// @return 0: *t has result.
/// @return -1: error.
///
/// @see ctime_gm() 
/// @see gmtime_r() 
/// @see localtime_r()

int uint32_to_tm(uint32_t epoch, int32_t offset, tm_t * t)
{
	int year, month, tmp;
	int flag = 0;
	int32_t days;

	memset(t, 0, sizeof(tm_t));

	if (epoch >= 0xFFFFFFFFFFFD5D00ULL)
		return (-1);

	t->tm_gmtoff = offset;
	epoch -= offset;

	if (epoch >= 0xFFFFFFFFFFFEAE80ULL) {
		epoch -= 0xFFFFFFFFFFFEAE80ULL;
		flag = 1;
	}

	t->tm_sec = epoch % 60;
	epoch /= 60;

	t->tm_min = epoch % 60;
	epoch /= 60;

	t->tm_hour = epoch % 24;
	epoch /= 24;

	days = epoch;

	if (flag) {
		t->tm_year = 69;
		t->tm_mon = 11;
		t->tm_mday = 31;
		t->tm_wday = (EPOCH_WDAY - 1) % 7;
	} else {
		t->tm_wday = (EPOCH_WDAY + days) % 7;

		year = EPOCH_YEAR;
		while (days >= (tmp = Days_Per_Year(year))) {
			++year;
			days -= tmp;
		}

		t->tm_year = year - 1900;
		t->tm_yday = days;

		month = 0;

		while (days > 0 && month < 12) {
			tmp = __days[month];
			if (month == 1 && IS_Leap(year))
				++tmp;
			if (days < tmp)
				break;
			days -= tmp;
			++month;
		}

		t->tm_mon = month;
		t->tm_mday = days + 1;
	}
	return (0);
}

/// @brief Convert tm_t structure as GMT time into seconds since 1900.
///
/// - Standards: GNU and BSD.
/// - Limits:   year(1900..2199).
/// - Assume:  epoch size is uint32_t uint32_t;
///
/// @see mktime() POSIX function.
/// @see timegm() POSIX function.
/// @return Seconds since EPOCH_YEAR Jan 1st.
/// @return -1 on error.
uint32_t timegm(tm_t * t)
{
	uint32_t days, seconds;

	int year = t->tm_year + 1900;
	int mon = t->tm_mon;	// 0..11
	int mday = t->tm_mday;	// 1..28,29,30,31
	int hour = t->tm_hour;	// 0..23
	int min = t->tm_min;	// 0..59
	int sec = t->tm_sec;	// 0..59

	if (year < EPOCH_YEAR || year > 2106)
		return (-1);

	if (mon > 12 || mday > 31 || hour > 23 || min > 59 || sec > 59)
		return (-1);

	if (mon < 0 || mday < 0 || hour < 0 || min < 0 || sec < 0)
		return (-1);

	--mday;			// remove offset

///  Note: To simplify we caculate Leap Day contributions in stages

	days = (year - EPOCH_YEAR);

	days *= (uint32_t) 365L;

	days += (uint32_t) __days_sum[mon];

	days += (uint32_t) mday;

	days += (uint32_t) Leap_Days_Since_1900(year);

	days -= (uint32_t) 17;

	if (mon > 1 && IS_Leap(year))
		++days;

	seconds = days;

	seconds *= (uint32_t) 24L;	// hours
	seconds += (uint32_t) hour;
	seconds *= (uint32_t) 60L;	// Minutes
	seconds += (uint32_t) min;
	seconds *= (uint32_t) 60L;	// Seconds
	seconds += (uint32_t) sec;
	return (seconds);
}

/// @brief Convert epoch GMT uint32_t *tp into POSIX tm_t *result.
///
/// @param[in] tp: uint32_t * time input.
/// @param[out] result: tm_t *result.
///
/// @return tm_t *result.
tm_t *gmtime_r(uint32_t * tp, tm_t * result)
{
	uint32_t epoch = *tp;
	uint32_to_tm(epoch, 0L, result);
	return (result);
}

/// @brief Convert epoch GMT uint32_t *tp into POSIX static tm_t *t.
///
/// @param[in] tp: uint32_t * time input.
///
/// @return tm_t t.
/// @warning result is overwritten on each call.
tm_t *gmtime(uint32_t * tp)
{
	static tm_t t;
	gmtime_r(tp, &t);
	return (&t);
}

#if 0

/// @todo  implement strftime() and strptime()

/// @brief Convert tm_t *t structure into POSIX asctime() ASCII string *buf.
///
/// @param[in] t: tm_t structure pointer.
/// @param[out] buf: user buffer for POSIX asctime() string result.
/// - Example output: "Thu Dec  8 21:45:05 EST 2011".
///
/// @return buf string pointer.
char *asctime_r(tm_t * t, char *buf)
{
	snprintf(buf, 32, "%s %s %2d %02d:%02d:%02d %4d",
		 __WDay[t->tm_wday],
		 __Month[t->tm_mon],
		 t->tm_mday,
		 t->tm_hour, t->tm_min, t->tm_sec, t->tm_year + 1900);
	return (buf);
}

/// @brief String storage for asctime().
static char __ctime_buf[32];

/// @brief Convert tm_t *t structure into POSIX asctime() ASCII string.
///
/// @param[in] t: struct tm * time input.
///
/// @return __ctime_buf[] string pointer in POSIX asctime() format.
/// - Example output: "Thu Dec  8 21:45:05 EST 2011".
/// @warning result is overwritten on each call.
char *asctime(tm_t * t)
{
	return (asctime_r(t, __ctime_buf));
}

/// @brief Convert local uint32_t *t epoch time into POSIX asctime() ASCII string *buf.
///
/// @param[in] t: uint32_t * time input.
/// @param[out] buf: string output.
///  - Example output: "Thu Dec  8 21:45:05 EST 2011"
///
/// @return  buf string pointer.
char *ctime_r(uint32_t * t, char *buf)
{
	return (asctime_r(localtime(t), buf));
}

/// @brief Convert local uint32_t *t epoch time into POSIX asctime() string __ctime_buf[]
///
/// @param[in] tp: uint32_t * time input.
///
/// @return  __ctime_buf[].
///  - Example: "Thu Dec  8 21:45:05 EST 2011".
char *ctime(uint32_t * tp)
{
	return (asctime_r(localtime(tp), __ctime_buf));
}

/// @brief GMT version of POSIX ctime().
///
/// @param[in] tp: uint32_t * time input.
///
/// @return  __ctime_buf[].
///  - Example: "Thu Dec  8 21:45:05 EST 2011".
/// @see ctime()
/// @warning result is overwritten on each call.
char *ctime_gm(uint32_t * tp)
{
	uint32_t epoch = *tp;
	tm_t tm;
	uint32_to_tm(epoch, 0L, &tm);
	return (asctime_r(&tm, __ctime_buf));
}

/// @brief Convert POSIX tm_t *t struct into POSIX epoch time in seconds with UTC offset adjustment.
///
/// @param[in] t: uint32_t * epoch time input.
///
// @return uint32_t epoch time.
uint32_t mktime(tm_t * t)
{

	uint32_t val;
	long int offset;
	tz_t tz;

	gettimezone(&tz);
	offset = 60L * tz.tz_minuteswest;

	val = timegm(t);
	val = val - offset;

	return (val);
}

/// @brief Get current timezone in struct timezone *tz - POSIX function.
///
/// @param[out] tz: timezone result.
///
/// @return  0
int gettimezone(tz_t * tz)
{
	tz->tz_minuteswest = __tzone.tz_minuteswest;
	tz->tz_dsttime = __tzone.tz_dsttime;
	return (0);
}

/// @brief Set current timezone with struct timezone *tz - POSIX function.
///
/// @param[in] tz: timezone result.
///
/// @return 0.
int settimezone(tz_t * tz)
{
	__tzone.tz_minuteswest = tz->tz_minuteswest;
	__tzone.tz_dsttime = tz->tz_dsttime;
	return (0);
}

/// @brief Get current time struct timeval *tv and struct timezone *tz - POSIX function.
///
/// @param[in] tv: time.
/// @param[in] tz: timezone.
///
/// @return  0
int gettimeofday(tv_t * tv, tz_t * tz)
{

	ts_t ts;

	clock_gettime(0, (ts_t *) & ts);

	tv->tv_sec = ts.tv_sec;
	tv->tv_usec = ts.tv_nsec / 1000L;

	gettimezone(tz);
	return (0);
}

/// @brief Return second from epoch - POSIX function.
///
///  @param[in,out] t: pointer to store time in.
///   - Notes:  If t is non-NULL, store the return value there also.
/// @return uint32_t seconds from epoch.
/// @see clock_gettime().
uint32_t time(uint32_t * t)
{
	ts_t ts;
	clock_gettime(0, (ts_t *) & ts);
	if (t != NULL)
		*t = ts.tv_sec;
	return (ts.tv_sec);
}

/// @brief Set current time struct timeval *tv and struct timezone *tz - POSIX function.
///
/// @param[in] tv: time.
/// @param[in] tz: timezone.
///
/// @return  0
int settimeofday(tv_t * tv, tz_t * tz)
{
	ts_t ts;

	ts.tv_sec = tv->tv_sec;
	ts.tv_nsec = tv->tv_usec * 1000L;

	clock_settime(0, (ts_t *) & ts);

	settimezone(tz);

	return (0);
}
#endif

/* return the UTC seconds */
uint32_t str2seconds(char *buf)
{
	/* +CCLK: 21/02/26,06:22:38+32 */

	tm_t tm;
	int tz_h = 0;

	tm.tm_year = 0;
	tm.tm_mon = 0;
	tm.tm_mday = 0;
	tm.tm_hour = 0;
	tm.tm_min = 0;
	tm.tm_sec = 0;

	sscanf(buf, "%d/%d/%d,%d:%d:%d+%d",
	       &tm.tm_year,
	       &tm.tm_mon,
		   &tm.tm_mday,
		   &tm.tm_hour,
		   &tm.tm_min,
		   &tm.tm_sec,
		   &tz_h);

	tm.tm_mon--;
	tm.tm_year += 2000;
	__tzone.tz_minuteswest = (tz_h % 24) * 60;

	if (tm.tm_year < 1970 || tm.tm_year > 2038) {
		return (-1);
	}
	if (tm.tm_year >= 1900)
		tm.tm_year -= 1900;
	if (tm.tm_mon < 0 || tm.tm_mon > 11) {
		return (-1);
	}
	if (tm.tm_mday < 1 || tm.tm_mday > 31) {
		return (-1);
	}
	if (tm.tm_hour < 0 || tm.tm_hour > 23) {
		return (-1);
	}
	if (tm.tm_min < 0 || tm.tm_min > 59) {
		return (-1);
	}

	return timegm(&tm);
}
