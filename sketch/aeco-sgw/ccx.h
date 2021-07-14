#include "rtcdriver.h"

bool is_my_did(uint8_t *p);

bool check_crc(uint8_t *p, int plen)
{
	int i, len = 0;
	uint16_t hh = 0, sum = 0;

	len = plen - 6;
	sum = p[len] << 8 | p[len+1];

	for (i = 0; i < len; i++) {
		hh += p[i];
	}

	if (hh == sum)
		return true;
	else
		return false;
}

// check the devid not in interrupt contex
bool is_our_did(uint8_t *p)
{
	if (p[3] != 0 || p[4] != 0 || p[5] != 0 || p[6] > 0x17) {

		// invalid devid. max_id = 0x17 ff ff ff ff (1_030.79.21.5103)
		#ifdef DEBUG_DEVID
		INFOLN("ivd0");
		#endif
		return false;
	}

	uint64_t devid = 0UL;
	uint8_t *pd = (uint8_t *) &devid;

	for(int i = 0; i < 8; i++) {
		pd[7-i] = p[3+i];
	}

	if (99999999999ULL == devid) {
		#ifdef DEBUG_DEVID
		INFOLN("999 ok");
		#endif
		return true;
	}

	if (devid > 99999999999ULL) {
		#ifdef DEBUG_DEVID
		INFOLN("ivd3");
		#endif
		return false;
	}

	uint32_t temp = devid / 10000;
	uint32_t tt = (uint32_t)(temp / 100);
	uint32_t wk_yr = (uint32_t)(temp - tt * 100);

	if (wk_yr > 52) {

		#ifdef DEBUG_DEVID
		INFOLN("ivd1");
		#endif

		return false;
	}

    temp = devid / 100000000;
    tt = (uint32_t)(temp / 100);
	wk_yr = (uint32_t)(temp - tt * 100);

	if (wk_yr > 30 || wk_yr < 18) {

		#ifdef DEBUG_DEVID
		INFOLN("ivd2");
		#endif
		return false;
	}

	#ifdef DEBUG_DEVID
	INFOLN("did ok");
	#endif
	return true;
}

bool is_our_pkt(uint8_t *p, int len)
{

	/*
	 * 31: 17
	 * 32: 19
	 * 33: 20
	*/
	if (len < 23) return false;

	if (p[0] != 0x47 || p[1] != 0x4F) {

		// invalide pkt header
		return false;
	}

	// check the devid quickly in interrupt contex
	if (p[3] != 0 || p[4] != 0 || p[5] != 0 || p[6] > 0x17) {

		// invalid devid. max_id = 0x17 ff ff ff ff (1_030.79.21.5103)
		return false;
	}

	if (p[2] < 0x32 || p[2] > 0x36) {

		// support only the 0x33/34/35/36 version
		return false;
	}

	if (check_crc(p, len) == false) {

		return false;
	}

	return true;
}

////////////////////////////////////////////////////////////////////////////
char devid_buf[24];
char my_devid[24];
char dev_vbat[6];
char dev_data[48];

char *uint64_to_str(char *dest, uint64_t n)
{
	/*
	 * >>> 0xffffffffffffffff
	 * 18446744073709551615
	*/
	dest += 20;
	*dest-- = 0;
	while (n) {
		*dest-- = (n % 10) + '0';
		n /= 10;
	}
	return dest + 1;
}

char *decode_devid(uint8_t *pkt, char *didbuf)
{
	int a = 0, b = 0;

	uint64_t did = 0UL;

	for (a = 3; a < 11; a++, b++) {

		*(((uint8_t *)&did) + 7 - b) = pkt[a];
	}

	return uint64_to_str(didbuf, did);
}

/* only support .0001 */
char *ftoa(char *a, float f, int preci)
{
	long p[] =
	    {0, 10, 100, 1000, 10000};

	char *ret = a;

	long ipart = (long)f;

	//INFOLN("%d", ipart);

	itoa(ipart, a, 10);		//int16, -32,768 ~ 32,767

	while (*a != '\0')
		a++;

	*a++ = '.';

	long fpart = abs(f * p[preci] - ipart * p[preci]);

	//INFOLN("%d", fpart);

	if (fpart > 0) {
		if (fpart < p[preci]/10) {
			*a++ = '0';
		}
		if (fpart < p[preci]/100) {
			*a++ = '0';
		}
		if (fpart < p[preci]/1000) {
			*a++ = '0';
		}
	}

	itoa(fpart, a, 10);
	return ret;
}
uint8_t decode_cmd(uint8_t *pkt)
{
	uint8_t cmd = 0;

	switch(pkt[2]) {

		case 0x33:
		case 0x34:
			cmd = pkt[15];
			break;
		default:
			cmd = 255;
			break;
	}

	return cmd;
}

char *decode_vbat(uint8_t *pkt)
{
	uint16_t vbat = 0;

	switch(pkt[2]) {
		case 0x31:
		case 0x32:
		case 0x33:
		case 0x34:
			vbat = pkt[13] << 8 | pkt[14];
			break;
	}

	ftoa(dev_vbat, (float)(vbat / 1000.0), 3);
	return dev_vbat;
}

uint32_t get_dev_type(uint8_t *p)
{
	uint64_t devid = 0UL;
	uint8_t *pd = (uint8_t *) &devid;

	for(int i = 0; i < 8; i++) {
		pd[7-i] = p[3+i];
	}

	uint64_t temp = devid / 1000000;
	uint32_t tt = (uint32_t)(temp / 100);
	return (uint32_t)(temp - tt * 100);
}

char *decode_sensor_data(uint8_t *pkt)
{
	uint64_t ddid = 0ULL;
	int16_t data = 0;
	float dd  = 0;
	char data_buf[8] = {0};
	uint32_t ttss = 0;

	data = (pkt[11]  << 8) | pkt[12];

	uint32_t dt = get_dev_type(pkt);

/*
	switch(dt) {

		case 0:
		case 2:
		case 6:
		case 20:
		case 21:
			// Temperature: 00, 02, 04
			dd = (float)(data / 10.0);
			ftoa(data_buf, dd, 1);
			sprintf(dev_data, "T\":%s", data_buf);
			break;

		case 1:
		case 3:
			// Pressure sensor: 01, 03
			dd = (float)(data / 100.0);
			ftoa(data_buf, dd, 2);
			sprintf(dev_data, "P\":%s`\"iT\":%d", data_buf, (int8_t)(pkt[21]));
			break;
		case 5:
			// ET-Pump
			sprintf(dev_data, "X\":0x%08X", data);
			break;
		case 7:
			// water level (pressure), unit is 'M'
			dd = (float)(data / 100.0);
			ftoa(data_buf, dd, 2);
			sprintf(dev_data, "L\":%s`\"iT\":%d", data_buf, (int8_t)(pkt[21]));
			break;
		case 8:
			// Humi&Temp Sensor
			sprintf(dev_data, "H\":%d`\"T\":%d`\"iT\":%d", (int)(data/10.0+0.5), (int8_t)(pkt[20]), (int8_t)(pkt[21]));
			break;
		case 9:
			// Moving Sensor, unit is 'mm'
			sprintf(dev_data, "M\":%d", data);
			break;
		case 12:
			// Vibration Sensor
			sprintf(dev_data, "V\":%d", data);
			break;
		case 13:
			// Float & Temp Sensor
			dd = (float)(data / 10.0);
			ftoa(data_buf, dd, 1);
			sprintf(dev_data, "T\":%s`\"L\":%d`\"iT\":%d", data_buf, (int8_t)(pkt[20]), (int8_t)(pkt[21]));
			break;
		case 14:
			// Water Leak Sensor
			dd = (float)(data / 10.0);
			ftoa(data_buf, dd, 1);
			sprintf(dev_data, "L\":%s", data_buf);
			break;
		case 16:
			// GOTh with oled, Temp&Humi
			dd = (float)(data / 10.0);
			ftoa(data_buf, dd, 1);
			sprintf(dev_data, "T\":%s`\"H\":%d`\"iT\":%d", data_buf, (int8_t)(pkt[20]), (int8_t)(pkt[21]));
			break;
	}
*/


	if (0 == dt || 2 == dt || 6 == dt || 20 == dt || 21 == dt) {
		// Temperature: 00, 02, 04
		dd = (float)(data / 10.0);
		ftoa(data_buf, dd, 1);
		sprintf(dev_data, "T\":%s", data_buf);
	} else if (1 == dt || 3 == dt) {
		// Pressure sensor: 01, 03
		dd = (float)(data / 100.0);
		ftoa(data_buf, dd, 2);
		sprintf(dev_data, "P\":%s`\"iT\":%d", data_buf, (int8_t)(pkt[21]));
	} else if (5 == dt) {
		// ET-Pump
		sprintf(dev_data, "X\":0x%08X", data);
	} else if (7 == dt) {
		// water level (pressure), unit is 'M'
		dd = (float)(data / 100.0);
		ftoa(data_buf, dd, 2);
		sprintf(dev_data, "L\":%s`\"iT\":%d", data_buf, (int8_t)(pkt[21]));
	} else if (8 == dt) {
		// Humi&Temp Sensor
		sprintf(dev_data, "H\":%d`\"T\":%d`\"iT\":%d", (int)(data/10.0+0.5), (int8_t)(pkt[20]), (int8_t)(pkt[21]));
	} else if (9 == dt) {
		// Moving Sensor, unit is 'mm'
		sprintf(dev_data, "M\":%d", data);

	} else if (12 == dt) {
		// Vibration Sensor
		sprintf(dev_data, "V\":%d", data);

	} else if (13 == dt) {
		// Float & Temp Sensor
		dd = (float)(data / 10.0);
		ftoa(data_buf, dd, 1);
		sprintf(dev_data, "T\":%s`\"L\":%d`\"iT\":%d", data_buf, (int8_t)(pkt[20]), (int8_t)(pkt[21]));

	} else if (14 == dt) {
		// Water Leak Sensor
		dd = (float)(data / 10.0);
		ftoa(data_buf, dd, 1);
		sprintf(dev_data, "L\":%s", data_buf);

	} else if (16 == dt) {
		// GOTh with oled, Temp&Humi
		dd = (float)(data / 10.0);
		ftoa(data_buf, dd, 1);
		sprintf(dev_data, "T\":%s`\"H\":%d`\"iT\":%d", data_buf, (int8_t)(pkt[20]), (int8_t)(pkt[21]));

	} else if (dt == 28) {
		// Paired-TCC
		// max did: 0x02.FF.FF.FF.FF = 12884901887
		dd = (float)(data / 10.0);
		ftoa(data_buf, dd, 1);

		*(((uint8_t *)&ddid) + 4) = 0x2;
		for (int i = 0; i <= 3; i++) {
			*(((uint8_t *)&ddid) + 3 - i) = pkt[20+i];
		}
		sprintf(dev_data, "T\":%s`\"PID\":%s", data_buf, uint64_to_str(devid_buf, ddid));

	} else if (dt == 29) {
		// Float & Temp Sensor dd = (float)(data / 10.0);
		ftoa(data_buf, dd, 1);
		//ttss = (pkt[18] << 24 | pkt[19] << 16 | pkt[24] << 8 | pkt[25]);
		*(((uint8_t *)&ttss) + 3) = pkt[18];
		*(((uint8_t *)&ttss) + 2) = pkt[19];
		*(((uint8_t *)&ttss) + 1) = pkt[24];
		*(((uint8_t *)&ttss) + 0) = pkt[25];
		sprintf(dev_data, "T\":%s`\"L\":%d`\"iT\":%d`\"TS\":%lu", data_buf, (int8_t)(pkt[20]), (int8_t)(pkt[21]), ttss);

	} else if (dt == 31) {

		// Smoke or DI sensor
		dd = (float)(data / 10.0);
		ftoa(data_buf, dd, 1);
		sprintf(dev_data, "L\":%s`\"iT\":%d", data_buf, (int8_t)pkt[21]);

	} else {
		dd = (float)(data / 10.0);
		ftoa(data_buf, dd, 1);
		sprintf(dev_data, "X\":%s`\"iX\":%d", data_buf, *((uint32_t *)(pkt+20)));
	}

	return dev_data;
}
