#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#include "ets_sys.h"
#include "osapi.h"
#include "user_interface.h"
#include "os_type.h"
#include "mem.h"
#include "gpio.h"
#include "math.h"

#include "driver/uart.h"
#include "driver/key.h"
#include "noduino.h"
#include "led.h"

#include "cJSON.h"
#include "mjyun.h"
#include "httpclient.h"
#include "xkey.h"
#include "led.h"
#include "param.h"
#include "low-power.h"
#include "pt1000.h"

#define USE_OLED				1
#define USE_MCP342X				1

//#define LOW_POWER				1
//#define	TESTING_LOW_POWER		1

#define	DEBUG			    	1
#define	CONFIG_CHECK_HOTDATA	1

#define	HTTP_UPLOAD_URL	"http://api.noduino.org/dev/temp?devid=%s&temp=%s&vbat=%s&time=%u&mac=%s"

#ifdef DEBUG
#define INFO( format, ... ) os_printf( format, ## __VA_ARGS__ )
#else
#define INFO( format, ... )
#endif

char *dtostrf(double n, signed char w, unsigned char p, char *s);

#endif
