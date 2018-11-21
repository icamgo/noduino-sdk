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

#define	DEBUG			    	1
#define	CONFIG_CHECK_HOTDATA	1

#define	HTTP_UPLOAD_URL	"http://api.noduino.org/dev/gas?devid=%s&CH4=%d&time=%u&mac=%s"

#ifdef DEBUG
#define INFO( format, ... ) os_printf( format, ## __VA_ARGS__ )
#else
#define INFO( format, ... )
#endif

int get_ch4(float *v);
void http_upload(int ch4);
void publish_sensor_data(int ch4, int uv, int v0);

#endif
