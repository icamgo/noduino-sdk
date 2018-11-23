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
#include "noduino.h"
#include "led.h"

#include "cJSON.h"
#include "mjyun.h"
#include "httpclient.h"

#define	CONFIG_DEBUG			1
#define	CONFIG_CHECK_HOTDATA	1
#define CONFIG_MQTT_ROBUST		1

#define	HTTP_UPLOAD_URL	"http://api.noduino.org/dev/ikair?devid=%s&temp=%s&humi=%s&light=%d&time=%u&mac=%s"

#define	HTTP_CHECK_ONLINE_URL	"http://api.noduino.org/dev/online?devid=%s"

#ifdef CONFIG_DEBUG
#define INFO( format, ... ) os_printf( format, ## __VA_ARGS__ )
#else
#define INFO( format, ... )
#endif

#endif
