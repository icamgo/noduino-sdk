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

#include "mjyun.h"
#include "httpclient.h"

#define	DEBUG			    1
#define	MQTT_SEND_RATE_SEC	2
#define	HTTP_SEND_RATE_MIN	3

#define	HTTP_UPLOAD_URL	"http://api.noduino.org/tempdata/updata.php?dvid=%s&productid=%s&temp=%s&humi=%s&time=%u"

#endif
