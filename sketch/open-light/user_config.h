#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#include "osapi.h"
#include "ets_sys.h"
#include "os_type.h"
#include "mem.h"
#include "limits.h"
#include "user_interface.h"
#include "driver/uart.h"
#include "mjyun.h"
#include "mjpwm.h"
#include "cJSON.h"
#include "espnow.h"

#include "app.h"
#include "upnp.h"

#include "group.h"
#include "math.h"

#define CONFIG_ALEXA		1

#define DEBUG				1

#define DEFAULT_VOICE_NAME	"open light"

#ifdef DEBUG
#define INFO( format, ... ) os_printf( format, ## __VA_ARGS__ )
#else
#define INFO( format, ... )
#endif

#define USE_OPTIMIZE_PRINTF
#define PIN_DI 				13
#define PIN_DCKI 			15

#endif
