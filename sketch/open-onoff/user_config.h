#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#include "ets_sys.h"
#include "osapi.h"
#include "user_interface.h"
#include "os_type.h"
#include "mem.h"
#include "gpio.h"

#include "driver/key.h"
#include "driver/uart.h"
#include "noduino.h"

#include "udp_srv.h"

#include "upnp.h"
#include "httpd.h"

#include "mjyun.h"

#define	DEBUG			1

#ifdef DEBUG
#define INFO( format, ... ) os_printf( format, ## __VA_ARGS__ )
#else
#define INFO( format, ... )
#endif

#define	XKEY_NUM		1

#define XKEY_IO_MUX		PERIPHS_IO_MUX_GPIO0_U
#define XKEY_IO_NUM		0
#define XKEY_IO_FUNC	FUNC_GPIO0

// NOTICE: --- For 1024KB spi flash
// 0xFA000
#define PARAM_START_SEC		0xFA

void xkey_init();

void relay_on();
void relay_off();
void relay_init();
void relay_set_status(uint8_t status);
void relay_set_status_and_publish(uint8_t status);
uint8_t relay_get_status();
void relay_publish_status();

void param_set_status(uint8_t status);
uint8_t param_get_status(void);
void param_save(void);
void param_init();

void led_init();
void wifi_led_enable();
void wifi_led_disable();
void led_on();
void led_off();
void led_set(uint8_t st);

#endif
