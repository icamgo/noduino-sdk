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

#include "mjyun.h"
#include "cJSON.h"

//#define	DEBUG			1

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
// 0x3C000
#define PARAM_START_SEC		0xFA

void xkey_init();

void curtain_on();
void curtain_off();
void curtain_init();
void curtain_set_status(int status, int pos);
void curtain_set_status_and_publish(int status, int pos);
int curtain_get_status();
int curtain_get_position();
void curtain_publish_status();

void param_set_status(int status);
void param_set_positon(int pos);
int param_get_status(void);
int param_get_position(void);
void param_save(void);
void param_init();

bool encoder_direction();
int encoder_pos();
void encoder_reset();
void encoder_init();
uint32_t encoder_circle();

//void led_init();
//void wifi_led_enable();
//void wifi_led_disable();

#endif
