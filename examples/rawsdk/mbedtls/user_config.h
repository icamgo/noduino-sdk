#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#define DEBUG
#define USE_OPTIMIZE_PRINTF

#ifdef DEBUG
	#define INFO os_printf
#else
	#define	INFO 
#endif

#define CONFIG_KEY_NUM 1
#define CONFIG_KEY_IO_MUX PERIPHS_IO_MUX_GPIO0_U
#define CONFIG_KEY_IO_NUM 0
#define CONFIG_KEY_IO_FUNC FUNC_GPIO0

#define WIFI_LED_IO_MUX PERIPHS_IO_MUX_GPIO2_U
#define WIFI_LED_IO_NUM 2
#define WIFI_LED_IO_FUNC FUNC_GPIO2
#define USER_LED_IO_MUX PERIPHS_IO_MUX_GPIO2_U
#define USER_LED_IO_NUM 2
#define USER_LED_IO_FUNC FUNC_GPIO2
#define LEDON 1
#define LEDOFF 0

#define SERVER_IP 0
#define PEM_FORMAT 1

#define SERVER_PORT 443
#define SERVER_DOMAIN "apiiot.mjyun.com"
#define HTTP_REQUEST "GET / HTTP/1.0\r\n\r\n"
#define REQUEST_INTERVAL 10000

#endif

