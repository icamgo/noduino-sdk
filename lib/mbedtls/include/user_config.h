#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#define ESP_DEBUG
#define USE_OPTIMIZE_PRINTF

#ifdef USE_INTER_LED
	#undef ESP_DEBUG
#endif

#ifdef ESP_DEBUG
	#define ESP_DBG os_printf
#else
	#define ESP_DBG
#endif

#define CONFIG_KEY_NUM 1
#define CONFIG_KEY_IO_MUX PERIPHS_IO_MUX_GPIO0_U
#define CONFIG_KEY_IO_NUM 0
#define CONFIG_KEY_IO_FUNC FUNC_GPIO0

#ifdef USE_INTER_LED
	#define WIFI_LED_IO_MUX PERIPHS_IO_MUX_U0TXD_U
	#define WIFI_LED_IO_NUM 1
	#define WIFI_LED_IO_FUNC FUNC_GPIO1
	#define USER_LED_IO_MUX PERIPHS_IO_MUX_U0TXD_U
	#define USER_LED_IO_NUM 1
	#define USER_LED_IO_FUNC FUNC_GPIO1
	#define LEDON 0
	#define LEDOFF 1
#else
	#define WIFI_LED_IO_MUX PERIPHS_IO_MUX_GPIO2_U
	#define WIFI_LED_IO_NUM 2
	#define WIFI_LED_IO_FUNC FUNC_GPIO2
	#define USER_LED_IO_MUX PERIPHS_IO_MUX_GPIO2_U
	#define USER_LED_IO_NUM 2
	#define USER_LED_IO_FUNC FUNC_GPIO2
	#define LEDON 1
	#define LEDOFF 0
#endif

#define SERVER_IP 0
#define PEM_FORMAT 1

#define SERVER_PORT 443
#define SERVER_DOMAIN "iot.espressif.cn"
#define HTTP_REQUEST "GET / HTTP/1.0\r\n\r\n"
#define REQUEST_INTERVAL 10000

#endif

