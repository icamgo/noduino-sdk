/*
 *  Copyright (c) 2019 - 2029 MaiKe Labs
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
*/

#ifndef __QUARK_NB_H__
#define __QUARK_NB_H__

#include <stdarg.h>
#include <math.h>
#include "lwip/netdb.h"
#include "lwip/api.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sockets.h"
#include "hal_uart.h"
#include "hal_eint.h"
#include "hal_gpio.h"
#include "ril.h"
#include "hal_pwm.h"
#include "serial_port.h"
#include "timers.h"
#define socklen_t int
#include "other.h"
#include "spi.h"
#include "opencpu_onenet.h"
#include "opencpu_ct.h"

#include <time.h>

typedef unsigned short uint16_t;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef void(* rtc_sw_timer_callback_t)(void *);

typedef enum {
	PING_PACKET_RESULT,
	PING_TOTAL_RESULT
} ping_result_type_t;

typedef  unsigned char u8;
typedef  unsigned int  u32;
typedef void (* ping_request_result_t)(ping_result_type_t type, void* result);
typedef void (* dns_request_result_t)(unsigned char*);

typedef enum {
    HAL_I2C_FREQUENCY_50K  = 0,          /**<  50kbps. */
    HAL_I2C_FREQUENCY_100K = 1,          /**<  100kbps. */
    HAL_I2C_FREQUENCY_200K = 2,          /**<  200kbps. */
    HAL_I2C_FREQUENCY_300K = 3,          /**<  300kbps. */
    HAL_I2C_FREQUENCY_400K = 4,          /**<  400kbps. */
    HAL_I2C_FREQUENCY_1M   = 5,          /**<  1mbps. */
    HAL_I2C_FREQUENCY_MAX                /**<  The total number of supported I2C frequencies (invalid I2C frequency).*/
} hal_i2c_frequency_t;

typedef struct _ping_result
{
	uint32_t min_time;
	uint32_t max_time;
	uint32_t avg_time;
	uint32_t total_num;
	uint32_t lost_num;
	uint32_t recv_num;
	ip4_addr_t ping_target;
} ping_result_t;

typedef struct {
	bool is_timeout;		 /*When it is true, other data is invalid.*/
	uint32_t rtt;			/*The unit is ms.*/
	uint32_t ttl;			/*The TTL value in ping response packet.*/
	uint32_t packet_size;	/*The unit is byte.*/
	bool is_ipv4;			/*ipv4 is true, ipv6 is false.*/
	uint16_t ip_address[8];  /*The address has been translated by ping thread.*/
} ping_packet_result_t;

#define CMDMP_MAX_APPKEY	15
#define CMDMP_MAX_PSWD		40

typedef struct {
	uint8  addr_mode; 	//!< 服务器信息，0:商用服务器，1:测试服务器
	uint8  mode;		//!< 0:disable 1:CMCC
	uint16 interval;	//!< 心跳间隔，分钟
	uint16 version;		//!< DM版本号，无效参数（兼容性设置)，默认V2.0
	uint8 appkey[CMDMP_MAX_APPKEY]; //!< APPKEY，厂商唯一
	uint8 pswd[CMDMP_MAX_PSWD];		//!< password，厂商唯一
} dm_config_t;

#define ATTR_ZIDATA_IN_NONCACHED_RAM_4BYTE_ALIGN  __attribute__ ((__section__(".noncached_zidata"),__aligned__(4)))

typedef void (*oc_sntp_callback_t)       (unsigned char * info);

#include "opencpu_api.h"

void opencpu_printf (const char *str, ...);
void opencpu_i2c_test(void);
void test_opencpu_start(void);
void opencpu_task_main(void);

void push_data_via_udp(void);
void push_data_via_tcp(void);

float get_vbat();
void show_signal_strength(void);
void show_iccid();

void gpio_init();
void sensor_power_on();
void sensor_power_off();

long bmp180_get_press();

void test_dns(void);
void test_cmdns_cb(unsigned char *ip);

void uart_init();
void custom_uart_init();
extern unsigned char uart_cmd;
extern int update_status;
#endif
