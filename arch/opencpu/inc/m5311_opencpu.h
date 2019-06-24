#ifndef __OPENCPU_H_
#define __OPENCPU_H_

#include <stdarg.h>
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
#include "serial_port.h"
#include "timers.h"
#define socklen_t int
#include "other.h"
#include "spi.h"
#include "opencpu_onenet.h"




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
    bool is_timeout;         /*When it is true, other data is invalid.*/
    uint32_t rtt;            /*The unit is ms.*/
    uint32_t ttl;            /*The TTL value in ping response packet.*/
    uint32_t packet_size;    /*The unit is byte.*/
    bool is_ipv4;            /*ipv4 is true, ipv6 is false.*/
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



#include "opencpu_api.h"


//操作示例函数
void opencpu_printf (const char *str, ...);
void opencpu_i2c_test(void);
void test_opencpu_start(void);
void opencpu_task_main(void);
void udp_test(void);
void tcp_test(void);
void get_signal_strength(void);
unsigned char get_network_state(void);
void test_ping_cb(ping_result_type_t type, void* p);
void test_ping(void);
void test_dns(void);
void test_cmdns_cb(unsigned char *ip);
void test_get_cellinfo(void);
void test_get_iccid(void);
void test_get_psminfo(void);
void test_all_in_one();
void test_gpio_irq();
void test_gpio();
void uart_init();
void test_get_imei_imsi();
void test_adc();
void test_pwm_init();
void custom_uart_init();
void test_dm(void);
extern unsigned char uart_cmd;
extern int update_status;
#endif