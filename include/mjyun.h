#ifndef __MJYUN_H__
#define __MJYUN_H__

#if defined(GLOBAL_DEBUG_ON)
#define MJYUN_DEBUG_ON
#endif
#if defined(MJYUN_DEBUG_ON)
#define MJYUN_DEBUG(format, ...) os_printf(format, ##__VA_ARGS__)
#else
#define MJYUN_DEBUG(format, ...)
#endif

/* mjyun flag */
#define	WITH_MQTT				0x1
#define	WITH_HTTPS				0x2
#define	WITHOUT_PING			0x4

/*
 * 对mjyun库进行配置
 */
typedef struct
{
	const char* product_id; 		/* 产品id, 可从mjyun官网后台获得 [必填]*/
	const char* hardware_version;	/*设置硬件版本号[必填]*/
	const char* firmware_version;	/*设置固件版本号[必填]*/
	const char* online_words; 		/*设置mjyun服务器发送给app的设备上线(online)的附加数据 [该数据缺省为设备id] [选填]*/
	const char* offline_words; 		/*设置mjyun服务器发送给app的设备离线消息(offline)的附加数据 [该数据缺省为设备id] [选填]*/
	uint32_t run_flag;				/* WITH_MQTT WITH_HTTPS */
} mjyun_config_t;


typedef enum {
	WIFI_IDLE, 						/*Wi-Fi空闲，常出现在设备刚刚启动且没有连接过任何SSID时*/
	WIFI_SMARTLINK_START, 			/*设备进入智能连接状态*/
	WIFI_SMARTLINK_LINKING, 		/*设备正在进行智能连接*/
	WIFI_SMARTLINK_FINDING, 		/*设备正在进行智能连接，正在查找手机所在的信道与发出的特征包*/
	WIFI_SMARTLINK_TIMEOUT, 		/*设备智能连接超时*/
	WIFI_SMARTLINK_GETTING, 		/*设备正在进行智能连接，已经锁定手机信道，正在解码数据包*/
	WIFI_SMARTLINK_OK, 				/*设备智能连接成功*/
	WIFI_AP_OK, 					/*设备在AP模式*/
	WIFI_AP_ERROR, 					/*设备在AP模式并发生了某些错误*/
	WIFI_AP_STATION_OK, 			/*设备在AP-STATION混合模式*/
	WIFI_AP_STATION_ERROR, 			/*设备在AP-STATION混合模式并发生了某些错误*/
	WIFI_STATION_OK, 				/*设备在STATION模式*/
	WIFI_STATION_ERROR, 			/*设备在STATION模式并发生了某些错误*/
	WIFI_STA_DISCONNECTED,			/*设备与路由器连接断开*/
	WIFI_TRY_CACHED_AP,
	MJYUN_CONNECTING, 				/*正在连接摩羯云*/
	MJYUN_CONNECTING_ERROR, 		/*连接摩羯云失败*/
	MJYUN_CONNECTED, 				/*已连接摩羯云*/
	MJYUN_DISCONNECTED, 			/*与摩羯云的连接断开*/
} mjyun_state_t;

typedef enum {
	SMARTLINK,
	WIFI,
	MJYUN_CLOUD,
} mjyun_msgtype_t;

/*----------------------- */
/*mjyun APIs*/
/*----------------------- */

typedef void (*mjyun_callback)();
typedef void (*mjyun_data_callback)(const char *event_name, const char *event_data);
typedef void (*mjyun_state_callback)(/* mjyun_msgtype_t type ,*/mjyun_state_t state);

/******************************************************************************
 * FunctionName : mjyun_run
 * Description  : device start to run mjyun platform
 * Parameters   : const char* product_id -- mjyun product id
 *              : const char* offline_words -- device will word
 * Returns      : return 0 success, others fails
 *******************************************************************************/
int mjyun_run(const mjyun_config_t* conf);
void mjyun_onconnected(mjyun_callback connectedCb);
void mjyun_ondisconnected(mjyun_callback disconnectedCb);
/*
void mjyun_connect();
void mjyun_disconnect();
*/
void mjyun_ondata(mjyun_data_callback dataCb);
void mjyun_onpublished(mjyun_callback sendCb);
BOOL mjyun_publish(const char* event_name, const char* event_data);
BOOL mjyun_publishstatus(const char* status_data);


void mjyun_statechanged(mjyun_state_callback stateChangedCb);
mjyun_state_t mjyun_state();

/*----------------------- */
/*others APIs*/
/*----------------------- */

/******************************************************************************
 * FunctionName : MJYUN_GetDevicePasswd
 * Description  : get mjyun passwd of this device
 * Parameters   : none 
 * Returns      : if this device didn't init, return NULL
*******************************************************************************/
uint8_t* mjyun_getdevicepasswd();

/******************************************************************************
 * FunctionName : mjyun_getdeviceid
 * Description  : get mjyun DID of this device
 * Parameters   : none 
 * Returns      : passwd, if did didn't init, return NULL
*******************************************************************************/
uint8_t* mjyun_getdeviceid();

//mjyun_get_firmware_id( uint8_t slot );
//mjyun_get_firmware_version( void );
//mjyun_get_hardware_version( void );

/******************************************************************************
 * FunctionName : mjyun_systemrecovery
 * Description  : recovery mjyun data
 * Parameters   : none 
 * Returns      : none
*******************************************************************************/
void mjyun_systemrecovery(); //系统恢复，抹掉所有mjcloud相关数据 --> 一般对应物理按键

void mjyun_forceentersmartlinkmode(); //强制进入配网模式 --> 一般对应物理按键

void mjyun_setssidprefix(const char* sPrefix); //设置物理硬件作为AP时的SSID前缀

void mjyun_check_update(); //检查固件更新

void mjyun_mini_ota_start(const char * path);

bool mjyun_lan_start(void);
bool mjyun_lan_stop(void);

#endif /*  __MJYUN_H__ */
