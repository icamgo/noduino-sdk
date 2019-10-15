#ifndef OPENCPU_LWM2M_H_
#define OPENCPU_LWM2M_H_

//************************************OPENCPU接口************************************************************
enum {
NBIOT_SUCCESS               = 0,        //100
NBIOT_SYNTAX_ERROR          = 50,       //601   语法，句法错误
NBIOT_MEMORY_ERROR          = 23,       //100	内存错误或无此资源
NBIOT_IN_PROGRESS           = 51,       //651   操作不被允许
NBIOT_PROGRESS_ERROR        = 4,        //100   操作不支持,notify时：无observe权限或未被订阅
NBIOT_FLOW_CONTROL          = 159,      //652   Uplink Busy
NBIOT_NOT_REGISTERED        = 600,      //602   设备未注册
NBIOT_REGISTEREDORING       = 601,      //602   设备注册中或已注册
NBIOT_RESOURE_FAIL			= 602,      //653	资源操作错误，申请dev时已存在，其他操作资源不存在
NBIOT_TOKEN_MISSING			= 603,	    //653
NBIOT_NO_OBSERVE			= 604,      //653
NBIOT_MSG_OUT				= 605,      //100
NBIOT_MSG_BUSY				= 606,      //100
};//错误码定义

//操作结果返回码
#define RESULT_000_ERROR				0
#define RESULT_205_CONTENT				1
#define RESULT_204_CHANGED				2
#define RESULT_400_BADREQUEST			11
#define RESULT_401_UNAUTHORIZED			12
#define RESULT_404_NOTFOUND				13
#define RESULT_405_METHODNOTALLOWED		14
#define RESULT_406_NOTACCEPTABLE		15
#define RESULT_231_CONTINUE				16
#define RESULT_408_REQUESTINCOMPLTETE	17
#define RESULT_413_REQUESTTOOLARGE		18
#define RESULT_415_FORMATUNSUPPORTED	19

//状态事件上报类型
#define CIS_EVENT_BASE 				0x00
#define CIS_EVENT_BOOTSTRAP_START 	CIS_EVENT_BASE + 1
#define CIS_EVENT_BOOTSTRAP_SUCCESS CIS_EVENT_BASE + 2
#define CIS_EVENT_BOOTSTRAP_FAILED 	CIS_EVENT_BASE + 3
#define CIS_EVENT_CONNECT_SUCCESS 	CIS_EVENT_BASE + 4
#define CIS_EVENT_CONNECT_FAILED 	CIS_EVENT_BASE + 5
#define CIS_EVENT_REG_SUCCESS 		CIS_EVENT_BASE + 6
#define CIS_EVENT_REG_FAILED 		CIS_EVENT_BASE + 7
#define CIS_EVENT_REG_TIMEOUT       CIS_EVENT_BASE + 8
#define CIS_EVENT_LIFETIME_TIMEOUT 	CIS_EVENT_BASE + 9
#define CIS_EVENT_STATUS_HALT 		CIS_EVENT_BASE + 10
#define CIS_EVENT_UPDATE_SUCCESS 	CIS_EVENT_BASE + 11
#define CIS_EVENT_UPDATE_FAILED     CIS_EVENT_BASE + 12
#define CIS_EVENT_UPDATE_TIMEOUT    CIS_EVENT_BASE + 13
#define CIS_EVENT_UNREG_DONE 	    CIS_EVENT_BASE + 15
#define CIS_EVENT_RESPONSE_FAILED   CIS_EVENT_BASE + 20
#define CIS_EVENT_RESPONSE_SUCCESS  CIS_EVENT_BASE + 21
#define CIS_EVENT_NOTIFY_FAILED     CIS_EVENT_BASE + 25
#define CIS_EVENT_NOTIFY_SUCCESS    CIS_EVENT_BASE + 26
#define CIS_EVENT_FIRMWARE_TRIGGER  CIS_EVENT_BASE + 50 //设备触发升级流程
/**********************************************************************
* 描  述: 状态事件上报回调函数，需用户实现
* 参  数:
          event: 状态事件类型，参照状态事件类型定义
* 返回值:
 ***********************************************************************/
typedef void (*cot_event_cb_t)       (int event);

/**********************************************************************
* 描  述: notify上报响应回调函数，需用户实现
* 参  数:
          ack_id: notify的
* 返回值:
 ***********************************************************************/
typedef void (*cot_notify_cb_t)      (int ack_id);

/**********************************************************************
* 描  述: 平台read操作回调函数，需用户实现
* 参  数:
          mid: 操作消息的ID
          objid:    被操作的object ID
          insid:    被操作的instance ID
          resid:    被操作的resource ID
* 返回值:
 ***********************************************************************/
typedef void (*cot_read_cb_t)        (int mid, int objid, int insid, int resid);

/**********************************************************************
* 描  述: 平台write操作回调函数，需用户实现
* 参  数:
          mid: 操作消息的ID
          objid:    被操作的object ID
          insid:    被操作的instance ID
          resid:    被操作的resource ID
          type:     写资源的类型，如该资源未曾被notify或read，皆作opaque类型处理
	  flag:	    多条写指令上报时消息标识 1、第一条消息； 2、中间消息； 0、最后一条消息
          len:      数据长度
          data:     被写入的数据
* 返回值:
 ***********************************************************************/
typedef void (*cot_write_cb_t)		 (int mid, int objid, int insid, int resid, int type, int flag, int len, char *data);

/**********************************************************************
* 描  述: 平台excute操作回调函数，需用户实现
* 参  数:
          mid: 操作消息的ID
          objid:    被操作的object ID
          insid:    被操作的instance ID
          resid:    被操作的resource ID
          len:      数据长度
          data:     执行的参数
* 返回值:
 ***********************************************************************/
typedef void (*cot_execute_cb_t)     (int mid, int objid, int insid, int resid, int len, char *data);

/**********************************************************************
* 描  述: 平台observe操作回调函数，需用户实现
* 参  数:
          mid: 操作消息的ID
          observe:  订阅标志 0：订阅；1：取消订阅
          objid:    被操作的object ID
          insid:    被操作的instance ID
          resid:    被操作的resource ID
* 返回值:
 ***********************************************************************/
typedef void (*cot_observe_cb_t)     (int mid, int observe, int objid, int insid, int resid);

/**********************************************************************
* 描  述: 平台设置策略参数请求回调函数，需用户实现
* 参  数:
          mid: 操作消息的ID
          observe:  订阅标志 0：订阅；1：取消订阅
          objid:    被操作的object ID
          insid:    被操作的instance ID
          resid:    被操作的resource ID
          len:      数据长度
          parameter:策略参数，格式如:pmin=xxx; pmax=xxx; gt=xxx; lt=xxx; stp=xxx
                    pmin: int类型，上传数据的最小时间间隔
                    pmax: int类型，上传数据的最大时间间隔
                    gt:   double类型，当数据大于该值上传
                    lt:   double类型，当数据小于该值上传
                    stp:  double类型，当两个数据点相差大于或者等于该值上传
* 返回值:
 ***********************************************************************/
typedef void (*cot_params_cb_t)      (int mid, int objid, int insid, int resid, int len, char *parameter);

/**********************************************************************
* 描  述: 平台discover操作回调函数，需用户实现
* 参  数:
          mid: 操作消息的ID
          objid:    被操作的object ID
* 返回值:
 ***********************************************************************/
typedef void (*cot_discover_cb_t)     (int mid, int objid);
typedef struct cot_cb
{
    cot_read_cb_t       onRead;
    cot_write_cb_t      onWrite;
    cot_execute_cb_t    onExec;
	cot_observe_cb_t    onObserve;
	cot_params_cb_t     onParams;
    cot_event_cb_t      onEvent;
    cot_notify_cb_t     onNotify;
	 cot_discover_cb_t   onDiscover;
}cot_cb_t;

#endif