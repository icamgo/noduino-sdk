#ifndef OPENCPU_ONENET_H_
#define OPENCPU_ONENET_H_

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

/**********************************************************************
* 描  述: 初始化OneNET任务，在使用OneNET相关函数之前调用
* 参  数:
* 返回值:
 ***********************************************************************/
extern void opencpu_onenet_init();

/**********************************************************************
* 描  述: 创建cot设备，当前只允许创建一个设备
* 参  数:
          ip:       需注册的OneNET平台服务器IP
          is_bs:    是否bootstrap服务器
          cot_cb:   需注册的回调函数
* 返回值:
          0: 创建成功，创建成功后
          >0:创建失败，错误码参照错误码定义
 ***********************************************************************/
extern int opencpu_onenet_create(char *ip, int is_bs, cot_cb_t *cot_cb);


/**********************************************************************
* 描  述: 创建cot设备，当前只允许创建一个设备
* 参  数:
          ip:       需注册的OneNET平台服务器IP
          is_bs:    是否bootstrap服务器
          cot_cb:   需注册的回调函数
          auth_code:注册服务器认证参数，不设值时平台无认证参数设置为NULL
* 返回值:
          0: 创建成功，创建成功后
          >0:创建失败，错误码参照错误码定义
 ***********************************************************************/
extern int opencpu_onenet_create_ex(char *ip, int is_bs, cot_cb_t *cot_cb,char *auth_code);

/**********************************************************************
* 描  述: 创建cot设备，当前只允许创建一个设备
* 参  数:
          ip:       需注册的OneNET平台服务器IP
          is_bs:    是否bootstrap服务器
          cot_cb:   需注册的回调函数
          auth_code:注册服务器认证参数，不设值时平台无认证参数设置为NULL
		  psk:		配置与平台建立DTLS连接的psk秘钥
* 返回值:
          0: 创建成功，创建成功后
          >0:创建失败，错误码参照错误码定义
 ***********************************************************************/
extern int opencpu_onenet_create_dlts(char *ip, int is_bs, cot_cb_t *cot_cb,char *auth_code,char *psk);
/**********************************************************************
* 描  述: 注销设备与OneNET连接 或 清理OneNET设备
* 参  数:
          level: 2 注销与OneNET连接，清除resource数据
                 4 关闭与OneNET连接，清除所有resource、instance、object数据
* 返回值:
          0: 操作成功
             若有注销OneNET平台操作，cot_event_cb_t将被回调
          >0:操作失败，错误码参照错误码定义
 ***********************************************************************/
extern int opencpu_onenet_close(int level);

/**********************************************************************
* 描  述: 添加一个object
* 参  数:
          objid:    object id,本次需添加的object，如3200
          inscount: 该object对象的instance个数，前添加的object有5个实例，则为5
          bitmap:   实例位图，字符串格式，每一个字符表示为一个实例，其中1表示可用，0表示不可用。
                    例如当前添加的object有5个实例，其中，0,3可用，则实例位图为10010
          attcount: 属性个数，当前版本暂不使用，置0即可
          actcount: 操作个数，当前版本暂不使用，置0即可
* 返回值:
          0: 添加成功
          >0:添加失败，错误码参照错误码定义
 ***********************************************************************/
extern int opencpu_onenet_add_obj(int objid, int inscount, unsigned char* bitmap, int attcount, int actcount);

/**********************************************************************
* 描  述: 删除一个object，同时将删除相关的所有instance和resource数据
* 参  数:
          objid:    object id,本次需删除的object，如3200
* 返回值:
          0: 删除成功
          >0:删除失败，错误码参照错误码定义
 ***********************************************************************/
extern int opencpu_onenet_del_obj(int objid);

/**********************************************************************
* 描  述: 向平台发起注册请求
* 参  数:
          timeout:    本次注册的超时时长，单位为s，超时后将关闭当前操作，并上报。不可为0
          lifetime:   本次注册成功后生命周期，单位为s。
                      生命周期超时后如未及时UPDATE将注销当前连接，并上报
* 返回值:
          0: 操作发起成功，操作是否成功依据模组cot_event_cb_t回调结果
          >0:操作发起失败，错误码参照错误码定义
 ***********************************************************************/
extern int opencpu_onenet_open(unsigned int timeout, unsigned int lifetime);

/**********************************************************************
* 描  述: 向平台发起更新生命周期或当前object请求
* 参  数:
          lifetime:    更新的lifetime值，单位为s，如果小于10则表示使用默认的lifetime值，86400s
          flag:        是否需要同时更新注册的Object对象
                       0:不更新     1:更新
* 返回值:
          0: 操作发起成功，操作是否成功依据模组cot_event_cb_t回调结果
          >0:操作发起失败，错误码参照错误码定义
 ***********************************************************************/
extern int opencpu_onenet_update(unsigned int lifetime, int flag);

/**********************************************************************
* 描  述: 向平台发起查询是否在注册状态
* 参  数:
* 返回值:
          0:  向平台发起查询成功，操作是否成功需等待模组cot_event_cb_t回调结果
          >0: 向平台发起查询失败，错误码参照错误码定义
          600:设备未注册
 ***********************************************************************/
extern int opencpu_onenet_is_open(void);

/**********************************************************************
* 描  述: 声明指定object的资源列表
* 参  数:
          objid: 需discover操作的对象ID
          len: data的长度
          data:object的resource要求，多个resource之间使用分号";"隔开"1101;1102;1103"
* 返回值:
          0:  操作成功
          >0: 操作失败，错误码参照错误码定义
 ***********************************************************************/
extern int opencpu_onenet_discover(int objid, int len, char *data);


/**********************************************************************
* 描  述: 向平台上报指定resource数据的变化
* 参  数:
          objid: object id,本次需上报数据的对象ID，如3200
          insid: instance id，本次需上报数据的实例ID
          resid: resource id，本次需上报数据的资源ID
          type:  发送的数据类型，不透明类型为十六进制字符串直接上报
                 1      2       3       4       5
                 string opaque  integer float   bool
                 字符串 不透明  整形    浮点型  布尔型
          value: 发送的数据
          flag:  消息标识 0:该Notify消息暂不上报 1:上报未上报的Notify消息(同一instance下)
          ack_id:指定该消息是否以响应形式上报  -1:非响应模式  >=0:响应模式
                 如果为响应模式，该条Notify消息成功时将被cot_notify_cb_t回调
                 该ack_id应该为不同的值
          mode:  是否使用RAI功能，当前暂不提供该功能
* 返回值:
          0:  操作成功
          >0: 操作失败，错误码参照错误码定义
 ***********************************************************************/
extern int opencpu_onenet_notify(int objid, int insid, int resid, int type, char *value, int flag, int ack_id, int mode);

/**********************************************************************
* 描  述: cot_read_cb被回调后，回复平台读取到的资源的值。
* 参  数:
          mid:   收到的+MIPLREAD的消息携带的mid。
          objid: object id,本次需回复数据的对象ID，如3200
          insid: instance id，本次需回复数据的实例ID
          resid: resource id，本次需回复数据的资源ID
          type:  回复的数据类型，不透明类型为十六进制字符串直接上报
                 1      2       3       4       5
                 string opaque  integer float   bool
                 字符串 不透明  整形    浮点型  布尔型
          value: 回复的数据
          flag:  消息标识 0:该read回复暂不回复 1:回复未回复的read消息(同一instance下)
* 返回值:
          0:  操作成功
          >0: 操作失败，错误码参照错误码定义
 ***********************************************************************/
extern int opencpu_onenet_read(int mid, int objid, int insid, int resid, int type, char *value, int flag);

/**********************************************************************
* 描  述: 以下回调函数被回调后，回复平台操作结果
          cot_write_cb
          cot_execute_cb
          cot_observe_cb
          cot_parameter_cb
* 参  数:
          mid:       收到的操作消息携带的mid
          result:    操作结果返回码
          is_observe:是否是MIPLOBSERVE操作
* 返回值:
          0:  操作成功
          >0: 操作失败，错误码参照错误码定义
 ***********************************************************************/
extern int opencpu_onenet_result(int mid, int result, int is_observe);

//************************************OPENCPU接口************************************************************

#endif