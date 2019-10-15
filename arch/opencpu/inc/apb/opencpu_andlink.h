#ifndef OPENCPU_ANDLINK_H_
#define OPENCPU_ANDLINK_H_

#include "opencpu_lwm2m.h"

/**********************************************************************
* 描  述: 初始化andlink任务，在使用andlink相关函数之前调用
* 参  数:
* 返回值:
 ***********************************************************************/
extern void opencpu_andlink_init();

/**********************************************************************
* 描  述: 创建cot设备，当前只允许创建一个设备
* 参  数:
          ip:       需注册的andlink平台服务器IP
          is_bs:    是否bootstrap服务器
          cot_cb:   需注册的回调函数
* 返回值:
          0: 创建成功，创建成功后
          >0:创建失败，错误码参照错误码定义
 ***********************************************************************/
extern int opencpu_andlink_create(char *ip, int is_bs, cot_cb_t *cot_cb);



/**********************************************************************
* 描  述: 注销设备与andlink连接 或 清理andlink设备
* 参  数:
          level: 2 注销与andlink连接，清除resource数据
                 4 关闭与andlink连接，清除所有resource、instance、object数据
* 返回值:
          0: 操作成功
             若有注销andlink平台操作，cot_event_cb_t将被回调
          >0:操作失败，错误码参照错误码定义
 ***********************************************************************/
extern int opencpu_andlink_close(int level);

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
extern int opencpu_andlink_add_obj(int objid, int inscount, unsigned char* bitmap, int attcount, int actcount);

/**********************************************************************
* 描  述: 删除一个object，同时将删除相关的所有instance和resource数据
* 参  数:
          objid:    object id,本次需删除的object，如3200
* 返回值:
          0: 删除成功
          >0:删除失败，错误码参照错误码定义
 ***********************************************************************/
extern int opencpu_andlink_del_obj(int objid);

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
extern int opencpu_andlink_open(unsigned int timeout, unsigned int lifetime);

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
extern int opencpu_andlink_update(unsigned int lifetime, int flag);

/**********************************************************************
* 描  述: 向平台发起查询是否在注册状态
* 参  数:
* 返回值:
          0:  向平台发起查询成功，操作是否成功需等待模组cot_event_cb_t回调结果
          >0: 向平台发起查询失败，错误码参照错误码定义
          600:设备未注册
 ***********************************************************************/
extern int opencpu_andlink_is_open(void);



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
extern int opencpu_andlink_notify(int objid, int insid, int resid, int type, char *value, int flag, int ack_id, int mode);

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
extern int opencpu_andlink_read(int mid, int objid, int insid, int resid, int type, char *value, int flag);

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
extern int opencpu_andlink_result(int mid, int result, int is_observe);

/**********************************************************************
* 描  述: 配置平台设备ID
* 参  数:
          value: 平台设备ID
* 返回值:
          0:  操作成功
          >0: 操作失败，错误码参照错误码定义
 ***********************************************************************/
extern int opencpu_andlink_config(char *value);
//************************************OPENCPU接口************************************************************

#endif