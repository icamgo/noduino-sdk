#include "m5311_opencpu.h"

int is_ref = 0;//是否注册

void opencpu_event_cb(int event)
{
	opencpu_printf("event:%d\n",event);
	switch(event)
	{
		case CIS_EVENT_REG_SUCCESS: 
		{
			is_ref = 1;
			opencpu_printf("onenet register done!\n");
			break;
		}
		case CIS_EVENT_UNREG_DONE:
		{
			is_ref = 0;
			opencpu_printf("onenet register failed\n");
			break;
		}
		case CIS_EVENT_FIRMWARE_TRIGGER:
		{
			opencpu_printf("new firmware is online\n");
			break;
		}
		default:
     		break;
	}
}

void opencpu_notify_cb(int ack_id)
{
    //TODO
    //...
    opencpu_printf("ack_id:%d\n",ack_id);
}

void opencpu_read_cb_error(int mid, int objid, int insid, int resid)
{
    //TODO
    //...
    opencpu_onenet_result(mid, RESULT_400_BADREQUEST, 0);//返回表示该操作结果错误
}

void opencpu_read_cb_ok(int mid, int objid, int insid, int resid)
{
    //TODO
    //...
    //根据传入的objid_insid_resid决定需上传的数据
    if((objid == 3200) && (resid == 5750))
    {
        opencpu_onenet_read(mid, objid, insid, resid, 1, "test read!", 1);//返回表示该操作结果数据,示例资源类型为string型
    }
    else
    {
        opencpu_onenet_result(mid, RESULT_400_BADREQUEST, 0);//返回表示该操作结果错误
    }
}

void opencpu_write_cb(int mid, int objid,int insid, int resid, int type, int flag, int len, char *data)
{
    //TODO
    //...
    opencpu_printf("write:%d %s\n",len,data);
    opencpu_onenet_result(mid, RESULT_204_CHANGED, 0);//操作正确完成返回204
}

void opencpu_execute_cb(int mid, int objid, int insid, int resid, int len, char *data)
{
    //TODO
    //...
    opencpu_printf("write:%d %s\n",len,data);
    opencpu_onenet_result(mid, RESULT_204_CHANGED, 0);//操作正确完成返回204
}

void opencpu_observe_cb(int mid, int observe, int objid, int insid, int resid)
{
    //TODO
    //...
    opencpu_printf("%d_%d_%d: %d\n", objid, insid, resid, observe);//对应的objid被observe后方可notify上报
    opencpu_onenet_result(mid, RESULT_205_CONTENT, 1);//操作正确完成返回204
}

void opencpu_parameter_cb(int mid,int objid, int insid, int resid, int len, char *parameter)
{
    //TODO
    //...
    opencpu_onenet_result(mid, RESULT_204_CHANGED, 0);//操作正确完成返回204
}

void test_onenet()
{
	cot_cb_t callback;
	callback.onRead = opencpu_read_cb_ok;
	callback.onWrite = opencpu_write_cb;
	callback.onExec = opencpu_execute_cb;
	callback.onObserve = opencpu_observe_cb;
	callback.onParams = opencpu_parameter_cb;
	callback.onEvent = opencpu_event_cb;
	callback.onNotify = opencpu_notify_cb;
	callback.onDiscover = NULL;
	
	opencpu_onenet_init();//初始化任务
	opencpu_onenet_create("183.230.40.39", 1, &callback);//创建设备
   	opencpu_onenet_add_obj(3200, 2, "10", 0, 0);
    opencpu_onenet_discover(3200, 19, "5500;5501;5505;5750");//返回将用到的资源列表
	opencpu_onenet_add_obj(3202, 1, "1", 0, 0);
    opencpu_onenet_discover(3202, 4, "5600");//返回将用到的资源列表
    opencpu_printf("open onenet");
	if(opencpu_onenet_open(30, 86400) == 0)
    {
        while(!is_ref)
        {
            vTaskDelay(100);//休眠
        }
        vTaskDelay(3000);//休眠
        opencpu_onenet_notify(3200, 0, 5750, 1, "test", 0, 0, 0);
        opencpu_onenet_notify(3200, 0, 5500, 5, "1", 1, 6, 0);
        opencpu_onenet_notify(3202, 0, 5600, 4, "20.8", 1, -1, 0);
        while(is_ref)
        {
            vTaskDelay(100);//休眠
        }
        opencpu_printf("close onenet");
        opencpu_onenet_close(4);//清理设备
    }
	
}
