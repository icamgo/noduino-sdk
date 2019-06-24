/*
   opencpu_fota.c
   created by xgl,2019/1/22
*/

#include "m5311_opencpu.h"

#define OPENCPU_SDK_CUSTOM_VERSION  "Version1.1.1"

int update_status = 0;
void opencpu_fota_progress_cb(int current,int total)
{
	if(get_run_mode()==0)
		return;
	opencpu_printf("total:%d,current:%d\n",total,current);
}

void opencpu_fota_event_cb(int event,int state)
{
	//if(get_run_mode()==0)
	//	return;
	opencpu_printf("RSFOTA:%d,%d\n",event,state);
	switch(event)
	{
		case 0://检测流程
			switch(state)
			{
				case 0://无升级包
					opencpu_printf("NO packet\n");
					break;
				case 1://有升级包可下载
					opencpu_printf("find new packet,ready to download\n");
					//opencpu_fota_download();
					break;
				case 2://又升级包且已下载
					opencpu_printf("find new packet,and has already been downloaded\n");
					break;
				default:
					break;
			}
			break;
		case 1://下载流程
			switch(state)
			{
				case 0://开始下载
					opencpu_printf("start download\n");
					break;
				case 2://下载停止,调用opencpu_fota_try_download()之后会从断点处继续下载
					opencpu_printf("download pause\n");
					break;
				case 3://下载成功
					opencpu_printf("download success\n");
					break;
				case 4:	//下载失败，并且丢弃当前进度，调用opencpu_fota_try_download()会重新查询升级包并下载
					opencpu_printf("download failed");
					break;
				case 5:	//平台拒绝下载
					opencpu_printf("download rejected");
					break;
				default:
					break;
			}
			break;
		case 2://升级流程,运行到此分支时，opencpu用户线程尚未启动，故不能使用打印函数等
			switch(state)
			{
				case 3:
					update_status = 1;
					break;
				case 4:
					update_status = -1;
				default:
					break;
					
			}
			break;
		default:
			break;		
	}
}

unsigned long opencpu_fota_version_cb()
{
	return OPENCPU_SDK_CUSTOM_VERSION;
}
