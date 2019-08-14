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

#include "quark-nb.h"

#define OPENCPU_SDK_CUSTOM_VERSION  "1.2.0"

int update_status = 0;
void opencpu_fota_progress_cb(int current,int total)
{
	if(get_run_mode()==0)
		return;
	opencpu_printf("total: %d, current: %d\n", total, current);
}

void opencpu_fota_event_cb(int event,int state)
{
	opencpu_printf("RSFOTA: %d, %d\n", event, state);

	switch(event) {
		case 0:
			//检测流程
			switch(state) {
				case 0:
					opencpu_printf("NO packet\n");
					break;
				case 1:
					opencpu_printf("Find new packet,ready to download\n");
					//opencpu_fota_download();
					break;
				case 2:
					opencpu_printf("Find new packet,and has already been downloaded\n");
					break;
				default:
					break;
			}
			break;

		case 1:
			//下载流程
			switch(state)
			{
				case 0:
					opencpu_printf("Start download\n");
					break;
				case 2:
					//下载停止,调用opencpu_fota_try_download()之后会从断点处继续下载
					opencpu_printf("Download pause\n");
					break;
				case 3:
					opencpu_printf("Download success\n");
					break;
				case 4:
					//下载失败，并且丢弃当前进度，调用opencpu_fota_try_download()会重新查询升级包并下载
					opencpu_printf("Download failed");
					break;
				case 5:
					//平台拒绝下载
					opencpu_printf("Download rejected");
					break;
				default:
					break;
			}
			break;

		case 2:
			//升级流程,运行到此分支时，opencpu用户线程尚未启动，故不能使用打印函数等
			switch(state) {
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
