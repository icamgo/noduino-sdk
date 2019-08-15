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

float get_vbat()
{
	int adc = 0;		/* mV */
    adc = opencpu_adc(HAL_ADC_CHANNEL_0);
	opencpu_printf("ADC = %d\n", adc);
	return adc / 1000.0 * (1470.0 / 470.0);
}

void show_iccid()
{
	unsigned char buf[30];
	int i = 0;
	memset(buf, 0, 30);
	while(opencpu_iccid(buf) != 0)
	{
		i++;
		vTaskDelay(10);
		if(i > 20) {
			opencpu_printf("ICCID timeout\n");
			return;
		}
	}
	opencpu_printf("ICCID: %s\n", buf);
}

void show_signal_strength()
{
	int rssi,rxlevel;

	opencpu_csq(&rssi, &rxlevel);
	opencpu_printf("CESQ: %d, %d\n", rssi, rxlevel);
}

static int char_to_int(unsigned char *s)
{
	int i;  
	int n = 0;  
	for (i = 0; s[i] >= '0' && s[i] <= '9'; ++i) {  
	    n = 10 * n + (s[i] - '0');  
	}  
	return n;  
}

void show_rtc_time()
{
	unsigned char time_string[32];

	memset(time_string, 0, 32);

	opencpu_rtc_get_time(time_string);

	/* 2019/8/15,12:22:34GMT+8 */
	opencpu_printf("TIME: %s\n", time_string);

#if 0
	unsigned char time_build[32] = {0};
	unsigned char len_mess[4];
	unsigned char *p1;
	unsigned char *p2;

	strcpy(time_build, time_string);

	p1 = strchr(time_string, '/');
	*p1 = '-';
	p1 = strchr(p1,'/');
	*p1 = '-';
	p1 = strchr(p1,',');
	*p1 = ' ';

	p2 = strchr(time_string,'+');

	/* 2019-8-15 12:22:34GMT+8 */
	opencpu_printf("TIME: %s\n",time_string);

	sprintf(p1+1, "%d", char_to_int(&time_build[p1 - time_string + 1]) + char_to_int(&time_build[p2 - time_string + 1]));

	/* 2019-8-15 20 */
	opencpu_printf("TIME: %s\n", time_string);

	p2  = strchr(time_build,'G');
	*p2 = 0;

	strcat(time_string, strchr(time_build, ':'));
	strcat(time_string, "\0");
#endif
}

void test_dm()
{
	dm_config_t dm_config = {
		1, //测试
		1, //enable
		2, //心跳间隔2min
		20, //版本号，默认2.0
		"M100000052", //appkey
		"n525A97z0M7Vyh91b0508l7j0U5g2g9Y"};
	opencpu_dm_start(dm_config);
}

void flash_test()
{
	unsigned char temp_read[10];
	unsigned char temp_write[] = "123123";

	memset(temp_read, 0, 10);

	opencpu_flash_erase(0, 6);
	opencpu_flash_write(1, temp_write, strlen(temp_write));
	opencpu_flash_read(1, temp_read, 6);
	opencpu_printf("Read: %s\n",temp_read);
}

void rtc_timer_cb()
{
	opencpu_printf("RTC Timer expires\n");
	push_data_via_tcp();
}

void rtc_timer_start()
{
	unsigned int handle;
	//定时 900S，到期执行 rtc_timer_cb 函数，循环执行
	opencpu_rtc_timer_create(&handle, 900, true, rtc_timer_cb);
	opencpu_rtc_timer_start(handle);
	opencpu_printf("RTC Timer starts\n");
}

/*
 * 此函数为opencpu产线模式相关的回调函数，返回1则版本下载到模组后
 * 以AT命令方式启动，需要先执行AT+ATCLOSE命令，之后才会以opencpu方式启动。
 * 返回0则以opencpu方式启动
 * 请务必联系技术支持后再确定返回值
*/
int get_factory_mode(void)
{
	return 1;
}

/*
 * 此函数为wakeup引脚中断回调函数，在wakein引脚拉低时触发执行
 * 睡眠状态下测试时，打印函数不会生效，因为系统刚刚恢复，
 * 还未初始化uart
*/
void opencpu_wakeup_callback()
{
	opencpu_printf("opencpu wakeup\n");
}

