/*
   opencpu_base_func.c
   created by xgl,2018/4/2
*/

#include "m5311_opencpu.h"

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

static int char_to_int(unsigned char *s)
{
	int i;  
    int n = 0;  
    for (i = 0; s[i] >= '0' && s[i] <= '9'; ++i)  
    {  
        n = 10 * n + (s[i] - '0');  
    }  
    return n;  
}

/* 测试获取实时时间 */
void test_get_time()
{
	unsigned char time_string[50];
	unsigned char time_build[50] = {0};
	unsigned char len_mess[4];
	unsigned char *p1;
	unsigned char *p2;
	memset(time_string,0,50);
	opencpu_rtc_get_time(time_string);
	opencpu_printf("TIME:%s\n",time_string);
	strcpy(time_build,time_string);
	p1 = strchr(time_string,'/');
		*p1 = '-';
		p1 = strchr(p1,'/');
		*p1 = '-';
		p1 = strchr(p1,',');
		*p1 = ' ';
		p2 = strchr(time_string,'+');
		opencpu_printf("TIME:%s\n",time_string);
		sprintf(p1+1,"%d",char_to_int(&time_build[p1-time_string+1])+char_to_int(&time_build[p2-time_string+1]));
		opencpu_printf("TIME:%s\n",time_string);
		p2  = strchr(time_build,'G');
		*p2 = 0;
		strcat(time_string,strchr(time_build,':'));
		strcat(time_string,"\0");
}

/* 测试设置RTC时间 */
void test_set_time()
{
/*
 * <time>:  string type value; format is "yy/MM/dd,hh:mm:ss+-zz"
 *          where characters indicate the last two digits of
 *          year,month,day,hour,minute,second and time zone.
 *          The time zone is expressed in quarters of an hour
 *          between the local time and GMT; range (-47...+48)
 *          eg. 6th May 1994 22:10:00 GMT+2 hours equals
 *          "94/05/06,22:10:00+08"
*/
	opencpu_rtc_set_time("94/05/06,22:10:00+08");
	opencpu_printf("time set ok\n");
	
}

unsigned char local_imei[40] = {0};
void test_get_imei()
{
	memset(local_imei,0,40);
	opencpu_printf("result:%d\n",opencpu_get_imei(local_imei));
	opencpu_printf("IMEI:%s\n",local_imei);
	
}

unsigned char local_imsi[40] = {0};
void test_get_imsi()
{
	memset(local_imsi,0,40);
	opencpu_printf("result:%d\n",opencpu_get_imsi(local_imsi));
	opencpu_printf("IMSI:%s\n",local_imsi);
	
}

void test_get_iccid()
{
	unsigned char buf[30];
	int i = 0;
	memset(buf,0,30);
	while(opencpu_iccid(buf)!= 0)
	{
		i++;
		vTaskDelay(10);
		if(i>20)
		{
			opencpu_printf("iccid timeout\n");
			return;
		}
	}
	opencpu_printf("ICCID:%s\n",buf);
}

void get_signal_strength()
{
	int rssi,rxlevel;
	opencpu_csq(&rssi,&rxlevel);
	opencpu_printf( "%d,%d\n",rssi,rxlevel);
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
	memset(temp_read,0,10);
	opencpu_flash_erase(0,6);
	opencpu_flash_write(1,temp_write,strlen(temp_write));
	opencpu_flash_read(1,temp_read,6);
	opencpu_printf("read:%s\n",temp_read);
}
