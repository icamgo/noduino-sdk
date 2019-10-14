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
#include "compile.h"

void quark_main()
{
	int i;
	unsigned char temp;
	unsigned  char iccid[40]={0};
	
	int iccid_len;
	unsigned char *p = 0;
	unsigned char temp1,temp2;

	custom_uart_init();

	opencpu_printf("\r\n\r\n%s\r\n", noduino_banner);

	opencpu_printf(opencpu_fota_version_cb());

	opencpu_printf("\nHW VERSION:%d\r\n",get_band_version());
	opencpu_printf("Update Status:%d\r\n",update_status);
	opencpu_printf("Run Mode:%d\r\n",get_run_mode());

	//reset to AT mode
	//opencpu_printf("reset to AT mode...\r\n");
	//opencpu_at_open();

	if(opencpu_is_boot_from_sleep()==1)
	{
		opencpu_printf("BOOT CAUSE: WAKE FROM SLEEP\r\n");
	} else {
		opencpu_printf("BOOT CAUSE: POWER_ON OR RESET\r\n");
	}

	opencpu_printf("Waiting for network...\r\n");

	opencpu_lock_light_sleep();

	//阻塞方式获取SIM卡 ICCID
	//ICCID: 898602B4031600005170
	show_iccid();

	//获取网络注册状态，并阻塞等待网络注册成功
	opencpu_printf("Network Registering...\r\n");

	while(opencpu_cgact() != 1)
	{
		opencpu_printf(".");
		vTaskDelay(1000);
	}

	opencpu_printf("Network Register Success!\r\n");

	show_signal_strength();

	show_rtc_time();

	push_data_via_tcp();

	uart_cmd = 0;

	while(1)
	{
		if(uart_cmd == 'p')
		{
			push_data_via_tcp();
			uart_cmd = 0;
		}

		if(uart_cmd == 'r')
		{
			opencpu_reboot();
			uart_cmd = 0;
		}

		if(uart_cmd == 'a')
		{
			test_dm();
			opencpu_printf("test pm\n");
			uart_cmd = 0;
		}

		if(uart_cmd == 'D')
		{
			test_dns();
			uart_cmd = 0;
		}

		if(uart_cmd == 'C')
		{
			show_rtc_time();
			uart_cmd = 0;
		}

		//设置psm
		if(uart_cmd == '6')
		{
			ril_power_saving_mode_setting_req_t psm_req1;
			psm_req1.mode=1;
			psm_req1.req_prdc_rau=NULL;
			psm_req1.req_gprs_rdy_tmr=NULL;
			psm_req1.req_prdc_tau="00101011";
			psm_req1.req_act_time="00100100";
	
			opencpu_set_psmparam(&psm_req1);
			uart_cmd = 0;
		}

		//读取psm设置值
		if(uart_cmd == '7')
		{
			ril_power_saving_mode_setting_rsp_t psm_rsp1;
			opencpu_get_psmparam(&psm_rsp1);
			opencpu_printf("%d,%s,%s,%s,%s\n",psm_rsp1.mode,psm_rsp1.req_prdc_rau,
				psm_rsp1.req_gprs_rdy_tmr,psm_rsp1.req_prdc_tau,psm_rsp1.req_act_time);
			uart_cmd = 0;
		}

		//测试EDRX参数读取
		 if(uart_cmd == 'E')
		{
			int temp_type;
			unsigned char temp_value[10];
				opencpu_read_edrx(&temp_type,temp_value);		
				opencpu_printf("type:%d,value:%s\n",temp_type,temp_value);
				uart_cmd = 0;			
		}

		//测试EDRX参数设置
		if(uart_cmd == 'e')
		{
			opencpu_set_edrx(1,5,"0101");
			opencpu_printf("edrx set ok\n");
			uart_cmd = 0;	
		}

		vTaskDelay(10);
	}
}

//opencpu主任务函数
void opencpu_task_main()
{
	quark_main();
	vTaskDelete(NULL);
}

/*
 新建opencpu任务，这个函数用户不可更改
*/
void test_opencpu_start()
{
	xTaskCreate(opencpu_task_main, "opencpu", 1024, NULL, TASK_PRIORITY_NORMAL, NULL);
}
