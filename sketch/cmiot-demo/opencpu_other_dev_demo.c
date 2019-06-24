/*
   opencpu_other_dev.c
   created by xgl,2019/1/22
   brief:模组一些其他外设的示例
*/

#include "m5311_opencpu.h"

void test_pwm_init()
{
	#define DUTY_RATIO  50
	#define PWM_FREQ   200
	opencpu_pwm_init(PWM_FREQ,DUTY_RATIO);
}

//RTC timer超时回调函数
void rtc_timer_callback()
{
	opencpu_printf("rtc timer expires\n");
}

void rtc_timer_test()
{
	unsigned int handle;
	//定时5S，到期执行rtc_timer_callback函数，循环执行
	opencpu_rtc_timer_create(&handle,50, true,rtc_timer_callback);
	opencpu_rtc_timer_start(handle);
	opencpu_printf("rtc timer starts\n");
}

//测试ADC
void test_adc()
{
    opencpu_printf("vol:%d\n",opencpu_adc(HAL_ADC_CHANNEL_0));
}
